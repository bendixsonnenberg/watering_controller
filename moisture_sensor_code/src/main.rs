#![no_std]
#![no_main]

use core::{
    fmt::Error,
    sync::atomic::{AtomicBool, AtomicI16, AtomicU16, AtomicU8, Ordering},
};

use assign_resources::assign_resources;
use can::{
    filter::{self, Mask16},
    Can,
};
use can_contract::*;
use defmt::*;
use dht_sensor::DhtReading;
use embassy_executor::Spawner;

use embassy_stm32::{
    gpio::{AnyPin, Input},
    interrupt::InterruptExt,
    timer::simple_pwm::{PwmPin, SimplePwm},
};
use embassy_stm32::{rcc::Sysclk, time::Hertz, *};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Delay, Duration, Instant, Timer, WithTimeout};
use gpio::{Level, Output, Speed};
use rand::{rngs::SmallRng, Rng, SeedableRng};

use {defmt_rtt as _, panic_probe as _};
// define constants
const CAN_BITRATE: u32 = 125_000; // bitrate for can bus. We are not transfering large amounts of
                                  // data ,so lets keep this low
const CONTROLLER_ID: u8 = 0;
static LED_SIGNAL: Signal<CriticalSectionRawMutex, LedState> = Signal::new();

assign_resources! {
    valve_control: ValveResources {
        adc: ADC1,
        valve_pin: PC13,
        moisture_pin: PA0,
    },
    can_control: CanResources {
        can: CAN,
        can_rx: PA11,
        can_tx: PA12,
    },
    dev_id: DevIdResources {
        b0: PB11,
        b1: PB10,
        b2: PB1,
        b3: PB0,
        b4: PA7,
        b5: PA6,
        b6: PA5,
        b7: PA4,

    }
    // the led pins and timer have to be together, it is impossible to do something forbidden here, since rustc will not compile
    led_pins: LedResources {
        timer: TIM2,
        red: PA2,
        green: PA1,
        blue: PA3,
    }

    // dht11
    enviroment_sensor_pins: EnvironmentResources {
        open_drain: PB12,
    }
}
#[derive(Default)]
struct SharedData {
    threshold: AtomicU16,
    moisture: AtomicU16,
    watering_time: AtomicU16,
    backoff_time: AtomicU16,
    sensor_exists: AtomicBool,
    tempertature: AtomicI16,
    humidity: AtomicU16,
}
static SHARED: SharedData = SharedData {
    threshold: AtomicU16::new(0),
    moisture: AtomicU16::new(0),
    watering_time: AtomicU16::new(30),
    backoff_time: AtomicU16::new(15),
    sensor_exists: AtomicBool::new(false),
    tempertature: AtomicI16::new(0),
    humidity: AtomicU16::new(0),
};
static DEV_ID: AtomicU8 = AtomicU8::new(0);
bind_interrupts!(struct Irqs {
    ADC1_2 => adc::InterruptHandler<peripherals::ADC1>;
    USB_LP_CAN1_RX0 => can::Rx0InterruptHandler<peripherals::CAN>;
    CAN1_RX1 => can::Rx1InterruptHandler<peripherals::CAN>;
    CAN1_SCE => can::SceInterruptHandler<peripherals::CAN>;
    USB_HP_CAN1_TX => can::TxInterruptHandler<peripherals::CAN>;
});
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Hallo ");
    let mut config = Config::default();
    config.rcc.hse = Some(rcc::Hse {
        freq: Hertz::mhz(8),
        mode: rcc::HseMode::Oscillator,
    });
    config.rcc.sys = Sysclk::HSE;
    let p = embassy_stm32::init(config);
    let r = split_resources!(p);

    // checking id
    {
        let r = r.dev_id;
        let mut dev_id: u8 = 0;
        let pull = gpio::Pull::Up;
        // this is ugly, should fix later
        let pins: [AnyPin; 8] = [
            r.b0.into(),
            r.b1.into(),
            r.b2.into(),
            r.b3.into(),
            r.b4.into(),
            r.b5.into(),
            r.b6.into(),
            r.b7.into(),
        ];
        for pin in pins {
            dev_id *= 2;
            let input = Input::new(pin, pull);
            if input.is_low() {
                dev_id += 1;
            }
        }
        if dev_id == 0 {
            error!("dev_id is incorrect");
            return;
        }
        DEV_ID.store(dev_id, Ordering::Relaxed);
        info!("got device id: {}", dev_id);
    }

    // setup relay
    _spawner.spawn(handle_valve(r.valve_control)).unwrap();
    _spawner
        .spawn(handle_can_communication(r.can_control))
        .unwrap();
    _spawner.spawn(led_control(r.led_pins)).unwrap();
    _spawner
        .spawn(enviroment_sensors(r.enviroment_sensor_pins))
        .unwrap();
}
#[embassy_executor::task]
async fn enviroment_sensors(env: EnvironmentResources) {
    let mut dht_pin = gpio::OutputOpenDrain::new(env.open_drain, Level::High, Speed::VeryHigh);
    Timer::after_millis(1000).await;
    loop {
        match dht_sensor::dht11::Reading::read(&mut embassy_time::Delay, &mut dht_pin) {
            Ok(dht_sensor::dht11::Reading {
                temperature,
                relative_humidity,
            }) => {
                SHARED.sensor_exists.store(true, Ordering::Relaxed);
                SHARED
                    .humidity
                    .store(relative_humidity as u16, Ordering::Relaxed);
                SHARED
                    .tempertature
                    .store(temperature as i16, Ordering::Relaxed);
            }
            Err(e) => match (e) {
                dht_sensor::DhtError::ChecksumMismatch => {
                    error!("checksum error")
                }
                dht_sensor::DhtError::Timeout => {
                    error!("timeout")
                }
                dht_sensor::DhtError::PinError(e) => {
                    error!("pin error{:?}", e)
                }
            },
        }
        Timer::after_secs(1).await;
    }
}
enum LedState {
    Color(u16, u16, u16),
    Random,
    Off,
}
#[embassy_executor::task]
async fn led_control(led: LedResources) {
    let led_pins = led;
    let red_pwm_pin = PwmPin::new_ch3(led_pins.red, gpio::OutputType::PushPull);
    let green_pwm_pin = PwmPin::new_ch2(led_pins.green, gpio::OutputType::PushPull);
    let blue_pwm_pin = PwmPin::new_ch4(led_pins.blue, gpio::OutputType::PushPull);
    let pwm = SimplePwm::new(
        led_pins.timer,
        None,
        Some(green_pwm_pin),
        Some(red_pwm_pin),
        Some(blue_pwm_pin),
        Hertz(50_000),
        timer::low_level::CountingMode::EdgeAlignedUp,
    );
    let channels = pwm.split();
    let mut blue_channel = channels.ch3;
    let mut green_channel = channels.ch4;
    let mut red_channel = channels.ch2;
    red_channel.set_duty_cycle_fully_off();
    green_channel.set_duty_cycle_fully_off();
    blue_channel.set_duty_cycle_fully_off();
    red_channel.enable();
    green_channel.enable();
    blue_channel.enable();
    let mut current_state = LedState::Off;
    const MAX: u16 = 0b11111;
    loop {
        use LedState::*;
        // the timeout makes it possible for the random setting to change color regularly
        if let Ok(new_current_state) = LED_SIGNAL
            .wait()
            .with_timeout(Duration::from_millis(250))
            .await
        {
            current_state = new_current_state;
        };
        match current_state {
            Off => {
                red_channel.set_duty_cycle_fully_off();
                green_channel.set_duty_cycle_fully_off();
                blue_channel.set_duty_cycle_fully_off();
            }
            Color(red, green, blue) => {
                red_channel.set_duty_cycle_fraction(red, MAX);
                green_channel.set_duty_cycle_fraction(green, MAX);
                blue_channel.set_duty_cycle_fraction(blue, MAX);
            }
            Random => {
                let mut rng = SmallRng::seed_from_u64(Instant::now().as_ticks());
                red_channel.set_duty_cycle_fraction(rng.random(), MAX);
                green_channel.set_duty_cycle_fraction(rng.random(), MAX);
                blue_channel.set_duty_cycle_fraction(rng.random(), MAX);
            }
        }
    }
}

#[embassy_executor::task]
async fn handle_valve(resources: ValveResources) {
    let mut adc = adc::Adc::new(resources.adc);
    // maximum sample time for more stable readings
    adc.set_sample_time(adc::SampleTime::CYCLES239_5);
    info!("starting handle valve");
    let mut moisture_level_pin = resources.moisture_pin;
    let mut valve_pin = Output::new(resources.valve_pin, Level::Low, Speed::Low);
    // let mut vrefint = adc.enable_vref();
    unsafe {
        // adc needs a hack in embassy to work. See embassy issue #2162
        interrupt::ADC1_2.enable();
    }
    info!("hack done");

    // let vref_sample = adc.read(&mut vrefint).await;
    // info!("vref_sample: {}", vref_sample);

    let threshold = 4000;
    SHARED.threshold.store(threshold, Ordering::Relaxed);
    let hystere = 20;
    let mut last_watering: Option<Instant> = None;
    loop {
        let v = adc.read(&mut moisture_level_pin).await;
        // info!("Sample: {}", v);
        SHARED.moisture.store(v, Ordering::Relaxed);

        let threshold = SHARED.threshold.load(Ordering::Relaxed);
        if v > threshold - hystere {
            // wet condition, set to fill put water,
            // check if we are still in backoff time
            let time_taken: Duration = {
                if let Some(watering) = last_watering {
                    if let Some(time_taken) = Instant::now().checked_duration_since(watering) {
                        time_taken
                    } else {
                        Duration::MAX
                    }
                } else {
                    Duration::MAX
                }
            };
            if time_taken
                > Duration::from_secs(SHARED.backoff_time.load(Ordering::Relaxed) as u64 * 60)
            {
                info!("watering now");
                valve_pin.set_high();
                Timer::after_secs(SHARED.watering_time.load(Ordering::Relaxed) as u64).await;
                valve_pin.set_low();
                last_watering = Some(Instant::now());
            }
        }
        Timer::after_millis(100).await;
    }
}

#[embassy_executor::task]
async fn handle_can_communication(can_resources: CanResources) {
    let mut can = init_can(can_resources).await;
    trace!("can initiated");

    let dev_id = DEV_ID.load(Ordering::Relaxed);
    // send_data_over_can(&mut can, 16, Commands::Threshold, dev_id).await;

    loop {
        let reg = stm32_metapac::can::Can::esr(stm32_metapac::CAN);

        trace!("Rec: {:?}", reg.read().rec());
        trace!("Tec: {:?}", reg.read().tec());
        let res = can.read().await;
        trace!("got frame: {:?}", res);
        if let Ok(env) = res {
            let frame = env.frame;
            trace!("frame received: {:?}", env);
            let Some(container) = frame_to_command_data(frame) else {
                // ignore
                // device id, has to be the id of this device anyways
                continue;
            };
            let to_send = match container {
                CommandDataContainer::Data {
                    target_id: _,
                    src_id: _,
                    data,
                } => match data {
                    CommandData::Settings(threshold, backoff_time, watering_time) => {
                        SHARED.threshold.store(threshold, Ordering::Relaxed);
                        SHARED.backoff_time.store(backoff_time, Ordering::Relaxed);
                        SHARED.watering_time.store(watering_time, Ordering::Relaxed);
                        None
                    }
                    CommandData::Light(red, green, blue) => {
                        LED_SIGNAL.signal(LedState::Color(red, green, blue));
                        None
                    }
                    CommandData::LightRandom => {
                        LED_SIGNAL.signal(LedState::Random);
                        None
                    }
                    CommandData::LightOff => {
                        LED_SIGNAL.signal(LedState::Off);
                        None
                    }
                    CommandData::Sensors(_, _, _) => None,
                    CommandData::Announce(_, _) => None,
                },
                CommandDataContainer::Remote { target_id: _, data } => match data {
                    CommandData::Settings(_, _, _) => Some(CommandDataContainer::Data {
                        target_id: CONTROLLER_ID,
                        src_id: dev_id,
                        data: CommandData::Settings(
                            SHARED.threshold.load(Ordering::Relaxed),
                            SHARED.watering_time.load(Ordering::Relaxed),
                            SHARED.backoff_time.load(Ordering::Relaxed),
                        ),
                    }),
                    CommandData::Sensors(_, _, _) => {
                        let (temperature, humidity) =
                            match SHARED.sensor_exists.load(Ordering::Relaxed) {
                                true => (
                                    Some(SHARED.tempertature.load(Ordering::Relaxed)),
                                    Some(SHARED.humidity.load(Ordering::Relaxed)),
                                ),
                                false => (None, None),
                            };
                        Some(CommandDataContainer::Data {
                            target_id: CONTROLLER_ID,
                            src_id: dev_id,
                            data: CommandData::Sensors(
                                Some(SHARED.moisture.load(Ordering::Relaxed)),
                                temperature,
                                humidity,
                            ),
                        })
                    }
                    CommandData::Light(_, _, _)
                    | CommandData::LightRandom
                    | CommandData::LightOff => None,
                    CommandData::Announce(_, _) => None,
                },
            };
            if let Some(to_send) = to_send {
                send_data_over_can(&mut can, to_send).await;
            } else {
                info!("no response required");
            }
        }
    }
}
async fn send_data_over_can(can: &mut Can<'_>, data: CommandDataContainer) {
    if let Some(frame) = command_data_to_frame::<can::Frame>(data) {
        let _ = can.write(&frame).await;
    } else {
        error!("failed creating can frame");
        return;
    }
}

async fn init_can(resourses: CanResources) -> Can<'static> {
    info!("starting can init");
    let dev_id = DEV_ID.load(Ordering::Relaxed);
    info!("dev_id_for can{}", dev_id);
    // first we define the filter, which will accept all messages directed at this device
    let mask = MASK;
    let id = get_filter_from_id(dev_id); // id is 8 bit, this will later be determined by dip switches

    let mask: Mask16 = Mask16::frames_with_std_id(id, mask);

    let mut can = Can::new(resourses.can, resourses.can_rx, resourses.can_tx, Irqs);

    can.modify_filters().enable_bank(
        0,
        can::Fifo::Fifo0,
        filter::BankConfig::Mask16([mask, mask]),
    );
    can.modify_config()
        .set_bitrate(CAN_BITRATE)
        .set_loopback(false)
        .set_silent(false);
    info!("enabeling can");
    can.enable().await;
    info!("sending announcement");
    Timer::after_millis((dev_id as u64 * 100) + 4000).await;
    send_data_over_can(
        &mut can,
        CommandDataContainer::Data {
            target_id: CONTROLLER_ID,
            src_id: dev_id,
            data: CommandData::Announce(dev_id, SensorFlags::empty()),
        },
    )
    .await;
    can
}
