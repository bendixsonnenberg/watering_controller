#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU16, AtomicU8, Ordering};

use assign_resources::assign_resources;
use can::{
    filter::{self, Mask16},
    Can,
};
use can_contract::*;
use defmt::*;
use embassy_executor::Spawner;

use embassy_stm32::{
    gpio::{AnyPin, Input},
    interrupt::InterruptExt,
};
use embassy_stm32::{rcc::Sysclk, time::Hertz, *};
use embassy_time::Timer;
use embedded_hal_bus::i2c::AtomicDevice;
use gpio::{Level, Output, Speed};

use {defmt_rtt as _, panic_probe as _};
mod time_source;
// define constants
const LOG_INTERVAL: u64 = 5; // at n*LOG_INTERVAL seconds a log entry is written
const CAN_BITRATE: u32 = 125_000; // bitrate for can bus. We are not transfering large amounts of
                                  // data ,so lets keep this low
const CONTROLLER_ID: u8 = 0;

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
}
#[derive(Default)]
struct SharedData {
    threshold: AtomicU16,
    hysterese: AtomicU16,
    moisture: AtomicU16,
    watering_time: AtomicU16,
    backoff_time: AtomicU16,
}
static SHARED: SharedData = SharedData {
    threshold: AtomicU16::new(0),
    hysterese: AtomicU16::new(0),
    moisture: AtomicU16::new(0),
    watering_time: AtomicU16::new(30),
    backoff_time: AtomicU16::new(15),
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
        let pins: [AnyPin; 7] = [
            r.b0.into(),
            r.b1.into(),
            r.b2.into(),
            r.b3.into(),
            r.b4.into(),
            r.b5.into(),
            r.b6.into(),
            // r.b7.into(),
        ];
        for pin in pins {
            let input = Input::new(pin, pull);
            if input.is_low() {
                info!("is high");
                dev_id += 1;
            }
            info!("dev_id:{}", dev_id);
            dev_id *= 2;
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
        .spawn(handle_modify_threshold(r.can_control))
        .unwrap();

    //
    // create spi device
}

#[embassy_executor::task]
async fn handle_valve(resources: ValveResources) {
    let mut adc = adc::Adc::new(resources.adc);
    let mut moisture_level_pin = resources.moisture_pin;
    let mut valve_pin = Output::new(resources.valve_pin, Level::Low, Speed::Low);
    // let mut vrefint = adc.enable_vref();
    unsafe {
        // adc needs a hack in embassy to work. See embassy issue #2162
        interrupt::ADC1_2.enable();
    }

    // let vref_sample = adc.read(&mut vrefint).await;
    // info!("vref_sample: {}", vref_sample);

    let value_dry = 2400;
    let value_wet = 2200;
    let threshold = (value_dry - value_wet) / 2 + value_wet;
    SHARED.threshold.store(threshold, Ordering::Relaxed);
    let hystere = 20;
    loop {
        let v = adc.read(&mut moisture_level_pin).await;
        trace!("Sample: {}", v);
        SHARED.moisture.store(v, Ordering::Relaxed);

        let threshold = SHARED.threshold.load(Ordering::Relaxed);
        if v > threshold - hystere {
            // wet condition, set to fill put water, then wait for 2min before trying to water
            // again
            info!("watering now");
            valve_pin.set_high();
            Timer::after_secs(SHARED.watering_time.load(Ordering::Relaxed) as u64).await;
            valve_pin.set_low();
            info!("waiting for backoff");
            Timer::after_secs(SHARED.backoff_time.load(Ordering::Relaxed) as u64 * 60).await;
        }
        Timer::after_millis(100).await;
    }
}

#[embassy_executor::task]
async fn handle_modify_threshold(can_resources: CanResources) {
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
            let Some((command, _, data)) = frame_to_command_data(frame) else {
                // ignore
                // device id, has to be the id of this device anyways
                continue;
            };
            match command {
                Commands::Threshold => {
                    info!("Threshold: {:?}", data);
                    // THRESHOLD
                    if frame.header().rtr() {
                        // send data
                        send_data_over_can(
                            &mut can,
                            SHARED.threshold.load(Ordering::Relaxed),
                            command,
                            CONTROLLER_ID,
                            dev_id,
                        )
                        .await;
                    } else {
                        let len = frame.header().len();
                        if len >= 2 {
                            let new_threshold = data.expect("not rtr, should exists");
                            SHARED.threshold.store(new_threshold, Ordering::Relaxed);
                        }
                    }
                }
                Commands::Hysterese => {
                    info!("Hysterese: {:?}", data);
                    // hysterses
                    if frame.header().rtr() {
                        // send data
                        send_data_over_can(
                            &mut can,
                            SHARED.hysterese.load(Ordering::Relaxed),
                            command,
                            CONTROLLER_ID,
                            dev_id,
                        )
                        .await;
                    } else {
                        let len = frame.header().len();
                        if len >= 2 {
                            let new_threshold = data.expect("not rtr, should exists");
                            SHARED.hysterese.store(new_threshold, Ordering::Relaxed);
                        }
                    }
                }
                Commands::Moisture => {
                    info!("Moisture: {:?}", data);
                    // moisture
                    if frame.header().rtr() {
                        // send data
                        send_data_over_can(
                            &mut can,
                            SHARED.moisture.load(Ordering::Relaxed),
                            command,
                            CONTROLLER_ID,
                            dev_id,
                        )
                        .await;
                    }
                }
                Commands::BackoffTime => {
                    info!("BackoffTime: {:?}", data);
                    if !frame.header().rtr() {
                        SHARED
                            .backoff_time
                            .store(data.expect("not rtr should exists"), Ordering::Relaxed);
                    } else {
                        send_data_over_can(
                            &mut can,
                            SHARED.backoff_time.load(Ordering::Relaxed),
                            command,
                            CONTROLLER_ID,
                            dev_id,
                        )
                        .await;
                    }
                }
                Commands::WateringTime => {
                    info!("WateringTime: {:?}", data);
                    if !frame.header().rtr() {
                        SHARED
                            .watering_time
                            .store(data.expect("not rtr should exists"), Ordering::Relaxed);
                    } else {
                        send_data_over_can(
                            &mut can,
                            SHARED.watering_time.load(Ordering::Relaxed),
                            command,
                            CONTROLLER_ID,
                            dev_id,
                        )
                        .await;
                    }
                }
                Commands::Announce => {
                    info!("Received announce");
                    if !frame.header().rtr() {
                        // some other sensor sent a announcement with our device id.
                        // just
                    } else {
                        send_data_over_can(&mut can, 0, command, CONTROLLER_ID, dev_id).await;
                    }
                }
            }
        }
    }
}
async fn send_data_over_can(
    can: &mut Can<'_>,
    data: u16,
    command: Commands,
    target_id: u8,
    source_id: u8,
) {
    let buf = can_contract::create_data_buf(data, source_id);
    let Some(std_id) = can_contract::id_from_command_and_dev_id(command, target_id) else {
        error!("failed generating id");
        return;
    };
    let Ok(frame) = can::Frame::new_standard(std_id.as_raw(), &buf) else {
        error!("failed creating frame");
        return;
    };
    let r = can.write(&frame).await;
}

async fn init_can(resourses: CanResources) -> Can<'static> {
    let dev_id = DEV_ID.load(Ordering::Relaxed);
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
    can.enable().await;
    info!("sending announcement");
    send_data_over_can(&mut can, 0, Commands::Announce, CONTROLLER_ID, dev_id).await;
    can
}
