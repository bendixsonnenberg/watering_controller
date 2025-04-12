#![no_std]
#![no_main]

use core::{
    cell::RefCell,
    sync::atomic::{AtomicU16, Ordering},
};

use assign_resources::assign_resources;
use can::{
    filter::{self, Mask16},
    Can, StandardId,
};
use can_contract::*;
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::interrupt::InterruptExt;
use embassy_stm32::*;
use embassy_time::{Instant, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_sdmmc::{self, sdcard::AcquireOpts, SdCard, VolumeIdx, VolumeManager};
use gpio::{Level, Output, Speed};

use time_source::DummyTimeSource;
use {defmt_rtt as _, panic_probe as _};
mod time_source;
// define constants
const LOG_INTERVAL: u64 = 5; // at n*LOG_INTERVAL seconds a log entry is written
const CAN_BITRATE: u32 = 125_000; // bitrate for can bus. We are not transfering large amounts of
                                  // data ,so lets keep this low
assign_resources! {
    valve_control: ValveResources {
        adc: ADC1,
        valve_pin: PC13,
        moisture_pin: PA0,
    },
    level_setting: SettingResources {
        adc: ADC2,
        measure_pin: PA4,
    },
    can_control: CanResources {
        can: CAN,
        can_rx: PA11,
        can_tx: PA12,
    },
    sd_card: SdCardResources {
        spi: SPI1,
        sclk: PA5,
        miso: PA6,
        mosi: PA7,
        ss: PA9
    }
}
struct SharedData {
    threshold: AtomicU16,
    hysterese: AtomicU16,
    moisture: AtomicU16,
}
static SHARED: SharedData = SharedData {
    threshold: AtomicU16::new(0),
    hysterese: AtomicU16::new(0),
    moisture: AtomicU16::new(0),
};
bind_interrupts!(struct Irqs {
    ADC1_2 => adc::InterruptHandler<peripherals::ADC1>, adc::InterruptHandler<peripherals::ADC2>;
    USB_LP_CAN1_RX0 => can::Rx0InterruptHandler<peripherals::CAN>;
    CAN1_RX1 => can::Rx1InterruptHandler<peripherals::CAN>;
    CAN1_SCE => can::SceInterruptHandler<peripherals::CAN>;
    USB_HP_CAN1_TX => can::TxInterruptHandler<peripherals::CAN>;
});
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Hallo ");
    let p = embassy_stm32::init(Default::default());
    let r = split_resources!(p);

    // setup relay
    _spawner.spawn(handle_valve(r.valve_control)).unwrap();
    _spawner
        .spawn(handle_modify_threshold(r.level_setting, r.can_control))
        .unwrap();

    // _spawner.spawn(write_to_sd(r.sd_card)).unwrap();
    // sd cards can be addressed by spi.
    // there is a crate to hande the file system stuff, but it needs a embedded_hal abstraction,
    // therefore we use embassy_embedded_hal to adapt between them.
    //
    // create spi device
}

#[embassy_executor::task]
async fn write_to_sd(resources: SdCardResources) {
    let mut config = spi::Config::default();
    config.frequency = time::Hertz::khz(400);
    let spi = spi::Spi::new_blocking(
        resources.spi,
        resources.sclk,
        resources.mosi,
        resources.miso,
        config,
    );
    let sd_control_pin = Output::new(resources.ss, Level::High, Speed::Low);
    let spi_dev = ExclusiveDevice::new_no_delay(spi, sd_control_pin).unwrap();

    let aquire_options = AcquireOpts {
        acquire_retries: 5,
        use_crc: false,
    };
    // setup sd card
    info!("Using {} hz", embassy_time::TICK_HZ);
    let sd_card = SdCard::new_with_options(spi_dev, embassy_time::Delay, aquire_options);
    info!("card size is:  {}", sd_card.num_bytes().unwrap());

    // need time source, maybe write self, just return zero always
    let mut vol = VolumeManager::new(sd_card, DummyTimeSource::dummy());

    // always select the first volume on the card, sacrificing a 8gb sd card is not something that
    // has to be avoided. We assume this sd card is dedicated to this task.
    let volume = vol.open_volume(VolumeIdx(0));
    info!("{:?}", volume);
    if let Ok(mut vol0) = volume {
        if let Ok(mut root_dir) = vol0.open_root_dir() {
            let file = root_dir.open_file_in_dir(
                "log_file.csv",
                embedded_sdmmc::Mode::ReadWriteCreateOrTruncate,
            );
            info!("{:?}", file);
            if let Ok(mut file) = file {
                // file was successfully opened, otherwise this will have failed
                //
                // writing formatting line!()
                // let _ = file.write(b"s, m, t, h\n");
                loop {
                    // let mut buffer: String<32> = String::new();
                    //
                    // let fmt_res = core::write!(
                    //     &mut buffer,
                    //     "{}, {}, {}, {}",
                    //     Instant::now().as_secs(),
                    //     SHARED.moisture.load(Ordering::Relaxed),
                    //     SHARED.threshold.load(Ordering::Relaxed),
                    //     SHARED.hysterese.load(Ordering::Relaxed)
                    // );
                    // let write_res = file.write(buffer.as_bytes());
                    // let flush_res = file.flush();
                    // if fmt_res.is_err() || write_res.is_err() || flush_res.is_err() {
                    //     info!(
                    //         "Writing to sd card failed, fmt: {:?}, write: {:?}, flush: {:?}",
                    //         fmt_res.is_err(),
                    //         write_res.is_err(),
                    //         flush_res.is_err()
                    //     );
                    // }
                    Timer::after_secs(10).await;
                }
            } else {
                info!("failed opening file");
            }
        } else {
            info!("Failed opening root_dir");
        }
    } else {
        info!("Failed opening volume");
    };
    info!("Opening sd card failed");
    let mut i = 0;
    loop {
        info!(
            "Time: {}s, Moisture: {}, Threshold: {}, Hysterese: {}",
            Instant::now().as_secs(),
            SHARED.moisture.load(Ordering::Relaxed),
            SHARED.threshold.load(Ordering::Relaxed),
            SHARED.hysterese.load(Ordering::Relaxed)
        );
        Timer::at(Instant::from_secs(LOG_INTERVAL * i)).await;
        i += 1;
    }
}
#[embassy_executor::task]
async fn handle_valve(resources: ValveResources) {
    let mut adc = adc::Adc::new(resources.adc);
    let mut moisture_level_pin = resources.moisture_pin;
    let mut valve_pin = Output::new(resources.valve_pin, Level::Low, Speed::Low);
    let mut vrefint = adc.enable_vref();
    unsafe {
        // adc needs a hack in embassy to work. See embassy issue #2162
        interrupt::ADC1_2.enable();
    }
    let vref_sample = adc.read(&mut vrefint).await;
    info!("vref_sample: {}", vref_sample);

    let value_dry = 2400;
    let value_wet = 2200;
    let threshold = (value_dry - value_wet) / 2 + value_wet;
    SHARED.threshold.store(threshold, Ordering::Relaxed);
    let hystere = 20;
    loop {
        //info!("starting loop");
        let v = adc.read(&mut moisture_level_pin).await;

        SHARED.moisture.store(v, Ordering::Relaxed);
        info!("Sample: {}", v);

        let threshold = SHARED.threshold.load(Ordering::Relaxed);
        if v > threshold - hystere {
            // wet condition, set to fill put water, then wait for 2min before trying to water
            // again
            info!("watering now");
            valve_pin.set_high();
            Timer::after_secs(30).await;
            valve_pin.set_low();
            info!("waiting for backoff");
            Timer::after_secs(15 * 60).await;
        }
        Timer::after_millis(100).await;
        //info!("Timer Done");
    }
}

#[embassy_executor::task]
async fn handle_modify_threshold(resources: SettingResources, can_resources: CanResources) {
    let mut can = init_can(can_resources).await;
    info!("can initiated");

    let dev_id = 1;
    send_data_over_can(&mut can, 16, Commands::Moisture, dev_id).await;
    info!("sent dummy message");
    loop {
        let res = can.read().await;
        info!("got frame: {:?}", res);
        if let Ok(env) = res {
            let frame = env.frame;
            info!("frame received: {:?}", env);
            let Some((command, _, data)) = frame_to_command_data(frame) else {
                // ignore
                // device id, has to be the id of this device anyways
                continue;
            };
            match command {
                Commands::Threshold => {
                    // THRESHOLD
                    if frame.header().rtr() {
                        // send data
                        send_data_over_can(
                            &mut can,
                            SHARED.threshold.load(Ordering::Relaxed),
                            command,
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
                    // hysterses
                    if frame.header().rtr() {
                        // send data
                        send_data_over_can(
                            &mut can,
                            SHARED.hysterese.load(Ordering::Relaxed),
                            command,
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
                    // moisture
                    if frame.header().rtr() {
                        // send data
                        send_data_over_can(
                            &mut can,
                            SHARED.moisture.load(Ordering::Relaxed),
                            command,
                            dev_id,
                        )
                        .await;
                    }
                }
            }
        }
    }
}
async fn send_data_over_can(can: &mut Can<'_>, data: u16, command: Commands, dev_id: u8) {
    let buf = &mut [0, 0];
    can_contract::write_data_buff(data, buf);
    let Some(std_id) = can_contract::id_from_command_and_dev_id(command, dev_id) else {
        error!("failed generating id");
        return;
    };
    let Ok(frame) = can::Frame::new_standard(std_id.as_raw(), buf) else {
        error!("failed creating frame");
        return;
    };
    can.write(&frame).await;
}

async fn init_can(resourses: CanResources) -> Can<'static> {
    // first we define the filter, which will accept all messages directed at this device
    let mask = MASK;
    let id = get_filter_from_id(1); // id is 8 bit, this will later be determined by dip switches
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
    can
}
