#![no_std]
#![no_main]

use core::{
    cell::RefCell,
    sync::atomic::{AtomicU16, Ordering},
};

use assign_resources::assign_resources;
use byteorder::ByteOrder;
use can::{
    filter::{self, Mask16},
    Can, StandardId,
};
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
const CAN_BITRATE: u32 = 500_000; // bitrate for can bus. We are not transfering large amounts of
                                  // data ,so lets keep this low
const ID_MASK: u16 = 0b000_1111_1111; // maks for can, masks out the order type
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
    //_spawner.spawn(handle_valve(r.valve_control)).unwrap();
    //_spawner
    //    .spawn(handle_modify_threshold(r.level_setting, r.can_control))
    //    .unwrap();

    _spawner.spawn(write_to_sd(r.sd_card)).unwrap();
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
                loop {
                    let write_res = file.write(b"how are you\n");
                    info!("{:?}", write_res);
                    if write_res.is_err() {
                        info!("writing to sd card failed");
                    }

                    file.flush();
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
    let mut valve_pin = Output::new(resources.valve_pin, Level::High, Speed::Low);
    let mut vrefint = adc.enable_vref();
    unsafe {
        // adc needs a hack in embassy to work. See embassy issue #2162
        interrupt::ADC1_2.enable();
    }
    let vref_sample = adc.read(&mut vrefint).await;
    info!("vref_sample: {}", vref_sample);

    let value_dry = 2800;
    let value_wet = 1200;
    let threshold = (value_dry - value_wet) / 2 + value_wet;
    SHARED.threshold.store(threshold, Ordering::Relaxed);
    let hystere = 100;
    loop {
        //info!("starting loop");
        let v = adc.read(&mut moisture_level_pin).await;

        SHARED.moisture.store(v, Ordering::Relaxed);
        //info!("Sample: {}", v);

        let threshold = SHARED.threshold.load(Ordering::Relaxed);
        if v < threshold - hystere {
            // wet condition, set to fill put water, then wait for 2min before trying to water
            // again
            valve_pin.set_high();
        } else if v > threshold + hystere {
            valve_pin.set_low();
        }
        Timer::after_millis(100).await;
        //info!("Timer Done");
    }
}

#[embassy_executor::task]
async fn handle_modify_threshold(resources: SettingResources, can_resources: CanResources) {
    let adc = adc::Adc::new(resources.adc);
    unsafe {
        // adc needs a hack in embassy to work. See embassy issue #2162
        interrupt::ADC1_2.enable();
    }
    //let mut read_pin = resources.measure_pin;
    //   loop {
    //   Timer::after_secs(5).await;
    //    let v = adc.read(&mut read_pin).await;
    //    info!("comparison: {}", v);
    //    SHARED.threshold.fetch_add(1, Ordering::Relaxed);
    //    SHARED.hysterese.store(v, Ordering::Relaxed);
    //}
    let mut can = init_can(can_resources).await;
    info!("can initiated");

    send_data_over_can(&mut can, 16, 3).await;
    info!("sent dummy message");
    loop {
        let res = can.read().await;
        info!("got frame: {:?}", res);
        if let Ok(env) = res {
            let frame = env.frame;
            info!("frame received: {:?}", env);
            if let can::Id::Standard(message_id) = frame.id() {
                let command = message_id.as_raw() >> 8; // removing the device id and only
                                                        // keeping the command
                match command {
                    0 => {
                        // THRESHOLD
                        if frame.header().rtr() {
                            // send data
                            send_data_over_can(
                                &mut can,
                                SHARED.threshold.load(Ordering::Relaxed),
                                command,
                            )
                            .await;
                        } else {
                            let len = frame.header().len();
                            if len >= 2 {
                                let new_threshold = byteorder::LE::read_u16(frame.data());
                                SHARED.threshold.store(new_threshold, Ordering::Relaxed);
                            }
                        }
                    }
                    1 => {
                        // hysterses
                        if frame.header().rtr() {
                            // send data
                            send_data_over_can(
                                &mut can,
                                SHARED.hysterese.load(Ordering::Relaxed),
                                command,
                            )
                            .await;
                        } else {
                            let len = frame.header().len();
                            if len >= 2 {
                                let new_threshold = byteorder::LE::read_u16(frame.data());
                                SHARED.hysterese.store(new_threshold, Ordering::Relaxed);
                            }
                        }
                    }
                    2 => {
                        // moisture
                        if frame.header().rtr() {
                            // send data
                            send_data_over_can(
                                &mut can,
                                SHARED.moisture.load(Ordering::Relaxed),
                                command,
                            )
                            .await;
                        }
                    }

                    _ => (),
                }
            }
        }
    }
}
async fn send_data_over_can(can: &mut Can<'_>, data: u16, command: u16) {
    let buf = &mut [0, 0];
    byteorder::LE::write_u16(buf, data);
    let frame =
        can::frame::Frame::new(create_header_from_command_device_id(command, 0), buf).unwrap();
    can.write(&frame).await;
}
fn create_header_from_command_device_id(command: u16, device: u16) -> can::frame::Header {
    let id = device | (command << 8);
    let id = can::Id::Standard(can::StandardId::new(id).expect("failed to convert id"));
    can::frame::Header::new(id, 2, false)
}

async fn init_can(resourses: CanResources) -> Can<'static> {
    // first we define the filter, which will accept all messages directed at this device
    let mask = StandardId::new(ID_MASK).expect("hardcoded values should conform");
    let id = StandardId::new(0b0000_0001).unwrap(); // id is 8 bit, this will later be determined by dip switches
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
