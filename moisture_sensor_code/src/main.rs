#![no_std]
#![no_main]

use core::{
    cell::RefCell,
    sync::atomic::{AtomicU16, Ordering},
};

use assign_resources::assign_resources;
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::interrupt::InterruptExt;
use embassy_stm32::*;
use embassy_time::{Instant, Timer};
use embedded_sdmmc::{self, SdCard, VolumeIdx, VolumeManager};
use gpio::{Level, Output, Speed};

use time_source::dummyTimeSource;
use {defmt_rtt as _, panic_probe as _};
mod time_source;
// define constants
const LOG_INTERVAL: u64 = 5; // at n*LOG_INTERVAL seconds a log entry is written
assign_resources! {
    valve_control: ValveResources {
        adc: ADC1,
        valve_pin: PC13,
        moisture_pin: PA0,
    },
    level_setting: SettingResources {
        adc: ADC2,
        measure_pin: PA4,
    }
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
});
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Hallo ");
    let p = embassy_stm32::init(Default::default());
    let r = split_resources!(p);

    // setup relay
    _spawner.spawn(handle_valve(r.valve_control)).unwrap();
    _spawner
        .spawn(handle_modify_threshold(r.level_setting))
        .unwrap();

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
    let spi_bus = RefCell::new(spi);
    let spi_bus: embassy_sync::blocking_mutex::Mutex<
        embassy_sync::blocking_mutex::raw::NoopRawMutex,
        _,
    > = embassy_sync::blocking_mutex::Mutex::new(spi_bus);
    let sd_control_pin = Output::new(resources.ss, Level::High, Speed::Medium);
    let spi_dev =
        embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice::new(&spi_bus, sd_control_pin);

    // setup sd card
    let sd_card = SdCard::new(spi_dev, embassy_time::Delay);
    config.frequency = time::mhz(16);

    // need time source, maybe write self, just return zero always
    let mut vol = VolumeManager::new(sd_card, dummyTimeSource::dummy());

    // always select the first volume on the card, sacrificing a 8gb sd card is not something that
    // has to be avoided. We assume this sd card is dedicated to this task.
    if let Ok(mut vol0) = vol.open_volume(VolumeIdx(0)) {
        if let Ok(mut root_dir) = vol0.open_root_dir() {
            if let Ok(mut file) =
                root_dir.open_file_in_dir("log_file.csv", embedded_sdmmc::Mode::ReadWriteTruncate)
            {
                // file was successfully opened, otherwise this will have failed
                loop {
                    let write_res = file.write(b"how are you\n");
                    if write_res.is_err() {
                        info!("writing to sd card failed");
                    }
                    Timer::after_secs(10).await;
                }
            }
        }
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
async fn handle_modify_threshold(resources: SettingResources) {
    let mut adc = adc::Adc::new(resources.adc);
    unsafe {
        // adc needs a hack in embassy to work. See embassy issue #2162
        interrupt::ADC1_2.enable();
    }
    let mut read_pin = resources.measure_pin;
    loop {
        Timer::after_secs(5).await;
        let v = adc.read(&mut read_pin).await;
        info!("comparison: {}", v);
        SHARED.threshold.fetch_add(1, Ordering::Relaxed);
        SHARED.hysterese.store(v, Ordering::Relaxed);
    }
}
