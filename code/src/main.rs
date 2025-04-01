#![no_std]
#![no_main]

use core::{
    cell::RefCell,
    sync::atomic::{AtomicU16, Ordering},
};

use adc::Adc;
use assign_resources::assign_resources;
use defmt::*;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::interrupt::InterruptExt;
use embassy_stm32::*;
use embassy_sync::{
    blocking_mutex::{
        raw::{CriticalSectionRawMutex, NoopRawMutex},
        CriticalSectionMutex,
    },
    mutex::{self, Mutex},
};
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_sdmmc::{self, SdCard, VolumeManager};
use gpio::{Input, Level, Output, Speed};
use peripherals::{ADC1, SPI1};
use spi::Spi;
use static_cell::StaticCell;
use usart::{Config, DataBits, Parity, StopBits, Uart};

use time_source::dummyTimeSource;
use {defmt_rtt as _, panic_probe as _};
mod time_source;
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
}
static ATOMIC_THRESHOLD: AtomicU16 = AtomicU16::new(0);
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

    // sd cards can be addressed by spi.
    // there is a crate to hande the file system stuff, but it needs a embedded_hal abstraction,
    // therefore we use embassy_embedded_hal to adapt between them.
    //
    // create spi device
    let mut config = spi::Config::default();
    config.frequency = time::Hertz::mhz(32);
    let spi = spi::Spi::new_blocking(p.SPI1, p.PA5, p.PA7, p.PA6, config);
    let spi_bus = RefCell::new(spi);
    let spi_bus: embassy_sync::blocking_mutex::Mutex<
        embassy_sync::blocking_mutex::raw::NoopRawMutex,
        _,
    > = embassy_sync::blocking_mutex::Mutex::new(spi_bus);
    let sd_control_pin = Output::new(p.PA9, Level::High, Speed::Medium);
    let spi_dev =
        embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice::new(&spi_bus, sd_control_pin);

    // setup sd card
    let mut sd_card = SdCard::new(spi_dev, embassy_time::Delay);

    // need time source, maybe write self, just return zero always
    let mut vol = VolumeManager::new(sd_card, dummyTimeSource::dummy());
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
    ATOMIC_THRESHOLD.store(threshold, Ordering::Relaxed);
    let hystere = 100;
    loop {
        //info!("starting loop");
        let v = adc.read(&mut moisture_level_pin).await;

        //info!("Sample: {}", v);

        let threshold = ATOMIC_THRESHOLD.load(Ordering::Relaxed);
        //info!("THRESHOLD: {}", threshold);
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
        ATOMIC_THRESHOLD.fetch_add(1, Ordering::Relaxed);
    }
}
