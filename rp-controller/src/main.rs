//! This example tests the RP Pico 2 W onboard LED.
//!
//! It does not work with the RP Pico 2 board. See `blinky.rs`.

#![no_std]
#![no_main]

use core::cell::RefCell;
use core::panic;

use assign_resources::*;
use can_contract::*;
use cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER};
use defmt::unwrap;
use embassy_embedded_hal::SetConfig;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{self, DMA_CH0, PIO0, SPI1, USB};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::spi::{Blocking, Spi};
use embassy_rp::usb::Driver;
use embassy_rp::Peri;
use embassy_rp::{bind_interrupts, spi};
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, ThreadModeRawMutex};
use embassy_sync::blocking_mutex::{self, NoopMutex};
use embassy_time::{Delay, Duration, Timer};
use embedded_can::Frame;
use embedded_hal_bus::spi::ExclusiveDevice;
use log::*;
use mcp2515::frame::CanFrame;
use mcp2515::*;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

type CanBus = MCP2515<
    embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice<
        'static,
        NoopRawMutex,
        Spi<'static, SPI1, Blocking>,
        Output<'static>,
    >,
>;
// Program metadata for `picotool info`.
// This isn't needed, but it's recommended to have these minimal entries.
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"Blinky Example"),
    embassy_rp::binary_info::rp_program_description!(
        c"This example tests the RP Pico 2 W's onboard LED, connected to GPIO 0 of the cyw43 \
        (WiFi chip) via PIO 0 over the SPI bus."
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];
assign_resources! {
    spi_can: SpiCan {
        spi: SPI1,
        miso: PIN_8,
        mosi: PIN_11,
        clk: PIN_10,
        cs: PIN_13
    }
}

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<USB>;
});

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}
#[embassy_executor::task]
async fn logger(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let fw = include_bytes!("../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../cyw43-firmware/43439A0_clm.bin");

    // To make flashing faster for development, you may want to flash the firmwares independently
    // at hardcoded addresses, instead of baking them into the program with `include_bytes!`:
    //     probe-rs download ../../cyw43-firmware/43439A0.bin --binary-format bin --chip RP2040 --base-address 0x10100000
    //     probe-rs download ../../cyw43-firmware/43439A0_clm.bin --binary-format bin --chip RP2040 --base-address 0x10140000
    //let fw = unsafe { core::slice::from_raw_parts(0x10100000 as *const u8, 230321) };
    //let clm = unsafe { core::slice::from_raw_parts(0x10140000 as *const u8, 4752) };

    let r = split_resources!(p);
    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        DEFAULT_CLOCK_DIVIDER,
        pio.irq0,
        cs,
        p.PIN_24,
        p.PIN_29,
        p.DMA_CH0,
    );

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (_net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    // setting up usb logging
    let driver = Driver::new(p.USB, Irqs);
    unwrap!(spawner.spawn(cyw43_task(runner)));
    unwrap!(spawner.spawn(logger(driver)));
    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;
    let can_mutex: blocking_mutex::Mutex<ThreadModeRawMutex, RefCell<CanBus>> =
        blocking_mutex::Mutex::new(RefCell::new(init_can(r.spi_can)));

    // setting up can bus

    let delay = Duration::from_millis(250);
    loop {
        info!("led on!");
        control.gpio_set(0, true).await;
        Timer::after(delay).await;

        info!("led off!");
        control.gpio_set(0, false).await;
        Timer::after(delay).await;
    }
}

fn init_can(can_resources: SpiCan) -> CanBus {
    static SPI_CAN_BUS: StaticCell<NoopMutex<RefCell<Spi<SPI1, Blocking>>>> = StaticCell::new();
    let config = spi::Config::default();
    let spi = Spi::new_blocking(
        can_resources.spi,
        can_resources.clk,
        can_resources.mosi,
        can_resources.miso,
        config,
    );
    let spi_bus = NoopMutex::new(RefCell::new(spi));
    let spi_bus = SPI_CAN_BUS.init(spi_bus);
    let cs_pin = Output::new(can_resources.cs, Level::High);
    let spi_device =
        embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice::new(spi_bus, cs_pin);

    let mut can = MCP2515::new(spi_device);
    let settings = mcp2515::Settings {
        mode: regs::OpMode::Normal,
        can_speed: CanSpeed::Kbps500,
        mcp_speed: McpSpeed::MHz16,
        clkout_en: false,
    };
    let Ok(()) = can.init(&mut Delay, settings) else {
        info!("Failed to init can_speed");
        panic!("failed")
    };
    let Ok(_) = can.set_filter(
        filter::RxFilter::F0,
        embedded_can::Id::Standard(get_filter_from_id(0)),
    ) else {
        return can;
    }; // controller
       // gets t have id 0
    let Ok(_) = can.set_mask(filter::RxMask::Mask0, embedded_can::Id::Standard(MASK)) else {
        return can;
    };
    can
}
