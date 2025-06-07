//! This example tests the RP Pico 2 W onboard LED.
//!
//! It does not work with the RP Pico 2 board. See `blinky.rs`.

#![no_std]
#![no_main]
mod can;
mod menu_implementation;
mod sd_card;
use core::cell::RefCell;
use core::sync::atomic::AtomicU16;

use assign_resources::*;
use bitmaps::Bitmap;
use can::CanChannel;
use can::CanPayload;
use can::CanSender;
use can::can_task;
use can::init_can;
use can_contract::*;
use cyw43_pio::{DEFAULT_CLOCK_DIVIDER, PioSpi};
use defmt::unwrap;
use embassy_executor::Spawner;
use embassy_rp::Peri;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{self, DMA_CH0, PIO0, PIO1, USB};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::usb::Driver;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_time::{Duration, Timer};
use log::*;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

// self imports
use crate::sd_card::sd_card_log;
use menu_implementation::menu_handle;
// considred changed
/// type that contains all information for sending frame;
type SensorBitmap = embassy_sync::blocking_mutex::Mutex<ThreadModeRawMutex, RefCell<Bitmap<256>>>;

/// struct for shared data
struct DefaultSettings {
    threshold: AtomicU16,
    watering_time: AtomicU16,
    backoff_time: AtomicU16,
}
static SHARED_SETTINGS: DefaultSettings = DefaultSettings {
    threshold: AtomicU16::new(1800),
    watering_time: AtomicU16::new(15),
    backoff_time: AtomicU16::new(20),
};
// Program metadata for `picotool info`.
// This isn't needed, but it's recommended to have these minimal entries.
#[unsafe(link_section = ".bi_entries")]
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
        spi: SPI0,
        miso: PIN_4,
        mosi: PIN_3,
        clk: PIN_2,
        cs: PIN_5,
    },
    can_int: CanInt {
        int: PIN_6,
    },
    spi_sdcard: SpiSdcard {
        spi: SPI1,
        miso: PIN_8,
        mosi: PIN_11,
        clk: PIN_10,
        cs: PIN_13,
    },
    display_i2c: DisplayI2C {
        sda: PIN_18,
        sdc: PIN_19,
        i2c: I2C1,
    },
    menu_input: MenuInput {
        back: PIN_16,
        enter: PIN_22,
        pio: PIO1,
        encoder_clock: PIN_20,
        encoder_dt: PIN_21,
    },
}

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
    PIO1_IRQ_0 => InterruptHandler<PIO1>;
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<USB>;
    ADC_IRQ_FIFO => embassy_rp::adc::InterruptHandler;

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
    info!("hello");
    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    let can = init_can(r.spi_can).await;

    static CAN_SEND_CHANNEL: CanChannel = CanChannel::new();
    static CAN_RECEIVE_CHANNEL: CanChannel = CanChannel::new();

    // this channel are the messages that go from some tasks to the can
    let can_send_rx = CAN_SEND_CHANNEL.receiver();
    let can_send_tx = CAN_SEND_CHANNEL.sender();
    // this channel are the messages that go from the can to the tasks(sd_card_log)
    let can_receive_rx = CAN_RECEIVE_CHANNEL.receiver();
    let can_receive_tx = CAN_RECEIVE_CHANNEL.sender();
    // setting up a bitmap of existing sensors
    static SENSORS: StaticCell<SensorBitmap> = StaticCell::new();
    let sensors = SENSORS.init(embassy_sync::blocking_mutex::Mutex::new(RefCell::new(
        Bitmap::<256>::new(),
    )));

    unwrap!(spawner.spawn(can_task(
        can,
        Input::new(r.can_int.int, Pull::Up),
        sensors,
        can_receive_tx,
        can_send_rx,
        can_send_tx,
    )));
    info!("after can task");

    unwrap!(spawner.spawn(menu_handle(
        can_send_tx,
        can_receive_rx,
        sensors,
        r.display_i2c,
        r.menu_input
    )));

    unwrap!(spawner.spawn(sd_card_log(
        can_send_tx,
        can_receive_rx,
        sensors,
        r.spi_sdcard
    )));

    let delay = Duration::from_millis(250);
    loop {
        control.gpio_set(0, true).await;
        Timer::after(delay).await;

        control.gpio_set(0, false).await;
        Timer::after(delay).await;
    }
}

#[embassy_executor::task]
async fn poll_sensors(can_tx: CanSender) {
    for i in 1..DevId::MAX {
        // id 0 is the controller
        // ask every id if it exists
        info!("polling: {}", i);
        can_tx
            .send((Commands::Announce, i, CanPayload::Remote))
            .await;
    }
}
