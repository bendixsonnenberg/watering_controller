//! This example tests the RP Pico 2 W onboard LED.
//!
//! It does not work with the RP Pico 2 board. See `blinky.rs`.

#![no_std]
#![no_main]

use core::cell::RefCell;
use core::panic;

use assign_resources::*;
use can_contract::*;
use cyw43_pio::{DEFAULT_CLOCK_DIVIDER, PioSpi};
use defmt::unwrap;
use embassy_embedded_hal::SetConfig;
use embassy_executor::Spawner;
use embassy_rp::Peri;
use embassy_rp::adc::{self, Adc, Channel, Sample};
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{self, DMA_CH0, PIO0, SPI1, USB};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::spi::{Blocking, Spi};
use embassy_rp::usb::Driver;
use embassy_rp::{bind_interrupts, spi};
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, ThreadModeRawMutex};
use embassy_sync::blocking_mutex::{self, Mutex, NoopMutex};
use embassy_time::{Delay, Duration, Timer};
use embedded_can::Frame;
use embedded_graphics::primitives::line;
use embedded_hal_bus::spi::ExclusiveDevice;
use log::*;
use mcp2515::frame::CanFrame;
use mcp2515::*;
use serde::de::value;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};
const REFERENCE_RESISTOR_OHM: u16 = 100;
const ADC_JITTER_ALLOWANCE: u16 = 100; // the amount the adc reading has to differ for it to be
// considred changed
const ADC_MAX: u16 = 4096;
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
        spi: SPI1,
        miso: PIN_8,
        mosi: PIN_11,
        clk: PIN_10,
        cs: PIN_13
    },
    adcs: Adcs{
        adc: ADC,
        dma: DMA_CH1,
        pin0: PIN_26,
        pin1: PIN_27,
        pin2: PIN_28,
    }
}

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
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
    Timer::after_secs(5).await;
    info!("hello");
    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;
    let can_mutex: blocking_mutex::Mutex<ThreadModeRawMutex, RefCell<CanBus>> =
        blocking_mutex::Mutex::new(RefCell::new(init_can(r.spi_can).await));

    unwrap!(spawner.spawn(values_from_adc(can_mutex, r.adcs)));
    // setting up can bus

    let delay = Duration::from_millis(250);
    loop {
        control.gpio_set(0, true).await;
        Timer::after(delay).await;

        control.gpio_set(0, false).await;
        Timer::after(delay).await;
    }
}

#[embassy_executor::task]
async fn values_from_adc(can: Mutex<ThreadModeRawMutex, RefCell<CanBus>>, adc_ressources: Adcs) {
    info!("starting adcs");
    let mut adc = Adc::new(adc_ressources.adc, Irqs, embassy_rp::adc::Config::default());
    let mut dma = adc_ressources.dma;

    let mut channel0 = Channel::new_pin(adc_ressources.pin0, embassy_rp::gpio::Pull::None);
    let mut channel1 = Channel::new_pin(adc_ressources.pin1, embassy_rp::gpio::Pull::None);
    let mut channel2 = Channel::new_pin(adc_ressources.pin2, embassy_rp::gpio::Pull::None);

    let mut channels = [channel0, channel1, channel2];
    const SAMPLES: usize = 100;
    const NUM_CHANNELS: usize = 3;
    let mut threshold: u16 = 2250;
    let mut hysterese: u16 = 20;
    let mut example: u16 = 100;
    loop {
        info!("entering adc loop");
        let mut buf = [Sample::default(); { SAMPLES * NUM_CHANNELS }];
        let div = 48_000_000 / (100_000 * NUM_CHANNELS) - 1;
        adc.read_many_multichannel_raw(&mut channels, &mut buf, div as u16, dma.reborrow())
            .await;
        let delta = |(sum, count): (u32, u32), value: &Sample| {
            if value.good() {
                (sum + value.value() as u32, count + 1)
            } else {
                (sum, count)
            }
        };
        let (sum, count) = buf.iter().step_by(NUM_CHANNELS).fold((0, 0), delta);
        let v_0 = sum / count;
        info!("before can calls");
        let r_0 = voltage_divider_resistance_upper(v_0, REFERENCE_RESISTOR_OHM);
        let new_threshold = linear_correction(1900, 2500, 100, 9000, r_0);
        let (sum, count) = buf.iter().skip(1).step_by(NUM_CHANNELS).fold((0, 0), delta);
        let v_1 = sum / count;
        let r_1 = voltage_divider_resistance_upper(v_1, REFERENCE_RESISTOR_OHM);
        let new_hysterese = linear_correction(1900, 2500, 100, 9000, r_1);
        let (sum, count) = buf.iter().skip(2).step_by(NUM_CHANNELS).fold((0, 0), delta);
        let v_2 = sum / count;
        let r_2 = voltage_divider_resistance_upper(v_2, REFERENCE_RESISTOR_OHM);
        let new_example = linear_correction(1900, 2500, 100, 9000, r_2);
        if threshold.abs_diff(new_threshold) > ADC_JITTER_ALLOWANCE {
            threshold = new_threshold;
            info!("have to update threshold");
            can.lock(|x| {
                set_value(&mut x.borrow_mut(), Commands::Threshold, threshold, 1);
            });
        }
        if hysterese.abs_diff(new_hysterese) > ADC_JITTER_ALLOWANCE {
            hysterese = new_hysterese;
            info!("have to update hysterese");
            can.lock(|x| {
                set_value(&mut x.borrow_mut(), Commands::Hysterese, hysterese, 1);
            });
        }
        if example.abs_diff(new_example) > ADC_JITTER_ALLOWANCE {
            example = new_example;
            info!("have to update example");
        }
        Timer::after_millis(100).await;
    }
}

fn set_value(can: &mut CanBus, command: Commands, data: u16, dev_id: u8) {
    let Some(id) = can_contract::id_from_command_and_dev_id(command, dev_id) else {
        return;
    };
    let mut buf = [0_u8; 2];
    can_contract::write_data_buff(data, &mut buf);
    let Some(frame) = CanFrame::new(id, &buf) else {
        return;
    };
    match can.send_message(frame) {
        Ok(_) => {
            info!("sent can message")
        }
        Err(e) => {
            info!("{:?}", e)
        }
    }
}
// calculate the resistance of a unknown resistor, by using a known reference resistor. The
// measured resistance is the resistor to voltage
fn voltage_divider_resistance_upper(mesurement: u32, known: u16) -> u16 {
    let alpha = known as u32 * ADC_MAX as u32;
    let division = (alpha) / mesurement;
    (division as u16) - known
}
fn linear_correction(
    corrected_min: u16,
    corrected_max: u16,
    in_min: u16,
    in_max: u16,
    input: u16,
) -> u16 {
    let relative = (input - in_min) / (in_max - in_min);
    (corrected_max - corrected_min) * relative + corrected_min
}
async fn init_can(can_resources: SpiCan) -> CanBus {
    info!("initiating can");
    static SPI_CAN_BUS: StaticCell<NoopMutex<RefCell<Spi<SPI1, Blocking>>>> = StaticCell::new();
    let config = spi::Config::default();
    info!("creating spi device");
    let spi = Spi::new_blocking(
        can_resources.spi,
        can_resources.clk,
        can_resources.mosi,
        can_resources.miso,
        config,
    );
    let spi_bus = NoopMutex::new(RefCell::new(spi));
    let spi_bus = SPI_CAN_BUS.init(spi_bus);
    info!("spi_bus created");
    let cs_pin = Output::new(can_resources.cs, Level::High);
    let spi_device =
        embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice::new(spi_bus, cs_pin);

    info!("spie_device created");
    let mut can = MCP2515::new(spi_device);
    info!("can created");
    let settings = mcp2515::Settings {
        mode: regs::OpMode::Normal,
        can_speed: CanSpeed::Kbps125,
        mcp_speed: McpSpeed::MHz8,
        clkout_en: false,
    };
    match can.init(&mut Delay, settings) {
        Ok(()) => {}
        Err(e) => {
            info!("Failed to init can: {:?}", e);
            Timer::after_secs(1).await;
            panic!("failed")
        }
    };
    let Ok(_) = can.set_filter(
        filter::RxFilter::F0,
        embedded_can::Id::Standard(get_filter_from_id(0)),
    ) else {
        info!("Failed to set filter ");
        return can;
    }; // controller
    // gets t have id 0
    let Ok(_) = can.set_mask(filter::RxMask::Mask0, embedded_can::Id::Standard(MASK)) else {
        return can;
    };
    info!("successful initilization");
    can
}
