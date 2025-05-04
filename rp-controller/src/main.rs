//! This example tests the RP Pico 2 W onboard LED.
//!
//! It does not work with the RP Pico 2 board. See `blinky.rs`.

#![no_std]
#![no_main]

use core::cell::RefCell;
use core::panic;

use assign_resources::*;
use bitmaps::Bitmap;
use can_contract::*;
use core::fmt::Write;
use cyw43_pio::{DEFAULT_CLOCK_DIVIDER, PioSpi};
use defmt::unwrap;
use embassy_executor::Spawner;
use embassy_futures::select;
use embassy_rp::adc::{Adc, Channel, Sample};
use embassy_rp::gpio::{AnyPin, Input, Level, Output, Pull};
use embassy_rp::peripherals::{self, DMA_CH0, PIO0, SPI1, USB};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::spi::{Blocking, Spi};
use embassy_rp::usb::Driver;
use embassy_rp::{Peri, gpio};
use embassy_rp::{bind_interrupts, spi};
use embassy_sync::blocking_mutex::NoopMutex;
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, ThreadModeRawMutex};
use embassy_time::{Delay, Duration, Instant, Ticker, Timer, WithTimeout};
use embedded_can::Frame;
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_sdmmc::{SdCard, VolumeIdx, VolumeManager};
use heapless::String;
use log::*;
use mcp2515::frame::CanFrame;
use mcp2515::regs::CanInte;
use mcp2515::*;
use static_cell::{ConstStaticCell, StaticCell};
use {defmt_rtt as _, panic_probe as _};
const REFERENCE_RESISTOR_OHM: u16 = 100;
const ADC_JITTER_ALLOWANCE: u16 = 100; // the amount the adc reading has to differ for it to be
// considred changed
const ADC_MAX: u16 = 4096;
const CONTROLLER_DEV_ID: DevId = 0;
type CanBus = MCP2515<
    embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice<
        'static,
        NoopRawMutex,
        Spi<'static, SPI1, Blocking>,
        Output<'static>,
    >,
>;
enum CanData {
    Remote,
    Data(can_contract::CanData),
}
type CanChannel =
    embassy_sync::channel::Channel<ThreadModeRawMutex, (Commands, can_contract::DevId, CanData), 3>;
type CanSender = embassy_sync::channel::Sender<
    'static,
    ThreadModeRawMutex,
    (Commands, can_contract::DevId, CanData),
    3,
>;
type CanReceiver = embassy_sync::channel::Receiver<
    'static,
    ThreadModeRawMutex,
    (Commands, can_contract::DevId, CanData),
    3,
>;
/// type that contains all information for sending frame;
type CanChannelData = (Commands, DevId, CanData);
type SensorBitmap = embassy_sync::blocking_mutex::Mutex<ThreadModeRawMutex, RefCell<Bitmap<256>>>;
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
        cs: PIN_13,
    },
    can_int: CanInt {
        int: PIN_14,
    }
    adcs: Adcs{
        adc: ADC,
        dma: DMA_CH1,
        pin0: PIN_26,
        pin1: PIN_27,
        pin2: PIN_28,
    },
    spi_sdcard: SpiSdcard {
        spi: SPI0,
        miso: PIN_4,
        mosi: PIN_3,
        clk: PIN_2,
        cs: PIN_5,
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
    info!("hello");
    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    let can = init_can(r.spi_can).await;

    static CAN_CHANNEL: CanChannel = CanChannel::new();

    let can_rx = CAN_CHANNEL.receiver();
    let can_tx = CAN_CHANNEL.sender();
    // setting up a bitmap of existing sensors
    static SENSORS: StaticCell<SensorBitmap> = StaticCell::new();
    let sensors = SENSORS.init(embassy_sync::blocking_mutex::Mutex::new(RefCell::new(
        Bitmap::<256>::new(),
    )));

    unwrap!(spawner.spawn(can_task(
        can,
        Input::new(r.can_int.int, Pull::Up),
        sensors,
        can_tx,
        can_rx
    )));

    // populate the bitmap,
    // we want to do this before the other tasks are spawned to reduce the chance of the channels
    // overflowing
    poll_sensors(can_tx).await;

    unwrap!(spawner.spawn(values_from_adc(can_tx, r.adcs)));
    unwrap!(spawner.spawn(sd_card_log(can_tx, can_rx, sensors, r.spi_sdcard)));

    let delay = Duration::from_millis(250);
    loop {
        control.gpio_set(0, true).await;
        Timer::after(delay).await;

        control.gpio_set(0, false).await;
        Timer::after(delay).await;
    }
}
#[embassy_executor::task]
async fn menu_handle() {}

struct DummyTimesource();

impl embedded_sdmmc::TimeSource for DummyTimesource {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        embedded_sdmmc::Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

#[embassy_executor::task]
async fn sd_card_log(
    mut can_tx: CanSender,
    mut can_rx: CanReceiver,
    sensors: &'static SensorBitmap,
    mut sd_card_resources: SpiSdcard,
) {
    // this overarching loop enables the sd card to be inserted later
    loop {
        let mut config = spi::Config::default();
        config.frequency = 400_000; // for initilization the frequency has to be below 400khz
        let spi = Spi::new_blocking(
            sd_card_resources.spi.reborrow(),
            sd_card_resources.clk.reborrow(),
            sd_card_resources.mosi.reborrow(),
            sd_card_resources.miso.reborrow(),
            config,
        );
        let cs = Output::new(sd_card_resources.cs.reborrow(), Level::High);
        info!("got spi");
        let spi_dev = ExclusiveDevice::new_no_delay(spi, cs);
        info!("got spi_dev");

        let sdcard = SdCard::new(spi_dev, embassy_time::Delay);
        info!("got sd_card");

        let mut volume_mgr = VolumeManager::new(sdcard, DummyTimesource());

        let Ok(mut vol0) = volume_mgr.open_volume(VolumeIdx(0)) else {
            error!("Failed opening volume, not logging anymore, retrying");
            continue;
        };

        let Ok(mut root_dir) = vol0.open_root_dir() else {
            error!("Failed opening dir");
            continue;
        };

        let Ok(mut log_file) = root_dir.open_file_in_dir(
            "log_file.csv",
            embedded_sdmmc::Mode::ReadWriteCreateOrTruncate,
        ) else {
            error!("failed opening file");
            continue;
        };

        let Ok(_) = log_file
            .write(b"time, moisture1, moisture2, threshold, hysterese, backoff, wateringTime\n")
        else {
            continue;
        };
        let Ok(_) = log_file.flush() else {
            continue;
        };
        let mut ticker = Ticker::every(Duration::from_secs_floor(30));
        loop {
            let time = Instant::now().as_secs();
            let moisture1;
            let moisture2;
            let threshold;
            let hysterese;
            let backoff;
            let watering_time;
            // we need to drop can after using it
            {
                moisture1 =
                    get_value(&mut can_tx, &mut can_rx, Commands::Moisture, 1, sensors).await;
                moisture2 =
                    get_value(&mut can_tx, &mut can_rx, Commands::Moisture, 2, sensors).await;
                threshold =
                    get_value(&mut can_tx, &mut can_rx, Commands::Threshold, 1, sensors).await;
                hysterese =
                    get_value(&mut can_tx, &mut can_rx, Commands::Hysterese, 1, sensors).await;
                backoff =
                    get_value(&mut can_tx, &mut can_rx, Commands::BackoffTime, 1, sensors).await;
                watering_time =
                    get_value(&mut can_tx, &mut can_rx, Commands::WateringTime, 1, sensors).await;
            }
            // N needs to be large enough to hold all possible content written in the following
            // line
            // len(time) + 4 (moist1) + 4 (moist2) + 4(threshold) + 1 (hyst) + 2(backoff) +
            // 2(watering_time) + 2*6(, ) = len(time) + 29
            let mut buffer: String<64> = String::new();
            let Ok(_) = core::writeln!(
                &mut buffer,
                "{}, {}, {}, {}, {}, {}, {}",
                time,
                moisture1.unwrap_or(0),
                moisture2.unwrap_or(0),
                threshold.unwrap_or(0),
                hysterese.unwrap_or(0),
                backoff.unwrap_or(0),
                watering_time.unwrap_or(0),
            ) else {
                break;
            };
            let Ok(_) = log_file.write(buffer.as_bytes()) else {
                break;
            };
            let Ok(_) = log_file.flush() else {
                break;
            };
            ticker.next().await;
        }
        error!("failed at logging");
        Timer::after_secs(60).await;
    }
}

async fn poll_sensors(can_tx: CanSender) {
    for i in 1..DevId::MAX {
        // id 0 is the controller
        // ask every id if it exists
        can_tx.send((Commands::Announce, i, CanData::Remote)).await;
    }
}
/// handles all the can related stuff, mainly sending messages and routing incoming messages to the
/// correct receiver
#[embassy_executor::task]
async fn can_task(
    mut can: CanBus,
    mut can_interrupt_pin: Input<'static>,
    sensors: &'static SensorBitmap,
    receive_channel: CanSender,
    send_channel: CanReceiver,
) {
    loop {
        // the only interrupt configure is if we received a message
        // try to read, if we read a frame, return ready future
        // or create function for can bus that check the registers to see if there is a message
        // ready as async
        let Ok(status) = can.read_status() else {
            Timer::after_millis(100).await;
            continue;
        };

        // check if we already have something to read, otherwise wait for interrupt
        let receive_future = async {
            if status.rx0if() || status.rx1if() {
            } else {
                can_interrupt_pin.wait_for_low().await
            };
        };

        let tranceive_future = send_channel.receive();
        match select::select(receive_future, tranceive_future).await {
            select::Either::First(_) => {
                // reading from the can object also clears the interrupt bit for tranceive
                // this leads to the interrupt pin being high again
                // unless another message is already waiting in another receive buffer
                let Ok(frame) = can.read_message() else {
                    continue;
                };
                let Some(frame) = frame_to_command_data(frame) else {
                    continue;
                };
                match frame {
                    (Commands::Announce, Some(dev_id), _) => sensors.lock(|sensors| {
                        // tell everyone that this sensor is connected
                        let mut s = sensors.borrow_mut();
                        s.set(dev_id as usize, true);
                    }),
                    (command, Some(dev_id), Some(data)) => {
                        receive_channel.try_send((command, dev_id, CanData::Data(data)));
                    }
                    (_command, Some(_dev_id), None) => {
                        info!("received remote frame, need to handle it")
                    }
                    (_, _, _) => continue,
                }
            }
            // transmit this message
            select::Either::Second((command, dev_id, data)) => {
                let Some(id) = can_contract::id_from_command_and_dev_id(command, dev_id) else {
                    continue;
                };

                let frame = match data {
                    CanData::Remote => CanFrame::new_remote(id, can_contract::DLC),

                    CanData::Data(data) => {
                        let buf = can_contract::create_data_buf(data, CONTROLLER_DEV_ID);
                        CanFrame::new(id, &buf)
                    }
                };
                let Some(frame) = frame else { continue };

                // wait until we have a free transmit buffer
                loop {
                    if can.find_free_tx_buf().is_ok() {
                        break;
                    }
                    Timer::after_millis(1).await;
                }
                match can.send_message(frame) {
                    Ok(_) => {
                        info!("sent can message");
                    }
                    Err(e) => {
                        info!("{:?}", e);
                    }
                }
            }
        }
    }
}
#[embassy_executor::task]
async fn values_from_adc(mut can_tx: CanSender, adc_ressources: Adcs) {
    info!("starting adcs");
    let mut adc = Adc::new(adc_ressources.adc, Irqs, embassy_rp::adc::Config::default());
    let mut dma = adc_ressources.dma;

    let channel0 = Channel::new_pin(adc_ressources.pin0, embassy_rp::gpio::Pull::None);
    let channel1 = Channel::new_pin(adc_ressources.pin1, embassy_rp::gpio::Pull::None);
    let channel2 = Channel::new_pin(adc_ressources.pin2, embassy_rp::gpio::Pull::None);

    let mut channels = [channel1, channel2, channel0];
    const SAMPLES: usize = 100;
    const NUM_CHANNELS: usize = 3;
    let mut old_r0 = 5000;
    let mut old_r1 = 5000;
    let mut old_r2 = 5000;
    loop {
        let mut buf = [Sample::default(); { SAMPLES * NUM_CHANNELS }];
        let div = 48_000_000 / (100_000 * NUM_CHANNELS) - 1;

        adc.read_many_multichannel_raw(&mut channels, &mut buf, div as u16, dma.reborrow())
            .await;
        info!("read adc");
        let delta = |(sum, count): (u32, u32), value: &Sample| {
            if value.good() {
                (sum + value.value() as u32, count + 1)
            } else {
                (sum, count)
            }
        };
        // deinterlacing buf so that only the results for a single pin are considered
        let (sum, count) = buf.iter().step_by(NUM_CHANNELS).fold((0, 0), delta);
        let v_0 = sum / count;
        let r_0 = voltage_divider_resistance_upper(v_0, REFERENCE_RESISTOR_OHM);
        let new_threshold = linear_correction(2500, 1500, 100, 10100, r_0);
        let (sum, count) = buf.iter().skip(1).step_by(NUM_CHANNELS).fold((0, 0), delta);
        let v_1 = sum / count;
        let r_1 = voltage_divider_resistance_upper(v_1, REFERENCE_RESISTOR_OHM);
        let new_watering_time = linear_correction(15, 90, 100, 10100, r_1);
        let (sum, count) = buf.iter().skip(2).step_by(NUM_CHANNELS).fold((0, 0), delta);
        let v_2 = sum / count;
        let r_2 = voltage_divider_resistance_upper(v_2, REFERENCE_RESISTOR_OHM);
        let new_backoff_time = linear_correction(5, 30, 100, 10100, r_2);
        info!(
            "threshold: {}, {},{},| watering_time: {}, {}, {}| backoff_time: {}, {}, {}",
            new_threshold, v_0, r_0, new_watering_time, v_1, r_1, new_backoff_time, v_2, r_2
        );
        if r_0.abs_diff(old_r0) > ADC_JITTER_ALLOWANCE {
            old_r0 = r_0;
            info!("have to update threshold");
            info!("got can");
            set_value(&mut can_tx, Commands::Threshold, new_threshold, 1).await;
            set_value(&mut can_tx, Commands::Threshold, new_threshold, 2).await;
        }
        if r_1.abs_diff(old_r1) > ADC_JITTER_ALLOWANCE {
            old_r1 = r_1;
            info!("have to update watering time");
            set_value(&mut can_tx, Commands::WateringTime, new_watering_time, 1).await;
            set_value(&mut can_tx, Commands::WateringTime, new_watering_time, 2).await;
        }
        if r_2.abs_diff(old_r2) > ADC_JITTER_ALLOWANCE {
            old_r2 = r_2;
            info!("have to update backoff_time");
            set_value(&mut can_tx, Commands::BackoffTime, new_backoff_time, 1).await;
            set_value(&mut can_tx, Commands::BackoffTime, new_backoff_time, 2).await;
        }
        Timer::after_millis(100).await;
    }
}

// can set and get values
async fn set_value(
    can_tx: &mut CanSender,
    command: Commands,
    data: can_contract::CanData,
    dev_id: can_contract::DevId,
) {
    can_tx.send((command, dev_id, CanData::Data(data))).await
}
async fn get_value(
    can_tx: &mut CanSender,
    can_rx: &mut CanReceiver,
    command: Commands,
    dev_id: u8,
    sensors: &SensorBitmap,
) -> Option<can_contract::CanData> {
    can_tx.send((command, dev_id, CanData::Remote)).await;

    let Ok((r_command, r_source_id, CanData::Data(r_data))) = can_rx
        .receive()
        .with_timeout(Duration::from_millis(500))
        .await
    else {
        // the sensor is not responding, remove it from the list of active sensors
        //TODO: add error reporting
        sensors.lock(|sensor| sensor.borrow_mut().set(dev_id as usize, false));
        return None;
    };

    if r_command == command && r_source_id == dev_id {
        return Some(r_data);
    }
    None
}
// calculate the resistance of a unknown resistor, by using a known reference resistor. The
// measured resistance is the resistor to voltage
fn voltage_divider_resistance_upper(mesurement: u32, known: u16) -> u16 {
    let alpha = known as u32 * ADC_MAX as u32;
    // prevent division by 0
    let mesurement = mesurement.max(1);
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
    // Ensure the input is within the specified range
    if input < in_min {
        return corrected_min;
    } else if input > in_max {
        return corrected_max;
    }

    // Calculate the corrected value using linear interpolation
    let slope = (corrected_max as f32 - corrected_min as f32) / (in_max as f32 - in_min as f32);
    let corrected_value = slope * (input as f32 - in_min as f32) + corrected_min as f32;

    // Convert the result back to u16 and return
    corrected_value as u16
}
/*
fn linear_correction(
    corrected_min: u16,
    corrected_max: u16,
    in_min: u16,
    in_max: u16,
    input: u16,
) -> u16 {
    let m_upper = corrected_max.saturating_sub(corrected_min);
    let m_lower = in_max.saturating_sub(in_min).max(1);
    let b = m_upper.saturating_mul(in_min).saturating_div(corrected_min);
    info!("m_0: {}, m_1: {}, b: {}", m_upper, m_lower, b);
    let upper: u32 = (input as u32).saturating_mul(m_upper as u32);
    let lower = upper.saturating_div(m_lower as u32);
    let res = lower.saturating_sub(b as u32);
    info!("upper: {}, lower: {}, res: {}", upper, lower, res);
    res as u16
}
*/
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
    // this enables the interrupt for RX0IE, RX1IE, MERRE, ERRE
    // we only want intterupts for receiving
    match can.init(&mut Delay, settings) {
        Ok(()) => {}
        Err(e) => {
            info!("Failed to init can: {:?}", e);
            Timer::after_secs(1).await;
            panic!("failed")
        }
    };

    info!("successful initilization");
    can
}
