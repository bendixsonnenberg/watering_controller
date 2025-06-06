//! This example tests the RP Pico 2 W onboard LED.
//!
//! It does not work with the RP Pico 2 board. See `blinky.rs`.

#![no_std]
#![no_main]

use core::cell::RefCell;
use core::panic;
use core::sync::atomic::AtomicU16;

use assign_resources::*;
use bitmaps::Bitmap;
use can_contract::*;
use core::fmt::Write;
use cyw43_pio::{DEFAULT_CLOCK_DIVIDER, PioSpi};
use defmt::unwrap;
use embassy_executor::Spawner;
use embassy_futures::select;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{self, DMA_CH0, I2C1, PIO0, PIO1, SPI0, USB};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::pio_programs::rotary_encoder::{Direction, PioEncoder, PioEncoderProgram};
use embassy_rp::spi::Spi;
use embassy_rp::usb::Driver;
use embassy_rp::{Peri, i2c};
use embassy_rp::{bind_interrupts, spi};
use embassy_sync::blocking_mutex::NoopMutex;
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, ThreadModeRawMutex};
use embassy_sync::channel::TrySendError;
use embassy_time::{Delay, Duration, Instant, Ticker, Timer, WithTimeout};
use embedded_can::Frame;
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_sdmmc::{SdCard, VolumeIdx, VolumeManager};
use heapless::String;
use lcd_lcm1602_i2c::sync_lcd::Lcd;
use log::*;
use mcp2515::frame::CanFrame;
use mcp2515::*;
use menu::{MenuRunner, Sensor};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};
// considred changed
const CONTROLLER_DEV_ID: DevId = 0;
const TX_BACKOFF_TIME: u64 = 5; // time to wait if no tx buffer is available in ms
const CHANNEL_SIZE: usize = 100;

type CanBus = MCP2515<
    embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice<
        'static,
        NoopRawMutex,
        Spi<'static, SPI0, spi::Blocking>,
        Output<'static>,
    >,
>;
type CanChannelData = (Commands, can_contract::DevId, CanPayload);
#[derive(Debug)]
enum CanPayload {
    Remote,
    Data(can_contract::CanData),
}
type CanChannel = embassy_sync::channel::Channel<ThreadModeRawMutex, CanChannelData, CHANNEL_SIZE>;
type CanSender =
    embassy_sync::channel::Sender<'static, ThreadModeRawMutex, CanChannelData, CHANNEL_SIZE>;
type CanReceiver =
    embassy_sync::channel::Receiver<'static, ThreadModeRawMutex, CanChannelData, CHANNEL_SIZE>;
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
    backoff_time: AtomicU16::new(5),
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
#[derive(Clone, Copy)]
struct SensorBuilder {
    sensors: &'static SensorBitmap,
    can_tx: CanSender,
    can_rx: CanReceiver,
}
impl SensorBuilder {
    fn lean_sensor_from_id(self, id: Option<usize>) -> Option<MenuSensor> {
        if let Some(id) = id {
            if self.sensors.lock(|map| map.borrow().get(id as usize)) {
                return Some(MenuSensor {
                    builder: self,
                    id: id as u8,
                    threshold: None,
                });
            }
        }
        // id = 0 shows that no sensor was found
        None
    }
    // try to get the threshold for this sensor, if it failes return none
    async fn populate(mut self, mut sensor: MenuSensor) -> Option<MenuSensor> {
        info!(
            "populating with: id: {}, threshold: {:?}",
            sensor.id, sensor.threshold
        );
        if let Some(threshold) = get_value(
            &mut self.can_tx,
            &mut self.can_rx,
            Commands::Threshold,
            sensor.id,
            &self.sensors,
        )
        .await
        {
            sensor.threshold = Some(threshold);
            Some(sensor)
        } else {
            error!("failed at getting threshold value");
            None
        }
    }
}
#[derive(Clone)]
struct MenuSensor {
    #[allow(dead_code)]
    builder: SensorBuilder,
    id: u8,
    threshold: Option<u16>,
}
impl Sensor<SensorBuilder> for MenuSensor {
    fn first(builder: SensorBuilder) -> Option<Self> {
        let id = builder.sensors.lock(|cell| cell.borrow().first_index());
        builder.lean_sensor_from_id(id)
    }
    fn next(self, builder: SensorBuilder) -> Option<Self> {
        let id = builder
            .sensors
            .lock(|cell| cell.borrow().next_index(self.id as usize));
        match id {
            Some(id) => builder.lean_sensor_from_id(Some(id)),
            None => Sensor::first(builder),
        }
    }
    async fn populate(self, builder: SensorBuilder) -> Option<Self> {
        builder.populate(self).await
    }
    fn prev(self, builder: SensorBuilder) -> Option<Self> {
        let id = builder.sensors.lock(|cell| {
            let mut id = cell.borrow().prev_index(self.id as usize);
            if id.is_none() {
                id = cell.borrow().last_index();
            }
            id
        });
        builder.lean_sensor_from_id(id)
    }
    fn get_setting(&self) -> u16 {
        self.threshold.unwrap_or(0)
    }
    fn get_id(&self) -> u8 {
        self.id
    }
    fn increase_setting(mut self, mut _builder: SensorBuilder) -> Option<Self> {
        info!("increase");
        if let Some(threshold) = self.threshold {
            self.threshold = Some(threshold.saturating_add(1));
        } else {
            self.threshold = None
        }
        self.send_over_can();
        Some(self)
    }
    fn decrease_setting(mut self, mut _builder: SensorBuilder) -> Option<Self> {
        info!("decrease");
        if let Some(threshold) = self.threshold {
            self.threshold = Some(threshold.saturating_sub(1));
        } else {
            self.threshold = None
        }
        self.send_over_can();
        Some(self)
    }
}

impl MenuSensor {
    fn send_over_can(&mut self) {
        if let Some(threshold) = self.threshold {
            let mut tx = self.builder.can_tx;
            set_value(&mut tx, Commands::Threshold, threshold, self.id)
        }
    }
}
fn write_two_lines(
    lcd: &mut Lcd<'_, i2c::I2c<'_, I2C1, i2c::Blocking>, embassy_time::Delay>,
    text: &str,
) {
    let _ = lcd.clear();
    let _ = lcd.set_cursor(0, 0);
    let (first, second) = match text.split_once(|c| c == '\n') {
        Some((first, second)) => (first, second),
        None => (text, ""),
    };
    match lcd.write_str(first) {
        Ok(_) => {}
        Err(e) => {
            info!("{:?}", e);
        }
    }
    let _ = lcd.set_cursor(1, 0);
    match lcd.write_str(second) {
        Ok(_) => {}
        Err(e) => {
            info!("{:?}", e);
        }
    }
}
#[embassy_executor::task]
async fn menu_handle(
    can_tx: CanSender,
    can_rx: CanReceiver,
    sensors: &'static SensorBitmap,
    display_resources: DisplayI2C,
    menu_resources: MenuInput,
) {
    const ADDR: u8 = 0x27;
    let mut runner: MenuRunner<_, SensorBuilder> =
        MenuRunner::<MenuSensor, SensorBuilder>::new(SensorBuilder {
            sensors,
            can_tx,
            can_rx,
        });
    let mut i2c = i2c::I2c::new_blocking(
        display_resources.i2c,
        display_resources.sdc,
        display_resources.sda,
        i2c::Config::default(),
    );
    info!("created i2c");
    let mut delay = embassy_time::Delay;
    let mut lcd = match lcd_lcm1602_i2c::sync_lcd::Lcd::new(&mut i2c, &mut delay)
        .with_address(ADDR)
        .with_rows(2)
        .with_cursor_on(false)
        .init()
    {
        Ok(lcd) => lcd,
        Err(e) => {
            error!("failed at init of i2c: {:?}", e);
            Timer::after_millis(50).await;
            return;
        }
    };
    let mut back_button = Input::new(menu_resources.back, Pull::Up);
    let mut enter_button = Input::new(menu_resources.enter, Pull::Down);

    let Pio {
        mut common, sm0, ..
    } = Pio::new(menu_resources.pio, Irqs);
    let prg = PioEncoderProgram::new(&mut common);
    let mut encoder = PioEncoder::new(
        &mut common,
        sm0,
        menu_resources.encoder_clock,
        menu_resources.encoder_dt,
        &prg,
    );

    loop {
        runner.update().await;
        let mut buffer: String<33> = String::new();
        let Ok(_) = core::writeln!(&mut buffer, "{}", runner) else {
            break;
        };
        write_two_lines(&mut lcd, buffer.as_str());
        match embassy_futures::select::select3(
            back_button.wait_for_falling_edge(),
            enter_button.wait_for_falling_edge(),
            encoder.read(),
        )
        .await
        {
            select::Either3::First(_) => {
                info!("back");
                runner.input(menu::Input::Back).await;
            }
            select::Either3::Second(_) => {
                info!("enter");
                runner.input(menu::Input::Enter).await;
            }
            select::Either3::Third(Direction::Clockwise) => {
                info!("right");
                runner.input(menu::Input::Right).await;
            }
            select::Either3::Third(Direction::CounterClockwise) => {
                info!("left");
                runner.input(menu::Input::Left).await;
            }
        }
    }
}

#[allow(dead_code)]
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

        let mut ticker = Ticker::every(Duration::from_secs_floor(30));
        loop {
            let time = Instant::now().as_secs();
            // we need to drop can after using it
            let mut buf: String<32> = String::new();

            let Ok(_) = core::write!(&mut buf, "Time: {}", time) else {
                error!("failed writing time");
                break;
            };
            let mut failed = false;
            {
                let sensors_map = sensors.lock(|sensor| sensor.borrow().clone());
                let mut buffer: String<64> = String::new();
                for sensor in sensors_map.into_iter() {
                    let id = sensor as u8;
                    let Some(moisture) =
                        get_value(&mut can_tx, &mut can_rx, Commands::Moisture, id, sensors).await
                    else {
                        continue;
                    };
                    let Some(threshold) =
                        get_value(&mut can_tx, &mut can_rx, Commands::Threshold, id, sensors).await
                    else {
                        continue;
                    };
                    let Ok(_) = core::write!(
                        &mut buffer,
                        "Sen: {}, Thr: {}, Mo: {}",
                        id,
                        moisture,
                        threshold
                    ) else {
                        failed = true;
                        break;
                    };
                    let Ok(_) = log_file.write(buffer.as_bytes()) else {
                        failed = true;
                        break;
                    };
                }
                if failed {
                    error!("failed writing to sd card");

                    break;
                }
                let Ok(_) = log_file.write("\n".as_bytes()) else {
                    error!("failed newline");
                    break;
                };
                let Ok(_) = log_file.flush() else {
                    error!("failed flushing");
                    break;
                };
            }
            // N needs to be large enough to hold all possible content written in the following
            // line
            // len(time) + 4 (moist1) + 4 (moist2) + 4(threshold) + 1 (hyst) + 2(backoff) +
            // 2(watering_time) + 2*6(, ) = len(time) + 29
            ticker.next().await;
        }
        error!("failed at logging");
        Timer::after_secs(60).await;
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
/// sends settings to the given channel. May fail at sending if the channel is full
fn send_settings_to_sensor(
    dev_id: can_contract::DevId,
    channel: CanSender,
) -> Result<(), TrySendError<(Commands, can_contract::DevId, CanPayload)>> {
    channel.try_send((
        Commands::Threshold,
        dev_id,
        CanPayload::Data(
            SHARED_SETTINGS
                .threshold
                .load(core::sync::atomic::Ordering::Relaxed),
        ),
    ))?;
    channel.try_send((
        Commands::WateringTime,
        dev_id,
        CanPayload::Data(
            SHARED_SETTINGS
                .watering_time
                .load(core::sync::atomic::Ordering::Relaxed),
        ),
    ))?;
    channel.try_send((
        Commands::BackoffTime,
        dev_id,
        CanPayload::Data(
            SHARED_SETTINGS
                .backoff_time
                .load(core::sync::atomic::Ordering::Relaxed),
        ),
    ))
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
    self_send: CanSender,
) {
    loop {
        // or create function for can bus that check the registers to see if there is a message
        // ready as async
        let Ok(status) = can.read_status() else {
            Timer::after_millis(100).await;
            continue;
        };
        // these futures can be blocked, so do not modify the channals in them or do something that
        // changes a status

        // check if we already have something to read, otherwise wait for interrupt
        // info!("send channel full: {:?}", send_channel.is_full());
        // info!("rx0if: {:?}, rx1if: {:?}", status.rx0if(), status.rx1if());
        let receive_future = async {
            if status.rx0if() || status.rx1if() {
            } else {
                can_interrupt_pin.wait_for_low().await
            };
            // delay if we wont be able to do anything with the message
            // only a shot delay, because otherwise we miss messages on the bus
            if send_channel.is_full() {
                // this allows other tasks to do something with the message
                Timer::after_millis(TX_BACKOFF_TIME).await;
            };
        };

        let tranceive_future = async {
            loop {
                if can.find_free_tx_buf().is_ok() {
                    break;
                }
                Timer::after_millis(TX_BACKOFF_TIME).await;
            }
            // after we can send something, wait until we have something to send.
            send_channel.receive().await
            // tx cant get full again because the only way to fill them is if this future finishes
        };
        match select::select(receive_future, tranceive_future).await {
            select::Either::First(_) => {
                // info!("got frame");
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
                        // info!("got announcment fomr {}", dev_id);
                        // tell everyone that this sensor is connected
                        let mut s = sensors.borrow_mut();
                        s.set(dev_id as usize, true);
                        // info!("{:?}", s.first_index());
                        // tell the new sensor what the settings are

                        let _ = send_settings_to_sensor(dev_id, self_send);
                    }),
                    (command, Some(dev_id), Some(data)) => {
                        // use try send here to avoid blocking the task from interacting with the
                        // can
                        let r = receive_channel.try_send((command, dev_id, CanPayload::Data(data)));
                        match r {
                            Ok(_) => {}
                            Err(e) => {
                                info!("{:?}", e);
                            }
                        }
                    }
                    (_command, Some(_dev_id), None) => {
                        info!("received remote frame, need to handle it")
                    }
                    (_, _, _) => continue,
                }
            }
            // transmit this message
            select::Either::Second((command, dev_id, data)) => {
                // we have a free tx buffer and we have got a message, so we should not encounter
                // tx busy
                info!(
                    "sending can frame with: {:?}, id: {} data: {:?}",
                    command, dev_id, data
                );
                let Some(id) = can_contract::id_from_command_and_dev_id(command, dev_id) else {
                    continue;
                };

                let frame = match data {
                    CanPayload::Remote => CanFrame::new_remote(id, can_contract::DLC),

                    CanPayload::Data(data) => {
                        let buf = can_contract::create_data_buf(data, CONTROLLER_DEV_ID);
                        CanFrame::new(id, &buf)
                    }
                };
                let Some(frame) = frame else { continue };

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

// can set and get values
fn set_value(
    can_tx: &mut CanSender,
    command: Commands,
    data: can_contract::CanData,
    dev_id: can_contract::DevId,
) {
    if let Err(e) = can_tx.try_send((command, dev_id, CanPayload::Data(data))) {
        error!(
            "failed setting value for {:?}, dev_id {}, data {}, {:?}",
            command, dev_id, data, e
        );
    };
}
async fn get_value(
    can_tx: &mut CanSender,
    can_rx: &mut CanReceiver,
    command: Commands,
    dev_id: u8,
    sensors: &SensorBitmap,
) -> Option<can_contract::CanData> {
    can_tx.send((command, dev_id, CanPayload::Remote)).await;

    let Ok((r_command, r_source_id, CanPayload::Data(r_data))) = can_rx
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
async fn init_can(can_resources: SpiCan) -> CanBus {
    info!("initiating can");
    static SPI_CAN_BUS: StaticCell<NoopMutex<RefCell<Spi<SPI0, spi::Blocking>>>> =
        StaticCell::new();
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
    info!("set filter");
    // gets t have id 0
    let Ok(_) = can.set_mask(filter::RxMask::Mask0, embedded_can::Id::Standard(MASK)) else {
        return can;
    };
    info!("set mask");
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
    info!("initilized can");

    can
}
