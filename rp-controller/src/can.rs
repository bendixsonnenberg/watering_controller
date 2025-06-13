use crate::SHARED_SETTINGS;
use crate::SensorBitmap;
use crate::SpiCan;
use can_contract::{CommandData, Commands, DevId, frame_to_command_data, get_filter_from_id};
use core::cell::RefCell;
use core::panic;
use embassy_futures::select;
use embassy_rp::gpio::{Input, Level, Output};
use embassy_rp::peripherals::SPI0;
use embassy_rp::spi;
use embassy_rp::spi::Spi;
use embassy_sync::blocking_mutex::NoopMutex;
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, ThreadModeRawMutex};
use embassy_sync::channel::TrySendError;
use embassy_time::{Delay, Duration, Timer, WithTimeout};
use embedded_can::Frame;
use log::*;
use mcp2515::frame::CanFrame;
use mcp2515::*;
use static_cell::StaticCell;

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
type CanChannelData = (Commands, can_contract::DevId, CommandData);
pub type CanChannel =
    embassy_sync::channel::Channel<ThreadModeRawMutex, CanChannelData, CHANNEL_SIZE>;
pub type CanSender =
    embassy_sync::channel::Sender<'static, ThreadModeRawMutex, CanChannelData, CHANNEL_SIZE>;
pub type CanReceiver =
    embassy_sync::channel::Receiver<'static, ThreadModeRawMutex, CanChannelData, CHANNEL_SIZE>;
/// sends settings to the given channel. May fail at sending if the channel is full
fn send_settings_to_sensor(
    dev_id: can_contract::DevId,
    channel: CanSender,
) -> Result<(), TrySendError<(Commands, can_contract::DevId, CommandData)>> {
    channel.try_send((
        Commands::Threshold,
        dev_id,
        CommandData::Threshold(
            SHARED_SETTINGS
                .threshold
                .load(core::sync::atomic::Ordering::Relaxed),
        ),
    ))?;
    channel.try_send((
        Commands::WateringTime,
        dev_id,
        CommandData::WateringTime(
            SHARED_SETTINGS
                .watering_time
                .load(core::sync::atomic::Ordering::Relaxed),
        ),
    ))?;
    channel.try_send((
        Commands::BackoffTime,
        dev_id,
        CommandData::BackoffTime(
            SHARED_SETTINGS
                .backoff_time
                .load(core::sync::atomic::Ordering::Relaxed),
        ),
    ))
}
/// handles all the can related stuff, mainly sending messages and routing incoming messages to the
/// correct receiver
#[embassy_executor::task]
pub async fn can_task(
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
                info!("{:?}", frame);
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
                    (_command, Some(_dev_id), CommandData::Remote) => {
                        info!("received remote frame, need to handle it")
                    }
                    (command, Some(dev_id), data) => {
                        // use try send here to avoid blocking the task from interacting with the
                        // can
                        //
                        // this channel goes to the log file
                        let r = receive_channel.try_send((command, dev_id, data));
                        match r {
                            Ok(_) => {}
                            Err(e) => {
                                info!("{:?}", e);
                            }
                        }
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
                    CommandData::Remote => CanFrame::new_remote(id, can_contract::DLC),

                    _ => {
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
pub fn set_value(
    can_tx: &mut CanSender,
    command: Commands,
    data: CommandData,
    dev_id: can_contract::DevId,
) {
    if let Err(e) = can_tx.try_send((command, dev_id, data)) {
        error!(
            "failed setting value for {:?}, dev_id {}, data {:?}, {:?}",
            command, dev_id, data, e
        );
    };
}
pub async fn get_value(
    can_tx: &mut CanSender,
    can_rx: &mut CanReceiver,
    command: Commands,
    dev_id: u8,
    sensors: &SensorBitmap,
) -> Option<can_contract::CommandData> {
    can_tx.send((command, dev_id, CommandData::Remote)).await;

    let Ok((r_command, r_source_id, r_data)) = can_rx
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
pub async fn init_can(can_resources: SpiCan) -> CanBus {
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
    let Ok(_) = can.set_mask(
        filter::RxMask::Mask0,
        embedded_can::Id::Standard(can_contract::MASK),
    ) else {
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
