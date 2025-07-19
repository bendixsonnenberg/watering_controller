use can_contract::CommandData;
use core::fmt::Write;
use cyw43::{Control, JoinOptions, NetDriver};
use embassy_executor::Spawner;
use embassy_net::{
    Config, DhcpConfig, StackResources,
    tcp::{TcpSocket, TcpWriter},
};
use embassy_rp::clocks::RoscRng;
use embassy_time::{Duration, Timer};
use embedded_io_async::Read;
use heapless::String;
use log::{error, info};
use serde::{Deserialize, Serialize};
use static_cell::StaticCell;

use crate::{
    SensorBitmap,
    can::{CanReceiver, CanSender, get_value},
    settings::{PASSWORD, SSID},
};
#[embassy_executor::task]
pub async fn net_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
}
struct SocketIterator<R: Read, const N: usize> {
    pub f: R,
    pub buf: [u8; N],
    pub offset: usize,
}
#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
struct SendSensor {
    threshold: u16,
    watering_time: u16,
    backoff_time: u16,
    moisture: Option<u16>,
    temperature: Option<i16>,
    humidity: Option<u16>,
}

impl<R: Read, const N: usize> SocketIterator<R, N> {
    async fn next(&mut self) -> Option<char> {
        match self.buf.len() - self.offset {
            0 => match self.f.read(&mut self.buf).await {
                Ok(0) => None,
                Ok(_count) => {
                    self.offset = 0;
                    self.offset += 1;
                    Some(self.buf[self.offset - 1] as char)
                }
                Err(e) => {
                    error!("{:?}", e);
                    None
                }
            },
            _ => {
                self.offset += 1;
                Some(self.buf[self.offset - 1] as char)
            }
        }
    }
}

#[embassy_executor::task]
pub async fn server(
    mut control: Control<'static>,
    net_device: NetDriver<'static>,
    spawner: Spawner,
    mut can_tx: CanSender,
    mut can_rx: CanReceiver,
    sensors: &'static SensorBitmap,
) {
    let mut rng = RoscRng;
    let dhcp_config = DhcpConfig::default();
    let config = Config::dhcpv4(dhcp_config);
    static RESOURCES: StaticCell<StackResources<10>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        net_device,
        config,
        RESOURCES.init(StackResources::new()),
        rng.next_u64(),
    );
    spawner.spawn(net_task(runner)).unwrap();

    loop {
        if let Ok(()) = control
            .join(
                SSID.get().await,
                JoinOptions::new(PASSWORD.get().await.as_bytes()),
            )
            .await
        {
            break;
        }
        Timer::after_secs(5).await;
    }
    // control should be  connected with wifi network
    info!("connected to wifi");
    while !stack.is_config_up() {
        Timer::after_millis(1000).await;
    }
    info!("dhcp connected");
    if let Some(ipv4) = stack.config_v4() {
        info!("address {}", ipv4.address);
    }

    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];
    let buf = [0; 4096];
    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(10)));
        if let Err(e) = socket.accept(8000).await {
            log::error!("accept error{:?}", e);
            continue;
        }
        let (reader, mut writer) = socket.split();
        let mut iterator = SocketIterator {
            f: reader,
            buf,
            offset: 0,
        };
        // get method and path
        let mut method: String<16> = String::new();
        while let Some(c) = iterator.next().await {
            if c == ' ' {
                break;
            }
            let _ = method.push(c);
        }
        let mut path: String<32> = String::new();
        while let Some(c) = iterator.next().await {
            if c == '\n' {
                break;
            }
            let _ = path.push(c);
        }

        //TODO get data
        match (method.as_str(), path.as_str()) {
            ("GET", "/sensors") => return_sensors(&mut writer, sensors).await,
            ("GET", path) if path.starts_with("/sensor/") => {
                return_sensor(&mut writer, path, sensors, &mut can_tx, &mut can_rx).await
            }
            ("POST", "/sensor") => todo!(),
            _ => return_missing(&mut writer).await,
        }
    }

    //TODO refactor into 2 funcitons to use ? instead of this many if lets
    async fn return_sensor(
        mut writer: &mut TcpWriter<'_>,
        path: &str,
        sensors: &SensorBitmap,
        mut can_tx: &mut CanSender,
        mut can_rx: &mut CanReceiver,
    ) {
        // get id; check if id exist
        if let Ok(id) = path["/sensor/".len()..].parse::<u8>()
            && sensors.lock(|sensors| sensors.borrow().get(id as usize))
        {
            let settings = get_value(
                &mut can_tx,
                &mut can_rx,
                CommandData::Settings(0, 0, 0),
                id,
                sensors,
            )
            .await;
            let sensors = get_value(
                &mut can_tx,
                &mut can_rx,
                CommandData::Sensors(None, None, None),
                id,
                sensors,
            )
            .await;
            if let Some(CommandData::Settings(threshold, backoff_time, watering_time)) = settings
                && let Some(CommandData::Sensors(moisture, temperature, humidity)) = sensors
            {
                let to_send = SendSensor {
                    threshold,
                    backoff_time,
                    watering_time,
                    moisture,
                    temperature,
                    humidity,
                };
                if let Ok(data) = serde_json_core::to_string::<SendSensor, 1024>(&to_send) {
                    let _ = writer.write("HTTP/1.1 200 OK\n\n".as_bytes()).await;
                    let _ = writer.write(data.as_bytes()).await;
                } else {
                    return_missing(&mut writer).await;
                }
            } else {
                return_missing(&mut writer).await;
            }
        } else {
            return_missing(&mut writer).await;
        }
    }

    async fn return_missing(writer: &mut TcpWriter<'_>) {
        let s: String<1024> = String::try_from("HTTP/1.1 404 Not found\n\n").expect("from const");

        if let Err(e) = writer.write(s.as_bytes()).await {
            error!("failed at sending: {:?}", e);
        }
    }

    async fn return_sensors(socket: &mut TcpWriter<'_>, sensors: &SensorBitmap) {
        let mut s: String<2048> =
            String::try_from("HTTP/1.1 200 Allowed\n\n[").expect("from const");

        sensors.lock(|sensors| {
            sensors.borrow().into_iter().for_each(|index| {
                let mut buf: String<5> = String::new();
                let _ = write!(buf, "{},", index);
                let _ = s.push_str(buf.as_str());
            })
        });
        // remove last ","
        s.pop();
        if let Err(_e) = s.push(']') {
            error!("too many sensors");
            return;
        };
        if let Err(e) = socket.write(s.as_bytes()).await {
            error!("failed at sending: {:?}", e);
        }
    }
    // get_value(
    //     &mut can_tx,
    //     &mut can_rx,
    //     CommandData::Sensors(None, None, None),
    //     0,
    //     sensors,
    // )
    // .await;
}
