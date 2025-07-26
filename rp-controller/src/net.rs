use can_contract::{CommandData, CommandDataContainer};
use core::fmt::Write;
use cyw43::{Control, JoinOptions, NetDriver};
use embassy_executor::Spawner;
use embassy_net::{Config, DhcpConfig, StackResources, tcp::TcpSocket};
use embassy_rp::clocks::RoscRng;
use embassy_time::{Duration, Timer};
use heapless::{String, Vec};
use log::{error, info};
use serde::{Deserialize, Serialize};
use static_cell::StaticCell;

use crate::{
    SensorBitmap,
    can::{CanReceiver, CanSender, get_value, set_value},
    error::get_errors,
    party,
    settings::{PASSWORD, SSID},
};
const MAX_ALLOCATED_SPACE_STRING: usize = 512;
#[embassy_executor::task]
pub async fn net_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
}
#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
struct SendSensor {
    id: u8,
    threshold: u16,
    watering_time: u16,
    backoff_time: u16,
    moisture: Option<u16>,
    temperature: Option<i16>,
    humidity: Option<u16>,
}
#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
struct PostSensor {
    id: u8,
    threshold: u16,
    watering_time: u16,
    backoff_time: u16,
}

// / it is the iterator to read from, newline count has to be the number of consecutive \n pulled from the iterator immediatly before the
async fn get_data_from_network<const N: usize>(
    socket: &mut TcpSocket<'_>,
    mut already_read: String<N>,
) -> Option<String<N>> {
    // the data starts with two \n after another
    let mut buf = [0; N];
    let mut result = String::<N>::new();
    let mut newline_count = 0;
    let mut data_flag = false;
    loop {
        if already_read.is_empty() {
            if let Ok(0) = socket.read(&mut buf).await {
                break;
            }
            let mut vec: Vec<u8, N> = Vec::new();
            let _ = vec.extend_from_slice(&buf);
            let parsed = String::from_utf8(vec);
            if parsed.is_err() {
                return None;
            }
            already_read = parsed.expect("Checked for err");
        }
        let c = already_read.remove(0);
        if c == '\n' {
            newline_count += 1;
        } else {
            newline_count = 0;
        }

        if newline_count >= 2 {
            data_flag = true;
        }

        if data_flag {
            let _ = result.push(c); // if this is an err the string was to short
        }
    }
    Some(result)
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
    let mut buf = [0; 128];
    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(10)));
        if let Err(e) = socket.accept(8000).await {
            log::error!("accept error{:?}", e);
            continue;
        }
        let Ok(_) = socket.read(&mut buf).await else {
            continue;
        };

        let vec = Vec::from_slice(&buf).unwrap();
        info!("got vec");
        Timer::after_millis(50).await;
        let mut header_string: String<MAX_ALLOCATED_SPACE_STRING> = String::from_utf8(vec).unwrap();
        info!("got header string: {}", header_string);
        Timer::after_millis(50).await;
        // get method and path
        let mut method: String<16> = String::new();
        loop {
            let c = header_string.remove(0);
            info!("got method char: {}", c);
            Timer::after_millis(50).await;
            if c == ' ' {
                break;
            }
            let _ = method.push(c);
        }
        info!("method is: {}", method);
        let mut path: String<32> = String::new();
        loop {
            let c = header_string.remove(0);
            if c == ' ' {
                break;
            }
            let _ = path.push(c);
        }
        info!("path is: {}", path);

        //TODO get data
        let data = get_data_from_network(&mut socket, header_string).await;
        match (method.as_str(), path.as_str()) {
            ("GET", "/static") => return_index(&mut socket).await,
            ("GET", "/sensors") => return_sensors(&mut socket, sensors).await,
            ("GET", path) if path.starts_with("/sensor/") => {
                info!("getting single sensor");
                return_sensor(&mut socket, path, sensors, &mut can_tx, &mut can_rx).await
            }
            ("GET", "/errors") => return_errors(&mut socket).await,
            ("POST", "/sensor") => set_sensor(&mut socket, data, sensors, &mut can_tx).await,
            ("POST", "/party/on") => party::party(sensors, can_tx, true).await,
            ("POST", "/party/off") => party::party(sensors, can_tx, false).await,
            _ => {
                let _ = socket.write("HTTP/1.1 404 MISSING\n\n".as_bytes()).await;
            }
        }
        socket.close();
        let _ = socket.flush().await;
        socket.abort();
    }
    async fn return_errors(writer: &mut TcpSocket<'_>) {
        let errors = get_errors().await;
        let Ok(error_string): Result<
            serde_json_core::heapless::String<MAX_ALLOCATED_SPACE_STRING>,
            serde_json_core::ser::Error,
        > = serde_json_core::to_string(&errors) else {
            return;
        };
        let _ = writer.write("HTTP/1.1 200 OK\n\n".as_bytes()).await;
        let _ = writer.write(error_string.as_bytes()).await;
        let _ = writer.write("\n\n".as_bytes()).await;
    }
    async fn return_index(mut writer: &mut TcpSocket<'_>) {
        let main_page = include_bytes!("../index.html");
        return_ok(&mut writer).await;

        let _ = writer.write(main_page).await;
    }

    async fn set_sensor(
        mut writer: &mut TcpSocket<'_>,
        data: Option<String<MAX_ALLOCATED_SPACE_STRING>>,
        sensors: &SensorBitmap,
        mut can_tx: &mut CanSender,
    ) {
        let Some(data) = data else {
            return_malformed(&mut writer).await;
            return;
        };
        let Ok((sensor, _size)) = serde_json_core::from_str::<PostSensor>(&data) else {
            return_malformed(&mut writer).await;
            return;
        };
        if !sensors.lock(|sensors| sensors.borrow().get(sensor.id as usize)) {
            return_missing(&mut writer).await;
            return;
        }
        // sensor that we want to set exists
        set_value(
            &mut can_tx,
            CommandDataContainer::Data {
                target_id: sensor.id,
                src_id: 0,
                data: CommandData::Settings(
                    sensor.threshold,
                    sensor.backoff_time,
                    sensor.watering_time,
                ),
            },
        );
        return_ok(&mut writer).await;
    }

    //TODO refactor into 2 funcitons to use ? instead of this many if lets
    async fn return_sensor(
        mut writer: &mut TcpSocket<'_>,
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
                    id,
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
                    let _ = writer.write("\n\n".as_bytes()).await;
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

    async fn return_missing(writer: &mut TcpSocket<'_>) {
        let s: String<MAX_ALLOCATED_SPACE_STRING> =
            String::try_from("HTTP/1.1 404 Not found\n\n").expect("from const");

        if let Err(e) = writer.write(s.as_bytes()).await {
            error!("failed at sending: {:?} (404)", e);
        }
    }

    async fn return_malformed(writer: &mut TcpSocket<'_>) {
        let s: String<MAX_ALLOCATED_SPACE_STRING> =
            String::try_from("HTTP/1.1 400 Bad Request\n\n").expect("from const");

        if let Err(e) = writer.write(s.as_bytes()).await {
            error!("failed at sending: {:?}(400)", e);
        }
    }
    async fn return_ok(writer: &mut TcpSocket<'_>) {
        let s: String<MAX_ALLOCATED_SPACE_STRING> =
            String::try_from("HTTP/1.1 200 OK\n\n").expect("from const");

        if let Err(e) = writer.write(s.as_bytes()).await {
            error!("failed at sending: {:?} (200)", e);
        }
    }
    async fn return_sensors(socket: &mut TcpSocket<'_>, sensors: &SensorBitmap) {
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
        if let Err(_e) = s.push_str("]\n") {
            error!("too many sensors");
            return;
        };
        info!("to write: {}", s);
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
