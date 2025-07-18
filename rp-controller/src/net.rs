use can_contract::CommandData;
use cyw43::{Control, JoinOptions, NetDriver};
use embassy_net::{Config, DhcpConfig, StackResources, tcp::TcpSocket};
use embassy_rp::clocks::RoscRng;
use embassy_time::{Duration, Timer};
use embedded_io_async::Read;
use heapless::String;
use log::{error, info};
use static_cell::StaticCell;

use crate::{
    SensorBitmap,
    can::{CanReceiver, CanSender, get_value},
    settings::{PASSWORD, SSID},
};
struct SocketIterator<R: Read, const N: usize> {
    pub f: R,
    pub buf: [u8; N],
    pub offset: usize,
}

impl<R: Read, const N: usize> SocketIterator<R, N> {
    async fn next(&mut self) -> Option<char> {
        match self.buf.len() - self.offset {
            0 => match self.f.read(&mut self.buf).await {
                Ok(0) => None,
                Ok(_count) => {
                    self.offset = 0;
                    self.next().await
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
async fn server(
    mut control: Control<'static>,
    net_device: NetDriver<'static>,
    mut can_tx: CanSender,
    mut can_rx: CanReceiver,
    sensors: &'static SensorBitmap,
) {
    let mut rng = RoscRng;
    let mut dhcp_config = DhcpConfig::default();
    let config = Config::dhcpv4(dhcp_config);
    static RESOURCES: StaticCell<StackResources<10>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        net_device,
        config,
        RESOURCES.init(StackResources::new()),
        rng.next_u64(),
    );

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
        let mut iterator = SocketIterator {
            f: socket,
            buf,
            offset: 0,
        };
        // get method and path
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
