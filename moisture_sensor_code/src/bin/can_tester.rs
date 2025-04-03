#![no_std]
#![no_main]

use byteorder::{ByteOrder, LE};

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::can::filter::{self, BankConfig};
use embassy_stm32::can::frame::Header;
use embassy_stm32::can::{
    Frame, Id, Rx0InterruptHandler, Rx1InterruptHandler, SceInterruptHandler, StandardId,
    TxInterruptHandler,
};
use embassy_stm32::gpio::Output;
use embassy_stm32::peripherals::CAN;
use embassy_stm32::{can::Can, Config};
use {defmt_rtt as _, panic_probe as _};
bind_interrupts!(struct Irqs {
    USB_LP_CAN1_RX0 => Rx0InterruptHandler<CAN>;
    CAN1_RX1 => Rx1InterruptHandler<CAN>;
    CAN1_SCE => SceInterruptHandler<CAN>;
    USB_HP_CAN1_TX => TxInterruptHandler<CAN>;
});
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Config::default());

    embassy_stm32::pac::AFIO
        .mapr()
        .modify(|w| w.set_can1_remap(2));

    let mut can = Can::new(p.CAN, p.PB8, p.PB9, Irqs);

    can.modify_filters().enable_bank(
        0,
        embassy_stm32::can::Fifo::Fifo0,
        filter::Mask32::accept_all(),
    );
    can.modify_config()
        .set_bitrate(500_000)
        .set_loopback(false)
        .set_silent(false);
    can.enable().await;
    can.write(&create_frame(1, 2, true, 6)).await;
    info!("Frame sent");

    loop {
        let result = can.read().await;
        info!("{:?}", result);
    }
    let mut output = Output::new(
        p.PC13,
        embassy_stm32::gpio::Level::High,
        embassy_stm32::gpio::Speed::Low,
    );
    output.toggle();
}
fn create_frame(target: u16, command: u16, remote: bool, data: u16) -> Frame {
    let id = target | (command << 8);
    let id = StandardId::new(id).unwrap();
    let id = Id::Standard(id);
    let header = Header::new(id, 2, remote);
    let mut buf = &mut [0, 0];
    LE::write_u16(buf, data);
    Frame::new(header, buf).unwrap()
}
