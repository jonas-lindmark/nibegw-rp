use cortex_m::asm::nop;
use defmt::debug;
use embassy_rp::gpio::Output;
use embassy_rp::peripherals::UART1;
use embassy_rp::uart::{BufferedUart, BufferedUartTx, Config};
use embedded_io_async::Write;
use static_cell::StaticCell;

use crate::{Irqs, UartResources};

pub struct SerialTx<'a> {
    tx: BufferedUartTx<'a, UART1>,
    dir_pin: Output<'a>,
}

impl<'a> SerialTx<'a> {
    pub fn new(tx: BufferedUartTx<'a, UART1>, dir_pin: Output<'a>) -> Self {
        Self { tx, dir_pin }
    }

    pub async fn write_byte(&mut self, byte: u8) {
        self.write(&[byte]).await;
    }

    pub async fn write(&mut self, buf: &[u8]) {
        debug!("Writing UART packet: {=[u8]:02x}", buf);
        self.dir_pin.set_high();
        self.tx.write_all(buf).await.unwrap();
        while self.tx.busy() {
            nop(); // timer is too inaccurate and may cause too long wait interfering with next read
        }
        self.dir_pin.set_low();
    }
}

pub async fn init_serial(p: UartResources) -> BufferedUart<'static, UART1> {
    static TX_BUF: StaticCell<[u8; 16]> = StaticCell::new();
    let tx_buf = &mut TX_BUF.init([0; 16])[..];
    static RX_BUF: StaticCell<[u8; 16]> = StaticCell::new();
    let rx_buf = &mut RX_BUF.init([0; 16])[..];
    let config = {
        let mut conf = Config::default();
        conf.baudrate = 9600;
        conf
    };
    BufferedUart::new(p.uart, Irqs, p.tx_pin, p.rx_pin, tx_buf, rx_buf, config)
}
