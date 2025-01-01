use embassy_rp::peripherals::UART1;
use embassy_rp::uart::{BufferedUart, Config};
use static_cell::StaticCell;

use crate::{Irqs, UartResources};

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
