//! Program for Raspberry Pi Pico simulating a Nibe heat pump sending some test data.
//!
//! Will send read and write tokens repeatedly and if no ACK packet is received within 10 seconds
//! the program will go into "alarm mode" keeping the LED lit.

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::UART1;
use embassy_rp::uart::{BufferedInterruptHandler, BufferedUart, BufferedUartRx, Config};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::Timer;
use embedded_io_async::{Read, Write};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    UART1_IRQ => BufferedInterruptHandler<UART1>;
});

const PING_TIMEOUT_SECONDS: u64 = 10;
const ALARM_RESET_SECONDS: u64 = 10;

const ACK: u8 = 0x06;
const WRITE_TOKEN_MESSAGE: [u8; 6] = [0x5C, 0x00, 0x20, 0x6B, 0x00, 0x4B];
const READ_TOKEN_MESSAGE: [u8; 6] = [0x5C, 0x00, 0x20, 0x69, 0x00, 0x49];
const DATA_MESSAGE: [u8; 87] = [
    0x06, 0x5C, 0x00, 0x20, 0x68, 0x50, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF,
    0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF,
    0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF,
    0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF,
    0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF,
    0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x18,
];

static ACK_CHANNEL: Channel<CriticalSectionRawMutex, (), 1> = Channel::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let (tx_pin, rx_pin, uart) = (p.PIN_8, p.PIN_9, p.UART1);
    let mut led = Output::new(p.PIN_25, Level::Low);

    static TX_BUF: StaticCell<[u8; 16]> = StaticCell::new();
    let tx_buf = &mut TX_BUF.init([0; 16])[..];
    static RX_BUF: StaticCell<[u8; 16]> = StaticCell::new();
    let rx_buf = &mut RX_BUF.init([0; 16])[..];
    let config = {
        let mut conf = Config::default();
        conf.baudrate = 9600;
        conf
    };
    let uart = BufferedUart::new(uart, Irqs, tx_pin, rx_pin, tx_buf, rx_buf, config);
    let (mut tx, rx) = uart.split();

    unwrap!(spawner.spawn(reader(rx)));

    let receiver = ACK_CHANNEL.receiver();
    let mut missed_pings: u64 = 0;
    loop {
        let mut got_ping: bool = false;

        info!("Sending write token");
        led.set_high();
        tx.write_all(&WRITE_TOKEN_MESSAGE).await.unwrap();
        led.set_low();
        Timer::after_millis(20).await;
        if let Ok(_) = receiver.try_receive() {
            got_ping = true
        }

        info!("Sending read token");
        led.set_high();
        tx.write_all(&READ_TOKEN_MESSAGE).await.unwrap();
        led.set_low();
        Timer::after_millis(20).await;
        if let Ok(_) = receiver.try_receive() {
            got_ping = true
        }

        info!("Sending data message");
        led.set_high();
        tx.write_all(&DATA_MESSAGE).await.unwrap();
        led.set_low();
        Timer::after_millis(20).await;
        if let Ok(_) = receiver.try_receive() {
            got_ping = true
        }

        if !got_ping {
            info!("Missed pings: {:?}", missed_pings);
            missed_pings = missed_pings + 1;
        }

        Timer::after_millis(860).await;

        if missed_pings > PING_TIMEOUT_SECONDS {
            led.set_high();
            Timer::after_secs(ALARM_RESET_SECONDS).await;
            missed_pings = 0;
        }
    }
}

#[embassy_executor::task]
async fn reader(mut rx: BufferedUartRx<'static, UART1>) {
    let sender = ACK_CHANNEL.sender();

    loop {
        let mut buf: [u8; 1] = [0; 1];
        match rx.read(&mut buf).await {
            Ok(l) => {
                if l == 1 && buf[0] == ACK {
                    info!("Got ping");
                    sender.send(()).await;
                }
            }
            Err(e) => {
                error!("Read error {:?}", e);
            }
        }
    }
}
