#![no_std]
#![no_main]

use core::cell::RefCell;

use assign_resources::assign_resources;
use cortex_m::asm::nop;
use defmt::{debug, error, info};
use embassy_executor::Spawner;
use embassy_net::udp::UdpSocket;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{self, PIO0, UART1};
use embassy_rp::uart::BufferedUartTx;
use embassy_rp::watchdog::Watchdog;
use embassy_rp::{bind_interrupts, pio, uart};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::channel;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use embedded_io_async::Write;
use static_cell::StaticCell;
#[allow(unused_imports)]
use {defmt_rtt as _, panic_probe as _};

use crate::network::{init_network, remote_endpoint};
use crate::reader::{compute_checksum, AsyncReader, Error, MessageType};
use crate::serial::init_serial;
use crate::wifi::init_wifi;

mod network;
mod reader;
mod serial;
mod wifi;

const START: u8 = 0x5c;
const ACK: u8 = 0x06;
const NACK: u8 = 0x15;
const ADDRESS_MODBUS40: u16 = 0x0020;

static WATCHDOG_COUNTER: Mutex<ThreadModeRawMutex, RefCell<u32>> = Mutex::new(RefCell::new(0));
static READ_CHANNEL: Channel<CriticalSectionRawMutex, Packet, 1> = Channel::new();
static WRITE_CHANNEL: Channel<CriticalSectionRawMutex, Packet, 1> = Channel::new();

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
    UART1_IRQ => uart::BufferedInterruptHandler<UART1>;
});

assign_resources! {
    wifi: WifiResources {
        pwr_pin: PIN_23,
        cs_pin: PIN_25,
        dio_pin: PIN_24,
        clk_pin: PIN_29,
        pio: PIO0,
        dma_ch: DMA_CH0,
    }
    uart: UartResources {
        tx_pin: PIN_8,
        rx_pin: PIN_9,
        uart: UART1,
    }
    uart_dir: UartDirResources {
        dir_pin:  PIN_10,
    }
    watchdog: WatchdogResources {
        watchdog: WATCHDOG,
    }
    led: LedResources {
        green: PIN_27,
        red: PIN_26,
    }
}

struct UartTxType<'a> {
    tx: BufferedUartTx<'a, UART1>,
    dir_pin: Output<'a>,
}

struct Packet {
    length: usize,
    buf: [u8; 2048],
}

#[embassy_executor::task]
async fn watchdog_task(watchdog: WatchdogResources) {
    let mut watchdog = Watchdog::new(watchdog.watchdog);

    // set long timeout initially to not trigger by slow Wi-Fi startup
    watchdog.start(Duration::from_millis(8_300));
    Timer::after(Duration::from_millis(8_000)).await;

    // set more reasonable timeout of 1.5 sec
    watchdog.start(Duration::from_millis(1_500));
    loop {
        let counter = WATCHDOG_COUNTER.lock(|f| {
            let val = f.borrow_mut().wrapping_add(1);
            f.replace(val)
        });
        match counter {
            0..=1 => watchdog.feed(),
            2..=35 => {
                watchdog.feed();
                info!("Watchdog {}", counter);
            }
            _ => info!("Watchdog {} not feeding", counter),
        }
        Timer::after(Duration::from_millis(1000)).await;
    }
}

fn clear_watchdog() {
    WATCHDOG_COUNTER.lock(|f| {
        f.replace(0);
    });
}

#[embassy_executor::task]
async fn udp_read_task(
    socket: UdpSocket<'static>,
) {
    let sender = READ_CHANNEL.sender();
    let mut buf = [0; 2048];
    loop {
        let (n, ep) = socket.recv_from(&mut buf).await.unwrap();
        debug!("Got UDP read message from {}: {=[u8]:02x}", ep, &buf[..n]);
        sender.send(Packet { length: n, buf }).await;
    }
}

#[embassy_executor::task]
async fn udp_write_task(
    socket: UdpSocket<'static>,
) {
    let sender = WRITE_CHANNEL.sender();
    let mut buf = [0; 2048];
    loop {
        let (n, ep) = socket.recv_from(&mut buf).await.unwrap();
        debug!("Got UDP write message from {}: {=[u8]:02x}", ep, &buf[..n]);
        sender.send(Packet { length: n, buf }).await;
    }
}

#[embassy_executor::task]
async fn receive_udp_write_task() {}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let r = split_resources!(p);

    let mut led_green = Output::new(r.led.green, Level::Low);
    let mut led_red = Output::new(r.led.red, Level::Low);

    led_red.set_high();

    //spawner.spawn(watchdog_task(r.watchdog)).unwrap();

    let (mut control, stack) = init_wifi(spawner, r.wifi).await;
    control.gpio_set(0, true).await;

    let sockets = init_network(&stack);

    let remote_endpoint = remote_endpoint();

    control.gpio_set(0, false).await;

    led_red.set_low();

    let (rx, tx) = init_serial(r.uart).await.split();
    let uart_dir_pin = Output::new(r.uart_dir.dir_pin, Level::Low);
    static UART_TX_REF_CELL: StaticCell<RefCell<UartTxType>> = StaticCell::new();
    let uart_tx_ref_cell = UART_TX_REF_CELL.init(RefCell::new(UartTxType {
        tx,
        dir_pin: uart_dir_pin,
    }));

    spawner.must_spawn(udp_read_task(sockets.read_socket));
    spawner.must_spawn(udp_write_task(sockets.write_socket));

    let mut reader = AsyncReader::new(rx);

    let read_receiver = READ_CHANNEL.receiver();
    let write_receiver = WRITE_CHANNEL.receiver();


    loop {
        //clear_watchdog();

        match reader.next_message().await {
            Ok(opt) => match opt {
                Some(message) => {
                    if message.address() != ADDRESS_MODBUS40 {
                        debug!("Skipping packet addressed to {}", message.address());
                        return;
                    }

                    debug!(
                        "Sending to {:?}: {=[u8]:02x}",
                        remote_endpoint,
                        message.raw_frame()
                    );

                    match message.message_type() {
                        MessageType::ReadToken => {
                            if let Ok(packet) = read_receiver.try_receive() {
                                info!("Got read token");
                                write_serial(uart_tx_ref_cell, &packet.buf[..packet.length]).await;
                            } else {
                                write_serial(uart_tx_ref_cell, &[ACK]).await;
                            }
                        }
                        MessageType::WriteToken => {
                            if let Ok(packet) = write_receiver.try_receive() {
                                info!("Got write token");
                                write_serial(uart_tx_ref_cell, &packet.buf[..packet.length]).await;
                            } else {
                                write_serial(uart_tx_ref_cell, &[ACK]).await;
                            }
                        }
                        MessageType::Other => {
                            let result = sockets
                                .send_socket
                                .send_to(message.raw_frame(), remote_endpoint)
                                .await;
                            if let Err(err) = result {
                                error!("Failed to send UDP packet: {:?}", err);
                            };
                            write_serial(uart_tx_ref_cell, &[ACK]).await;
                        }
                    }

                    flash_led(&mut led_green).await;
                }
                None => {
                    debug!("Found no message");
                }
            },
            Err(error) => {
                match error {
                    Error::ReadError(err) => {
                        error!("Read error {:?}", err);
                        // nack?
                    }
                    Error::ChecksumMismatch => {
                        error!("Checksum mismatch");
                        write_serial(uart_tx_ref_cell, &[NACK]).await;
                    }
                }
                flash_led(&mut led_red).await;
            }
        }
    }
}

async fn flash_led(led: &mut Output<'_>) {
    led.set_high();
    Timer::after(Duration::from_millis(10)).await;
    led.set_low();
}

async fn write_serial(uart_tx_ref_cell: &'static RefCell<UartTxType<'_>>, buf: &[u8]) {
    let mut uart_tx = uart_tx_ref_cell.borrow_mut();
    debug!("Writing UART packet: {=[u8]:02x}", buf);
    uart_tx.dir_pin.set_high();
    uart_tx.tx.write_all(buf).await.unwrap();
    while uart_tx.tx.busy() {
        nop(); // timer is too inaccurate and may cause too long wait interfering with next read
    }
    uart_tx.dir_pin.set_low();
}
