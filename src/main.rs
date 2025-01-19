#![no_std]
#![no_main]

use core::cell::RefCell;

use assign_resources::assign_resources;
use defmt::{debug, error, info};
use embassy_executor::Spawner;
use embassy_net::udp::UdpSocket;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{self, PIO0, UART1};
use embassy_rp::watchdog::Watchdog;
use embassy_rp::{bind_interrupts, pio, uart};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::channel::{Channel, Receiver};
use embassy_time::{Duration, Timer};
#[allow(unused_imports)]
use {defmt_rtt as _, panic_probe as _};

use crate::network::{init_network, remote_endpoint, EndpointType};
use crate::reader::{AsyncReader, Error, Message, MessageType};
use crate::serial::{init_serial, SerialTx};
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
type ReceiverType<'a> = Receiver<'a, CriticalSectionRawMutex, Packet, 1>;

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

struct Packet {
    length: usize,
    buf: [u8; 2048],
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let r = split_resources!(p);

    let mut led_green = Output::new(r.led.green, Level::Low);
    let mut led_red = Output::new(r.led.red, Level::Low);

    led_red.set_high();

    spawner.spawn(watchdog_task(r.watchdog)).unwrap();

    let (mut control, stack) = init_wifi(spawner, r.wifi).await;
    control.gpio_set(0, true).await;

    let sockets = init_network(stack);

    let remote_endpoint = remote_endpoint();

    control.gpio_set(0, false).await;

    led_red.set_low();

    let (tx, rx) = init_serial(r.uart).await.split();
    let mut uart_tx = SerialTx::new(tx, Output::new(r.uart_dir.dir_pin, Level::Low));
    let mut reader = AsyncReader::new(rx);

    spawner.must_spawn(udp_read_task(sockets.read_socket));
    spawner.must_spawn(udp_write_task(sockets.write_socket));

    let read_receiver = READ_CHANNEL.receiver();
    let write_receiver = WRITE_CHANNEL.receiver();

    loop {
        clear_watchdog();

        match reader.next_message().await {
            Ok(opt) => match opt {
                Some(message) => {
                    if message.address() != ADDRESS_MODBUS40 {
                        debug!("Skipping packet addressed to {}", message.address());
                        return;
                    }

                    match message.message_type() {
                        MessageType::ReadToken => {
                            send_queued_message_to_pump_or_ack(&read_receiver, &mut uart_tx).await
                        }
                        MessageType::WriteToken => {
                            send_queued_message_to_pump_or_ack(&write_receiver, &mut uart_tx).await
                        }
                        MessageType::Other => {
                            relay_message_to_server_and_ack(
                                &sockets.send_socket,
                                remote_endpoint,
                                &mut uart_tx,
                                &message,
                            )
                            .await;
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
                        uart_tx.write_byte(NACK).await;
                    }
                }
                flash_led(&mut led_red).await;
            }
        }
    }
}

#[embassy_executor::task]
async fn udp_read_task(socket: UdpSocket<'static>) {
    let sender = READ_CHANNEL.sender();
    let mut buf = [0; 2048];
    loop {
        let (n, ep) = socket.recv_from(&mut buf).await.unwrap();
        debug!("Got UDP read message from {}: {=[u8]:02x}", ep, &buf[..n]);
        sender.send(Packet { length: n, buf }).await;
    }
}

#[embassy_executor::task]
async fn udp_write_task(socket: UdpSocket<'static>) {
    let sender = WRITE_CHANNEL.sender();
    let mut buf = [0; 2048];
    loop {
        let (n, ep) = socket.recv_from(&mut buf).await.unwrap();
        debug!("Got UDP write message from {}: {=[u8]:02x}", ep, &buf[..n]);
        sender.send(Packet { length: n, buf }).await;
    }
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

async fn send_queued_message_to_pump_or_ack(
    receiver: &ReceiverType<'_>,
    serial_tx: &mut SerialTx<'_>,
) {
    if let Ok(packet) = receiver.try_receive() {
        serial_tx.write(&packet.buf[..packet.length]).await;
    } else {
        serial_tx.write_byte(ACK).await;
    }
}

async fn relay_message_to_server_and_ack(
    socket: &UdpSocket<'_>,
    remote_endpoint: EndpointType,
    serial_tx: &mut SerialTx<'_>,
    message: &Message,
) {
    debug!(
        "Sending to {:?}: {=[u8]:02x}",
        remote_endpoint,
        message.raw_frame()
    );
    let result = socket.send_to(message.raw_frame(), remote_endpoint).await;
    if let Err(err) = result {
        error!("Failed to send UDP packet: {:?}", err);
    };
    serial_tx.write_byte(ACK).await;
}

async fn flash_led(led: &mut Output<'_>) {
    led.set_high();
    Timer::after(Duration::from_millis(10)).await;
    led.set_low();
}
