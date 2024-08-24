#![no_std]
#![no_main]

use core::cell::RefCell;

use assign_resources::assign_resources;
use defmt::{debug, error, info};
use embassy_executor::Spawner;
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_rp::{bind_interrupts, pio, uart};
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{self, PIO0, UART1};
use embassy_rp::watchdog::Watchdog;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_time::{Duration, Timer};
use embedded_io_async::Write;

#[allow(unused_imports)]
use {defmt_rtt as _, panic_probe as _};

use crate::reader::{AsyncReader, Error};
use crate::serial::init_serial;
use crate::wifi::init_wifi;

mod wifi;
mod serial;
mod reader;

const ACK: u8 = 0x06;
const NACK: u8 = 0x15;


static WATCHDOG_COUNTER: Mutex<ThreadModeRawMutex, RefCell<u32>> = Mutex::new(RefCell::new(0));

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
    watchdog: WatchdogResources {
        watchdog: WATCHDOG,
    }
    led: LedResources {
        green: PIN_26,
        red: PIN_27,
    }
}


#[embassy_executor::task]
async fn watchdog_task(watchdog: WatchdogResources) {
    let mut watchdog = Watchdog::new(watchdog.watchdog);

    // set long timeout initially to not trigger by slow wifi startup
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

    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];
    let mut rx_meta = [PacketMetadata::EMPTY; 16];
    let mut tx_meta = [PacketMetadata::EMPTY; 16];
    let mut socket = UdpSocket::new(stack, &mut rx_meta, &mut rx_buffer, &mut tx_meta, &mut tx_buffer);
    socket.bind(1234).unwrap();

    control.gpio_set(0, false).await;

    led_red.set_low();

    let (rx, mut tx) = init_serial(r.uart).await.split();

    let mut reader = AsyncReader::new(rx);

    loop {
        clear_watchdog();

        match reader.next_message().await {
            Ok(opt) => {
                match opt {
                    Some(message) => {
                        // forward to UDP
                        //socket.send_to(message.raw_frame(), )
                        tx.write(&[ACK]).await.unwrap();
                        flash_led(&mut led_green).await;
                    }
                    None => {
                        debug!("Found no message");
                    }
                }
            }
            Err(error) => {
                match error {
                    Error::ReadError(err) => {
                        error!("Read error {:?}", err);
                        // nack?
                    }
                    Error::ChecksumMismatch => {
                        error!("Checksum mismatch");
                        tx.write(&[NACK]).await.unwrap();
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

