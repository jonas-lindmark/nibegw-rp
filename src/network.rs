use core::str::FromStr;
use cyw43::NetDriver;
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{Ipv4Address, Stack};
use static_cell::StaticCell;

const REMOTE_IP: &str = env!("REMOTE_IP");
const REMOTE_PORT: &str = env!("REMOTE_PORT");
const LOCAL_READ_PORT: &str = env!("LOCAL_READ_PORT");
const LOCAL_WRITE_PORT: &str = env!("LOCAL_WRITE_PORT");

pub type EndpointType = (Ipv4Address, u16);

pub struct NetworkSockets<'a> {
    pub send_socket: UdpSocket<'a>,
    pub read_socket: UdpSocket<'a>,
    pub write_socket: UdpSocket<'a>,
}

struct SocketResources {
    rx_buffer: [u8; 4096],
    tx_buffer: [u8; 4096],
    rx_meta: [PacketMetadata; 16],
    tx_meta: [PacketMetadata; 16],
}

impl SocketResources {
    pub fn new() -> SocketResources {
        SocketResources {
            rx_buffer: [0; 4096],
            tx_buffer: [0; 4096],
            rx_meta: [PacketMetadata::EMPTY; 16],
            tx_meta: [PacketMetadata::EMPTY; 16],
        }
    }
}

pub fn remote_endpoint() -> EndpointType {
    let address = Ipv4Address::from_str(REMOTE_IP).unwrap();
    (address, REMOTE_PORT.parse::<u16>().unwrap())
}

pub fn init_network<'a>(stack: &'a Stack<NetDriver<'a>>) -> NetworkSockets<'a> {
    static SEND_SOCKET_RESOURCES: StaticCell<SocketResources> = StaticCell::new();
    let send_resources = &mut *SEND_SOCKET_RESOURCES.init(SocketResources::new());
    let mut send_socket = UdpSocket::new(
        stack,
        &mut send_resources.rx_meta,
        &mut send_resources.rx_buffer,
        &mut send_resources.tx_meta,
        &mut send_resources.tx_buffer,
    );
    send_socket.bind(0).unwrap();

    static READ_SOCKET_RESOURCES: StaticCell<SocketResources> = StaticCell::new();
    let read_resources = &mut *READ_SOCKET_RESOURCES.init(SocketResources::new());
    let mut read_socket = UdpSocket::new(
        stack,
        &mut read_resources.rx_meta,
        &mut read_resources.rx_buffer,
        &mut read_resources.tx_meta,
        &mut read_resources.tx_buffer,
    );
    read_socket
        .bind(LOCAL_READ_PORT.parse::<u16>().unwrap())
        .unwrap();

    static WRITE_SOCKET_RESOURCES: StaticCell<SocketResources> = StaticCell::new();
    let write_resources = &mut *WRITE_SOCKET_RESOURCES.init(SocketResources::new());
    let mut write_socket = UdpSocket::new(
        stack,
        &mut write_resources.rx_meta,
        &mut write_resources.rx_buffer,
        &mut write_resources.tx_meta,
        &mut write_resources.tx_buffer,
    );
    write_socket
        .bind(LOCAL_WRITE_PORT.parse::<u16>().unwrap())
        .unwrap();

    NetworkSockets {
        send_socket,
        read_socket,
        write_socket,
    }
}
