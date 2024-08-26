use core::str::FromStr;
use cyw43::NetDriver;
use embassy_net::{Ipv4Address, Stack};
use embassy_net::udp::{PacketMetadata, UdpSocket};
use static_cell::StaticCell;

const REMOTE_IP: &str = env!("REMOTE_IP");
const REMOTE_PORT: &str = env!("REMOTE_IP");
const LOCAL_READ_PORT: &str = env!("LOCAL_READ_PORT");
const LOCAL_WRITE_PORT: &str = env!("LOCAL_WRITE_PORT");

pub struct NetworkSockets<'a> {
    pub remote_socket: UdpSocket<'a>,
    pub local_read_socket: UdpSocket<'a>,
    pub local_write_socket: UdpSocket<'a>,
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


pub fn remote_endpoint() -> (Ipv4Address, u16) {
    let address = Ipv4Address::from_str(REMOTE_IP).unwrap();
    (address, REMOTE_PORT.parse::<u16>().unwrap())
}


pub fn init_network<'a>(
    stack: &'a Stack<NetDriver<'a>>) -> NetworkSockets {
    static REMOTE_SOCKET_RESOURCES: StaticCell<SocketResources> = StaticCell::new();
    let remote_resources = &mut *REMOTE_SOCKET_RESOURCES.init(SocketResources::new());
    let remote_socket = UdpSocket::new(
        stack,
        &mut remote_resources.rx_meta,
        &mut remote_resources.rx_buffer,
        &mut remote_resources.tx_meta,
        &mut remote_resources.tx_buffer,
    );

    static LOCAL_READ_SOCKET_RESOURCES: StaticCell<SocketResources> = StaticCell::new();
    let local_read_resources = &mut *LOCAL_READ_SOCKET_RESOURCES.init(SocketResources::new());
    let mut local_read_socket = UdpSocket::new(
        stack,
        &mut local_read_resources.rx_meta,
        &mut local_read_resources.rx_buffer,
        &mut local_read_resources.tx_meta,
        &mut local_read_resources.tx_buffer,
    );
    local_read_socket.bind(LOCAL_READ_PORT.parse::<u16>().unwrap()).unwrap();

    static LOCAL_WRITE_SOCKET_RESOURCES: StaticCell<SocketResources> = StaticCell::new();
    let local_write_resources = &mut *LOCAL_WRITE_SOCKET_RESOURCES.init(SocketResources::new());
    let mut local_write_socket = UdpSocket::new(
        stack,
        &mut local_write_resources.rx_meta,
        &mut local_write_resources.rx_buffer,
        &mut local_write_resources.tx_meta,
        &mut local_write_resources.tx_buffer,
    );
    local_write_socket.bind(LOCAL_WRITE_PORT.parse::<u16>().unwrap()).unwrap();


    NetworkSockets { remote_socket, local_read_socket, local_write_socket }
}
