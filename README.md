# Modbus Gateway

Modbus gateway for Raspberry Pi Pico W forwarding serial 
RS-485 as UDP packages to remote server (ie. Homeassistant).

## Protocol

This application listening data from various Nibe heat pumps (RS485 bus)
and send valid frames to configurable IP/port address by UDP packets.
Application also acknowledge the valid packets to heat pump.

Serial settings: 9600 baud, 8 bits, Parity: none, Stop bits 1

MODBUS module support should be turned ON from the heat pump.

Frame format:
```text
+-----+-----+-----+-----+-----+-----+-+-+-+-----+
| 5C  | ADR | ADR | CMD | LEN |   DATA    | CHK |
+-----+-----+-----+-----+-----+-----+-+-+-+-----+
      |<----------- CHECKSUM ------------>|
```
Address:
```text
0x0016 = SMS40
0x0019 = RMU40
0x0020 = MODBUS40
```
Checksum: XOR

When valid data is received (checksum ok),
 `ACK (0x06)` should be sent to the heat pump.

When checksum mismatch,
 `NAK (0x15)` should be sent to the heat pump.

If heat pump does not receive acknowledge in certain time period,
pump will raise an alarm and alarm mode is activated.


## Configuration

Create config file `.env`:

```text
WIFI_NETWORK=<network_ssid>
WIFI_PASSWORD=<password>
MQTT_SERVER_IP=<ip_adress>
MQTT_USER=<user>
MQTT_PASSWORD=<password>
MQTT_TOPIC=<topic>
```

Load in environment:
```shell
export $(grep -v '^#' .env | xargs)
```
