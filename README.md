# Modbus Gateway

Modbus gateway for Raspberry Pi Pico W forwarding serial 
RS-485 as UDP packages to remote server (ie. Homeassistant).

### Configuration

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
