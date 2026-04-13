# ESP8266 air quality sensor

This is a simple firmware for an ESP8266 board such as NodeMCU to measure air quality
(carbon dioxide concentration, temperature, humidity) and reporting the measurements via MQTT.

* ESP8266 NodeMCU clone devkit
* MH-Z19 carbon dioxide sensor
* AM2301 temperature and humidity sensor

Used as part of a Home Assistant setup for measuring indoor air quality.

## Dependencies
- ESP8266 stack for Arduino development environment
- DHT sensor library: https://github.com/adafruit/DHT-sensor-library
- MH-Z19 sensor library: https://github.com/WifWaf/MH-Z19
- ESP software serial: https://github.com/plerup/espsoftwareserial/
- MQTT library: https://pubsubclient.knolleary.net/

## Arduino CLI setup

- Install Arduino CLI
- `arduino-cli config init`
- `arduino-cli config add board_manager.additional_urls https://arduino.esp8266.com/stable/package_esp8266com_index.json`
- `arduino-cli core update-index`
- `arduino-cli core install esp8266:esp8266`
- `make libs`
- `make compile`
- `make upload [PORT=/dev/ttyUSB0]`
- `make monitor [PORT=/dev/ttyUSB0]`
