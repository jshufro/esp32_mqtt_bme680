# ESP32 MQTT BME680

This Arduino IDE project is the code to power an [ESP32 Thing](https://www.sparkfun.com/products/15663) connected via I2C to a [BME680](https://www.sparkfun.com/products/16466) environmental sensor. The microcontroller connects to your WiFi network and publishes to an MQTT broker of your choice at a configurable interval.

This was purpose-built to be used in conjunction with other MQTT-compatible software... In this case, [Home Assistant](https://www.home-assistant.io/), but the same setup will work with anything similar.

## Requirements
* [ESP32 Thing](https://www.sparkfun.com/products/15663)
* [BME680](https://www.sparkfun.com/products/16466) 
* MQTT Broker of your choice (e.g. [mosquitto](https://mosquitto.org/))
* [Arduino IDE](https://www.arduino.cc/en/main/software)
	* [Espressif's library for the ESP32](https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/boards_manager.md)
	* If using Linux, your user will need to be in the `dialout` group.

## Directions

 1. Attach your BME680 to the ESP32 board using a convenient [Qwiic connector](https://www.sparkfun.com/products/14426).
 2. Open the project in Arduino IDE
 3. Modify at least the following code constants:
	 * NETWORK_ID
	 * PASSWORD
	 * MQTT_TOPIC
	 * MQTT_URI
4. Plug your ESP32 board into your computer using a micro USB cable, and flash the software.

## Debugging
| LED | Meaning | Actions |
|--|--| --|
| Off | No power or starting up. | Check power and wait at least 30 seconds. |
| Flashes once, with 2 seconds in between | BME680 wasn't found | Make sure the BME680 is wired in and working. If using a non-standard I2C mode, you may have to update the BME680_I2C_MODE configuration. |
| Flashes twice | WiFi error | Make sure wifi can reach the ESP32, and the NETWORK_ID/PASSWORD configuration is correct. |
| Flashes Trebly | MQTT client error | Make sure your MQTT broker is running and the connection settings are correct. |
| Lit Steady | No Error | Have some tea. Relax. Read a book. |

All other issues should be debugged using the IDE, with the DEBUG constant defined for verbose output to the console. Make sure to set the baud rate in the IDE to match SERIAL_SPEED (115200 unless changed).

## License
[MIT](https://choosealicense.com/licenses/mit/)
