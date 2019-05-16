# GPS Tracker and LapTimer

This is a small R&D project intendet to familiarize myself with programming of Mircoprocessors in the Arduino environment.
The goal is to create a GPS tracker and LapTimer that records laps performed at a race track and shows the delta time to the fastest lap.
The code is currently very messy and might get some cleanup at a later point.

## Features:
### Implemented
* define start/finish line
* detect start/finish crossing to start/end a lap
* show laptime, lap number and various other information
* show delta time to reference lap (with number and a red/green bar)
* save completed laps to SD card in a structured manner
* connect to WiFi and offer WebServer to retrieve recorded laps from the tracker
* analyse recorded laps in python on PC
* OTA update over wifi

### Planned
* navigation via touchscreen through a menu to allow 
	* loading a specific reference from SD
	* loading a predifine start/finish line location
	* setting up the WiFi connection
* record analog inputs (ie for break/gas pedal)
* get a proper GPS antenna

A commercial product would be eg [this one](https://www.vboxmotorsport.co.uk/index.php/de/products/performance-meters/vbox-laptimer).


## Parts:
* [NodeMCU-32s](https://hackaday.com/2016/09/15/esp32-hands-on-awesome-promise/)
* Arduino Leonardo
* Adafruit capacitive touchscreen
* [Adafruit ultimate GPS shield with SD interface](https://www.adafruit.com/product/1272)

The GPS module might not be accurate enough to realy calculate accurate delta times, but its cheap and its enough for early testing. 
Later, following modules might be interesting (suggested in [this article](http://grauonline.de/wordpress/?page_id=1468)):
http://www.unicorecomm.com/en/product/content_1611.html
https://www.ardusimple.com/product/simplertk2b/


## Setup:
The idea is to split work on two devices:
* ESP32s takes care of the GPS tracking and corresponding calculations as well as handling SD card data
* Leonardo to work the display

The two devices communicate via a very simple binary protocol over Serial port.

![alt text](resources/IMG_20190515_114456.jpg "The assembled tracker decoding test NMEA data fed from PC")


### Connections:
* ESP32 -> GPS (Serial)
* ESP32 -> SD (SPI)
* ESP32 -> Leonardo (Serial)
* Leonardo -> Display/Touch (SPI + I2C?)


## Development:
Eclipse IDE with the following platforms/libraries

### Platforms:
* arduino
* esp32
	* FFat, FS, SD, SPI
	* Update, WebServer, WiFi (needs [this fix](https://github.com/espressif/arduino-esp32/issues/1327#issuecomment-435839829))

### Libraries:
* Adafruit FT6206
* Adafruit GFX
* Adafruit GPS
* Adafruit ILI9441
* NeoGPS

(SD library is already included in esp32 platform!)

Note: To be able to flash the Leonardo from within Eclipse, one neeeds [avrdude_autoreset_wrapper](https://github.com/gotzl/avrdude_autoreset_wrapper).

