Naqua Water Parameters : Controller software (RPi)
--------------------------------------------------
Software runs on Raspberry Pi 3B.

A number of sensors are connected to the Pi's GPIO.
An Arduino Uno is connected via USB port to read from analog sensors.

Some of the sensors are on I2S bus and others on 1W (onewire) bus, hence the interfaces i2s and serial should be enabled.

the data transmitted to the central website (via wifi) is timestamped, so it is crucial that the date/time on the Pi is accurate.

website URL, as well as a host of other configuration parameters, are in pgoconfig.ini

errors.log and status.log are located in the same folder as the application script.

