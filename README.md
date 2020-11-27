MuCa - Firmware
===========


Before starting
------------
#### If you ordered the board online, the flashing is not required!



This code is a form from [ft5x06-tool](https://github.com/boundarydevices/ft5x06-tool), largely inspired from the [Focaltech GitHub driver](https://github.com/focaltech-systems/drivers-input-touchscreen-FTS_driver).


Requirements
------------

The source code itself isn't depending any external library, so no build dependency.

As for runtime dependency, the kernel must have `i2c-dev` support enabled.

You need to have a physical access to the i2c ports, like raspberry pi. 

To enable `i2c-dev`:
```
sudo raspi-config
5 > Interfacing options
P5 I2C
<Enable>
```


Build instructions
------------------

Very straight-forward:
```
$ git clone https://github.com/muca-board/muca-firmware.git
$ cd muca-firmware/
$ make
```


Usage With Flashing Rig
-----

I setup a flashing rig to flash the Muca board. You don't need this rig if you don't need to flash 650 boards. 


```
$ sudo python3 muca-flash-autorun.py
```

Usage
-----

To upload the firmware to the Muca Board (FT5316DME)

```
$ ./muca-flash -b 1 -c 0x0A -i MuCaFirmware.bin 
```

On a raspberry pi, the i2c bus is set to `1`. The chip id has to be forced because the firmware is empty. 



The tool has an help output which lists all the different parameters.
```
FT5x06 tool usage: ft5x06-tool [OPTIONS]
OPTIONS:
	-a, --address
		I2C address of the FT5x06 controller (hex). Default is 0x38.
	-b, --bus
		I2C bus the FT5x06 controller is on. Default is 1.
	-c, --chipid
		Force chip ID to the value (hex). Default is read from controller.
	-i, --input
		Input firmware file to flash.
	-o, --output
		Output firmware file read from FT5x06.
	-d, --debug
		Enable or disable I2C debug. Default 0, 1 to enable
	-h, --help
		Show this help and exit.
```
