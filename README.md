# FT5316dme: flashfirmware

We have an issue with the FT5316DME chip.  The chip FT5316DME seem to be sold without focaltech's firmware.
To develop the first prototypes, we had to desolder one chip of an existing touchpad and solder it to the board.

So far we didn't see any firmware file online, nor have received info from focaltech. However there are several hints online on how to flash a firmware to focaltech's FT5x chips. This can be done via i2c. 

## Solution 1: Flash the firmware
We don't have too many hints on exactly how to do it. However, 

#### Focals tech driver 
https://github.com/focaltech-systems/drivers-input-touchscreen-FTS_driver
This repo contains focaltech driver for linux kernel. The important bit is that it contains the "upgrade" function and the timings to put the chip in "upload" mode. But it didn't contain any firmware. 

#### FT5x06 Flash tool 
https://github.com/boundarydevices/ft5x06-tool/
This is an adaptd version of the focaltech driver, develop to flash a firmware to a ft5x06 chip (not exactly ours).  
One solution provided by this project could have been to read existing firmware and upload to another chip, but this feature is broken according to the developer.

I tried this on a raspy, without success. 

#### pcduino
https://github.com/pcduino/modules/tree/master/touch/ft5x
Maybe the most promising approach. 
PcDuino has a module for the ft5x touch driver. The repo contains the upgrade functions, but also two files that seems to be the firmware: `oflim-V31_20130909_app.h` and `ft_app.i`

#### Various linux kernel patches
There are several kernel patches implemnting driver for this chip and that includes upgrade functions, but the firmware is not provided.
https://github.com/azzazza/patch_kernel_q415/blob/master/drivers/input/touchscreen/ft5x06_ts.c
This implementation seem more solid than the other ones. 


## Solution 2: Buy new chips with flashed firmware.
But you to ensure the firmware is present and working ? 

## Solution 3: There is something wrong with our soldering ?
This might be possible. However, out of the 3 boards soldered with a "new" chip, non of them worked, and the 3 "desolder" chips are working just fine. 
