#!/usr/bin/env python2.7
import RPi.GPIO as GPIO
import time
import board
import neopixel

from subprocess import Popen, PIPE, CalledProcessError

pixels = neopixel.NeoPixel(board.D18, 1)
state = 2


BUTTON_GPIO = 17

if __name__ == '__main__':
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BUTTON_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    pressed = False
    while True:
        # button is pressed when pin is LOW
        if not GPIO.input(BUTTON_GPIO):
            if not pressed:
                print("Button pressed!")
                pressed = True
                
                for j in range(3):
                 pixels.fill((0, 0, 0))
                 pixels.show()
                 time.sleep(0.2);
                 pixels.fill((0, 0, 20))
                 pixels.show()
                 time.sleep(0.2);
                error = 0
                with Popen("./muca-flash -b 1 -c 0x0A -i MuCaFirmware.bin -d 0", shell = True, stdout=PIPE, bufsize=1, universal_newlines=True) as p:
                   for line in p.stdout:
                        print(line.rstrip()) # process line here
                        line = line.split()
                        if line[1] == 'ERROR':
                         print("Completed with Error")
                         error = 1
                        elif line[1] == 'SUCCESS':
                         print("Completed without Error")
                         error = 0
                         
                if p.returncode != 0:
                     raise CalledProcessError(p.returncode, p.args)

                state = 1+error
               
            
        # button not pressed (or released)
        else:
            pressed = False
            state = 0

        if state == 1:
            pixels.fill((0, 20, 0))
            pixels.show()
        elif state == 2 :
            pixels.fill((20, 00, 0))
            pixels.show()
        elif state == 3:
            pixels.fill((0, 00, 20))
            pixels.show()
            