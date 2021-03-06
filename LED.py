#------------------------------------------------------------------------------
# Filename:     LED.py
# Author(s):    David Taylor
# ShortDesc:    This code controls the NeoPixel
#
# Usage:
#
# Changelog
# Version   Date        Delta
# v0.1      2019-11-22  First test of the GPIO code (for Raspberry Pi, should port to Jetson Nano just fine)
#
#------------------------------------------------------------------------------

#from enum import Enum
from time import sleep # for delays :)

import Jetson.GPIO # The GPIO library for the Jetson Nano
#import RPi.GPIO as GPIO
import board # Adafruit library per https://circuitpython.readthedocs.io/projects/neopixel/en/latest/
import neopixel # library for controlling the LED ring

#GPIO.setmode(GPIO.BOARD)

#---LED Code for Robot---

led = neopixel.NeoPixel(board.D12, 24, pixel_order=neopixel.RGB) # control neopixel leds via GPIO pin 12 with tuple (R, G, B, W)

def setLED(mode):
    if mode:
        state = 1
    else:
        state = 0

    for num in range(len(led)):
        led[num] = (20*(1-state), 20*state, 0) # set all LEDs to either RED (255, 0, 0, 100) or GREEN (0, 255, 0, 100) based on Mode enum input
    led.show()

#---Code to Test---

# pixel = neopixel.NeoPixel(GPIO.D12, 1, pixel_order=neopixel.RGBW) # Need to verify arg1 datatype - attempting to control LED ring with GPIO pin 12
# pixel[0] = (30, 0, 20, 10) # set first LED in ring to RGBW value of (30,0,20,10)

while(1):
    setLED(True)
    print('ON')
    sleep(1.0)
    setLED(False)
    print('OFF')
    sleep(1.0)

#------------------
