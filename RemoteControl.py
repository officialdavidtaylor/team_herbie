#------------------------------------------------------------------------------
# Filename:     RemoteControl.py
# Author(s):    David Taylor
# ShortDesc:    This code enables manual remote control of the robot via a PlayStation DualShock4 bluetooth controller.
#
# Usage:        This code is to be used for testing, and eventually merged into a single file.
#
# Changelog
# Version   Date        Delta
# v0.1      2019-12-01  First code block for testing with Jetson Nano. Made table of key mappings.
#
# Resources:
# - PyGame Documentation    |   https://www.pygame.org/docs/
# - Example and button map  |   https://stackoverflow.com/questions/46557583/how-to-identify-which-button-is-being-pressed-on-ps4-controller-using-pygame
# - Good example            |   https://blog.mclemon.io/python-using-a-dualshock-4-with-pygame
# - Future low-level stuff  |   https://core-electronics.com.au/tutorials/using-usb-and-bluetooth-controllers-with-python.html
#------------------------------------------------------------------------------

#import Jetson.GPIO as GPIO # The GPIO library for the Jetson Nano
import RPi.GPIO as GPIO # For Raspberry Pi Testing

import pygame

# Note on controller indicies
#JOYBUTTON VALUES:
#------------------------
# 0     Ex
# 1     Circle
# 2     Triangle
# 3     Square
# 4     L Bumper
# 5     R Bumper
# 6     L Trigger
# 7     R Trigger
# 8     Share
# 9     Options
# 10    PS Button
# 11    L Stick
# 12    R Stick
#
#AXIS VALUES:
#------------------------
# 0     L X Axis
# 1     L Y Axis (inverted)
# 2     L Trigger (range starting from -1.0 to 1.0 at full press)
# 3     R X Axis
# 4     R Y Axis (inverted)
# 5     R Trigger (range starting from -1.0 to 1.0 at full press)

pygame.init()

controller = pygame.joystick.Joystick(0)
controller.init()

print(controller.get_numaxes())

try:
    while True:
        events = pygame.event.get()
        for event in events:
            if event.type == pygame.JOYBUTTONDOWN:
                print(controller.get_axis(4) * 100)

except KeyboardInterrupt:
    print("EXITING NOW")
    controller.quit()
    GPIO.cleanup()  # to be used when GPIO is active
