#------------------------------------------------------------------------------
# Filename:     herbie.py
# Author(s):    David Taylor
# ShortDesc:    This is the code behind our senior design robot
#
# Usage:
# - Run on boot on NVIDIA Jetson Nano
# - Control mode is toggled with the PlayStation main Button
#  - Auto: Thumb Up/Down signal will activate/deactivate following
#  - Manual: left/right joystick Y axis controls left/right motors
# - System shutdown toggled by pressing 'X' button on remote
#
# Changelog
# Version   Date        Delta
# v0.1      2019-12-01  List of libraries
#
# Resources:
# - resource    |   url
#------------------------------------------------------------------------------

#-----<LIBRARIES>-----
import Jetson.GPIO as GPIO  # The GPIO library for the Jetson Nano

# PlayStation DualShock4 interfacing
import pygame

# LED control
import board    # Adafruit library per https://circuitpython.readthedocs.io/projects/neopixel/en/latest/
import neopixel # library for controlling the LED ring

from time import sleep
#-----</LIBRARIES>-----

#-----<GLOBAL VARIABLES>-----
LED_PIN = board.D12 # GPIO Pin #12

MOTOR_A_PIN = board.D33
MOTOR_B_PIN = board.D35
#-----</GLOBAL VARIABLES>-----
