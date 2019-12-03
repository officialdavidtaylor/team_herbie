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
#import Jetson.GPIO as GPIO  # The GPIO library for the Jetson Nano
import RPi.GPIO as GPIO

# PlayStation DualShock4 interfacing
import pygame

# LED control
import board    # Adafruit library per https://circuitpython.readthedocs.io/projects/neopixel/en/latest/
import neopixel # library for controlling the LED ring
import rpi_dc_lib

from time import sleep

#-----</LIBRARIES>-----

#-----<GLOBAL VARIABLES>-----
LED_PIN = board.D12 # GPIO Pin #12

# MOTOR_A_PIN = board.D33
# MOTOR_B_PIN = board.D35
#-----</GLOBAL VARIABLES>-----


def motorone():
    # define instance of the class
    # (GPIO , GPIO , GPIO , freq , verbose, name)
    MotorOne = rpi_dc_lib.L298NMDc(19, 13, 26, 50, True, "motor_one")

    try:
        print("1. motor forward at 15")
        MotorOne.forward(15)
        input("press key to stop")
        print("motor stop\n")
        MotorOne.stop(0)
        sleep(3)

        print("2. motor forward ramp speed up 15 to 30 steps of 1")
        for i in range(15, 30):
            MotorOne.forward(i)
            sleep(1)
        MotorOne.stop(0)
        print("motor stoped\n")
        sleep(3)

        print("3. motor backward")
        MotorOne.backward(15)
        input("press key to stop")
        MotorOne.stop(0)
        print("motor stopped\n")
        sleep(3)

        print("4. motor backward ramp speed up up 15 to 30 steps of 1")
        for i in range(15, 30):
            MotorOne.backward(i)
            sleep(1)
        MotorOne.stop(0)
        print("motor stopped\n")
        sleep(3)

        print("5  brake check")
        MotorOne.forward(50)
        sleep(3)
        MotorOne.brake(0)
        print("motor brake\n")

    except KeyboardInterrupt:
        print("CTRL-C: Terminating program.")
    except Exception as error:
        print(error)
        print("Unexpected error:")
    else:
        print("No errors")
    finally:
        print("cleaning up")
        MotorOne.cleanup(True)


if __name__ == '__main__':
    motorone()
    exit()