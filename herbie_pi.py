#------------------------------------------------------------------------------
# Filename:     herbie_pi.py
# Author(s):    David Taylor, Dhruv Singhal
# ShortDesc:    This is the code of the Raspberry Pi on our senior design robot
#
# Usage:
# - Run on boot on Raspberry Pi Zero W
# - Control mode *will be* toggled with the PlayStation main Button
#  - Auto: Thumb Up/Down signal will activate/deactivate following
#  - Manual: left/right joystick Y axis controls left/right motors
#
# Changelog
# Version   Date        Delta
# v0.1      2019-12-01  List of libraries
# v1.0      2019-12-05  Funtional driving code
#
# Resources:
# - resource    |   url
#
# Feature Requests: (based on priority)
# - Integrate NVIDIA Jetson Nano (with either ML or OpenCV image processing)
# - Add a startup verification system (probably return a confirmation after all of the object constructors are run?) that gives LED feedback (progress bar for boot?).
#  - Add controller pairing failsafe
# - Add different controlling modes (rn we only have tank mode)
# - Replace pygame with homebrewed device file interpretation
#------------------------------------------------------------------------------

#-----<LIBRARIES>-----
import RPi.GPIO as GPIO  # The GPIO library for the Jetson Nano

from time import sleep

# Used for PlayStation DualShock4 interfacing
import pygame

# LED control
#import board    # Adafruit library per https://circuitpython.readthedocs.io/projects/neopixel/en/latest/
#import neopixel # library for controlling the LED ring

#import rpi_dc_lib  # not used in favor of Software PWM coded in our own DC_Motor_Controller class

#-----</LIBRARIES>-----

#-----<GLOBAL VARIABLES>-----

# GPIO PIN MAPPING
LED_PIN = 12

MOTOR_1A = 29   # Used for the right side of the vehicle
MOTOR_1B = 31   # "
MOTOR_2A = 33   # Used for the left side of the vehicle
MOTOR_2B = 35   # "

#-----</GLOBAL VARIABLES>-----


#-----<CLASSES>-----

class DC_Motor_Controller:

    """Object for controlling the DC motors with software PWM, courtesy of the RPi.GPIO library"""

    speed = 0

    # Pass the GPIO numbers for motor connections A and B
    def __init__(self, argA, argB):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)    # See RPi.GPIO docs for what this is

        GPIO.setup(int(argA), GPIO.OUT)  # Set pin as output
        GPIO.setup(int(argB), GPIO.OUT)  # "
        self.aPWM = GPIO.PWM(argA, 40)   # Set PWM frewquency to 40hz
        self.bPWM = GPIO.PWM(argB, 40)   # "

        self.aPWM.start(self.speed)   # Activate PWM for pin A
        self.bPWM.start(self.speed)   # Activate PWM for pin B

    def changeSpeed(self, newSpeed):
        """Input values between -100 and 100"""

        if self.speed != newSpeed:
            if newSpeed > 0:
                self.bPWM.ChangeDutyCycle(0)
                print("\n\nThis speed crashed it: ", newSpeed)
                self.aPWM.ChangeDutyCycle(newSpeed)
            elif newSpeed < 0:
                self.aPWM.ChangeDutyCycle(0)
                print("\n\nThis speed crashed it: ", newSpeed)
                self.bPWM.ChangeDutyCycle(newSpeed)
            else:
                print("\n\nThis speed crashed it: ", newSpeed)
                self.aPWM.ChangeDutyCycle(0)
                print("\n\nThis speed crashed it: ", newSpeed)
                self.bPWM.ChangeDutyCycle(0)
            self.speed = newSpeed

#class LED_Controller:
#    """Utilizes the Adafruit Neopixel library to control the output of the Neopixel LED ring."""
#
#    def __init__(self, arg):
#        super(LED_Controller, self).__init__()
#        self.arg = arg

class Remote_Control:
    """Use a DualShock4 controller to manually control the operation of the robot"""

    # Controller Button States
    Ex = False          # PyGame Button 0
    Circle = False      # PyGame Button 1
    Triangle = False    # PyGame Button 2
    Square = False      # PyGame Button 3
    L_Bumper = False    # PyGame Button 4
    R_Bumper = False    # PyGame Button 5
    L_Trigger = False   # PyGame Button 6
    R_Trigger = False   # PyGame Button 7
    Share = False       # PyGame Button 8
    Options = False     # PyGame Button 9
    PS = False        # PyGame Button 10
    L_Stick = False     # PyGame Button 11
    R_Stick = False     # PyGame Button 12

    # Controller Axis States
    L_X_Axis = 0        # PyGame Axis 0: [0, 1.0]
    R_X_Axis = 0        # PyGame Axis 3: [0, 1.0]
    L_Y_Axis = 0        # PyGame Axis 1: [0, 1.0]
    R_Y_Axis = 0        # PyGame Axis 4: [0, 1.0]
    L_Trigger = -1.0    # PyGame Axis 2: [-1.0, 1.0]
    R_Trigger = -1.0    # PyGame Axis 5: [-1.0, 1.0]

    def __init__(self):
        pygame.init()    # Initialize pygame library
        self.controller = pygame.joystick.Joystick(0) # Connect to the controller (and hope that it is paired with the Pi)
        self.controller.init()   # Prepare to read data from controller

    def update(self):   # Only update the relavent buttons/axies
        pygame.event.get()
        self.L_Y_Axis = self.controller.get_axis(1)
        self.R_Y_Axis = self.controller.get_axis(4)
        self.Ex = self.controller.get_button(0)
        self.PS = self.controller.get_button(10)

#class Autonomous_Control:
#   """NVIDIA Jetson Nano is used to provide motor control feedback based on realtime video processing"""

#-----</CLASSES>-----

#-----<FUNCTIONS>-----

def __main__():

    GPIO.cleanup()  # Clear any previously used GPIO modes

    # Initialize Neopixel ring
    #

    # Initialize motor controller objects
    Rmotor = DC_Motor_Controller(MOTOR_1A, MOTOR_1B)
    Lmotor = DC_Motor_Controller(MOTOR_2A, MOTOR_2B)

    # Initialize DualShock4 Controller Connection
    DS4 = Remote_Control()
    R_Y_AXIS_SCALE_VAL = 100    # Scale right stick Y-axis by 100 to match the changeSpeed method input range
    L_Y_AXIS_SCALE_VAL = 100    # Scale left stick Y-axis by 100 to match the changeSpeed method input range

    while True:
        DS4.update()
        Rmotor.changeSpeed(int(DS4.R_Y_Axis) * R_Y_AXIS_SCALE_VAL)
        Lmotor.changeSpeed(int(DS4.L_Y_Axis) * L_Y_AXIS_SCALE_VAL)


#-----</FUNCTIONS>-----

# Code Execution
__main__()
