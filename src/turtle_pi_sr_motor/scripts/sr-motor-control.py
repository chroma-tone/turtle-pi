#!/usr/bin/env python

import RPi.GPIO as GPIO
from time import sleep
from time import time

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32

class ShiftRegisterMotorControl:
    # Static state of latch across all ShiftRegisterMotorControl instances
    latchState = 0
    gpiosInitialized = False

    # Ref https://cdn-learn.adafruit.com/assets/assets/000/009/769/original/mshieldv1-schem.png
    # TODO: Motor 3 and 4 don't seem to be working - need to investigate why later
    # Bit positions in the 74HCT595 shift register output
    MOTOR1_A = 2
    MOTOR1_B = 3
    MOTOR2_A = 1
    MOTOR2_B = 4
    MOTOR3_A = 5
    MOTOR3_B = 7
    MOTOR4_A = 0
    MOTOR4_B = 6

    # Constants that the user passes in to the motor calls
    UNINITIALIZED = 0
    FORWARD = 1
    BACKWARD = 2
    BRAKE = 3
    RELEASE = 4

    # RPI pins: Ref https://www.element14.com/community/servlet/JiveServlet/previewBody/73950-102-11-339300/pi3_gpio.png
    # Interface to 74HCT595 latch

# Purpose  74HC595  Arduino  Rpi     Color
# !Latch   12       D12      GPIO19  White
# !CLK     11       D4       GPIO13  Grey
# !EN      13       D7       GPIO6   Purple
# DATA     14       D8       GPIO5   Blue

    MOTORLATCH = 19 
    MOTORCLK = 13 
    MOTORENABLE = 6 
    MOTORDATA = 5 

    # PWM motor speed control
    MOTOR1_EN = 12 # Arduino D11
    MOTOR2_EN = 16 # Arduino D3
    MOTOR3_EN = 20 # Arduino D5
    MOTOR4_EN = 21 # Arduino D6

    # PWM control frequency (Hz)
    PWM_FREQ = 50

    @staticmethod
    def initializeGpios():
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(ShiftRegisterMotorControl.MOTORLATCH, GPIO.OUT)
        GPIO.setup(ShiftRegisterMotorControl.MOTORCLK, GPIO.OUT)
        GPIO.setup(ShiftRegisterMotorControl.MOTORENABLE, GPIO.OUT)
        GPIO.setup(ShiftRegisterMotorControl.MOTORDATA, GPIO.OUT)
        GPIO.setup(ShiftRegisterMotorControl.MOTOR1_EN, GPIO.OUT)
        GPIO.setup(ShiftRegisterMotorControl.MOTOR2_EN, GPIO.OUT)
        GPIO.setup(ShiftRegisterMotorControl.MOTOR3_EN, GPIO.OUT)
        GPIO.setup(ShiftRegisterMotorControl.MOTOR4_EN, GPIO.OUT)

        GPIO.output(ShiftRegisterMotorControl.MOTORENABLE, GPIO.LOW)
        ShiftRegisterMotorControl.gpiosInitialized = True

    def __init__(this, motorNumber):
        if motorNumber > 4 or motorNumber < 1:
            raise ValueError, "motorNumber must be between 1 and 4 inclusive"

        if ~ShiftRegisterMotorControl.gpiosInitialized:
            ShiftRegisterMotorControl.initializeGpios()

        if motorNumber == 1:
            this.motorA = ShiftRegisterMotorControl.MOTOR1_A
            this.motorB = ShiftRegisterMotorControl.MOTOR1_B
            this.motorPwm = GPIO.PWM(ShiftRegisterMotorControl.MOTOR1_EN, ShiftRegisterMotorControl.PWM_FREQ)

        elif motorNumber == 2:
            this.motorA = ShiftRegisterMotorControl.MOTOR2_A
            this.motorB = ShiftRegisterMotorControl.MOTOR2_B
            this.motorPwm = GPIO.PWM(ShiftRegisterMotorControl.MOTOR2_EN, ShiftRegisterMotorControl.PWM_FREQ)

        elif motorNumber == 3:
            this.motorA = ShiftRegisterMotorControl.MOTOR3_A
            this.motorB = ShiftRegisterMotorControl.MOTOR3_B
            this.motorPwm = GPIO.PWM(ShiftRegisterMotorControl.MOTOR3_EN, ShiftRegisterMotorControl.PWM_FREQ)

        elif motorNumber == 4:
            this.motorA = ShiftRegisterMotorControl.MOTOR4_A
            this.motorB = ShiftRegisterMotorControl.MOTOR4_B
            this.motorPwm = GPIO.PWM(ShiftRegisterMotorControl.MOTOR4_EN, ShiftRegisterMotorControl.PWM_FREQ)

        # Initialize motor driver to idle state
        this.motorPwm.start(0)
        this.currentDirection = ShiftRegisterMotorControl.UNINITIALIZED
        this.setDirection(ShiftRegisterMotorControl.RELEASE)

    @staticmethod
    def latch_tx():
        latch_start = time()
        GPIO.output(ShiftRegisterMotorControl.MOTORLATCH, GPIO.LOW)

        # Set Clock Low initially
        GPIO.output(ShiftRegisterMotorControl.MOTORCLK, GPIO.LOW)

        # print "ShiftRegisterMotorControl.latchState = " + str(ShiftRegisterMotorControl.latchState)
        for i in range(0,8):
            # Set up data pin with next bit
            if (ShiftRegisterMotorControl.latchState << i) & 0x80 == 0x80:
                # print "Latching 1"
                GPIO.output(ShiftRegisterMotorControl.MOTORDATA, GPIO.HIGH)
            else:
                # print "Latching 0"
                GPIO.output(ShiftRegisterMotorControl.MOTORDATA, GPIO.LOW)

            sleep(0.001)

            # And pulse clock to serialize through
            GPIO.output(ShiftRegisterMotorControl.MOTORCLK, GPIO.HIGH)
            sleep(0.001)
            GPIO.output(ShiftRegisterMotorControl.MOTORCLK, GPIO.LOW)

        GPIO.output(ShiftRegisterMotorControl.MOTORLATCH, GPIO.HIGH)
        latch_end = time()
        latch_delta = latch_end - latch_start
        rospy.loginfo ("Latch delta {0:2}".format(latch_delta))

    def setDirection(this, direction):
        # Check if we're already going this way - no need to re-latch
        if this.currentDirection == direction:
            return;

        if direction == ShiftRegisterMotorControl.FORWARD:
            ShiftRegisterMotorControl.latchState |= 1 << this.motorA
            ShiftRegisterMotorControl.latchState &= ~(1 << this.motorB)

        if direction == ShiftRegisterMotorControl.BACKWARD:
            ShiftRegisterMotorControl.latchState &= ~(1 << this.motorA)
            ShiftRegisterMotorControl.latchState |= 1 << this.motorB

        if direction == ShiftRegisterMotorControl.RELEASE:
            ShiftRegisterMotorControl.latchState &= ~(1 << this.motorA)
            ShiftRegisterMotorControl.latchState &= ~(1 << this.motorB)

        ShiftRegisterMotorControl.latch_tx()
        this.currentDirection = direction

    def setSpeed(this, percentSpeed):
        print "Changing duty cycle to " + str(percentSpeed)
        this.motorPwm.ChangeDutyCycle(int(percentSpeed))

def test1():
    motor1 = ShiftRegisterMotorControl(1)
    motor1.setSpeed(100)

    for i in range(1,10):
        print "Forward"
        motor1.setDirection(ShiftRegisterMotorControl.FORWARD)
        sleep(1)

        print "Backwards"
        motor1.setDirection(ShiftRegisterMotorControl.BACKWARD)
        sleep(1)

        print "Release"
        motor1.setDirection(ShiftRegisterMotorControl.RELEASE)
        sleep(1)

    motor1.setSpeed(0)

def shift_test():
    try:
        ShiftRegisterMotorControl.initializeGpios()
        while True:
            for i in range(0,8):
                ShiftRegisterMotorControl.latchState = 1 << i
                ShiftRegisterMotorControl.latch_tx()
                sleep(1)
    except:
        pass

# test1()
# shift_test()

def rightWheelCallback(data):
    rospy.loginfo(rospy.get_caller_id() + " - Setting Right Wheel Direction %s", data.data)

    # TODO: Check if we're already going in the right direction - no need to reload if so

    if data.data > 0:
        motorR.setDirection(ShiftRegisterMotorControl.FORWARD)
        motorR.setSpeed(data.data)
    elif data.data < 0:
        motorR.setDirection(ShiftRegisterMotorControl.BACKWARD)
        motorR.setSpeed(-data.data)
    elif data.data == 0:
        motorR.setDirection(ShiftRegisterMotorControl.RELEASE)
        motorR.setSpeed(0)
    else:
        rospy.loginfo(rospy.get_caller_id() + " Unknown command %s", data.data)

def leftWheelCallback(data):
    rospy.loginfo(rospy.get_caller_id() + " - Setting Left Wheel Direction %s", data.data)

    if data.data > 0:
        motorL.setDirection(ShiftRegisterMotorControl.FORWARD)
        motorL.setSpeed(data.data)
    elif data.data < 0:
        motorL.setDirection(ShiftRegisterMotorControl.BACKWARD)
        motorL.setSpeed(-data.data)
    elif data.data == 0:
        motorL.setDirection(ShiftRegisterMotorControl.RELEASE)
        motorL.setSpeed(0)
    else:
        rospy.loginfo(rospy.get_caller_id() + " Unknown command %s", data.data)

def listener():
    rospy.init_node('turtle_pi', anonymous=True)
    rospy.Subscriber("rmotor", Float32, rightWheelCallback)
    rospy.Subscriber("lmotor", Float32, leftWheelCallback)
    rospy.spin()

if __name__ == '__main__':
    motorL = ShiftRegisterMotorControl(1)
    motorL.setSpeed(80.0)

    motorR = ShiftRegisterMotorControl(2)
    motorR.setSpeed(80.0)

    listener()
    motorL.setSpeed(0.0)
    motorR.setSpeed(0.0)

GPIO.cleanup()

