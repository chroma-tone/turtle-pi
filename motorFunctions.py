import RPi.GPIO as GPIO
from enum import Enum

MotorAEN = 22
MotorAIN1 = 17
MotorAIN2 = 27
MotorBEN = 23
MotorBIN1 = 24
MotorBIN2 = 25

class direction(Enum):
    forward = 1
    back = 2
    left = 3
    right = 4
    none = 5

def stop(state):
    GPIO.output(MotorAEN,GPIO.LOW)
    GPIO.output(MotorAIN1,GPIO.LOW)
    GPIO.output(MotorAIN2,GPIO.LOW)

    GPIO.output(MotorBEN,GPIO.LOW)
    GPIO.output(MotorBIN1,GPIO.LOW)
    GPIO.output(MotorBIN2,GPIO.LOW)
    state = direction.none

def goBack(state):
    if state != direction.back:
        GPIO.output(MotorAEN,GPIO.LOW)   
        GPIO.output(MotorBEN,GPIO.LOW)
        state = direction.back
    
        GPIO.output(MotorAIN1,GPIO.LOW)
        GPIO.output(MotorAIN2,GPIO.HIGH)
        GPIO.output(MotorAEN,GPIO.HIGH)

        GPIO.output(MotorBIN1,GPIO.LOW)
        GPIO.output(MotorBIN2,GPIO.HIGH)
        GPIO.output(MotorBEN,GPIO.HIGH)

def goLeft(state):
    if state != direction.left:
        GPIO.output(MotorAEN,GPIO.LOW)
        GPIO.output(MotorBEN,GPIO.LOW)
        state = direction.left
    
        GPIO.output(MotorAIN1,GPIO.HIGH)
        GPIO.output(MotorAIN2,GPIO.LOW)
        GPIO.output(MotorAEN,GPIO.HIGH)

def goRight(state):
    if state != direction.right:
        GPIO.output(MotorAEN,GPIO.LOW)
        GPIO.output(MotorBEN,GPIO.LOW)
        state = direction.right

        GPIO.output(MotorBIN1,GPIO.HIGH)
        GPIO.output(MotorBIN2,GPIO.LOW)
        GPIO.output(MotorBEN,GPIO.HIGH)

def goForward(state):
    if state != direction.forward:
        GPIO.output(MotorAEN,GPIO.LOW)
        GPIO.output(MotorBEN,GPIO.LOW)
        state = direction.forward

        GPIO.output(MotorAEN,GPIO.HIGH)
        GPIO.output(MotorAIN1,GPIO.HIGH)
        GPIO.output(MotorAIN2,GPIO.LOW)

        GPIO.output(MotorBEN,GPIO.HIGH)
        GPIO.output(MotorBIN1,GPIO.HIGH)
        GPIO.output(MotorBIN2,GPIO.LOW)

def setupMotor():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(MotorAEN,GPIO.OUT)
    GPIO.setup(MotorAIN1,GPIO.OUT)
    GPIO.setup(MotorAIN2,GPIO.OUT)

    GPIO.setup(MotorBEN,GPIO.OUT)
    GPIO.setup(MotorBIN1,GPIO.OUT)
    GPIO.setup(MotorBIN2,GPIO.OUT)
    
def cleanup():
    GPIO.cleanup()

