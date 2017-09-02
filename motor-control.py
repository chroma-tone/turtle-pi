#!/usr/bin/env python
import RPi.GPIO as GPIO
from time import sleep
import getch

GPIO.setmode(GPIO.BCM)
MotorAEN = 22
MotorAIN1 = 17
MotorAIN2 = 27


MotorBEN = 23
MotorBIN1 = 24
MotorBIN2 = 25

GPIO.setup(MotorAEN,GPIO.OUT)
GPIO.setup(MotorAIN1,GPIO.OUT)
GPIO.setup(MotorAIN2,GPIO.OUT)

GPIO.setup(MotorBEN,GPIO.OUT)
GPIO.setup(MotorBIN1,GPIO.OUT)
GPIO.setup(MotorBIN2,GPIO.OUT)

def stop():
    GPIO.output(MotorAEN,GPIO.LOW)
    GPIO.output(MotorAIN1,GPIO.LOW)
    GPIO.output(MotorAIN2,GPIO.LOW)

    GPIO.output(MotorBEN,GPIO.LOW)
    GPIO.output(MotorBIN1,GPIO.LOW)
    GPIO.output(MotorBIN2,GPIO.LOW)


def goForward():
    GPIO.output(MotorAEN,GPIO.LOW)
    GPIO.output(MotorAIN1,GPIO.LOW)
    GPIO.output(MotorAIN2,GPIO.HIGH)
    GPIO.output(MotorAEN,GPIO.HIGH)

    GPIO.output(MotorBEN,GPIO.LOW)
    GPIO.output(MotorBIN1,GPIO.LOW)
    GPIO.output(MotorBIN2,GPIO.HIGH)
    GPIO.output(MotorBEN,GPIO.HIGH)

def goRight():
    GPIO.output(MotorAEN,GPIO.LOW)
    GPIO.output(MotorAIN1,GPIO.LOW)
    GPIO.output(MotorAIN2,GPIO.HIGH)
    GPIO.output(MotorAEN,GPIO.HIGH)

    GPIO.output(MotorBEN,GPIO.LOW)


def goLeft():
    GPIO.output(MotorAEN,GPIO.LOW)

    GPIO.output(MotorBEN,GPIO.LOW)
    GPIO.output(MotorBIN1,GPIO.LOW)
    GPIO.output(MotorBIN2,GPIO.HIGH)
    GPIO.output(MotorBEN,GPIO.HIGH)

def goBack():
    GPIO.output(MotorAEN,GPIO.HIGH)
    GPIO.output(MotorAIN1,GPIO.HIGH)
    GPIO.output(MotorAIN2,GPIO.LOW)

    GPIO.output(MotorBEN,GPIO.HIGH)
    GPIO.output(MotorBIN1,GPIO.HIGH)
    GPIO.output(MotorBIN2,GPIO.LOW)

stop()
command = ""
sleeptime = 0.1
while command != "x":
    command = getch.getche()
    if command == 65:
        goForward()
        sleep(sleeptime)
        stop()
    elif command == "e":
        goBack()
        sleep(sleeptime)
        stop()
    elif command == "s":
        stop()

GPIO.cleanup()

