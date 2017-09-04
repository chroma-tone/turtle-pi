#!/usr/bin/env python
import RPi.GPIO as GPIO
from time import sleep, time
import readchar
import os
from apscheduler.schedulers.background import BackgroundScheduler
from enum import Enum
import logging
from threading import Thread
logging.basicConfig()

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

def goRight(state):
    if state != direction.right:
        GPIO.output(MotorAEN,GPIO.LOW)
        GPIO.output(MotorBEN,GPIO.LOW)
        state = direction.right
    
    GPIO.output(MotorAIN1,GPIO.LOW)
    GPIO.output(MotorAIN2,GPIO.HIGH)
    GPIO.output(MotorAEN,GPIO.HIGH)

def goLeft(state):
    if state != direction.left:
        GPIO.output(MotorAEN,GPIO.LOW)
        GPIO.output(MotorBEN,GPIO.LOW)
        state = direction.left

    GPIO.output(MotorBIN1,GPIO.LOW)
    GPIO.output(MotorBIN2,GPIO.HIGH)
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

def checkDelta(lastTime, sleeptime, state):
    #print "job : " + str(lastTime) + " " + str(sleeptime) + " " + str(state)
    delta = time() - lastTime
    if delta > sleeptime:
        stop(state)


MotorAEN = 22
MotorAIN1 = 17
MotorAIN2 = 27
MotorBEN = 23
MotorBIN1 = 24
MotorBIN2 = 25

def mainloop(state, lastTime, scheduler):
    command = ""
    try: 
        while command != "x":
            command = readchar.readchar()
            if command == '\033':
                readchar.readchar()
                command = readchar.readchar()
        
            if command == 'A':
                lastTime = time()
                goForward(state)
            elif command == "B":
                lastTime = time()
                goBack(state)
            elif command == "C":
                lastTime = time()
                goRight(state)
            elif command == "D":
                lastTime = time()
                goLeft(state)
            elif command == "s":
                stop(state)
    except (KeyboardInterrupt, SystemExit):
        print "system exception"    
    print "ctrl-c to exit"
    cleanUp(state, scheduler)

def main():
    lastTime = time()
    state = direction.none

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(MotorAEN,GPIO.OUT)
    GPIO.setup(MotorAIN1,GPIO.OUT)
    GPIO.setup(MotorAIN2,GPIO.OUT)

    GPIO.setup(MotorBEN,GPIO.OUT)
    GPIO.setup(MotorBIN1,GPIO.OUT)
    GPIO.setup(MotorBIN2,GPIO.OUT)
    
    stop(state)
    sleeptime = 0.3
    
    scheduler = BackgroundScheduler()
    scheduler.start()
    scheduler.add_job(checkDelta, 'interval', seconds=0.05, args=[lastTime, sleeptime, state])
    keyboardThread = Thread(target = mainloop, args = [state, lastTime, scheduler])
    keyboardThread.start()
    try:
        while 1:
            sleep(1)
    except (KeyboardInterrupt, SystemExit):
        print "system exception"    

def cleanUp(state, scheduler):        
    stop(state)
    GPIO.cleanup()
    scheduler.shutdown()

if __name__ == "__main__":
    main()
