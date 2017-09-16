#!/usr/bin/env python
import RPi.GPIO as GPIO
from time import sleep, time
import readchar
import os, sys
from apscheduler.schedulers.background import BackgroundScheduler
from enum import Enum
import logging
from threading import Thread, Lock
import argparse
parser = argparse.ArgumentParser(description='motor controller for the turtle pi')
parser.add_argument("--log",dest='log_level')
args = parser.parse_args()
log_level = args.log_level
if log_level != None:
    numeric_level = getattr(logging, log_level.upper(), None)
    if not isinstance(numeric_level, int):
        raise ValueError('Invalid log level: %s' % log_level)

    logging.basicConfig(filename='log', level=numeric_level)

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

def checkDelta(sleep_time, state, tlock):
    global last_time
    tlock.acquire()
    delta = time() - last_time
    tlock.release()
    logging.debug("job: {} > {} ?".format(delta, sleep_time))
    if delta > sleep_time:
        stop(state)


MotorAEN = 22
MotorAIN1 = 17
MotorAIN2 = 27
MotorBEN = 23
MotorBIN1 = 24
MotorBIN2 = 25

def mainloop(state, scheduler, tlock):
    command = ""
    global last_time
    try: 
        while command != "x":
            sleep(0.05)
            command = readchar.readchar()
            if command == '\033':
                readchar.readchar()
                command = readchar.readchar()
        
            if command == 'A':
                updateLastUpdatedTime(tlock)
                goForward(state)
            elif command == "B":
                updateLastUpdatedTime(tlock)
                goBack(state)
            elif command == "C":
                updateLastUpdatedTime(tlock)
                goRight(state)
            elif command == "D":
                updateLastUpdatedTime(tlock)
                goLeft(state)
            elif command == "s":
                stop(state)
    except (KeyboardInterrupt, SystemExit):
        print "exit keyboard thread"    
    print "ctrl-c to exit"
    cleanUp(state, scheduler)

def updateLastUpdatedTime(tlock):
    global last_time
    tlock.acquire()
    logging.debug("updating lastTime")
    last_time = time()
    tlock.release()

def main():
    state = direction.none

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(MotorAEN,GPIO.OUT)
    GPIO.setup(MotorAIN1,GPIO.OUT)
    GPIO.setup(MotorAIN2,GPIO.OUT)

    GPIO.setup(MotorBEN,GPIO.OUT)
    GPIO.setup(MotorBIN1,GPIO.OUT)
    GPIO.setup(MotorBIN2,GPIO.OUT)
    
    stop(state)
    sleep_time = 0.2
    tlock = Lock()
    scheduler = BackgroundScheduler()
    scheduler.start()
    scheduler.add_job(checkDelta, 'interval', seconds=0.05, args=[ sleep_time, state, tlock])
    keyboard_thread = Thread(target = mainloop, args = [state,  scheduler, tlock])
    keyboard_thread.start()
    try:
        while 1:
            sleep(1)
    except (KeyboardInterrupt, SystemExit):
        print "exit main"
        keyboard_thread.join()


def cleanUp(state, scheduler):        
    stop(state)
    GPIO.cleanup()
    scheduler.shutdown()

last_time = time()
if __name__ == "__main__":
    main()
