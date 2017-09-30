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
from motorFunctions import *

def initLogging():
    parser = argparse.ArgumentParser(description='motor controller for the turtle pi')
    parser.add_argument("--log",dest='log_level')
    args = parser.parse_args()
    log_level = args.log_level
    if log_level == None:
        logging.basicConfig(filename='log', level='WARNING')
    else:    
        numeric_level = getattr(logging, log_level.upper(), None)
        if not isinstance(numeric_level, int):
            raise ValueError('Invalid log level: %s' % log_level)
    
        logging.basicConfig(filename='log', level=numeric_level)

def checkDelta(sleep_time, state):
    global last_time
    global tlock
    tlock.acquire()
    delta = time() - last_time
    tlock.release()
    logging.debug("job: {} > {} ?".format(delta, sleep_time))
    if delta > sleep_time:
        stop(state)

def mainloop(state, scheduler):
    command = ""
    global last_time
    try: 
        while command != "x":
            #sleep(0.01)
            command = readchar.readchar()
            if command == '\033':
                readchar.readchar()
                command = readchar.readchar()
        
            if command == 'A':
                updateLastUpdatedTime()
                goForward(state)
            elif command == "B":
                updateLastUpdatedTime()
                goBack(state)
            elif command == "C":
                updateLastUpdatedTime()
                goRight(state)
            elif command == "D":
                updateLastUpdatedTime()
                goLeft(state)
            elif command == "s":
                stop(state)
    except (KeyboardInterrupt, SystemExit):
        print "exit keyboard thread"    
    print "ctrl-c to exit"
    cleanUp(state, scheduler)


def updateLastUpdatedTime():
    global last_time
    global tlock
    tlock.acquire()
    logging.debug("updating lastTime")
    last_time = time()
    tlock.release()

def main():
    print "Use the arrow keys to steer, 'x' to exit"
    initLogging()
    state = direction.none

    setupMotor()    
    stop(state)
    sleep_time = 0.2
    scheduler = BackgroundScheduler()
    scheduler.start()
    scheduler.add_job(checkDelta, 'interval', seconds = 0.1, args = [ sleep_time, state])

    keyboard_thread = Thread(target = mainloop, args = [state,  scheduler])
    keyboard_thread.start()
    try:
        while 1:
            sleep(1)
    except (KeyboardInterrupt, SystemExit):
        print "exit main"
        keyboard_thread.join()

def cleanUp(state, scheduler):        
    stop(state)
    cleanup()
    scheduler.shutdown()

tlock = Lock()
last_time = time()
if __name__ == "__main__":
    main()
