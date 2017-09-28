import sr-motor-control.py
import Rpi.GPIO as GPIO

def setupGpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(MOTORLATCH, GPIO.OUT)
    GPIO.setup(MOTORENABLE, GPIO.OUT)
    GPIO.setup(MOTORDATA, GPIO.OUT)
    GPIO.setup(MOTORCLK, GPIO.OUT)


