#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Int16

WHEEL_ENCODER_PIN = 14

def encoder_tick(channel):
    global tick_count
    global tick_topic

    tick_count += 1
    tick_topic.publish(tick_count)

def init_wheel_encoder():
    global tick_count
    global tick_topic

    rospy.init_node('wheel_encoder', anonymous=True)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(WHEEL_ENCODER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(WHEEL_ENCODER_PIN, GPIO.RISING)
    GPIO.add_event_callback(WHEEL_ENCODER_PIN, encoder_tick)

    tick_topic = rospy.Publisher('/wheel_ticks', Int16, queue_size=10)
    tick_count = 0

def waitForInterrupts():
    rospy.spin()

if __name__ == '__main__':
    init_wheel_encoder()
    waitForInterrupts()