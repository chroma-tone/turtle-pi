#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Int16

LWHEEL_ENCODER_PIN = 14
RWHEEL_ENCODER_PIN = 15

def l_encoder_tick(channel):
    global l_tick_count
    global l_tick_topic
    global l_tick_direction

    l_tick_count += l_tick_direction
    l_tick_topic.publish(l_tick_count)

def r_encoder_tick(channel):
    global r_tick_count
    global r_tick_topic
    global r_tick_direction

    r_tick_count += r_tick_direction
    r_tick_topic.publish(r_tick_count)

def init_wheel_encoders():
    global l_tick_count
    global l_tick_topic
    global l_tick_direction
    global r_tick_count
    global r_tick_topic
    global r_tick_direction
    
    rospy.init_node('wheel_encoder', anonymous=True)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LWHEEL_ENCODER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(LWHEEL_ENCODER_PIN, GPIO.RISING)
    GPIO.add_event_callback(LWHEEL_ENCODER_PIN, l_encoder_tick)

    GPIO.setup(RWHEEL_ENCODER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(RWHEEL_ENCODER_PIN, GPIO.RISING)
    GPIO.add_event_callback(RWHEEL_ENCODER_PIN, r_encoder_tick)

    l_tick_topic = rospy.Publisher('/left_wheel_ticks', Int16, queue_size=10)
    r_tick_topic = rospy.Publisher('/right_wheel_ticks', Int16, queue_size=10)
    l_tick_count = 0
    r_tick_count = 0

def right_wheel_callback(data):
    global r_tick_direction
    if data.data > 0:
        r_tick_direction = 1
    elif data.data < 0:
        r_tick_direction = -1
    elif data.data == 0:
        pass # leave as it was, assume that any further ticks are in the same direction

def left_wheel_callback(data):
    global l_tick_direction
    if data.data > 0:
        l_tick_direction = 1
    elif data.data < 0:
        l_tick_direction = -1
    elif data.data == 0:
        pass # leave as it was, assume that any further ticks are in the same direction
        
def monitor_wheel_commands():
    global r_tick_direction
    global l_tick_direction
    r_tick_direction = 0
    l_tick_direction = 0

    rospy.Subscriber("right_wheel", Int16, right_wheel_callback)
    rospy.Subscriber("left_wheel", Int16, left_wheel_callback)

def waitForInterrupts():
    rospy.spin()

if __name__ == '__main__':
    init_wheel_encoders()
    monitor_wheel_commands()
    rospy.spin()
    # waitForInterrupts()
