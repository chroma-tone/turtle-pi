#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from signal import signal,SIGINT
import sys

from flask import Flask
from flask import render_template
from flask import request

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

@app.route("/twist")
def set_twist():
    speed = int(request.args.get("speed"))
    rotation = int(request.args.get("rotation"))

    rospy.loginfo("Setting Speed to {0:d}, and rotation to {1:d}".format(speed, rotation))
    twist = Twist()
    twist.linear.x = speed
    twist.angular.z = rotation

    turtle_control.publish(twist)
    return "Twist value {0:d}, {1:d}".format(twist.linear.x, twist.angular.z)

def initRospy():
    global turtle_control
    rospy.init_node('web_control', anonymous=True)
    turtle_control = rospy.Publisher('/key_vel', Twist, queue_size=10)

def sigintHandler(signal, frame):
    rospy.loginfo("Terminating webserver")
    sys.exit(0)

if __name__ == '__main__':
    signal(SIGINT, sigintHandler)
    initRospy()
    print "Starting webserver"
    app.run(debug=True, host='0.0.0.0', port=3000, use_reloader=False)
