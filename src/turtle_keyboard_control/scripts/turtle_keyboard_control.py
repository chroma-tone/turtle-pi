#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def keyPressed(data):
    if data.linear.x > 0:
        rospy.loginfo("Going forward")
        pub.publish("Forward")
    elif data.linear.x < 0:
        rospy.loginfo("Going Backward")
        pub.publish("Backward")
    else:
        rospy.loginfo("Stopping")
        pub.publish("Release")

def handleKeys():
    rospy.init_node('turtle_keyboard_control')
    rospy.Subscriber('/key_vel', Twist, keyPressed)
    rospy.spin()

if __name__ == '__main__':
    try:
        pub = rospy.Publisher('/right_wheel', String, queue_size=10)
        handleKeys()
    except rospy.ROSInterruptException:
        pass
