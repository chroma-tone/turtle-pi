#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def keyPressed(data):
    if data.linear.x > 0:
        rospy.loginfo("Going forward")
        pubL.publish("Forward")
        pubR.publish("Forward")
    elif data.linear.x < 0:
        rospy.loginfo("Going Backward")
        pubL.publish("Backward")
        pubR.publish("Backward")
    else:
        rospy.loginfo("Stopping")
        pubL.publish("Release")
        pubR.publish("Release")

    if data.angular.z > 0:
        rospy.loginfo("Turning Right")
        pubL.publish("Backward")
        pubR.publish("Forward")

    elif data.angular.z < 0:
        rospy.loginfo("Turning Left")
        pubL.publish("Forward")
        pubR.publish("Backward")

def handleKeys():
    rospy.init_node('turtle_keyboard_control')
    rospy.Subscriber('/key_vel', Twist, keyPressed)
    rospy.spin()

if __name__ == '__main__':
    try:
        pubR = rospy.Publisher('/right_wheel', String, queue_size=10)
        pubL = rospy.Publisher('/left_wheel', String, queue_size=10)
        handleKeys()
    except rospy.ROSInterruptException:
        pass
