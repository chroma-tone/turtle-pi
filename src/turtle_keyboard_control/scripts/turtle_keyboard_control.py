#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist

def keyPressed(data):
    active = False

    if data.linear.x > 0:
        rospy.loginfo("Going forward")
        pubL.publish(1)
        pubR.publish(1)
        active = True
    elif data.linear.x < 0:
        rospy.loginfo("Going Backward")
        pubL.publish(-1)
        pubR.publish(-1)
        active = True

    if data.angular.z > 0:
        rospy.loginfo("Turning Right")
        pubL.publish(1)
        pubR.publish(-1)
        active = True

    elif data.angular.z < 0:
        rospy.loginfo("Turning Left")
        pubL.publish(-1)
        pubR.publish(1)
        active = True

    if not active:
        rospy.loginfo("Idling")
        pubL.publish(0)
        pubR.publish(0)

def handleKeys():
    rospy.init_node('turtle_keyboard_control')
    rospy.Subscriber('/key_vel', Twist, keyPressed)
    rospy.spin()

if __name__ == '__main__':
    try:
        pubR = rospy.Publisher('/right_wheel', Int16, queue_size=10)
        pubL = rospy.Publisher('/left_wheel', Int16, queue_size=10)
        handleKeys()
    except rospy.ROSInterruptException:
        pass
