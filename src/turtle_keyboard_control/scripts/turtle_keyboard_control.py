#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist

def control_update(data):
    active = False

    if data.linear.x > 0:
        rospy.loginfo("Going forward")
        left_wheel.publish(data.linear.x)
        right_wheel.publish(data.linear.x)
        active = True

    elif data.linear.x < 0:
        rospy.loginfo("Going Backward")
        left_wheel.publish(data.linear.x)
        right_wheel.publish(data.linear.x)
        active = True

    if data.angular.z > 0:
        rospy.loginfo("Turning Right")
        left_wheel.publish(1)
        right_wheel.publish(-1)
        active = True

    elif data.angular.z < 0:
        rospy.loginfo("Turning Left")
        left_wheel.publish(-1)
        right_wheel.publish(1)
        active = True

    if not active:
        rospy.loginfo("Idling")
        left_wheel.publish(0)
        right_wheel.publish(0)

def handle_control():
    rospy.init_node('turtle_keyboard_control')
    rospy.Subscriber('/key_vel', Twist, control_update)
    rospy.spin()

if __name__ == '__main__':
    try:
        right_wheel = rospy.Publisher('/right_wheel', Int16, queue_size=10)
        left_wheel = rospy.Publisher('/left_wheel', Int16, queue_size=10)
        handle_control()
    except rospy.ROSInterruptException:
        pass
