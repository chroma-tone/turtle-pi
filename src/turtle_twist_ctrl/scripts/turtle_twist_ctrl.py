#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from math import pi

class TurtleTwistControl:
    def __init__(self):
        rospy.init_node("turtle_twist_control")
        nodename = rospy.get_name()
        rospy.loginfo("%s started" % nodename)
    
        self.w = rospy.get_param("~base_width", 0.5)
        self.pub_lmotor = rospy.Publisher('/lmotor', Int16, queue_size=10)
        self.pub_rmotor = rospy.Publisher('/rmotor', Int16, queue_size=10)
    
        self.left = 0
        self.right = 0
        
    def twistCallback(self, msg):
        self.dx = msg.linear.x
        self.dr = -msg.angular.z * 180.0 / pi

        # dx = (l + r) / 2
        # dr = (r - l) / w
            
        self.right = min(int(100.0 * self.dx + self.dr * self.w / 2), 100)
        self.left = min(int(100.0 * self.dx - self.dr * self.w / 2), 100)
        rospy.loginfo("Twist: (%f,%f) => publishing: (%d, %d)", self.dx, self.dr, self.left, self.right) 
                
        self.pub_lmotor.publish(self.left)
        self.pub_rmotor.publish(self.right)

    def spin(self):
        rospy.Subscriber('/cmd_vel', Twist, self.twistCallback)
        rospy.spin()

if __name__ == '__main__':
    try:
        turtleTwistControl = TurtleTwistControl()
        turtleTwistControl.spin()

    except rospy.ROSInterruptException:
        pass
