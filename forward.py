#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32


class Forward():
#############################################################
#############################################################

    #############################################################
    def __init__(self):
    #############################################################
        rospy.init_node("forward")

        self.speed = 2.0
        self.pub_lmotor = rospy.Publisher('lwheel_vtarget', Float32, queue_size=10)
        self.pub_rmotor = rospy.Publisher('rwheel_vtarget', Float32, queue_size=10)


#############################################################
    def spin(self):
    #############################################################
        r = rospy.Rate(10)
    
        while not rospy.is_shutdown():
            self.pub_lmotor.publish(self.speed)
            self.pub_rmotor.publish(self.speed)
            r.sleep()

if __name__ == '__main__':
    """ main """
    try:
        forward = Forward()
        forward.spin()
    except rospy.ROSInterruptException:
        pass