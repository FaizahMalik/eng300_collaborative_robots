#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from python_tools import *
import math


def move(i):
    x = clamp(math.sin(i * 0.1) * 0.3, -0.2, 0.2)
    z = 0 # math.cos(i * 0.1) * 0.5
    # print("x: %f" % x)
    # print("z: %f" % z)
    vel_msg.linear.x = x
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = z
    velocity_publisher.publish(vel_msg)
    return


if __name__ == '__main__':
    rospy.init_node('robot_cleaner', anonymous=True)
    rate = rospy.Rate(5)
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    t = 0
    try:
        while not rospy.is_shutdown():
            move(t)
            t += 1
            rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass