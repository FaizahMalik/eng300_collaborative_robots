#!/usr/bin/env python


import rospy
import os
from python_tools import *
import numpy

import geometry_msgs.msg
import tf2_ros
import math
from autonomous_robot.msg import LR_wheel_odom


class DiffTf:
    def __init__(self):
        self.rate = load_param('diff_drive_odom/rate', 1)
        self.base_frame = load_tf_prefix(
            load_param('diff_drive_odom/base_frame_id', 'base_link'))
        self.odom_frame = load_tf_prefix(
            load_param('diff_drive_odom/odom_frame_id', 'odom'))
        self.base_width = load_param('robot/wheel_base', 0.35)
        self.wheel_radius = load_param('robot/wheel_Radius', 0.0325)  # drive wheel radius (m)

        self.odom_tf = geometry_msgs.msg.TransformStamped()
        self.odom_pub = tf2_ros.TransformBroadcaster()
        rospy.Subscriber("LR_odom", LR_wheel_odom, self.odom_callback)
        self.LW_dis = 0
        self.RW_dis = 0
        self.x = 0
        self.y = 0
        self.th = 0

    def odom_callback(self, msg):
        self.LW_dis = (float(msg.left) / (840 * 4)) * 2 * math.pi * self.wheel_radius
        self.RW_dis = (float(msg.right) / (840 * 4)) * 2 * math.pi * self.wheel_radius
        self.cartesian()
        self.quaternion()

    def init(self):
        self.cartesian()
        self.quaternion()

    def cartesian(self):
        dl = self.LW_dis
        dr = self.RW_dis
        d_lin = (dr + dl) / 2
        th = numpy.sin((dr - dl) / self.base_width)
        # th = ((dr - dl) / self.base_width)
        self.th += th
        dx = numpy.cos(self.th) * d_lin
        dy = numpy.sin(self.th) * d_lin
        self.x += dx
        self.y += dy

    def quaternion(self):
        self.odom_tf.header.stamp = rospy.Time.now()
        self.odom_tf.header.frame_id = self.odom_frame
        self.odom_tf.child_frame_id = self.base_frame
        self.odom_tf.transform.translation.x = self.x
        self.odom_tf.transform.translation.y = self.y
        self.odom_tf.transform.translation.z = 0.0
        self.odom_tf.transform.rotation.x = 0
        self.odom_tf.transform.rotation.y = 0
        self.odom_tf.transform.rotation.z = numpy.sin(self.th / 2)
        self.odom_tf.transform.rotation.w = numpy.cos(self.th / 2)
        self.odom_pub.sendTransform(self.odom_tf)

    def spin(self):
        r = rospy.Rate(self.rate)
        self.init()
        while not rospy.is_shutdown():
            r.sleep()


if __name__ == '__main__':
    rospy.init_node("diff_drive_odom")
    try:
        diffTf = DiffTf()
        diffTf.spin()
    except rospy.ROSInterruptException:
        pass
