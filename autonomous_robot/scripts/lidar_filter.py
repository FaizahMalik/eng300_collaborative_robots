#!/usr/bin/env python
import rospy

import tf2_ros
import numpy
import math
from sensor_msgs.msg import LaserScan
from python_tools import *


class Main:
    def __init__(self):
        self.rate = rospy.Rate(load_param('lidar_filter/rate', 5))
        self.robot_name = load_param('robot_name', 'error')
        self.robot_scan_frame = self.robot_name + 'base_scan'
        # print('robot_scan_frame', self.robot_scan_frame)
        self.other_robot_name = load_param('other_robot_name', 'error')
        self.other_scan_frame = self.other_robot_name + 'base_scan'
        # print('other_scan_frame', self.robot_scan_frame)
        self.robot_width = load_param('robot/wheel_base', 0.3)

        self.lidar_data = LaserScan()
        self.lidar_filtered = LaserScan()
        rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.lf_pub = rospy.Publisher('laser_scan_filtered', LaserScan, queue_size=10)
        self.n_samples = 360
        self.sample_min = 0
        self.sample_max = 0
        self.wrapped = False

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.th = 0
        self.d_th = 0

    def scan_callback(self, data):
        self.lidar_data = data
        if self.wrapped:
            for i in range(self.sample_max, self.n_samples + 1, 1):
                self.lidar_data[i] = 0
            for i in range(0, self.sample_min + 1, 1):
                self.lidar_data[i] = 0
        else:
            for i in range(self.sample_min, self.sample_max + 1, 1):
                self.lidar_data.ranges[i] = 0
        self.lidar_data.header.frame_id = self.robot_scan_frame + '_filtered'
        self.lf_pub.publish(self.lidar_data)

    def update_cartesian(self):
        dx = self.r_tf.transform.translation.x
        dy = self.r_tf.transform.translation.y
        distance = (dx ** 2 + dy ** 2) ** 0.5
        if distance == 0:
            distance = 0.0001
        self.th = numpy.arctan(dy / dx) + math.pi
        self.d_th = numpy.arctan(self.robot_width / distance) + math.pi
        angle_range = self.lidar_data.angle_max - self.lidar_data.angle_min
        self.n_samples = len(self.lidar_data.ranges)
        sample_0 = int(self.n_samples * (self.th / angle_range))
        sample_d = int(self.n_samples * (self.d_th / angle_range))
        sample_min = sample_0 - sample_d
        sample_max = sample_0 + sample_d
        self.sample_min = circular_data_wrap_logic(sample_min, self.n_samples)
        self.sample_max = circular_data_wrap_logic(sample_max, self.n_samples)
        if self.sample_min > self.sample_max:
            self.sample_min, self.sample_max = self.sample_max, self.sample_min
            self.wrapped = True
        else:
            self.wrapped = False

    def update_tf(self):
        try:
            self.r_tf = self.tfBuffer.lookup_transform('odom', 'base_link', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("eajlfdjlbafjsgb")
            pass

    def spin(self):
        while not rospy.is_shutdown():
            self.r_tf = self.tfBuffer.lookup_transform('odom', 'base_link', rospy.Time())
            # self.update_tf()
            self.update_cartesian()
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('lidar_filter')
    try:
        LF = Main()
        LF.spin()
    except rospy.ROSInterruptException:
        pass
