#!/usr/bin/env python


import rospy
import math
from python_tools import *
from geometry_msgs.msg import Twist
from autonomous_robot.msg import LR_wheel_target_speed
from autonomous_robot.msg import LR_wheel_speed
from autonomous_robot.msg import LR_wheel_odom


class Wheel(object):
    def __init__(self, orientation):

        """Wheel orientation assignment"""
        if orientation == 'L':  #
            self.LR = -1
        elif orientation == 'R':
            self.LR = 1
        else:
            print("LR assignment error. Orientation variable:")
            print(orientation)
            exit()

        """robot/wheel parameters"""
        self.base_width = load_param('Robot_Width', 0.25)  # robot drive wheel separation (m)
        self.wheel_radius = load_param('Wheel_Radius', 0.0325)  # drive wheel radius (m)

    def calc_target_speed(self, fv, r_av, t):
        speed_target_l = fv + (r_av * self.base_width * self.LR)  # linear V = sum of:
        #                                                           forward + angular * relative direction
        # this is the cursed conversion to how the arduino measures it's speed. It is the number of rotations made by
        #   the hall effect sensor per publishing rate. first the frequency of the wheel is calculated, and then the
        #   number of samples calculated using the 840:1 sensor to main shaft ratio * 4 for the four states of the
        #   hall effect sensor, and * t which is an estimation of the following publishing rate based on the previous.
        speed_target_sensor_freq = (speed_target_l / (2 * math.pi * self.wheel_radius))*(t * 840 * 4)
        # speed_target_a = speed_target_l / self.wheel_radius  # conversion to angular speed (rad/s)
        return int(speed_target_sensor_freq)


class Main:
    def __init__(self):
        self.robot_forward_velocity = 0
        self.robot_angular_velocity = 0
        self.rate = load_param('motor_controller/rate', 10)  # node processing frequency
        self.r = rospy.Rate(self.rate)
        self.base_width = load_param('Robot_Width', 0.25)  # drive wheel radius
        self.cv_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)  # setup subscriber
        self.ws_sub = rospy.Subscriber('LR_odom', LR_wheel_odom, self.lr_odom_callback)  # setup subscriber
        self.tws_pub = rospy.Publisher('LR_TWS', LR_wheel_target_speed, queue_size=10)
        self.ws_pub = rospy.Publisher('LR_WS', LR_wheel_speed, queue_size=10)
        self.LR_ws = LR_wheel_speed()
        self.LR_tws = LR_wheel_target_speed()
        t = rospy.Time.now()
        self.time_now = float(t.nsecs) * 1e-9 + float(t.secs)
        self.time_then = float(t.nsecs) * 1e-9 + float(t.secs)
        self.time_past = 0.1
        self.avg_time_past = 0.1

    def lr_odom_callback(self, msg):
        t = rospy.Time.now()
        self.time_now = float(t.nsecs) * 1e-9 + float(t.secs)
        self.time_past = self.time_now - self.time_then
        self.time_then = self.time_now
        self.avg_time_past = ((self.avg_time_past * 10) + self.time_past) / 11
        self.LR_ws.left = (msg.left * 2 * math.pi) / (self.avg_time_past * 4 * 840)
        self.LR_ws.right = (msg.right * 2 * math.pi) / (self.avg_time_past * 4 * 840)
        self.ws_pub.publish(self.LR_ws)

    def cmd_vel_callback(self, msg):
        self.robot_forward_velocity = msg.linear.x
        self.robot_angular_velocity = msg.angular.z

    def update_target_speeds(self):
        self.LR_tws.left = LW.calc_target_speed(self.robot_forward_velocity,
                                                self.robot_angular_velocity,
                                                self.time_past)
        self.LR_tws.right = RW.calc_target_speed(self.robot_forward_velocity,
                                                 self.robot_angular_velocity,
                                                 self.time_past)

    def pub_target_speeds(self):
        self.tws_pub.publish(self.LR_tws)


if __name__ == '__main__':
    rospy.init_node("arduino_interface")
    mci = Main()
    LW = Wheel('L')
    RW = Wheel('R')

    try:
        while not rospy.is_shutdown():
            mci.update_target_speeds()
            mci.pub_target_speeds()
            mci.r.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
