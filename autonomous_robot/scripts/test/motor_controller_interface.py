#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import math
from python_tools import *
from autonomous_robot.msg import LR_wheel_odometry
from DFRobot_RaspberryPi_DC_Motor import DFRobot_DC_Motor_IIC as Board
import time


class Wheel(object):
    def __init__(self, orientation):
        """sim/test switches"""  # TODO - potentially remove as no longer functional
        self.cmd_vel_to_odom = load_param('motor_controller/cmd_vel_to_odom', False)

        """Wheel orientation assignment"""
        if orientation == 'L':  # TODO - test self.Mn with i2c function
            self.LR = -1
            self.Mn = 'M1'
        elif orientation == 'R':
            self.LR = 1
            self.Mn = 'M2'
        else:
            print("LR assignment error. Orientation variable:")
            print(orientation)
            print("Exiting node")
            exit()

        """PID"""
        self.kp = 10
        self.ki = 0
        self.kd = 0
        self.ek_1 = 0  # e(k-1)
        self.ik = 0  # i(k)

        """robot/wheel parameters"""
        self.base_width = load_param('Robot_Width', 0.25)  # robot drive wheel separation (m)
        self.speed_now = 0
        self.speed_now_rads = 0
        self.speed_then = 0
        self.speed_target = 0
        self.speed_target_a = 0
        self.wheel_radius = load_param('Wheel_Radius', 0.05)  # drive wheel radius (m)
        self.wheel_displacement = 0

    def pid_control(self, rk, yk, ts):  # TODO check
        ek = rk - yk
        self.ik += ts * ek
        dk = (ek - self.ek_1) / ts
        uk = self.kp * ek + self.ki * self.ik + self.kd * dk
        print("rk: %d, yk: %d, ek: %d, ik: %d, dk: %d, uk: %d, " % (rk, yk, ek, self.ik, dk, uk))
        print("rk: %d, yk: %d, kp: %d, ki: %d, kd: %d, uk: %d, " % (rk, yk, self.kp * ek, self.ki * self.ik, self.kd * dk, uk))
        self.ek_1 = ek
        return uk

    def read_wheel_speed(self):
        if self.cmd_vel_to_odom:  # use cmd_vel for testing - potentially redundant
            # self.speed_now = self.l_vel
            print("reading from cmd_vel for actual wheel speed")
        else:
            self.speed_now_rads = board.get_encoder_speed(board.ALL)  # current wheel speed (rad/s)
            self.speed_now_rads = self.speed_now_rads[1]
            self.speed_now = self.speed_now_rads * self.wheel_radius * math.pi  # conversion (m/s)
            print("speed_now: %f.m/s" % self.speed_now)
            self.speed_then = self.speed_now  # setup speed_then for next iteration

    def calculate_new_wheel_speed(self, fv, r_av, ts):  # update motor
        self.speed_target = fv + (r_av * self.base_width * self.LR)  # linear V = sum of:
        #                                                            # forward + angular command * direction
        self.speed_target_a = self.speed_target / (self.wheel_radius * math.pi)  # conversion to angular speed (rad/s)
        print("target speed: %f.m/s" % self.speed_target)
        duty = clamp(self.pid_control(
            self.speed_target_a, self.speed_now_rads, ts), -95, 95)  # returns duty cycle (-95% < dc < 95%)
        print("Duty: %f" % duty)
        print(" ")
        if duty < 0:
            board.motor_movement([board.M2], board.CW, abs(duty))
        else:
            board.motor_movement([board.M2], board.CCW, abs(duty))

    def update_wheel_displacement(self, ts):
        if self.cmd_vel_to_odom:  # used for testing without actual motor
            self.speed_now = self.speed_target  # can potentially be removed as code no longer works without board
        self.wheel_displacement += ts * (self.speed_now + self.speed_then) / 2
        print('%s displacement: %f' % (self.Mn, self.wheel_displacement))
        return int(self.wheel_displacement * 1e3)  # return wheel displacement as int in mm


class Main:
    def __init__(self):
        self.robot_forward_velocity = 0
        self.robot_angular_velocity = 0
        self.rate = load_param('motor_controller/rate', 15)  # node processing frequency
        self.base_width = load_param('Robot_Width', 0.25)  # drive wheel radius
        self.cv_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)  # setup subscriber
        self.odom_pub = rospy.Publisher('LR_wheel_odometry', LR_wheel_odometry, queue_size=10)
        self.LW_odom = LR_wheel_odometry()

        self.time_now = float(rospy.Time.now().nsecs) * 1e-9 + float(rospy.Time.now().secs)
        self.time_then = float(rospy.Time.now().nsecs) * 1e-9 + float(rospy.Time.now().secs)
        self.time_past = 1

    def cmd_vel_callback(self, msg):
        self.robot_forward_velocity = msg.linear.x
        self.robot_angular_velocity = msg.angular.z

    def timer(self):
        t = rospy.Time.now()
        self.time_now = float(t.nsecs) * 1e-9 + float(t.secs)
        self.time_past = self.time_now - self.time_then
        self.time_then = self.time_now

    def pub_wheel_odom(self, lw, rw):
        self.LW_odom.left = lw
        self.LW_odom.right = rw
        self.odom_pub.publish(self.LW_odom)

    def spin(self):
        self.rate = rospy.Rate(mci.rate)
        while not rospy.is_shutdown():
            # measure time elapsed
            self.timer()
            # read wheel velocities
            LW.read_wheel_speed()
            RW.read_wheel_speed()
            # calculate wheel displacements (mm) & publish to odom node
            self.pub_wheel_odom(
                LW.update_wheel_displacement(self.time_past),
                RW.update_wheel_displacement(self.time_past))
            # calculate new target wheel vel & publish (wheel function also handles publishing)
            LW.calculate_new_wheel_speed(self.robot_forward_velocity,
                                         self.robot_angular_velocity,
                                         self.time_past)
            RW.calculate_new_wheel_speed(self.robot_forward_velocity,
                                         self.robot_angular_velocity,
                                         self.time_past)
            print(" ")
            self.rate.sleep()
            '''
            0. timer
            1. read wheel velocities
            2. calculate displacements
                2.1 send to odom node
            3. calculate new wheel vel
                3.1. PID
                3.2. i2c command
            4. sleep()
            '''


def board_detect():
    board_detected = board.detecte()
    print("Board list conform:")
    print(board_detected)


def print_board_status():
    if board.last_operate_status == board.STA_OK:
        print("board status: everything ok")
    elif board.last_operate_status == board.STA_ERR:
        print("board status: unexpected error")
    elif board.last_operate_status == board.STA_ERR_DEVICE_NOT_DETECTED:
        print("board status: device not detected")
    elif board.last_operate_status == board.STA_ERR_PARAMETER:
        print("board status: parameter error, last operate no effective")
    elif board.last_operate_status == board.STA_ERR_SOFT_VERSION:
        print("board status: unsupport board framware version")


if __name__ == '__main__':
    rospy.init_node("motor_controller_interface")

    board = Board(1, 0x10)
    board_detect()

    while board.begin() != board.STA_OK:  # Board begin and check board status
        print_board_status()
        print("board begin faild")
        time.sleep(2)
    print("board begin success")

    board.set_encoder_enable(board.ALL)
    board.set_encoder_reduction_ratio(board.ALL, 60)
    board.set_moter_pwm_frequency(600)

    try:
        mci = Main()
        LW = Wheel('L')
        RW = Wheel('R')
        mci.spin()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
