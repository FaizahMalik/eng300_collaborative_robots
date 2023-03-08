# -*- coding:utf-8 -*-
"""!
  @file DC_Motor_Demo.py
  @brief Connect board with raspberryPi.
  @n Make board power and motor connection correct.
  @n Run this demo.
  @n Motor 1 will move slow to fast, orientation clockwise,
  @n motor 2 will move fast to slow, orientation count-clockwise,
  @n then fast to stop. loop in few seconds.
  @n Motor speed will print on terminal
  @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license    The MIT License (MIT)
  @author     [tangjie](jie.tang@dfrobot.com)
  @version    V1.0.1
  @date       2022-04-19
  @url  https://github.com/DFRobot/DFRobot_RaspberryPi_Motor
"""
from __future__ import print_function
import sys
import os

sys.path.append("../")

import time

from DFRobot_RaspberryPi_DC_Motor import DFRobot_DC_Motor_IIC as Board

board = Board(1, 0x10)  # Select bus 1, set address to 0x10


def board_detect():
    l = board.detecte()
    print("Board list conform:")
    print(l)


''' print last operate status, users can use this variable to determine the result of a function call. '''


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


if __name__ == "__main__":

    board_detect()  # If you forget address you had set, use this to detected them, must have class instance

    # Set board controler address, use it carefully, reboot module to make it effective
    '''
    board.set_addr(0x10)
    if board.last_operate_status != board.STA_OK:
      print("set board address faild")
    else:
      print("set board address success")
    '''

    while board.begin() != board.STA_OK:  # Board begin and check board status
        print_board_status()
        print("board begin faild")
        time.sleep(2)
    print("board begin success")

    board.set_encoder_enable(board.ALL)  # Set selected DC motor encoder enable
    # board.set_encoder_disable(board.ALL)              # Set selected DC motor encoder disable
    board.set_encoder_reduction_ratio(board.ALL, 43)  # Set selected DC motor encoder reduction ratio,
    #                                                   test motor reduction ratio is 43.8

    board.set_moter_pwm_frequency(600)  # Set DC motor pwm frequency to 1000HZ

    while True:
        board.motor_movement([board.M1], 0, 95)
        board.motor_movement([board.M2], 0, 95)
        time.sleep(1)
        for duty in range(80, 140, 2):  # slow to fast
            board.motor_movement([board.M1], 2, 95)
            board.motor_movement([board.M2], 2, 95)
            board.set_encoder_reduction_ratio(board.ALL, duty)
            time.sleep(0.01)
            speed = board.get_encoder_speed(board.ALL)  # Use board.all to get all encoders speed
            print("2")
        for duty in range(80, 140, 2):  # slow to fast
            board.motor_movement([board.M1], 0, 95)
            board.motor_movement([board.M2], 0, 95)
            board.set_encoder_reduction_ratio(board.ALL, duty)
            time.sleep(0.01)
            print("0")
            speed = board.get_encoder_speed(board.ALL)  # Use board.all to get all encoders speed


        for duty in range(160, 10, -2):  # slow to fast
            board.motor_movement([board.M1], 1, 95)
            board.motor_movement([board.M2], 1, 95)
            board.set_encoder_reduction_ratio(board.ALL, duty)
            time.sleep(0.01)
            speed = board.get_encoder_speed(board.ALL)  # Use board.all to get all encoders speed
            print("1")


        # time.sleep(4)
