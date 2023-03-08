#!usr/bin/env_python

from __future__ import print_function
import sys
import os
import time
from DFRobot_RaspberryPi_DC_Motor import DFRobot_DC_Motor_IIC as Board

board = Board(1, 0x10)


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


def motor_duty(id, orientation, duty):
    board.motor_movement(id, orientation, duty)
    speed = board.get_encoder_speed(id)
    print("%d: duty: %d, speed: %rpm" % (id, duty, speed))


if __name__ == "__main__":
    board_detect()

    board.set_encoder_enable(board.ALL)
    board.set_encoder_reduction_ratio(board.ALL, 60)
    board.set_moter_pwm_frequency(1000)

    while board.begin() != board.STA_OK:  # Board begin and check board status
        print_board_status()
        print("board begin faild")
        time.sleep(2)
    print("board begin success")

    '''for duty in range(95, 1, -1):
        board.motor_movement([board.M1], board.CW, duty)
        board.motor_movement([board.M2], board.CW, duty)
        print(duty)
        time.sleep(0.1)'''

    board.motor_movement([board.M1], board.CW, 95)
    board.motor_movement([board.M2], board.CW, 95)
    time.sleep(1)
    board.motor_movement([board.M1], board.CW, 0)
    board.motor_movement([board.M2], board.CW, 0)
