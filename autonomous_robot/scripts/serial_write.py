#!/usr/bin/env python

import serial
import rospy
import tf2_ros

PORT = "/dev/ttyUSB0"
BAUD = 9600

ser = serial.Serial(PORT, BAUD)
# make sure port is open
if ser.isOpen():
    ser.close()
ser.open()
ser.isOpen()


def send(x, y, xo, yo, zo, w):
    # package data into a sring and add stop character 'x'
    data = (str(x) + ',' + str(y) + ',' + str(xo) + ',' + str(yo) + ',' + str(zo) + ',' + str(w) + 'x')
    print()
    # send data over serial ecoded as ASCII
    ser.write(data.encode())
    # wait untill all data in out buffer has been written
    ser.flush()
    state = 'read'
    # wait for one second before reading
    rospy.sleep(1)


rospy.init_node('serial_write', anonymous=True)

tfBuffer = tf2_ros.Buffer()

listener = tf2_ros.TransformListener(tfBuffer)

rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
    try:
        trans = tfBuffer.lookup_transform('odom', 'base_footprint', rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rate.sleep()
        continue

    # split data
    send(trans.transform.translation.x, trans.transform.translation.y, trans.transform.orientation.x,
         trans.transform.orientation.y, trans.transform.orientation.z, trans.transform.orientation.w)
