#!/usr/bin/env python

import rospy


def clamp(n, minn, maxn):  # little solution to min max a given value
    return max(min(maxn, n), minn)


def abs_clamp(n, minn, maxn):  # modified clamp to maintain respect to sign of input
    if n > 0:
        return max(min(maxn, abs(n)), minn)
    if n < 0:
        return -(max(min(maxn, abs(n)), minn))
    if n == 0:
        return minn


def load_param(ros_param_name, local_default):
    if rospy.has_param(ros_param_name):
        return rospy.get_param(ros_param_name)
    else:
        return local_default


def load_tf_prefix(tf_id):
    if rospy.has_param("robot_name"):
        return rospy.get_param("robot_name") + '_tf/' + tf_id
    else:
        return tf_id


def circular_data_wrap_logic(input_value, data_range):
    if input_value > data_range:
        return input_value - data_range
    elif input_value < 0:
        return input_value + data_range
    else:
        return input_value
