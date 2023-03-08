#!/usr/bin/env python
from test_import import *


def shortest_route(n1, n2, n_range):
    if n1 == n2:
        return [0, True]
    min_max = min_max_ordering(n1, n2)
    max_n = min_max[0]
    min_n = min_max[1]
    r_1 = max_n - min_n + 1
    r_2 = abs(max_n - min_n - n_range) + 1
    print("r_1: %d" % r_1)
    print("r_2: %d" % r_2)
    if r_1 < r_2:
        return [r_1, True]
    else:
        return [r_2, False]


def min_max_ordering(n1, n2):
    if n1 > n2:
        return [n1, n2]
    elif n1 < n2:
        return [n2, n1]
    return [n1, n1]


def circular_data_wrap_logic(input_value, data_range):
    if input_value > data_range:
        return input_value - data_range
    elif input_value < 0:
        return input_value + data_range
    else:
        return input_value


if __name__ == '__main__':
    angle_range = 360-1
    input_data = 3
    d_input = 1
    in_min = input_data - d_input
    in_max = input_data + d_input
    in_min = circular_data_wrap_logic(in_min, angle_range+1)
    in_max = circular_data_wrap_logic(in_max, angle_range+1)
    if in_min > in_max:
        in_min, in_max = in_max, in_min
        wrapped = True
    else:
        wrapped = False
    if wrapped:
        for i in range(in_max, angle_range+1, 1):
            print(i)
        for i in range(0, in_min+1, 1):
            print(i)
    else:
        for i in range(in_min, in_max+1, 1):
            print(i)
