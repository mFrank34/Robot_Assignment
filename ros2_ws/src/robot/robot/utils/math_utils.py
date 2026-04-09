"""
File: math_utils.py
About: math utilitys to reduce math noise within code.
Author: Michael Franks
"""

import math


def clean_ranges(ranges, range_min, range_max):
    return [r for r in ranges if range_min <= r <= range_max]


def median(values):
    if not values:
        return 0.0
    s = sorted(values)
    return s[len(s) // 2]


def average(values):
    if not values:
        return 0.0
    return sum(values) / len(values)


def get_slice(ranges, start, end):
    return ranges[start:end]


def get_front_slice(ranges, width):
    mid = len(ranges) // 2
    return ranges[mid - width: mid + width]


def angle_diff(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def clamp(val, min_val, max_val):
    return max(min_val, min(max_val, val))


def get_yaw_from_odom(odom):
    """Takes a robot’s orientation (in quaternion form) and returns which direction it’s facing"""
    q = odom.pose.pose.orientation
    sin = 2 * (q.w * q.z + q.x * q.y)
    cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(sin, cosy)

