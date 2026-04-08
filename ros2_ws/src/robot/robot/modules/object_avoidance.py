"""
File: object_avoidance.py
About: object avoidance system
Author: Michael Franks
"""


import math
import random
from rclpy.duration import Duration
from geometry_msgs.msg import Twist

from robot.data.state import State
from robot.data.dimensions import RobotDimensions

# ✅ NEW IMPORT
from robot.utils.math_utils import clean_ranges, median, get_front_slice, angle_diff


class ObjectAvoidance:
    def __init__(self, clock, logger=None, dims=None):
        self.clock = clock
        self.logger = logger
        self.state_ts = self.clock.now()

        self.state = State.FORWARD

        self.TURNING_TIME = 4.5
        self.REVERSING_TIME = 2.5
        self.SCAN_COOLDOWN = 0.3

        self.SPEED_FORWARD = 0.3
        self.SPEED_REVERSE = 0.25
        self.SPEED_ANGULAR = 0.7

        self.OBSTACLE_DISTANCE = 1.0

        dims = dims or RobotDimensions()
        half_length = max(abs(dims.front_axle_offset_x), abs(dims.rear_axle_offset_x)) + dims.wheel_radius
        half_width = (dims.wheel_separation / 2) + dims.wheel_radius
        self.TURN_CLEARANCE = math.sqrt(half_length ** 2 + half_width ** 2) + dims.safety_margin

        self.last_scan = None
        self.last_scan_time = None
        self.last_odom = None
        self.reverse_start_pos = None
        self.turn_direction = self.SPEED_ANGULAR

        self.turn_start_yaw = None
        self.turn_target_angle = math.radians(45)

    def log(self, msg):
        if self.logger:
            self.logger.info(msg)

    def go_state(self, new_state):
        if self.state != new_state:
            self.log(f"{self.state} -> {new_state}")
            self.state = new_state
            self.state_ts = self.clock.now()

            if new_state == State.REVERSE:
                self.reverse_start_pos = (
                    self.last_odom.pose.pose.position if self.last_odom else None
                )

            if new_state == State.TURN and self.last_odom:
                self.turn_start_yaw = self.get_yaw_from_odom(self.last_odom)

    def get_yaw_from_odom(self, odom):
        q = odom.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def path_clear(self):
        ranges = self.last_scan.ranges

        front_slice = get_front_slice(ranges, 60)
        vals = clean_ranges(front_slice, self.last_scan.range_min, self.last_scan.range_max)

        if not vals:
            return False

        return min(vals) > (self.OBSTACLE_DISTANCE + 0.3)

    def check_forward_2_reverse(self):
        if self.last_scan is None:
            return False

        ranges = self.last_scan.ranges

        front_slice = get_front_slice(ranges, 60)
        vals = clean_ranges(front_slice, self.last_scan.range_min, self.last_scan.range_max)

        if not vals:
            return False

        return min(vals) < self.OBSTACLE_DISTANCE

    def check_scan_stale(self):
        if self.last_scan_time is None:
            return True
        elapsed = self.clock.now() - self.last_scan_time
        return elapsed > Duration(seconds=self.SCAN_COOLDOWN)

    def check_reverse_complete(self):
        elapsed = self.clock.now() - self.state_ts

        if elapsed > Duration(seconds=self.REVERSING_TIME):
            return True

        if self.last_odom and self.reverse_start_pos:
            pos = self.last_odom.pose.pose.position
            dist = math.sqrt((pos.x - self.reverse_start_pos.x) ** 2 + (pos.y - self.reverse_start_pos.y) ** 2)
            return dist > self.TURN_CLEARANCE

        return False

    def check_turn_complete(self):
        if self.last_odom is None or self.turn_start_yaw is None:
            return False

        current_yaw = self.get_yaw_from_odom(self.last_odom)

        diff = angle_diff(current_yaw, self.turn_start_yaw)

        return diff >= self.turn_target_angle

    def forward(self, out_vel):
        out_vel.linear.x = self.SPEED_FORWARD
        out_vel.angular.z = 0.0

        if self.check_forward_2_reverse():
            self.go_state(State.REVERSE)
        elif self.check_scan_stale():
            self.go_state(State.STOP)

    def reverse(self, out_vel):
        out_vel.linear.x = -self.SPEED_REVERSE
        out_vel.angular.z = 0.0

        if self.check_reverse_complete():
            self.select_turn_direction()
            self.go_state(State.TURN)

    def turn(self, out_vel):
        out_vel.linear.x = 0.0
        out_vel.angular.z = self.turn_direction

        if self.check_turn_complete():
            self.go_state(State.FORWARD)

    def stop(self, out_vel):
        out_vel.linear.x = 0.0
        out_vel.angular.z = 0.0
        self.log("Waiting for sensor data...")

        elapsed = self.clock.now() - self.state_ts
        if self.last_scan is not None and elapsed > Duration(seconds=self.SCAN_COOLDOWN):
            if self.check_forward_2_reverse():
                self.go_state(State.REVERSE)
            else:
                self.go_state(State.FORWARD)

    def select_turn_direction(self):
        ranges = self.last_scan.ranges
        mid = len(ranges) // 2

        right_side = ranges[0:mid]
        left_side = ranges[mid:]

        def get_med(vals):
            clean = clean_ranges(vals, self.last_scan.range_min, self.last_scan.range_max)
            return median(clean)

        if get_med(left_side) > get_med(right_side):
            self.turn_direction = self.SPEED_ANGULAR * random.uniform(0.7, 1.0)
        else:
            self.turn_direction = -self.SPEED_ANGULAR * random.uniform(0.7, 1.0)

    def control_cycle(self):
        out_vel = Twist()
        if self.last_scan is None:
            return out_vel

        match self.state:
            case State.FORWARD:
                self.forward(out_vel)
            case State.REVERSE:
                self.reverse(out_vel)
            case State.TURN:
                self.turn(out_vel)
            case State.STOP:
                self.stop(out_vel)

        return out_vel

    def update(self, scan, odom=None):
        self.last_scan = scan
        self.last_scan_time = self.clock.now()
        self.last_odom = odom
        return self.control_cycle()
