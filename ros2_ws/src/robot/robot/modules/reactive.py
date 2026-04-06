"""
File: reactive.py
About: obstacle avoidance with smooth state transitions
Author: Michael Franks
"""

from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from robot.modules.state import State
from robot.data.dimensions import RobotDimensions
import math


class Reactive:
    def __init__(self, clock, logger=None, dims=None):
        self.clock = clock
        self.logger = logger
        self.state_ts = self.clock.now()

        # robot state
        self.state = State.FORWARD

        # timing system
        self.TURNING_TIME = 2.0
        self.REVERSING_TIME = 2.0
        self.SCAN_COOLDOWN = 0.3

        # speeds
        self.SPEED_LINEAR = 0.3
        self.SPEED_ANGULAR = 0.5

        # obstacle threshold
        self.OBSTACLE_DISTANCE = 1.0

        # derive minimum turn clearance from robot geometry
        dims = dims or RobotDimensions()
        half_length = max(abs(dims.front_axle_offset_x), abs(dims.rear_axle_offset_x)) + dims.wheel_radius
        half_width = (dims.wheel_separation / 2) + dims.wheel_radius
        self.TURN_CLEARANCE = math.sqrt(half_length ** 2 + half_width ** 2) + dims.safety_margin

        self.last_scan = None
        self.last_scan_time = None
        self.last_odom = None
        self.reverse_start_pos = None
        self.controller = None
        self.turn_direction = 0.5

    def log(self, msg):
        if self.logger:
            self.logger.info(msg)

    def go_state(self, new_state):
        if self.state != new_state:
            self.log(f"{self.state} -> {new_state}")
            self.state = new_state
            self.state_ts = self.clock.now()
            if new_state == State.REVERSE:
                # snapshot position when reverse begins
                self.reverse_start_pos = (
                    self.last_odom.pose.pose.position if self.last_odom else None
                )

    # TRANSITION CHECKS
    def check_forward_2_reverse(self):
        if self.last_scan is None or len(self.last_scan.ranges) == 0:
            return False
        mid = len(self.last_scan.ranges) // 2
        front_ranges = self.last_scan.ranges[mid - 10:mid + 10]
        front_vals = [r for r in front_ranges if r > 0.0 and not math.isinf(r)]
        if not front_vals:
            return False
        median_front = sorted(front_vals)[len(front_vals) // 2]
        return median_front < self.OBSTACLE_DISTANCE

    def check_forward_2_stop(self):
        if self.last_scan_time is None:
            return True
        elapsed = self.clock.now() - self.last_scan_time
        return elapsed > Duration(seconds=self.SCAN_COOLDOWN)

    def check_reverse_2_turn(self):
        elapsed = self.clock.now() - self.state_ts
        if elapsed < Duration(seconds=self.REVERSING_TIME):
            return False

        # use odom to check actual distance reversed if available
        if self.last_odom is not None and self.reverse_start_pos is not None:
            pos = self.last_odom.pose.pose.position
            dx = pos.x - self.reverse_start_pos.x
            dy = pos.y - self.reverse_start_pos.y
            dist = math.sqrt(dx ** 2 + dy ** 2)
            self.log(f"Reversed {dist:.2f}m, need {self.TURN_CLEARANCE:.2f}m")
            return dist > self.TURN_CLEARANCE

        # fallback: timer only
        return True

    def check_turn_2_forward(self):
        elapsed = self.clock.now() - self.state_ts
        return elapsed > Duration(seconds=self.TURNING_TIME)

    # STATE BEHAVIOURS
    def forward(self, out_vel):
        out_vel.linear.x = self.SPEED_LINEAR
        out_vel.angular.z = 0.0
        self.log("Moving forward...")

        if self.check_forward_2_reverse():
            self.go_state(State.REVERSE)
        elif self.check_forward_2_stop():
            self.go_state(State.STOP)

    def reverse(self, out_vel):
        out_vel.linear.x = -self.SPEED_LINEAR
        out_vel.angular.z = 0.0
        self.log("Reversing...")

        if self.check_reverse_2_turn():
            self.go_state(State.TURN)
            self.select_turn_direction()

    def turn(self, out_vel):
        out_vel.linear.x = 0.0
        out_vel.angular.z = self.turn_direction
        self.log("Turning...")

        if self.check_turn_2_forward():
            self.go_state(State.FORWARD)

    def stop(self, out_vel):
        out_vel.linear.x = 0.0
        out_vel.angular.z = 0.0
        self.log("Stopped, waiting for sensor data...")

        elapsed = self.clock.now() - self.state_ts
        if self.last_scan is not None and len(self.last_scan.ranges) > 0:
            if elapsed > Duration(seconds=self.SCAN_COOLDOWN):
                if self.check_forward_2_reverse():
                    self.go_state(State.REVERSE)
                else:
                    self.go_state(State.FORWARD)

    def select_turn_direction(self):
        if self.last_scan is None or len(self.last_scan.ranges) == 0:
            self.turn_direction = 0.5
            return

        third = len(self.last_scan.ranges) // 3
        left = self.last_scan.ranges[:third]
        right = self.last_scan.ranges[2 * third:]

        left_min = min([r for r in left if r > 0.0 and not math.isinf(r)] or [float('inf')])
        right_min = min([r for r in right if r > 0.0 and not math.isinf(r)] or [float('inf')])

        self.turn_direction = 0.5 if right_min > left_min else -0.5
        self.log(
            f"Turn direction: {'right' if self.turn_direction > 0 else 'left'} (L:{left_min:.2f} R:{right_min:.2f})")

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

    def update(self, scan, controller, odom=None):
        self.last_scan = scan
        self.last_scan_time = self.clock.now()
        self.controller = controller
        self.last_odom = odom
        return self.control_cycle()
