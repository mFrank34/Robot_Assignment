"""
File: object_avoidance.py
About: Obstacle avoidance with smooth state transitions.
       Takes a LaserScan and odometry, returns a Twist.
Author: Michael Franks
"""

import math
from rclpy.duration import Duration
from geometry_msgs.msg import Twist

from robot.data.state import State
from robot.data.dimensions import RobotDimensions


class Reactive:
    def __init__(self, clock, logger=None, dims=None):
        self.clock = clock
        self.logger = logger
        self.state_ts = self.clock.now()

        # robot state
        self.state = State.FORWARD

        # timing — increased to give more clearance in corners
        self.TURNING_TIME = 3.0
        self.REVERSING_TIME = 2.5
        self.SCAN_COOLDOWN = 0.3

        # speeds
        self.SPEED_FORWARD = 0.3
        self.SPEED_REVERSE = 0.2
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
        self.turn_direction = self.SPEED_ANGULAR

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

    def check_forward_2_reverse(self):
        if self.last_scan is None or len(self.last_scan.ranges) == 0:
            return False
        # forward is at the start of the array — take first 40 rays
        front_ranges = self.last_scan.ranges[:40]
        front_vals = [r for r in front_ranges if r > 0.0 and not math.isinf(r)]
        if not front_vals:
            return False
        median_front = sorted(front_vals)[len(front_vals) // 2]
        return median_front < self.OBSTACLE_DISTANCE

    def check_scan_stale(self):
        """True if no fresh scan has arrived within SCAN_COOLDOWN seconds."""
        if self.last_scan_time is None:
            return True
        elapsed = self.clock.now() - self.last_scan_time
        return elapsed > Duration(seconds=self.SCAN_COOLDOWN)

    def check_reverse_complete(self):
        """True once the robot has reversed far enough to safely turn."""
        elapsed = self.clock.now() - self.state_ts
        if elapsed < Duration(seconds=self.REVERSING_TIME):
            return False

        if self.last_odom is not None and self.reverse_start_pos is not None:
            pos = self.last_odom.pose.pose.position
            dx = pos.x - self.reverse_start_pos.x
            dy = pos.y - self.reverse_start_pos.y
            dist = math.sqrt(dx ** 2 + dy ** 2)
            self.log(f"Reversed {dist:.2f}m, need {self.TURN_CLEARANCE:.2f}m")
            return dist > self.TURN_CLEARANCE

        return True  # fallback: timer only

    def check_turn_complete(self):
        """
        True once turn timer has elapsed AND path ahead is clear.
        Prevents resuming forward into a wall after an insufficient turn.
        """
        elapsed = self.clock.now() - self.state_ts
        if elapsed < Duration(seconds=self.TURNING_TIME):
            return False
        return not self.check_forward_2_reverse()

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
        if self.last_scan is None or len(self.last_scan.ranges) == 0:
            self.turn_direction = self.SPEED_ANGULAR
            return

        ranges = self.last_scan.ranges
        third = len(ranges) // 3

        # with forward at index 0:
        # right side = end of array, left side = middle
        left = ranges[third:third * 2]
        right = ranges[third * 2:]

        def median(vals):
            clean = sorted([r for r in vals if r > 0.0 and not math.isinf(r)])
            return clean[len(clean) // 2] if clean else 0.0

        left_med = median(left)
        right_med = median(right)

        self.turn_direction = self.SPEED_ANGULAR if right_med > left_med else -self.SPEED_ANGULAR
        self.log(f"Turning {'right' if self.turn_direction > 0 else 'left'} (L:{left_med:.2f} R:{right_med:.2f})")

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
        """Takes a LaserScan and optional odometry, returns a Twist."""
        self.last_scan = scan
        self.last_scan_time = self.clock.now()
        self.last_odom = odom
        return self.control_cycle()