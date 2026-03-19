"""
File: reactive.py
About: obstacle avoidance
Author: Michael Franks
"""

from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import Twist
from robot.modules.state import State


class Reactive:
    def __init__(self, clock, logger=None):
        self.clock = clock
        self.logger = logger
        self.state_ts = self.clock.now()

        # robot state
        self.state = State.FORWARD
        self.last_ts = self.clock.now()

        # timing system
        self.TURNING_TIME = 2.0
        self.REVERSING_TIME = 2.0
        self.SCAN_TIMEOUT = 1.0

        # speeds
        self.SPEED_LINEAR = 0.3
        self.SPEED_ANGULAR = 0.3

        # obstacle threshold
        self.OBSTACLE_DISTANCE = 1.0

        self.last_scan = None
        self.controller = None

    def log(self, msg):
        if self.logger:
            self.logger.info(msg)

    def go_state(self, new_state):
        self.state = new_state
        self.state_ts = self.clock.now()

    def check_forward_2_reverse(self):
        pos = round(len(self.last_scan.ranges) / 2)
        return self.last_scan.ranges[pos] < self.OBSTACLE_DISTANCE

    def check_forward_2_stop(self):
        elapsed = self.clock.now() - Time.from_msg(self.last_scan.header.stamp)
        return elapsed > Duration(seconds=self.SCAN_TIMEOUT)

    def check_stop_2_forward(self):
        elapsed = self.clock.now() - Time.from_msg(self.last_scan.header.stamp)
        return elapsed < Duration(seconds=self.SCAN_TIMEOUT)

    def check_reverse_2_turn(self):
        elapsed = self.clock.now() - self.state_ts
        return elapsed > Duration(seconds=self.REVERSING_TIME)

    def check_turn_2_forward(self):
        elapsed = self.clock.now() - self.state_ts
        return elapsed > Duration(seconds=self.TURNING_TIME)

    def forward(self, out_vel):
        out_vel.linear.x = self.SPEED_LINEAR
        self.log('Moving forward ...')
        if self.check_forward_2_stop():
            self.go_state(State.STOP)
        elif self.check_forward_2_reverse():
            self.go_state(State.REVERSE)

    def reverse(self, out_vel):
        out_vel.linear.x = -self.SPEED_LINEAR
        self.log('Reversing ...')
        if self.check_reverse_2_turn():
            self.go_state(State.TURN)

    def turn(self, out_vel):
        out_vel.angular.z = self.SPEED_ANGULAR
        self.log('Turning ...')
        if self.check_turn_2_forward():
            self.go_state(State.FORWARD)

    def stop(self, out_vel):
        out_vel.linear.x = 0.0
        out_vel.angular.z = 0.0
        self.log('Stopped, waiting for sensor data ...')
        if self.check_stop_2_forward():
            self.go_state(State.FORWARD)

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

    def update(self, front_scan, controller):
        self.last_scan = front_scan
        self.controller = controller
        return self.control_cycle()
