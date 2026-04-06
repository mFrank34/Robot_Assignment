"""
File: server.py
About: server for robot to listen to topic and send request.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math

from robot.modules.state import State
from robot.modules.reactive import Reactive
from robot.modules.controller import Controller
from robot.data.dimensions import RobotDimensions


class RobotServer(Node):
    def __init__(self):
        super().__init__('robot_node')

        self.running = False
        self.controller = Controller(self)

        dims = RobotDimensions()
        self.reactive = Reactive(self.get_clock(), self.get_logger(), dims)

        self.current_odom = None
        self.front_scan = None

        self.last_turn_direction = 0.5

        # subscriptions
        self.create_subscription(Odometry, '/model/square_bot/odometry', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/front_lidar/scan', self.front_lidar_callback, 10)
        self.create_subscription(String, '/robot/command', self.command_callback, 10)

        self.timer = self.create_timer(0.1, self.update)

        self.get_logger().info('Robot server started')

    def odom_callback(self, msg):
        self.current_odom = msg

    def front_lidar_callback(self, msg):
        self.front_scan = msg

    # SAFE RANGE PROCESSING
    def split_scan(self, scan):
        ranges = [r for r in scan.ranges if r > 0.0 and not math.isnan(r)]
        if not ranges:
            ranges = [float('inf')]

        third = len(ranges) // 3
        left = ranges[:third]
        centre = ranges[third:third * 2]
        right = ranges[third * 2:]

        def safe_median(section):
            sorted_vals = sorted(section)
            mid = len(sorted_vals) // 2
            return sorted_vals[mid] if sorted_vals else float('inf')

        left_med = safe_median(left)
        centre_med = safe_median(centre)
        right_med = safe_median(right)

        all_vals = left + centre + right
        all_med = safe_median(all_vals)

        return left_med, centre_med, right_med, all_med

    # ACTION
    def action(self, state, front_centre):
        match state:
            case State.FORWARD:
                speed = max(0.2, min(1.0, front_centre * 0.5))
                self.controller.send_velocity(speed, 0.0)

            case State.REVERSE:
                self.controller.send_velocity(-0.15, 0.0)

            case State.TURN:
                self.controller.send_velocity(0.0, self.last_turn_direction)

            case State.STOP:
                self.controller.send_velocity(0.0, 0.0)

    def update(self):
        if not self.running or self.front_scan is None:
            return

        front = self.split_scan(self.front_scan)

        if front is None:
            self.get_logger().warn("Invalid scan data")
            return

        _, front_centre, _, _ = front

        # update decision system
        self.reactive.update(self.front_scan, self.controller, self.current_odom)

        # sync direction from reactive
        self.last_turn_direction = self.reactive.turn_direction

        # execute movement
        self.action(self.reactive.state, front_centre)

    def command_callback(self, msg):
        if msg.data == 'start':
            self.running = True
            self.get_logger().info('Autonomous mode started')

        elif msg.data == 'stop':
            self.running = False
            self.stop_robot()
            self.get_logger().info('Autonomous mode stopped')

    def stop_robot(self):
        self.controller.send_velocity(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = RobotServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()