"""
File: server.py
About: server for robot to listen to topic and send request.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float64

""" Personal Class and Enums """
from robot.modules.state import State
from robot.modules.reactive import Reactive
from robot.modules.controller import Controller


class RobotServer(Node):
    def __init__(self):
        super().__init__('robot_node')
        self.last_turn_direction = None
        self.running = False
        self.controller = Controller(self)
        self.reactive = Reactive(self.get_clock())

        # Store latest data
        self.current_odom = None
        self.front_scan = None
        self.back_scan = None

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/square_bot/odometry',
            self.odom_callback,
            10
        )
        self.front_lidar_sub = self.create_subscription(
            LaserScan,
            '/front_lidar/scan',
            self.front_lidar_callback,
            10
        )
        self.back_lidar_sub = self.create_subscription(
            LaserScan,
            '/back_lidar/scan',
            self.back_lidar_callback,
            10
        )
        self.command_sub = self.create_subscription(
            String,
            '/robot/command',
            self.command_callback,
            10
        )

        # Timer — runs update every 100ms
        self.timer = self.create_timer(0.1, self.update)

        self.get_logger().info('Robot server started')

    def odom_callback(self, msg):
        self.current_odom = msg

    def front_lidar_callback(self, msg):
        self.front_scan = msg

    def back_lidar_callback(self, msg):
        self.back_scan = msg

    def front_range(self):
        """Return min left, centre, right, and overall front distance"""
        front_ranges = [r for r in self.front_scan.ranges if r > 0.0 and r != float('inf')]

        if not front_ranges:
            return None, None, None, None

        total = len(front_ranges)
        third = total // 3

        min_left = min(front_ranges[:third])
        min_centre = min(front_ranges[third:third * 2])
        min_right = min(front_ranges[third * 2:])
        min_front = min(front_ranges)

        return min_left, min_centre, min_right, min_front

    def back_range(self):
        """Return min left, centre, right, and overall back distance"""
        back_ranges = [r for r in self.back_scan.ranges if r > 0.0 and r != float('inf')]

        if not back_ranges:
            return None, None, None, None

        total = len(back_ranges)
        third = total // 3

        min_left = min(back_ranges[:third])
        min_centre = min(back_ranges[third:third * 2])
        min_right = min(back_ranges[third * 2:])
        min_back = min(back_ranges)

        return min_left, min_centre, min_right, min_back

    def action(self, state, turn_direction=0.5, min_centre=1.0, back_centre=1.0):
        match state:
            case State.CLEAR:
                speed = max(0.2, min(1.0, min_centre * 0.5))
                self.controller.send_velocity(speed, 0.0)
            case State.OBSTACLE:
                self.controller.send_velocity(0.2, 0.0)
            case State.TOO_CLOSE:
                self.controller.send_velocity(0.0, turn_direction)
            case State.REVERSING:
                speed = max(0.1, min(0.3, back_centre * 0.3))
                self.controller.send_velocity(-speed, 0.0)
            case _:
                self.controller.send_velocity(0.0, 0.0)

    def update(self):
        if not self.running or self.front_scan is None or self.back_scan is None:
            return

        # get front and back ranges
        front_left, front_centre, front_right, min_front = self.front_range()
        back_left, back_centre, back_right, min_back = self.back_range()

        if None in (front_left, front_centre, front_right, min_front, back_left, back_centre, back_right, min_back):
            return  # safety if any sensor failed

        # get decision from Reactive
        state, last_state, turn_direction, back_centre = self.reactive.update(
            front_left, front_centre, front_right, min_front,
            back_left, back_centre, back_right, min_back
        )

        self.get_logger().info(
            f'State: {state}, Front L: {front_left:.2f}, C: {front_centre:.2f}, R: {front_right:.2f}, '
            f'Back L: {back_left:.2f}, C: {back_centre:.2f}, R: {back_right:.2f}, Min F: {min_front:.2f}, Min B: {min_back:.2f}'
        )

        self.action(state, turn_direction, front_centre, back_centre)

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
