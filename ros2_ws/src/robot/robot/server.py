"""
File: server.py
About: server for robot to listen to topic and send request.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float64

""" Personal Class and Enums """
from robot.modules.state import State
from robot.modules.reactive import Reactive
from robot.modules.commander import Commander


class RobotServer(Node):
    def __init__(self):
        super().__init__('robot_node')
        self.running = False
        self.controller = Commander(self)
        self.reactive = Reactive()

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

    def command_callback(self, msg):
        if msg.data == 'start':
            self.running = True
            self.get_logger().info('Autonomous mode started')
        elif msg.data == 'stop':
            self.running = False
            self.stop_robot()
            self.get_logger().info('Autonomous mode stopped')

    def update(self):
        if not self.running or self.front_scan is None or self.back_scan is None:
            return

        # Filter out invalid 0.0 and inf readings
        front_ranges = [r for r in self.front_scan.ranges if r > 0.0 and r != float('inf')]
        back_ranges = [r for r in self.back_scan.ranges if r > 0.0 and r != float('inf')]

        # Check we still have valid readings after filtering
        if not front_ranges or not back_ranges:
            return

        min_front = min(front_ranges)
        min_back = min(back_ranges)

        state, last_state = self.reactive.update(min_front, min_back)
        self.get_logger().info(f'State: {state}, Front: {min_front:.2f}, Back: {min_back:.2f}')

        match state:
            case State.CLEAR:
                self.controller.send_velocity(0.5, 0.0)

            case State.OBSTACLE:
                self.controller.send_velocity(0.3, 0.0)

            case State.TOO_CLOSE:
                # ALWAYS turn when too close
                self.controller.send_velocity(0.0, 0.5)

            case State.REVERSING:
                self.controller.send_velocity(-0.3, 0.0)

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
