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
from robot.modules.state import state


class RobotServer(Node):
    def __init__(self):
        super().__init__('robot_node')
        self.running = False

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

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pan_pub = self.create_publisher(Float64, '/camera_pan', 10)

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
        state = self.reactive.update(min_front, min_back)

        if state == State.CLEAR:
            pass
        elif state == State.REVERSING:
            pass
        elif state == State.OBSTACLE:
            pass
        elif state == State.TOO_CLOSE:
            pass


    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
