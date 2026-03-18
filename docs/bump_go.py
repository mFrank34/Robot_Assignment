"""
File: bump_go.py
About: simple bump and go robot controller using FSM
"""

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class BumpGoNode(Node):
    def __init__(self):
        super().__init__('bump_go')

        # states
        self.FORWARD = 0
        self.REVERSE = 1
        self.TURN = 2
        self.STOP = 3

        self.state = self.FORWARD
        self.state_ts = self.get_clock().now()

        # timing
        self.TURNING_TIME = 2.0
        self.REVERSING_TIME = 2.0
        self.SCAN_TIMEOUT = 1.0

        # speeds
        self.SPEED_LINEAR = 0.3
        self.SPEED_ANGULAR = 0.3

        # obstacle threshold
        self.OBSTACLE_DISTANCE = 1.0

        self.last_scan = None

        self.scan_sub = self.create_subscription(
            LaserScan,
            'input_scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.vel_pub = self.create_publisher(Twist, 'output_vel', 10)

        self.timer = self.create_timer(0.05, self.control_cycle)

    def scan_callback(self, msg):
        self.last_scan = msg

    def control_cycle(self):
        if self.last_scan is None:
            return

        out_vel = Twist()

        if self.state == self.FORWARD:
            out_vel.linear.x = self.SPEED_LINEAR
            self.get_logger().info('Moving forward ...')
            if self.check_forward_2_stop():
                self.go_state(self.STOP)
            if self.check_forward_2_reverse():
                self.go_state(self.REVERSE)

        elif self.state == self.REVERSE:
            out_vel.linear.x = -self.SPEED_LINEAR
            self.get_logger().info('Reversing ...')
            if self.check_reverse_2_turn():
                self.go_state(self.TURN)

        elif self.state == self.TURN:
            out_vel.angular.z = self.SPEED_ANGULAR
            self.get_logger().info('Turning ...')
            if self.check_turn_2_forward():
                self.go_state(self.FORWARD)

        elif self.state == self.STOP:
            out_vel.linear.x = 0.0
            out_vel.angular.z = 0.0
            self.get_logger().info('Stopped, waiting for sensor data ...')
            if self.check_stop_2_forward():
                self.go_state(self.FORWARD)

        self.vel_pub.publish(out_vel)

    def go_state(self, new_state):
        self.state = new_state
        self.state_ts = self.get_clock().now()

    def check_forward_2_reverse(self):
        pos = round(len(self.last_scan.ranges) / 2)
        return self.last_scan.ranges[pos] < self.OBSTACLE_DISTANCE

    def check_forward_2_stop(self):
        elapsed = self.get_clock().now() - Time.from_msg(self.last_scan.header.stamp)
        return elapsed > Duration(seconds=self.SCAN_TIMEOUT)

    def check_stop_2_forward(self):
        elapsed = self.get_clock().now() - Time.from_msg(self.last_scan.header.stamp)
        return elapsed < Duration(seconds=self.SCAN_TIMEOUT)

    def check_reverse_2_turn(self):
        elapsed = self.get_clock().now() - self.state_ts
        return elapsed > Duration(seconds=self.REVERSING_TIME)

    def check_turn_2_forward(self):
        elapsed = self.get_clock().now() - self.state_ts
        return elapsed > Duration(seconds=self.TURNING_TIME)


def main(args=None):
    rclpy.init(args=args)
    print('Robot Controller using FSM')
    bump_go_node = BumpGoNode()
    rclpy.spin(bump_go_node)
    bump_go_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()