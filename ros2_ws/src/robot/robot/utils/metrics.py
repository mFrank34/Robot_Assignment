import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import math
import time
import csv
import os

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import math
import time
import csv
import os


class MetricsNode(Node):
    def __init__(self):
        super().__init__('metrics_node')

        self.declare_parameter("env", "maze")
        self.declare_parameter("trial", 1)
        self.declare_parameter("duration", 300.0)

        self.env = self.get_parameter("env").value
        self.trial = int(self.get_parameter("trial").value)
        self.duration = float(self.get_parameter("duration").value)

        self.create_subscription(
            Odometry,
            '/model/square_bot/odometry',
            self.odom_callback,
            10
        )

        self.create_subscription(
            LaserScan,
            '/front_lidar/scan',
            self.scan_callback,
            10
        )

        self.start_time = time.time()
        self.last_position = None
        self.total_distance = 0.0

        self.min_obstacle = float('inf')
        self.collision_threshold = 0.25

        self.file_path = "results.csv"
        self.init_csv()

        self.create_timer(1.0, self.check_done)
        self.get_logger().info("Metrics node started (aligned with server)")

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        current = (x, y)

        if self.last_position is not None:
            dx = current[0] - self.last_position[0]
            dy = current[1] - self.last_position[1]
            self.total_distance += math.sqrt(dx * dx + dy * dy)

        self.last_position = current

    def scan_callback(self, msg):
        valid = [
            r for r in msg.ranges
            if math.isfinite(r) and r > 0.05
        ]

        if not valid:
            return

        min_r = min(valid)
        self.min_obstacle = min(self.min_obstacle, min_r)

    def init_csv(self):
        if not os.path.isfile(self.file_path):
            with open(self.file_path, 'w', newline='') as f:
                writer = csv.writer(f, delimiter=';')
                writer.writerow([
                    "env",
                    "trial",
                    "time",
                    "distance",
                    "collision",
                    "min_obstacle"
                ])

    def check_done(self):
        elapsed = time.time() - self.start_time

        if elapsed >= self.duration:
            self.save_results()
            self.get_logger().info("Run complete")
            rclpy.shutdown()

    def save_results(self):
        elapsed = time.time() - self.start_time

        collision = int(self.min_obstacle < self.collision_threshold)

        with open(self.file_path, 'a', newline='') as f:
            writer = csv.writer(f, delimiter=';')
            writer.writerow([
                self.env,
                self.trial,
                round(elapsed, 2),
                round(self.total_distance, 2),
                collision,
                round(self.min_obstacle, 3) if math.isfinite(self.min_obstacle) else "inf"
            ])

        self.get_logger().info("Metrics saved")


def main(args=None):
    rclpy.init(args=args)
    node = MetricsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
