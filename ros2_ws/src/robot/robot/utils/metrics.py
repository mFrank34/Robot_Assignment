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

        # ---- parameters ----
        self.declare_parameter("env", "maze")
        self.declare_parameter("trial", 1)
        self.declare_parameter("duration", 300.0)

        self.env = self.get_parameter("env").value
        self.trial = int(self.get_parameter("trial").value)
        self.duration = float(self.get_parameter("duration").value)

        # ---- subscriptions ----
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # ---- state ----
        self.start_time = time.time()
        self.last_position = None
        self.total_distance = 0.0

        self.min_obstacle = float('inf')
        self.collision_threshold = 0.25

        # ---- CSV setup ----
        self.file_path = "results.csv"
        self.init_csv()

        # timer checks if 5 min is done
        self.create_timer(1.0, self.check_done)

        self.get_logger().info("CSV Metrics Node started")

    def init_csv(self):
        file_exists = os.path.isfile(self.file_path)

        if not file_exists:
            with open(self.file_path, 'w', newline='') as f:
                writer = csv.writer(f, delimiter=';')
                writer.writerow([
                    "env",
                    "trial",
                    "time",
                    "distance",
                    "collisions",
                    "min_obstacle"
                ])

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
        valid = [r for r in msg.ranges if r > 0.0]
        if not valid:
            return

        min_r = min(valid)
        self.min_obstacle = min(self.min_obstacle, min_r)

    def check_done(self):
        elapsed = time.time() - self.start_time

        if elapsed >= self.duration:
            self.save_results()
            self.get_logger().info("Run complete, shutting down...")
            rclpy.shutdown()

    def save_results(self):
        elapsed = time.time() - self.start_time

        collisions = int(self.min_obstacle < self.collision_threshold)

        with open(self.file_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                self.env,
                self.trial,
                round(elapsed, 2),
                round(self.total_distance, 2),
                collisions,
                round(self.min_obstacle, 3)
            ])

        self.get_logger().info("Results saved to CSV")


def main(args=None):
    rclpy.init(args=args)
    node = MetricsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
