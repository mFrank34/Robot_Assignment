"""
File: metrics.py
About: metric collection to collect metric around the robot
Author: Michael Franks
"""

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

        self.create_subscription(Odometry, '/model/square_bot/odometry', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/front_lidar/scan', self.scan_callback, 10)

        self.start_time = time.time()

        self.last_position = None
        self.total_distance = 0.0

        self.min_obstacle = float('inf')
        self.collision_threshold = 0.35
        self.near_miss_threshold = 0.75

        self.collision_count = 0
        self.in_collision = False

        self.near_miss_count = 0
        self.in_near_miss = False

        self.stuck_duration = 0.0
        self.last_odom_time = None

        self.file_path = "results.csv"
        self.init_csv()

        self.create_timer(1.0, self.check_done)

        self.get_logger().info("Metrics node started")

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        now = time.time()

        if self.last_position is not None:
            dx = x - self.last_position[0]
            dy = y - self.last_position[1]
            dist = math.sqrt(dx * dx + dy * dy)
            self.total_distance += dist


            if dist < 0.01 and self.last_odom_time is not None:
                self.stuck_duration += now - self.last_odom_time

        self.last_position = (x, y)
        self.last_odom_time = now

    def scan_callback(self, msg):
        valid = [
            r for r in msg.ranges
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max
        ]

        if not valid:
            return

        min_r = min(valid)
        self.min_obstacle = min(self.min_obstacle, min_r)

        # collision event
        if min_r < self.collision_threshold:
            if not self.in_collision:
                self.collision_count += 1
                self.in_collision = True
        else:
            self.in_collision = False

        if self.collision_threshold <= min_r < self.near_miss_threshold:
            if not self.in_near_miss:
                self.near_miss_count += 1
                self.in_near_miss = True
        else:
            self.in_near_miss = False

    def init_csv(self):
        if not os.path.isfile(self.file_path):
            with open(self.file_path, 'w', newline='') as f:
                writer = csv.writer(f, delimiter=';')
                writer.writerow([
                    "env",
                    "trial",
                    "time",
                    "distance",
                    "avg_speed",
                    "collisions",
                    "near_misses",
                    "min_obstacle",
                    "stuck_seconds"
                ])

    def check_done(self):
        elapsed = time.time() - self.start_time

        if elapsed >= self.duration:
            self.save_results()
            self.get_logger().info("Run complete")
            rclpy.shutdown()

    def save_results(self):
        elapsed = time.time() - self.start_time
        avg_speed = self.total_distance / elapsed if elapsed > 0 else 0.0

        with open(self.file_path, 'a', newline='') as f:
            writer = csv.writer(f, delimiter=';')
            writer.writerow([
                self.env,
                self.trial,
                round(elapsed, 2),
                round(self.total_distance, 2),
                round(avg_speed, 3),
                self.collision_count,
                self.near_miss_count,
                round(self.min_obstacle, 3) if math.isfinite(self.min_obstacle) else "inf",
                round(self.stuck_duration, 2)
            ])

        self.get_logger().info("Metrics saved")


def main(args=None):
    rclpy.init(args=args)
    node = MetricsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_results()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()