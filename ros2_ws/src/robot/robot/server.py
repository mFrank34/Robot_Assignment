"""
File: server.py
About: server for robot to listen to topic and send request /
core system to tell how the robot modules interact with one another.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from robot.data.dimensions import RobotDimensions
from robot.data.state import State
from robot.data.mode import Mode

from robot.modules.actuator import Actuator
from robot.modules.object_avoidance import ObjectAvoidance
from robot.modules.dynamic_explore import DynamicExplore
from robot.modules.dynamic_speed import DynamicSpeed


class RobotServer(Node):
    """robot server"""

    def __init__(self):
        """constructor"""
        super().__init__('robot_node')

        self.running = False
        self.mode_type = Mode.HYBRID

        # actuator — single outbound comms layer
        self.actuator = Actuator(self)

        # behavior modules
        dims = RobotDimensions()
        self.reactive = ObjectAvoidance(self.get_clock(), self.get_logger(), dims)
        self.explore = DynamicExplore()
        self.speed = DynamicSpeed()

        # sensor state
        self.current_odom = None
        self.front_scan = None

        # subscriptions
        self.create_subscription(Odometry, '/model/square_bot/odometry', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/front_lidar/scan', self.front_lidar_callback, 10)
        self.create_subscription(String, '/robot/command', self.command_callback, 10)

        self.timer = self.create_timer(0.1, self.update)

        self.get_logger().info('Robot server started')

    def odom_callback(self, msg):
        """incoming odometry message"""
        self.current_odom = msg

    def front_lidar_callback(self, msg):
        """incoming lidar message"""
        self.front_scan = msg

    def get_front_centre(self, scan) -> float:
        """brake the scan in part and turn the main middle"""
        # Filter using msg metadata for safety
        ranges = [r for r in scan.ranges if scan.range_min <= r <= scan.range_max]
        if not ranges:
            return float('inf')
        # Take the middle third safely
        n = len(ranges)
        third = n // 3
        centre_slice = sorted(ranges[third: 2 * third])

        return centre_slice[len(centre_slice) // 2] if centre_slice else float('inf')

    def update(self):
        """main update loop and control which system is in control"""
        if not self.running or self.front_scan is None or self.current_odom is None:
            return

        front_centre = self.get_front_centre(self.front_scan)

        # --- HARD SWITCH BETWEEN STYLES ---
        if self.mode_type == Mode.REACTIVE:
            vel = self.reactive.update(self.front_scan, self.current_odom)

        elif self.mode_type == Mode.EXPLORE:
            vel = self.explore.update(self.front_scan, self.current_odom)

        elif self.mode_type == Mode.HYBRID:
            vel = self.reactive.update(self.front_scan, self.current_odom)

            if self.reactive.state == State.FORWARD and front_centre > 1.5:
                vel = self.explore.update(self.front_scan, self.current_odom)

        # speed stays shared
        self.speed.update(front_centre)

        if vel.linear.x > 0:
            vel.linear.x = self.speed.to_velocity()

        self.actuator.send_twist(vel)

    def command_callback(self, msg):
        """system to enable and disable robot autonomous mode"""
        if msg.data == 'start':
            self.running = True
            self.get_logger().info('Autonomous mode started')

        elif msg.data == 'stop':
            self.running = False
            self.actuator.stop()
            self.get_logger().info('Autonomous mode stopped')

        elif msg.data == 'reactive':
            self.mode_type = Mode.REACTIVE
            self.get_logger().info('Mode: Reactive only')

        elif msg.data == 'explore':
            self.mode_type = Mode.EXPLORE
            self.get_logger().info('Mode: Explore only')

        elif msg.data == 'hybrid':
            self.mode_type = Mode.HYBRID
            self.get_logger().info('Mode: Hybrid')


def main(args=None):
    rclpy.init(args=args)
    node = RobotServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
