"""
File: commander.py
About: system for sending commands to robot for client and server.
"""

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class Commander:
    def __init__(self, node):
        self.cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
        self.pan_pub = node.create_publisher(Float64, '/camera_pan', 10)
        self.tilt_pub = node.create_publisher(Float64, '/camera_tilt', 10)

    def send_velocity(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_pub.publish(msg)

    def send_pan(self, angle):
        msg = Float64()
        msg.data = angle
        self.pan_pub.publish(msg)

    def send_tilt(self, angle):
        msg = Float64()
        msg.data = angle
        self.tilt_pub.publish(msg)
