"""
File: actuator.py
About: System for sending commands to robot — velocity, camera pan/tilt.
"""

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

# safety clamps
MAX_LINEAR = 1.0
MIN_LINEAR = -1.0
MAX_ANGULAR = 1.5
MIN_ANGULAR = -1.5

# camera limits (radians)
PAN_MIN, PAN_MAX = -1.57, 1.57
TILT_MIN, TILT_MAX = -0.5, 0.5


class Actuator:
    """Class for sending commands to a robot."""
    def __init__(self, node):
        """Init the Actuator class."""
        self.cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
        self.pan_pub = node.create_publisher(Float64, '/camera_pan', 10)
        self.tilt_pub = node.create_publisher(Float64, '/camera_tilt', 10)

    def send_velocity(self, linear, angular):
        """Send a Twist directly, clamps values before publishing."""
        msg = Twist()
        msg.linear.x = max(MIN_LINEAR, min(MAX_LINEAR, linear))
        msg.angular.z = max(MIN_ANGULAR, min(MAX_ANGULAR, angular))
        self.cmd_vel_pub.publish(msg)

    def send_twist(self, twist: Twist):
        """Send a Twist directly, clamps values before publishing."""
        self.send_velocity(twist.linear.x, twist.angular.z)

    def stop(self):
        """A system that stop the robot."""
        self.send_velocity(0.0, 0.0)

    def send_pan(self, angle):
        """Send an angle directly to the robot camera."""
        msg = Float64()
        msg.data = max(PAN_MIN, min(PAN_MAX, angle))
        self.pan_pub.publish(msg)

    def send_tilt(self, angle):
        """Send a tilt angle directly to the robot camera."""
        msg = Float64()
        msg.data = max(TILT_MIN, min(TILT_MAX, angle))
        self.tilt_pub.publish(msg)
