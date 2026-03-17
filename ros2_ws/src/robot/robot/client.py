"""
File: client.py
About: Client controlling the robot system for now
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class ClientNode(Node):
    def __init__(self):
        super().__init__('client_node')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pan_pub = self.create_publisher(Float64, '/camera_pan', 10)
        self.tilt_pub = self.create_publisher(Float64, '/camera_tilt', 10)

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

    def stop(self):
        self.send_velocity(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = ClientNode()

    print("Controls:")
    print(" w = forward")
    print(" s = backward")
    print(" a = turn left")
    print(" d = turn right")
    print(" q = pan camera left")
    print(" e = pan camera right")
    print(" r = tilt camera up")
    print(" f = tilt camera down")
    print(" x = stop")
    print(" z = quit")

    while True:
        key = input("Enter command: ").strip().lower()

        if key == 'w':
            node.send_velocity(0.5, 0.0)
        elif key == 's':
            node.send_velocity(-0.5, 0.0)
        elif key == 'a':
            node.send_velocity(0.0, 0.5)
        elif key == 'd':
            node.send_velocity(0.0, -0.5)
        elif key == 'q':
            node.send_pan(0.785)
        elif key == 'e':
            node.send_pan(-0.785)
        elif key == 'r':
            node.send_tilt(-0.349)
        elif key == 'f':
            node.send_tilt(0.349)
        elif key == 'x':
            node.stop()
        elif key == 'z':
            node.stop()
            break
        else:
            print("Unknown key")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
