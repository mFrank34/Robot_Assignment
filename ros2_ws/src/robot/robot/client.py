"""
File: client.py
About: Client controlling the robot system for now
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from robot.modules.controller import Controller


class ClientNode(Node):
    def __init__(self):
        super().__init__('client_node')

        # Use shared commander for movement
        self.controller = Controller(self)

        # Command publisher for server
        self.command_pub = self.create_publisher(String, '/robot/command', 10)

    def send_command(self, command):
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ClientNode()

    print("Controls:")
    print("  w = forward")
    print("  s = backward")
    print("  a = turn left")
    print("  d = turn right")
    print("  q = pan camera left")
    print("  e = pan camera right")
    print("  r = tilt camera up")
    print("  f = tilt camera down")
    print("  o = start autonomous mode")
    print("  p = stop autonomous mode")
    print("  x = stop robot")
    print("  z = quit")

    while True:
        key = input("Enter command: ").strip().lower()

        match key:
            case 'w':
                node.controller.send_velocity(0.5, 0.0)
            case 's':
                node.controller.send_velocity(-0.5, 0.0)
            case 'a':
                node.controller.send_velocity(0.0, 0.5)
            case 'd':
                node.controller.send_velocity(0.0, -0.5)
            case 'q':
                node.controller.send_pan(0.785)
            case 'e':
                node.controller.send_pan(-0.785)
            case 'r':
                node.controller.send_tilt(-0.349)
            case 'f':
                node.controller.send_tilt(0.349)
            case 'o':
                node.send_command('start')
                print("Autonomous mode started")
            case 'p':
                node.send_command('stop')
                print("Autonomous mode stopped")
            case 'x':
                node.controller.send_velocity(0.0, 0.0)
            case 'z':
                node.controller.send_velocity(0.0, 0.0)
                break
            case _:
                print("Unknown key")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
