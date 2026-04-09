import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from robot.modules.actuator import Actuator
import sys
import termios
import tty


class ClientNode(Node):
    def __init__(self):
        super().__init__("client")

        self.command_pub = self.create_publisher(String, "/robot/command", 10)
        self.controller = Actuator(self)

        self.pan = 0.0
        self.tilt = 0.0

        self.pan_step = 0.1
        self.tilt_step = 0.1

    def send_system_command(self, state):
        msg = String()
        msg.data = state
        self.command_pub.publish(msg)
        self.get_logger().info(f"System: {state.upper()}")

    def update_camera(self):
        self.controller.send_pan(self.pan)
        self.controller.send_tilt(self.tilt)


def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        key = sys.stdin.read(1)
        if key == '\x1b':
            key += sys.stdin.read(2)  # Read full escape sequence while still in cbreak
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return key


def main():
    rclpy.init()
    node = ClientNode()

    print("Controls:")
    print("o = start | p = stop | x = center | z = exit")
    print("1 = reactive | 2 = explore | 3 = hybrid")
    print("arrows = camera pan/tilt")
    print("Ctrl+C = exit")

    try:
        while rclpy.ok():
            key = get_key()
            print(repr(key))

            if key == '\x03':
                print("Exiting...")
                break

            # SYSTEM
            if key == 'o':
                node.send_system_command("start")
            elif key == 'p':
                node.send_system_command("stop")
            elif key == 'x':
                node.pan = 0.0
                node.tilt = 0.0
                node.update_camera()  # Only update on camera change
                print("Camera centered")
            # MODES
            elif key == '1':
                node.send_system_command("reactive")
            elif key == '2':
                node.send_system_command("explore")
            elif key == '3':
                node.send_system_command("hybrid")

            elif key == 'z':
                break

            # ARROWS
            elif key == '\x1b[A':  # up
                node.tilt -= node.tilt_step
                node.update_camera()  # Only update on camera change
            elif key == '\x1b[B':  # down
                node.tilt += node.tilt_step
                node.update_camera()  # Only update on camera change
            elif key == '\x1b[C':  # right
                node.pan -= node.pan_step
                node.update_camera()  # Only update on camera change
            elif key == '\x1b[D':  # left
                node.pan += node.pan_step
                node.update_camera()  # Only update on camera change

    except KeyboardInterrupt:
        print("\nCtrl+C pressed")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
