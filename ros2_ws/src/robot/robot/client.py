import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty


class ClientNode(Node):
    def __init__(self):
        super().__init__("client")

        self.command_pub = self.create_publisher(String, "/robot/command", 10)

    def send_system_command(self, state):
        msg = String()
        msg.data = state
        self.command_pub.publish(msg)
        self.get_logger().info(f"System: {state.upper()}")


def get_key():
    """Read single key press from terminal"""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

    return key


def main():
    rclpy.init()
    node = ClientNode()

    print("Controls:")
    print("o = start | p = stop | x = reset | z = exit")
    print("Arrow keys supported partially (see below)")

    try:
        while rclpy.ok():
            key = get_key()

            # letters
            if key == 'o':
                node.send_system_command("start")

            elif key == 'p':
                node.send_system_command("stop")

            elif key == 'x':
                print("Camera reset")

            elif key == 'z':
                break

            # arrow keys (3-char escape sequence)
            elif key == '\x1b':  # ESC
                key2 = sys.stdin.read(2)
                if key2 == "[A":
                    print("UP")
                elif key2 == "[B":
                    print("DOWN")
                elif key2 == "[C":
                    print("RIGHT")
                elif key2 == "[D":
                    print("LEFT")

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
