from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                '/front_lidar/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                '/back_lidar/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
                '/camera_pan@std_msgs/msg/Float64]ignition.msgs.Double',
                '/camera_tilt@std_msgs/msg/Float64]ignition.msgs.Double',
                '/model/square_bot/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            ],
        ),
        Node(
            package='robot',
            executable='server',
        ),
    ])
