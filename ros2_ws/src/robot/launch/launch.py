"""
File: launch.py
About: file for launching the bridge connection to gazebo
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/front_lidar/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/camera_pan@std_msgs/msg/Float64@gz.msgs.Double',
                '/camera_tilt@std_msgs/msg/Float64@gz.msgs.Double',
                '/model/square_bot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            ],
        ),
        Node(
            package='robot',
            executable='server',
        ),
    ])
