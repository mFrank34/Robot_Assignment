#!/usr/bin/env python3

"""
System Designed Run the Robot Simulation
Author: Michael Franks
"""

import os
import time

information = [
    " 1. Start Gazebo Simulation",
    " 2. Launch Server & bridge",
    " 3. Run Client"
]


def run(command):
    os.system(f'bash -c "{command}; exec bash"')
    time.sleep(1)


def start():
    for index in information:
        print(index)

    user_input = input("Enter your command: ")
    match user_input:
        case "1":
            print("Starting Gazebo Simulation...")
            run("ign gazebo ~/gazebo_projects/worlds/hallway.sdf")
        case "2":
            print("Starting Server & bridge...")
            run("source ~/ros2_ws/install/setup.bash && ros2 launch robot launch.py")
        case "3":
            print("Starting Client...")
            run("source ~/ros2_ws/install/setup.bash && ros2 run robot client")
        case _:
            print("Invalid Input")


if __name__ == '__main__':
    start()
