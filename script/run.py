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

simulation = [
    "1. Maze",
    "2. Hallway",
    "3. Free Roam"
]


def run(command):
    os.system(f'bash -c "{command}; exec bash"')
    time.sleep(1)


def run_sim():
    print("\n--- Select World ---")
    for index in simulation:
        print(index)
    user_input = input("Enter your command: ")
    os.system("pkill -9 -f ign 2>/dev/null")
    match user_input:
        case "1":
            print("Starting Maze Simulation...")
            run("ign gazebo ~/gazebo_projects/worlds/maze.sdf")
        case "2":
            print("Starting Hallway Simulation...")
            run("ign gazebo ~/gazebo_projects/worlds/hallway.sdf")
        case "3":
            print("Starting Free Roam Simulation...")
            run("ign gazebo ~/gazebo_projects/worlds/free_roam.sdf")
        case _:
            print("Invalid Input")


def start():
    print("\n=== Main Menu ===")

    for index in information:
        print(index)

    user_input = input("Enter your command: ")
    match user_input:
        case "1":
            run_sim()
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
