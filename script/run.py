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
    " 3. Run Client",
    " 4. Run Camera Viewer",
    " 5. Run Metrics Logger",
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
        case "4":
            print("Starting Camera Viewer...")
            run("ros2 run rqt_image_view rqt_image_view")
        case "5":
            env = input("Enter environment (maze/hallway/free_roam): ")
            trial = input("Enter trial number: ")

            print("Starting Metrics Logger...")
            run(f"source ~/ros2_ws/install/setup.bash && ros2 run robot metrics --ros-args -p env:={env} -p trial:={trial} -p duration:=300.0")
        case _:
            print("Invalid Input")


if __name__ == '__main__':
    start()
