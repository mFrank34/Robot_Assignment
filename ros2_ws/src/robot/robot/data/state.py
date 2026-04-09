"""
File: state.py
About: state for robots move set...
Author: Michael Franks
"""

from enum import Enum


class State(Enum):
    """directions of the robot movement system"""
    FORWARD = 0
    REVERSE = 1
    TURN = 2
    STOP = 3
