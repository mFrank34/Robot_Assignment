"""
File: state.py
About: state for robots move set...
"""

from enum import Enum


class State(Enum):
    FORWARD = 0
    REVERSE = 1
    TURN = 2
    STOP = 3
