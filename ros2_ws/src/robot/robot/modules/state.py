"""
File: state.py
About: state for robots move set...
"""

from enum import Enum


class State(Enum):
    CLEAR = 'clear'
    OBSTACLE = 'obstacle'
    TOO_CLOSE = 'too_close'
    REVERSING = 'reversing'
