"""
File: mode.py
About: another static class for switching between modes
Author: Michael Franks
"""

from enum import Enum


class Mode(Enum):
    """directions of the robot movement system"""
    REACTIVE = 0
    EXPLORE = 1
    HYBRID = 2
