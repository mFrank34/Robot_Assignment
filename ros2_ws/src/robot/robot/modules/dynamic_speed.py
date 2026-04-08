"""
File: dynamic_speed.py
About: dynamic speed of robot control system.
Author: Michael Franks
"""


class DynamicSpeed(object):
    MINIMUM_SPEED = 0.1
    MAXIMUM_SPEED = 5.0

    SLOW_DISTANCE = 5.0
    STOP_DISTANCE = 0.5

    def __init__(self):
        self.percent = 100

    def update(self, front_centre: float):
        """Returns speed as a 0-100 percent based on nearest obstacle."""
        if front_centre >= self.SLOW_DISTANCE:
            self.percent = 100
        elif front_centre <= self.STOP_DISTANCE:
            self.percent = 0
        else:
            ratio = (front_centre - self.STOP_DISTANCE) / (self.SLOW_DISTANCE - self.STOP_DISTANCE)
            self.percent = round(ratio * 100)

        return self.percent

    def to_velocity(self):
        """Convert current percent to an actual linear velocity."""
        return self.MIN_SPEED + (self.percent / 100) * (self.MAX_SPEED - self.MIN_SPEED)
