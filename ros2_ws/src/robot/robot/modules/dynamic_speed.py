"""
File: dynamic_speed.py
About: dynamic speed of robot control system.
Author: Michael Franks
"""


class DynamicSpeed:
    MINIMUM_SPEED = 0.1
    MAXIMUM_SPEED = 0.5

    SLOW_DISTANCE = 2.0
    STOP_DISTANCE = 0.5

    def __init__(self):
        self.percent = 100

    def update(self, front_centre: float) -> int:
        """
        Update speed percent based on distance to nearest obstacle ahead.
        :param front_centre: median range in metres from center lidar segment
        :return: speed as 0-100 percent
        """
        if front_centre >= self.SLOW_DISTANCE:
            self.percent = 100
        elif front_centre <= self.STOP_DISTANCE:
            self.percent = 0
        else:
            ratio = (front_centre - self.STOP_DISTANCE) / (self.SLOW_DISTANCE - self.STOP_DISTANCE)
            self.percent = round(ratio * 100)

        return self.percent

    def to_velocity(self) -> float:
        """Convert current percent to a linear velocity value."""
        return self.MINIMUM_SPEED + (self.percent / 100) * (self.MAXIMUM_SPEED - self.MINIMUM_SPEED)
