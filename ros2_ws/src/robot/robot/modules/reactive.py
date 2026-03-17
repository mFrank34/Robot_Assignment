"""
File: reactive.py
About: obstacle avoidance
Author: Michael Franks
"""

from robot.modules import State

class Reactive:
    def __init__(self):
        self.state = State.CLEAR

    def update(self, forward_distance, backward_distance):
        if forward_distance > 2.0 > backward_distance > 2.0:
            # both clear — move forward freely
            self.state = State.CLEAR
        elif forward_distance < 2.0 < backward_distance:
            # blocked in front, clear behind — reverse
            self.state = State.REVERSING
        elif forward_distance > 2.0 > backward_distance:
            # clear in front, blocked behind — move forward
            self.state = State.OBSTACLE
        else:
            # blocked both sides — spin to find clear direction
            self.state = State.TOO_CLOSE
        return self.state
