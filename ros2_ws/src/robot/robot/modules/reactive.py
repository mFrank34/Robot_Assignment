"""
File: reactive.py
About: obstacle avoidance
Author: Michael Franks
"""

from robot.modules.state import State

from robot.modules.state import State


class Reactive:
    def __init__(self):
        # state of robot
        self.state = State.CLEAR
        self.last_state = State.CLEAR

        # range of stopping
        self.distance = 0.4

        # turing information
        self.turn_count = 0
        self.turning = False

    def update(self, forward_distance, backward_distance):
        self.last_state = self.state

        # --- TURNING MODE (commit properly) ---
        if self.turning:
            self.turn_count += 1

            # MUST turn for a minimum time
            if self.turn_count < 15:
                self.state = State.TOO_CLOSE
                return self.state, self.last_state

            # after that, only stop if actually clear
            if forward_distance > self.distance:
                self.turning = False
                self.turn_count = 0
                self.state = State.CLEAR
            else:
                self.state = State.TOO_CLOSE

            return self.state, self.last_state

        # --- NORMAL MODE ---
        # obstacle in front start turning
        if forward_distance < self.distance:
            self.turning = True
            self.turn_count = 0
            self.state = State.TOO_CLOSE

        else:
            self.state = State.CLEAR

        return self.state, self.last_state
