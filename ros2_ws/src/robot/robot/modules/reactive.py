"""
File: reactive.py
About: obstacle avoidance
Author: Michael Franks
"""

from rclpy.duration import Duration
from robot.modules.state import State


class Reactive:
    def __init__(self, clock):
        self.clock = clock
        self.state_ts = self.clock.now()

        # robot state
        self.state = State.CLEAR
        self.last_state = State.CLEAR

        # stopping distance
        self.distance = 0.5

        # hysteresis — require more clearance to exit manoeuvre than to enter
        self.clear_margin = 1.2

        # turning info
        self.turning = False
        self.turn_direction = 1.0  # positive = left, negative = right
        self.min_turn_time = Duration(seconds=1.5)
        self.max_turn_time = Duration(seconds=3.0)
        self.turn_attempts = 0
        self.max_turn_attempts = 2

        # reversing info
        self.reversing = False
        self.max_reverse_time = Duration(seconds=3.0)

    def go_state(self, turning=False, reversing=False):
        self.turning = turning
        self.reversing = reversing
        self.state_ts = self.clock.now()

    def update(self, front_left, front_centre, front_right, min_front,
               back_left, back_centre, back_right, min_back):
        self.last_state = self.state

        clear_threshold = self.distance * self.clear_margin
        elapsed = self.clock.now() - self.state_ts

        # only update turn direction when not already manoeuvring
        if not self.turning and not self.reversing:
            diff = abs(front_left - front_right)
            if diff > 0.2:
                self.turn_direction = 1.0 if front_left > front_right else -1.0

        # --- TURNING MODE ---
        if self.turning:

            # don't check exit until we've turned enough to actually reorient
            if elapsed > self.min_turn_time:
                if self.turn_direction > 0:
                    side_clear = front_left > clear_threshold
                else:
                    side_clear = front_right > clear_threshold

                if side_clear and min_front > clear_threshold:
                    self.turn_attempts = 0
                    self.state = State.CLEAR
                    self.go_state()
                    return self.state, self.last_state, self.turn_direction, back_centre

            # been turning too long
            if elapsed > self.max_turn_time:
                self.turn_attempts += 1

                if self.turn_attempts < self.max_turn_attempts:
                    # flip direction and try again
                    self.turn_direction *= -1
                    self.go_state(turning=True)
                    self.state = State.TOO_CLOSE
                elif min_back > self.distance:
                    # try reversing
                    self.turn_attempts = 0
                    self.state = State.REVERSING
                    self.go_state(reversing=True)
                else:
                    # back blocked too, keep turning
                    self.turn_attempts = 0
                    self.go_state(turning=True)
                    self.state = State.TOO_CLOSE
            else:
                self.state = State.TOO_CLOSE

            return self.state, self.last_state, self.turn_direction, back_centre

        # --- REVERSING MODE ---
        if self.reversing:

            # front is clear — pick best direction and turn to reorient
            if min_front > clear_threshold and front_left > clear_threshold and front_right > clear_threshold:
                if abs(front_left - front_right) > 0.2:
                    self.turn_direction = 1.0 if front_left > front_right else -1.0
                self.state = State.TOO_CLOSE
                self.go_state(turning=True)
                return self.state, self.last_state, self.turn_direction, back_centre

            # been reversing too long — pick best direction and turn
            if elapsed > self.max_reverse_time:
                if abs(front_left - front_right) > 0.2:
                    self.turn_direction = 1.0 if front_left > front_right else -1.0
                self.state = State.TOO_CLOSE
                self.go_state(turning=True)
            else:
                self.state = State.REVERSING

            return self.state, self.last_state, self.turn_direction, back_centre

        # --- NORMAL MODE ---
        if min_front < self.distance or front_left < self.distance or front_right < self.distance:
            self.go_state(turning=True)
            self.state = State.TOO_CLOSE
        elif min_back < self.distance:
            self.state = State.OBSTACLE
        else:
            self.state = State.CLEAR

        return self.state, self.last_state, self.turn_direction, back_centre
