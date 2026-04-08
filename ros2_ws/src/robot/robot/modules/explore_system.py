"""
File: explore_system.py
About: a system to dynamically explore the world with a different behavior / Moving toward the most open space.
Author: Michael Franks
"""

import math
from geometry_msgs.msg import Twist

# number of segments to split the scan into
SEGMENTS = 6
# forward speed while exploring
EXPLORE_SPEED = 0.2
# angular speed scalar
TURN_SCALE = 0.5
# how many recent directions to remember
HISTORY_SIZE = 8
# how close a new direction has to be (radians) to count as "recently visited"
HISTORY_THRESHOLD = 0.4
# minimum median range for a segment to be considered open
OPEN_THRESHOLD = 0.8


class ExploreSystem:
    def __init__(self):
        self.heading_history = []
        self.last_steering = 0.0

    def _segment_medians(self, scan):
        """
        divided up the scan into different segments of equal size
        :return: return a list of angle offsets and median range within tuple.
        """
        ranges = scan.ranges
        total = len(ranges)
        if total == 0:
            return []

        seg_size = total // SEGMENTS
        results = []

        for i in range(SEGMENTS):
            start = i * seg_size
            end = start + seg_size if i < SEGMENTS - 1 else total
            seg = [r for r in ranges[start:end] if r > 0.0 and not math.isnan(r) and not math.isinf(r)]

            if not seg:
                median = 0.0
            else:
                seg_sorted = sorted(seg)
                median = seg_sorted[len(seg_sorted) // 2]

            # map segment index to angle offset
            # segment 0 = leftmost, segment SEGMENTS-1 = rightmost
            centre_fraction = (i + 0.5) / SEGMENTS  # 0.0 to 1.0
            angle_offset = (centre_fraction - 0.5) * (scan.angle_max - scan.angle_min)
            results.append((angle_offset, median))

        return results

    def _recently_visited(self, angle):
        """
        function designed to find if angle been visited or is too close to robot
        :param angle: angle heading in degrees
        :return: if it has been visited or not
        """
        for past in self.heading_history:
            diff = abs(math.atan2(math.sin(angle - past), math.cos(angle - past)))
            if diff < HISTORY_THRESHOLD:
                return True
        return False

    def _add_history(self, angle):
        self.heading_history.append(angle)
        if len(self.heading_history) > HISTORY_SIZE:
            self.heading_history.pop(0)

    def update(self, scan) -> Twist:
        """
        takes a scan input and return twist steering to wards the most open unexplored direction
        :param scan: incoming scan
        :return: direction in twist
        """
        out_vel = Twist()

        if scan is None or len(scan.ranges) == 0:
            return out_vel

        segments = self._segment_medians(scan)
        if not segments:
            return out_vel

        # filter to open segments only
        open_segs = [(angle, dist) for angle, dist in segments if dist >= OPEN_THRESHOLD]

        if not open_segs:
            # everything blocked — rotate slowly to find open space
            out_vel.linear.x = 0.0
            out_vel.angular.z = TURN_SCALE
            return out_vel

        # prefer segments not recently visited
        fresh = [(angle, dist) for angle, dist in open_segs if not self._recently_visited(angle)]
        candidates = fresh if fresh else open_segs

        # pick the most open candidate
        best_angle, best_dist = max(candidates, key=lambda s: s[1])

        # record this direction in history
        self._add_history(best_angle)

        # smooth steering slightly to avoid jerky motion
        steering = best_angle * TURN_SCALE
        self.last_steering = 0.6 * steering + 0.4 * self.last_steering

        out_vel.linear.x = EXPLORE_SPEED
        out_vel.angular.z = self.last_steering

        return out_vel
