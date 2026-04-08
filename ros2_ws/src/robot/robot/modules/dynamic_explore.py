"""
File: dynamic_explore.py
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


class DynamicExplore:
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

    def get_yaw_from_odom(self, odom):
        """Extracts yaw (rotation around Z) from quaternion."""
        q = odom.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def update(self, scan, odom) -> Twist:
        out_vel = Twist()
        if scan is None or odom is None:
            return out_vel

        current_yaw = self.get_yaw_from_odom(odom)

        # ===== CORRIDOR DETECTION (NEW) =====
        ranges = scan.ranges
        mid = len(ranges) // 2

        def avg(vals):
            clean = [r for r in vals if r > 0.0 and not math.isnan(r) and not math.isinf(r)]
            return sum(clean) / len(clean) if clean else 0.0

        front = ranges[mid - 30: mid + 30]
        left = ranges[mid + 60: mid + 120]
        right = ranges[mid - 120: mid - 60]

        front_avg = avg(front)
        left_avg = avg(left)
        right_avg = avg(right)

        # Corridor = walls on both sides, open ahead
        if front_avg > 1.5 and left_avg < 1.0 and right_avg < 1.0:
            out_vel.linear.x = EXPLORE_SPEED
            out_vel.angular.z = 0.0
            return out_vel
        # ===== END CORRIDOR =====

        segments = self._segment_medians(scan)

        open_segs = [(ang, dist) for ang, dist in segments if dist >= OPEN_THRESHOLD]

        if not open_segs:
            out_vel.angular.z = TURN_SCALE
            self.last_steering = TURN_SCALE
            return out_vel

        fresh = []
        for ang, dist in open_segs:
            global_ang = current_yaw + ang
            if not self._recently_visited(global_ang):
                fresh.append((ang, dist, global_ang))

        if fresh:
            best_local_ang, _, best_global_ang = max(fresh, key=lambda s: s[1])
        else:
            best_local_ang, _ = max(open_segs, key=lambda s: s[1])
            best_global_ang = current_yaw + best_local_ang

        self._add_history(best_global_ang)

        # ===== REDUCED STEERING =====
        steering = best_local_ang * TURN_SCALE * 0.6

        # ===== STRONGER SMOOTHING =====
        self.last_steering = 0.4 * steering + 0.6 * self.last_steering

        out_vel.linear.x = EXPLORE_SPEED
        out_vel.angular.z = self.last_steering
        return out_vel