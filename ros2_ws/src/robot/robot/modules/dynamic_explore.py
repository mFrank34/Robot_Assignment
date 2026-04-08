"""
File: dynamic_explore.py
About: A system to dynamically explore the world.
Author: Michael Franks
"""

import math
from geometry_msgs.msg import Twist
from robot.utils.math_utils import median, average, angle_diff

SEGMENTS = 6
EXPLORE_SPEED = 0.2
TURN_SCALE = 0.5
HISTORY_SIZE = 8
HISTORY_THRESHOLD = 0.4
OPEN_THRESHOLD = 0.8


class DynamicExplore:
    def __init__(self):
        self.heading_history = []
        self.last_steering = 0.0

    def _segment_medians(self, scan):
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
            med = median(seg)

            centre_fraction = (i + 0.5) / SEGMENTS
            angle_offset = (centre_fraction - 0.5) * (scan.angle_max - scan.angle_min)

            results.append((angle_offset, med))

        return results

    def _recently_visited(self, angle):
        for past in self.heading_history:
            diff = angle_diff(angle, past)
            if diff < HISTORY_THRESHOLD:
                return True
        return False

    def _add_history(self, angle):
        self.heading_history.append(angle)
        if len(self.heading_history) > HISTORY_SIZE:
            self.heading_history.pop(0)

    def get_yaw_from_odom(self, odom):
        q = odom.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def update(self, scan, odom) -> Twist:
        out_vel = Twist()
        if scan is None or odom is None:
            return out_vel

        current_yaw = self.get_yaw_from_odom(odom)

        # ===== CORRIDOR DETECTION =====
        ranges = scan.ranges
        mid = len(ranges) // 2

        def clean(vals):
            return [r for r in vals if r > 0.0 and not math.isnan(r) and not math.isinf(r)]

        front = clean(ranges[mid - 30: mid + 30])
        left = clean(ranges[mid + 60: mid + 120])
        right = clean(ranges[mid - 120: mid - 60])

        front_avg = average(front)
        left_avg = average(left)
        right_avg = average(right)

        # Corridor = walls on both sides, open ahead
        if front_avg > 1.5 and left_avg < 1.0 and right_avg < 1.0:
            # ===== WALL CENTERING =====
            error = left_avg - right_avg  # +ve = more space left → steer left

            # small proportional control (tweakable)
            Kp = 0.8

            steering = Kp * error

            # clamp to avoid over-steering
            steering = max(-0.5, min(0.5, steering))

            out_vel.linear.x = EXPLORE_SPEED
            out_vel.angular.z = steering
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

        steering = best_local_ang * TURN_SCALE * 0.6
        self.last_steering = 0.4 * steering + 0.6 * self.last_steering

        out_vel.linear.x = EXPLORE_SPEED
        out_vel.angular.z = self.last_steering
        return out_vel
