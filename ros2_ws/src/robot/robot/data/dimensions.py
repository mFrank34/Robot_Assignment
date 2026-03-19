from dataclasses import dataclass

@dataclass
class RobotDimensions:
    chassis_length: float = 0.6   # X
    chassis_width: float = 0.5    # Y
    chassis_height: float = 0.1   # Z

    front_lidar_offset_x: float = 0.3
    back_lidar_offset_x: float = -0.3
    lidar_height: float = 0.08

    wheel_radius: float = 0.1
    wheel_separation: float = 0.6   # distance between wheels along Y
    rear_caster_radius: float = 0.1

    # optional safety margin
    safety_margin: float = 0.1