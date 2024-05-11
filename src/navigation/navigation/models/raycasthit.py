import math

from geometry_msgs.msg import Pose2D


class RayCastHit:
    def __init__(self, range, angle):
        self.range = range
        self.angle = angle

    def obstacle_position(self, position: Pose2D) -> tuple[float, float]:
        obstacle_absolute_angle = position.theta + self.angle
        obs_relative_position_x = self.range * math.cos(obstacle_absolute_angle)
        obs_relative_position_y = self.range * math.sin(obstacle_absolute_angle)
        return position.x + obs_relative_position_x, position.y + obs_relative_position_y


    def is_hit(self):
        pass
