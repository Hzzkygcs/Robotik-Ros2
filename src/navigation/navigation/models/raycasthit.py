from __future__ import annotations
import math

from geometry_msgs.msg import Pose2D, Point


class RayCastHit:
    def __init__(self, range, angle, pose: Pose2D):
        self.range = range
        self.angle = angle
        self.pose = pose

    def relative_position(self, position: Pose2D) -> ObstaclePoint:
        obstacle_absolute_angle = position.theta + self.angle
        obs_relative_position_x = self.range * math.cos(obstacle_absolute_angle)
        obs_relative_position_y = self.range * math.sin(obstacle_absolute_angle)
        return ObstaclePoint(position.x + obs_relative_position_x, position.y + obs_relative_position_y, self)


    def relative_pov_position(self, position: Pose2D):  # 0 = infront, - = right, + = left
        obstacle_pos = self.relative_position(position)
        rotation = obstacle_pos.abs_angle() - position.theta + math.pi/2
        length = obstacle_pos.length()
        return ObstaclePoint.from_polar(length, rotation, self)

    @property
    def obstacle_hit_abs_position(self):
        obs_relative_pos = self.relative_position(self.pose)
        return ObstaclePoint(self.pose.x + obs_relative_pos.x,obs_relative_pos.y + self.pose.y, self)


class ObstaclePoint:
    def __init__(self, x, y, originalRaycast):
        self.x = x
        self.y = y
        self.originalRaycast = originalRaycast

    def distance_to(self, other: ObstaclePoint) -> float:
        dx = other.x - self.x
        dy = other.y - self.y
        return math.sqrt(dx ** 2 + dy ** 2)

    def abs_angle(self):
        return math.atan2(self.y, self.x)

    def length(self):
        return self.distance_to(ObstaclePoint.zero)

    def __repr__(self):
        return f"({self.x},{self.y})"

    @classmethod
    def from_polar(self, distance, angle, originalRaycast):
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)
        return ObstaclePoint(x, y, originalRaycast)


ObstaclePoint.zero = ObstaclePoint(0, 0, None)