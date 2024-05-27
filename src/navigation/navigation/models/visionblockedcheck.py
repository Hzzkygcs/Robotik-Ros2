import math

import numpy as np
from geometry_msgs.msg import Pose2D


class VisionBlockedChecker():
    def __init__(self, vision_angle):
        assert vision_angle >= 0
        self.vision_angle = vision_angle
        self.obstacle_distances =np.array([])
        self.obstacle_angles =np.array([])

    def set_obstacle_distances(self, distances):
        self.obstacle_distances = distances

    def set_obstacle_angles(self, angles):
        """This function assumed to be sorted ascendingly"""
        self.obstacle_angles = angles

    # check if a point is blocked by any obstacle within +-angle_rads
    def is_vision_blocked(self, target_point, robot_pose: Pose2D):
        assert len(self.obstacle_distances) == len(self.obstacle_angles)
        if len(self.obstacle_distances) == 0:
            return True
        dx = target_point[0] - robot_pose.x
        dy = target_point[1] - robot_pose.y
        length = (dx**2 + dy**2)**0.5
        angle = math.atan2(dy, dx)
        angle -= robot_pose.theta
        if abs(angle) > math.pi:  # keep the angle range between -pi to +pi
            angle %= 2 * math.pi
        if angle > math.pi:
            angle -= 2 * math.pi
        closest_angle = np.searchsorted(self.obstacle_angles, angle)

        i = closest_angle
        while i < len(self.obstacle_angles) and abs(self.obstacle_angles[i] - angle) < self.vision_angle:
            if self.obstacle_distances[i] < length:
                return True
            i += 1
        if i == len(self.obstacle_angles):
            # we cant conclude because we dont have enough obstacle data (need wider obstacle data)
            # so its best to assume it to be blocked by obstacle
            return True
        i = closest_angle
        while i >= 0 and abs(self.obstacle_angles[i] - angle) < self.vision_angle:
            if self.obstacle_distances[i] < length:
                return True
            i -= 1
        if i <= -1:
            # we cant conclude because we dont have enough obstacle data (need wider obstacle data)
            # so its best to assume it to be blocked by obstacle
            return True
        return False
