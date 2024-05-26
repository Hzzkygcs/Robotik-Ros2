import math

import numpy as np

# in coppeliasim measurement unit
ROBOT_RADIUS = 0.27



class StraightVirtualRayCast:
    def __init__(self, raycast_length, robot_radius=ROBOT_RADIUS):
        assert robot_radius >= 0
        self._distances = np.array([])
        self._angles = np.array([])
        self._y = np.array([])  # should be sorted based on obstacle distance (distance, not y)
        self._x = np.array([])  # should be sorted based on obstacle distance (distance, not y)
        self._dirty_bit = False
        self.robot_radius = robot_radius
        self.raycast_length = raycast_length
        self._closest_left = None
        self._closest_right = None

    @property
    def obstacle_hits(self):
        return self._distances, self._angles

    @obstacle_hits.setter
    def obstacle_hits(self, value: tuple):
        self._distances: np.ndarray = value[0]
        self._angles: np.ndarray = value[1]
        in_front = (math.radians(-90) < self._angles) & (self._angles < math.radians(90))
        self._distances, self._angles = self._distances[in_front], self._angles[in_front]
        argsort = np.flip(self._distances.argsort())
        self._distances, self._angles = self._distances[argsort], self._angles[argsort]  # sort by distance descendingly
        self._dirty_bit = True

    def _update_xy(self):
        if not self._dirty_bit:
            return
        self._y = self._distances * np.sin(np.pi/2 + self._angles)
        self._x = self._distances * np.cos(np.pi/2 + self._angles)
        closest_left_index = np.where((self._x > -self.robot_radius) & (self._x <= 0), self._y, np.inf).argmin()
        closest_right_index = np.where((self._x > 0) & (self._x < self.robot_radius), self._y, np.inf).argmin()
        self._closest_left = self._x[closest_left_index], self._y[closest_left_index]
        self._closest_right = self._x[closest_right_index], self._y[closest_right_index]
        self._closest_left_index = closest_left_index  # debugging purpose only
        self.closest_right_index = closest_right_index  # debugging purpose only
        assert self._closest_left[1] >= 0
        assert self._closest_right[1] >= 0



    @property
    def closest_left(self):
        self._update_xy()
        if self._closest_left is None or self._closest_left[1] >= self.raycast_length:
            return None
        return self._closest_left

    @property
    def closest_right(self):
        self._update_xy()
        if self._closest_right is None or self._closest_right[1] >= self.raycast_length:
            return None
        return self._closest_right

    @property  # return (x, y, is_left)
    def closest_overall(self):
        left = self.closest_left
        right = self.closest_right
        if left is None:
            return right
        if right is None:
            return left
        if left[1] >= right[1]:
            return right
        return left