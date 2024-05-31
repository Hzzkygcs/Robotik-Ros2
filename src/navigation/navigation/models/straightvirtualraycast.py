import math

import cv2
import numpy as np
from matplotlib import pyplot as plt

# in coppeliasim measurement unit
ROBOT_RADIUS = 0.32



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
        coordinates = np.stack((self._x, self._y), axis=1)
        distances = np.linalg.norm(coordinates, axis=1)

        closest_left_index = np.where((self._x > -self.robot_radius) & (self._x <= 0), distances,
                                      np.inf).argmin()
        closest_right_index = np.where((self._x > 0) & (self._x < self.robot_radius), distances,
                                       np.inf).argmin()
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




class StraightVirtualRayCastVisualizer:
    def __init__(self, straightVirtualRaycast: StraightVirtualRayCast, fig=None, ax=None):
        self.straightRaycast = straightVirtualRaycast
        self.destinations = []
        self.fig = plt.figure() if fig is None else fig
        self.ax = self.fig.add_subplot(1, 1, 1) if ax is None else ax

        self.fig.show()
        self.start = (-5, -0.5)
        self.end = (5, 5)
        self.px_size = (600, 350)
        copied_canvas = self._get_new_canvas()
        self._canvas_axes = self.ax.imshow(self._get_new_canvas(), vmin=0, vmax=255)

        start_x, start_y = self.start
        end_x, end_y = self.end
        self.w = end_x - start_x
        self.h = end_y - start_y

    def _get_new_canvas(self):
        return np.zeros((self.px_size[1]+5, self.px_size[0]+5, 3), dtype=np.uint8)

    def update_frame(self, goal_point=None):  # goal point is in (alpha, distance) relative to current robot's orientation
        copied_canvas = self._get_new_canvas()
        self.draw_obstacles(copied_canvas)
        if goal_point is not None:
            alpha = goal_point[0]
            dist = goal_point[1]
            goal_point = (dist * math.cos(alpha), dist * math.sin(alpha))
            px_x, px_y = self.to_pixel(*goal_point)
            cv2.circle(copied_canvas, (px_x, px_y), 2, (0, 255, 255), 2)

        self._canvas_axes.set_data(copied_canvas)
        plt.pause(0.001)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def set_new_map(self, new_map):
        self.straightRaycast = new_map
        self.update_frame()
        return self

    def set_destinations(self, destinations):
        self.destinations = destinations

    def draw_obstacles(self, copied_canvas):
        obstacles = zip(self.straightRaycast._x, self.straightRaycast._y)


        x_start, _ = self.to_pixel(-self.straightRaycast.robot_radius, 0)
        x_end, _ = self.to_pixel(self.straightRaycast.robot_radius, 0)
        cv2.line(copied_canvas, (x_start, 0), (x_start, self.px_size[1]), (0,0,255), 2)
        cv2.line(copied_canvas, (x_end, 0), (x_end, self.px_size[1]), (0,0,255), 2)

        closest_left = self.straightRaycast.closest_left
        if closest_left is not None:
            cv2.line(copied_canvas, self.to_pixel(*closest_left), self.to_pixel(0,0), (0,255,0), 2)
        closest_right = self.straightRaycast.closest_right
        if closest_right is not None:
            cv2.line(copied_canvas, self.to_pixel(*closest_right), self.to_pixel(0,0), (0,255,0), 2)

        for x, y in obstacles:
            px_x, px_y = self.to_pixel(x, y)
            cv2.circle(copied_canvas, (px_x, px_y), 2, (255, 0, 0), 1)

    def to_pixel(self, x, y):
        return to_pixel(x, y, self.start[0], self.start[1], self.w, self.h, self.px_size[0], self.px_size[1])

def to_pixel(x, y, start_x, start_y, actual_w, actual_h, px_w, px_h):
    return round((x - start_x)/actual_w * px_w), round((1 - (y - start_y)/actual_h) * px_h)