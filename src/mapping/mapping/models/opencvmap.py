import math

import cv2
import numpy as np
import matplotlib.pyplot as plt

plt.ion()
MAP_START=(-8.3,-5.625)
MAP_END=(8.3, 5.4)

mngr = plt.get_current_fig_manager()


RESOLUTION = 0.01

PATH_OBSTACLE_RADIUS = 3
PATH_CLEAR_THICKNESS = 1

PATH_OBSTACLE_VALUE = 255
PATH_UNKNOWN_VALUE = 150
PATH_CLEAR_VALUE = 0



class OpenCvMap:
    def __init__(self, map_start: tuple[float, float]=MAP_START, mep_end: tuple[float, float]=MAP_END,
                 resolution=RESOLUTION):
        self.min_x, self.min_y = map_start
        self.max_x, self.max_y = mep_end
        self.resolution = resolution

        self.act_width = self.max_x - self.min_x
        self.act_height = self.max_y - self.min_y
        self.px_width = math.ceil(self.act_width / resolution)
        self.px_height = math.ceil(self.act_height / resolution)
        self.px_anchor_x = round(-self.min_x/resolution)
        self.px_anchor_y = round(-self.min_y/resolution)

        self.canvas = np.full((self.px_height, self.px_width, 1), PATH_UNKNOWN_VALUE, dtype=np.uint8)
        self.__canvas_axes = plt.imshow(self.canvas, cmap='gray', vmin=0, vmax=255)
        plt.show()

    def actual_to_px(self, point):
        return (self.x_to_px(point[0]), self.y_to_px(point[1]))

    def x_to_px(self, x: float) -> float:
        percentage = (x - self.min_x) / self.act_width
        ret = round(percentage * self.px_width)
        if ret < 0:
            return 0
        if ret > self.px_width:
            return self.px_width - 1
        return ret

    def y_to_px(self, y: float) -> float:
        # 1- to mirror the Y axis such that its displayed like the one in CoppeliaSim
        percentage = 1 - (y - self.min_y) / self.act_height
        ret = round(percentage * self.px_height)
        if ret < 0:
            return 0
        if ret >= self.px_height:
            return self.px_height - 1
        return ret

    def add_raycast(self, position, hit_pos):
        position_px = self.actual_to_px(position)
        hit_pos_px = self.actual_to_px(hit_pos)
        cv2.line(self.canvas, position_px, hit_pos_px, PATH_CLEAR_VALUE, PATH_CLEAR_THICKNESS)
        cv2.circle(self.canvas, hit_pos_px, PATH_OBSTACLE_RADIUS, PATH_OBSTACLE_VALUE)

    def update_frame(self):
        self.__canvas_axes.set_data(self.canvas)
        plt.pause(0.001)
        plt.draw()
