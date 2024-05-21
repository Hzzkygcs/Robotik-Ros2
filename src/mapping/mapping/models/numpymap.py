from __future__ import annotations
import math

import cv2
import numpy as np
import matplotlib.pyplot as plt

plt.ion()
MAP_START=(-8.3,-5.625)
MAP_END=(8.3, 5.4)

mngr = plt.get_current_fig_manager()


RESOLUTION = 0.02
ALGORITHM_RESOLUTION = 0.25

PATH_OBSTACLE_RADIUS = 3
PATH_CLEAR_THICKNESS = 1

PATH_OBSTACLE_VALUE = 255
PATH_UNKNOWN_VALUE = 0
PATH_CLEAR_VALUE = 1





class NumpyMap:
    def __init__(self, map_start: tuple[float, float]=MAP_START, map_end: tuple[float, float]=MAP_END,
                 resolution=RESOLUTION, show_image=True):
        self.map_start = map_start
        self.map_end = map_end
        self.min_x, self.min_y = map_start
        self.max_x, self.max_y = map_end
        self.resolution = resolution

        self.act_width = self.max_x - self.min_x
        self.act_height = self.max_y - self.min_y
        self.px_width = math.ceil(self.act_width / resolution)
        self.px_height = math.ceil(self.act_height / resolution)
        self.px_anchor_x = round(-self.min_x/resolution)
        self.px_anchor_y = round(-self.min_y/resolution)

        self.canvas = np.full((self.px_height, self.px_width, 1), PATH_UNKNOWN_VALUE, dtype=np.uint8)

    def get(self, actual_coordinate):
        pixel_coordinate = self.actual_to_px(actual_coordinate)
        return self.canvas[pixel_coordinate[1]][pixel_coordinate[0]]

    def set(self, actual_coordinate, value):
        pixel_coordinate = self.actual_to_px(actual_coordinate)
        self.canvas[pixel_coordinate[1]][pixel_coordinate[0]] = value

    def actual_to_px(self, point):
        return (self.x_to_px(point[0]), self.y_to_px(point[1]))

    def px_to_actual_rect(self, px: tuple) -> Rect:  # tuple of tuple of float
        px_x, px_y = px
        percentage_x = px_x / self.px_width
        percentage_x_end = (px_x+1) / self.px_width
        percentage_y = px_y / self.px_height
        percentage_y_end = (px_y+1) / self.px_height
        ret = Rect(self.min_x + self.act_width * percentage_x,
                   self.min_y + self.act_height * percentage_y,
                   self.min_x + self.act_width * percentage_x_end,
                   self.min_y + self.act_height * percentage_y_end)
        return ret

    def actual_rect_to_px_rect(self, actual_rect: Rect):
        x_start, y_start = self.actual_to_px(actual_rect.start)
        x_end, y_end = self.actual_to_px(actual_rect.end)

        # because +Y is going up on cartecius (actual) coordinate, but going down in array coordinate
        assert y_end < y_start
        y_start, y_end = y_end, y_start
        return Rect(x_start, y_start, x_end, y_end)


    def px_to_x(self, px: float) -> float:
        percentage = px / self.px_width
        ret = self.min_x + self.act_width * percentage
        if ret < self.min_x:
            return self.min_x
        if ret > self.max_x:
            return self.max_x
        return ret

    def px_to_y(self, py: float) -> float:
        percentage = 1 - py / self.px_height
        ret = self.min_y + self.act_height * percentage
        if ret < self.min_y:
            return self.min_y
        if ret > self.max_y:
            return self.max_y
        return ret

    def x_to_px(self, x: float) -> float:
        percentage = (x - self.min_x) / self.act_width
        ret = round(percentage * self.px_width)
        if ret < 0:
            return 0
        if ret >= self.px_width:
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
        self.draw_line(position_px, hit_pos_px)
        self.draw_circle(hit_pos_px)

    def draw_line(self, position_px, hit_pos_px):
        for x, y in bresenham_line(position_px, hit_pos_px):
            if self.canvas[y][x] <= 30:  # cant use max() because we operate with uint8 [0,255] that cant have negative
                self.canvas[y][x] = PATH_CLEAR_VALUE
            else:
                self.canvas[y][x] = self.canvas[y][x] - 30

    def draw_circle(self, hit_pos_px):
        for x, y, manhattan in generate_circle_pixels(hit_pos_px, PATH_OBSTACLE_RADIUS):
            color_addition = 150 / (1+manhattan)
            if self.canvas[y][x] <= 255 - color_addition:  # cant use min() because we operate with uint8 [0, 255]
                self.canvas[y][x] = self.canvas[y][x] + color_addition
            else:
                self.canvas[y][x] = 255

    def resize_dilated_but_efficient(self, new_resolution, show_image=False):
        ret = NumpyMap(self.map_start, self.map_end, new_resolution, show_image=show_image)
        for x in range(ret.px_width):
            for y in range(ret.px_height):
                actual_rect_of_ret = ret.px_to_actual_rect((x,y))
                px_rect_of_self = self.actual_rect_to_px_rect(actual_rect_of_ret)
                sliced = px_rect_of_self.slice_map(self.canvas)
                ret.canvas[y][x] = sliced.max()
        return ret

    def resize_accurate_but_inefficient(self, new_resolution, show_image=False):
        ret = NumpyMap(self.map_start, self.map_end, new_resolution, show_image=show_image)
        for x in range(self.px_width):
            x_actual = self.px_to_x(x)
            for y in range(self.px_height):
                y_actual = self.px_to_y(y)
                actual_coord = (x_actual, y_actual)
                new_value = max(ret.get(actual_coord), self.canvas[y][x])
                ret.set(actual_coord, new_value)
        return ret

    def resize_px_canvas(self, estimate_new_resolution):
        assert self.resolution < estimate_new_resolution
        multiplier = round(estimate_new_resolution // self.resolution)
        return max_pooling_2d_array(self.canvas, (multiplier, multiplier))



class NumpyMapDisplayer:
    def __init__(self, map: NumpyMap):
        self.map = map
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(2,1,1)
        self._canvas_axes = self.ax.imshow(self.map.canvas, cmap='gray', vmin=0, vmax=255)
        self.fig.show()


    def update_frame(self):
        self._canvas_axes.set_data(apply_thresholding(self.map.canvas))
        plt.pause(0.001)
        plt.draw()

    def set_new_map(self, new_map: NumpyMap):
        self.map = new_map
        self.update_frame()
        return self



def apply_dilation(np_canvas_array, copy=True):
    canvas = np.copy(np_canvas_array) if copy else np_canvas_array


def apply_thresholding(np_canvas_array, copy=True):
    canvas = np.copy(np_canvas_array) if copy else np_canvas_array
    upper_threshold = 130
    canvas[np_canvas_array >= upper_threshold] = 255
    canvas[(1 <= np_canvas_array) & (np_canvas_array < upper_threshold)] = 1
    canvas[np_canvas_array == 0] = 124
    return canvas



def max_pooling_2d_array(mat, kernel_size):
    """ https://stackoverflow.com/a/42463514/7069108 """
    M, N, *_ = mat.shape
    K, L = kernel_size
    # assert M % K == 0 and N % L == 0, "Image size must be divisible by kernel size"

    MK = M // K
    NL = N // L

    # split the matrix into 'quadrants'
    Q1 = mat[:MK * K, :NL * L].reshape(MK, K, NL, L).max(axis=(1, 3))
    Q2 = mat[MK * K:, :NL * L].reshape(-1, NL, L).max(axis=2)
    Q3 = mat[:MK * K, NL * L:].reshape(MK, K, -1).max(axis=1)
    Q4 = mat[MK * K:, NL * L:].max()

    # compose the individual quadrants into one new matrix
    return np.vstack([np.c_[Q1, Q3], np.c_[Q2, Q4]])




def generate_circle_pixels(center, radius):  # yield x,y,manhattan distance
    square_area_of_radius = (1 + (radius-1)*2)**2
    square_area_of_radius_min_1 = max(0, (1 + (radius-2)*2) ** 2)
    max_number_of_outter_cells = square_area_of_radius - square_area_of_radius_min_1

    number_of_degree_rotation = max_number_of_outter_cells // 2 + 1
    degree_rotation = (math.pi * 2) / number_of_degree_rotation

    prev_x = float('inf')
    for i in range(number_of_degree_rotation):
        degree = degree_rotation * i
        x = center[0] + round(radius * math.cos(degree))
        y1 = center[1] + round(radius * math.sin(degree))
        y0 = center[1] - round(radius * math.sin(degree))
        if x == prev_x:  # make sure no duplicates due to rounding
            continue
        prev_x = x
        for y in range(y0, y1):
            manhattan_dist = abs(x - center[0]) + abs(y - center[1])
            yield x,y,manhattan_dist


def bresenham_line(start, end):
    """Generate coordinates of the line from (x0, y0) to (x1, y1) using Bresenham's algorithm."""
    x0, y0 = start
    x1, y1 = end

    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy


class Rect:
    def __init__(self, min_x, min_y, max_x, max_y):
        self.min_x = min_x
        self.min_y = min_y
        self.max_x = max_x
        self.max_y = max_y

    def slice_map(self, np_canvas_array) -> np.ndarray:
        return np_canvas_array[self.min_y:self.max_y, self.min_x:self.max_x]

    @property
    def mid(self):
        return (self.min_x + self.max_x) / 2, (self.min_y + self.max_y) / 2

    @property
    def start(self):
        return self.min_x, self.min_y

    @property
    def end(self):
        return self.max_x, self.max_y

