from __future__ import annotations
import math

import cv2
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose2D

from .mapconstants import *

class NumpyMap:
    def __init__(self, map_start: tuple[float, float] = MAP_START, map_end: tuple[float, float] = MAP_END,
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
        self.px_anchor_x = round(-self.min_x / resolution)
        self.px_anchor_y = round(-self.min_y / resolution)

        # V
        self.robot_pos = None
        self.route = None

        self.canvas = np.full((self.px_height, self.px_width, 1), PATH_UNKNOWN_VALUE, dtype=np.uint8)

    def get(self, actual_coordinate):
        pixel_coordinate = self.actual_to_px(actual_coordinate)
        return self.canvas[pixel_coordinate[1]][pixel_coordinate[0]]

    def get_px(self, coordinate):
        return self.canvas[coordinate[1]][coordinate[0]]

    def set(self, actual_coordinate, value):
        pixel_coordinate = self.actual_to_px(actual_coordinate)
        self.canvas[pixel_coordinate[1]][pixel_coordinate[0]] = value

    def actual_to_px(self, point):
        return (self.x_to_px(point[0]), self.y_to_px(point[1]))

    def px_to_actual_rect(self, px: tuple) -> Rect:  # tuple of tuple of float
        px_x, px_y = px
        percentage_x = px_x / self.px_width
        percentage_x_end = (px_x + 1) / self.px_width

        # -1 isntead of +1 because pixel +y is go down meanwhile  cartecius +y is go up
        percentage_y1 = px_y / self.px_height
        percentage_y2 = (px_y + 1) / self.px_height
        y1 = self.max_y - self.act_height * percentage_y1  # max_y - something because the higher y_pixel, the lower y_actual. so we reverse it
        y2 = self.max_y - self.act_height * percentage_y2

        ret = Rect(self.min_x + self.act_width * percentage_x,
                   min(y1, y2),
                   self.min_x + self.act_width * percentage_x_end,
                   max(y1, y2))
        return ret

    def actual_rect_to_px_rect(self, actual_rect: Rect):
        x_start, y_start = self.actual_to_px(actual_rect.start)
        x_end, y_end = self.actual_to_px(actual_rect.end)

        # because +Y is going up on cartecius (actual) coordinate, but going down in array coordinate
        if y_end < y_start:
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

    def add_raycast(self, position, hit_pos, add_obstacle):
        position_px = self.actual_to_px(position)
        hit_pos_px = self.actual_to_px(hit_pos)
        # end_of_line_of_sight_pos = self.actual_to_px(magnitude_multiplier(position, hit_pos))
        path_clear_length = line_length(position_px, hit_pos_px)# - PATH_OBSTACLE_RADIUS
        # if path_clear_length > 0:
        self.draw_line(position_px, hit_pos_px)
        # self.draw_line(position_px, magnitude(position_px, hit_pos_px, path_clear_length))
        if add_obstacle:
            self.draw_circle(hit_pos_px)
        self.substract_circle(position_px)

    def draw_line(self, position_px, hit_pos_px):
        assert all(map(lambda x: isinstance(x, int), position_px+hit_pos_px))
        for x, y in bresenham_line(position_px, hit_pos_px):
            if not (0 <= x < self.px_width and 0 <= y < self.px_height):
                continue
            if self.canvas[y][x] <= 30:  # cant use max() because we operate with uint8 [0,255] that cant have negative
                self.canvas[y][x] = PATH_CLEAR_VALUE
            else:
                self.canvas[y][x] = self.canvas[y][x] - 30
            # if self.canvas[y][x] != PATH_OBSTACLE_VALUE:
            #     self.canvas[y][x] = PATH_CLEAR_VALUE

    def draw_circle(self, hit_pos_px):
        for x, y, manhattan in generate_circle_pixels(hit_pos_px, PATH_OBSTACLE_RADIUS):
            if x >= self.px_width:
                continue
            if y >= self.px_height:
                continue

            color_addition = 150 / (1 + manhattan)
            if self.canvas[y][x] <= 255 - color_addition:  # cant use min() because we operate with uint8 [0, 255]
                self.canvas[y][x] = self.canvas[y][x] + color_addition
            else:
                self.canvas[y][x] = 255

    def substract_circle(self, center_pos):
        for x, y, manhattan in generate_circle_pixels(center_pos, ROBOT_RADIUS):
            if x >= self.px_width:
                continue
            if y >= self.px_height:
                continue
            color_decrement = 30
            if self.canvas[y][x] > color_decrement:  # cant use min() because we operate with uint8 [0, 255]
                self.canvas[y][x] = self.canvas[y][x] - color_decrement
            else:
                self.canvas[y][x] = PATH_CLEAR_VALUE

    def resize_dilated_but_efficient(self, new_resolution, show_image=False):
        ret = NumpyMap(self.map_start, self.map_end, new_resolution, show_image=show_image)
        for x in range(ret.px_width):
            for y in range(ret.px_height):
                actual_rect_of_ret = ret.px_to_actual_rect((x, y))
                px_rect_of_self = self.actual_rect_to_px_rect(actual_rect_of_ret)
                sliced = px_rect_of_self.slice_map(self.canvas)
                ret.canvas[y][x] = sliced.max(initial=0)
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
    def __init__(self, map: NumpyMap, fig=None, ax=None):
        self.map = map
        self.destinations = []
        self.fig = plt.figure() if fig is None else fig
        self.ax = self.fig.add_subplot(2, 1, 1) if ax is None else ax
        self._canvas_axes = self.ax.imshow(self.map.canvas, vmin=0, vmax=255)
        self.fig.show()

    def update_frame(self):
        copied_canvas = to_rgb(apply_thresholding(self.map.canvas))
        self.draw_destinations(copied_canvas)

        if self.map.robot_pos is not None:
            copied_canvas = display_robot_on_canvas(copied_canvas, self.map)
        if self.map.route is not None:
            copied_canvas = display_route_on_canvas(copied_canvas, self.map)

        self._canvas_axes.set_data(copied_canvas)
        plt.pause(0.001)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def set_new_map(self, new_map: NumpyMap):
        self.map = new_map
        self.update_frame()
        return self

    def set_destinations(self, destinations):
        self.destinations = destinations

    def draw_destinations(self, copied_canvas):
        for i in range(len(self.destinations)-1):
            (prev_act_x, prev_act_y, _) = self.destinations[i]
            (act_x, act_y, _) = self.destinations[i+1]
            prev_px_x, prev_px_y = self.map.actual_to_px((prev_act_x, prev_act_y))
            px_x, px_y = self.map.actual_to_px((act_x, act_y))
            cv2.line(copied_canvas, (prev_px_x,prev_px_y),(px_x, px_y), (0, 122, 255), thickness=6)
            cv2.circle(copied_canvas, (px_x, px_y), 2, (0, 255, 0), thickness=10)



def line_length(start, end):
    dx = start[0] - end[0]
    dy = start[1] - end[1]
    length = math.sqrt(dx * dx + dy * dy)
    return length


def magnitude(start, end, magnitude):
    dx = start[0] - end[0]
    dy = start[1] - end[1]
    angle = math.atan2(dy, dx)
    ret_x = start[0] + math.cos(angle) * magnitude
    ret_y = start[1] + math.sin(angle) * magnitude
    return round(ret_x), round(ret_y)

def to_rgb(im):
    w, h, *_ = im.shape
    ret = np.empty((w, h, 3), dtype=np.uint8)
    ret[:, :, :] = im
    ret[:, :, 1] = ret[:, :, 2] = ret[:, :, 0]
    return ret

def display_robot_on_canvas(np_canvas_array: np.ndarray, map: NumpyMap, copy=True):
    canvas = np.copy(np_canvas_array) if copy else np_canvas_array
    raw_x = map.robot_pos.x
    raw_y = map.robot_pos.y
    px = map.x_to_px(raw_x)
    py = map.y_to_px(raw_y)

    robot_pixel_radius = int(ROBOT_RADIUS * (ROBOT_DEFAULT_RES/map.resolution))
    cv2.circle(canvas, (px, py), robot_pixel_radius, (255, 0, 0), -1)

    # Draw robot circle
    # for x, y, manhattan in generate_circle_pixels((px, py), robot_pixel_radius):
    #     canvas[y][x] = ROBOT_VALUE

    return canvas

def display_route_on_canvas(np_canvas_array: np.ndarray, map: NumpyMap, copy=True):
    canvas = np.copy(np_canvas_array) if copy else np_canvas_array
    for route in map.route:
        px = route[2][0]
        py = route[2][1]

        # Draw waypoints
        for x, y, manhattan in generate_circle_pixels((px, py), WAYPOINT_RADIUS):
            canvas[y][x] = WAYPOINT_VALUE

    return canvas


def apply_dilation(np_canvas_array, copy=True):
    canvas = np.copy(np_canvas_array) if copy else np_canvas_array


def apply_thresholding(np_canvas_array, copy=True):
    canvas = np.copy(np_canvas_array) if copy else np_canvas_array
    upper_threshold = PATH_OBSTACLE_TRESHOLD
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
    square_area_of_radius = (1 + (radius - 1) * 2) ** 2
    square_area_of_radius_min_1 = max(0, (1 + (radius - 2) * 2) ** 2)
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
            yield x, y, manhattan_dist


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
        assert self.min_x <= self.max_x
        assert self.min_y <= self.max_y

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


### UNIT TESTS
def unit_test():
    curr_map = NumpyMap(resolution=0.25)
    inp = np.array([3.5, 4.5])
    temp = curr_map.actual_to_px(inp)
    output = np.array(curr_map.px_to_actual_rect(temp).mid)
    assert (np.abs((inp - output)) <= 0.25).all()


unit_test()
