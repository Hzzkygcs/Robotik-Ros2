import math
import time

import cv2
import matplotlib
import numpy as np
from matplotlib import pyplot as plt

# in coppeliasim measurement unit
ROBOT_RADIUS = 0.32



class ObstacleAvoidancePathFinder:
    def __init__(self, robot_radius=ROBOT_RADIUS):
        assert robot_radius >= 0
        self._distances = np.array([])
        self._angles = np.array([])
        self._y = np.array([])  # should be sorted based on obstacle distance (distance, not y)
        self._x = np.array([])  # should be sorted based on obstacle distance (distance, not y)
        self.groups = np.array([])  # should be sorted based on obstacle distance (distance, not y)
        self._dirty_bit = False
        self.robot_radius = robot_radius
        self.goal_point = None
        self.current_position = None
        self.middle = None
        self.reassess_avoid_obstacle_to_right_caller = CallLimiter(self.reassess_avoid_obstacle_to_right, 0.1)

        self.distance_of_being_avoided_obstacle = 0
        self.avoid_obstacle_to_right = None
        self.distance_difference_threshold_for_different_obj = 1

    @property
    def obstacle_hits(self):
        return self._distances, self._angles

    @obstacle_hits.setter
    def obstacle_hits(self, value: tuple):
        self._distances: np.ndarray = value[0]
        self._angles: np.ndarray = value[1]
        self._dirty_bit = True

    def update_points(self, current_position: tuple, goal_point: tuple):
        assert len(current_position) == 2
        assert len(goal_point) == 2
        self.goal_point = np.array([[goal_point[0], goal_point[1]]])
        self.current_position = np.array([[current_position[0], current_position[1]]])


    def _update_xy(self):
        if not self._dirty_bit:
            return
        assert self.goal_point is not None
        assert self.current_position is not None

        self.middle = None
        self._y = self._distances * np.sin(np.pi/2 + self._angles)
        self._x = self._distances * np.cos(np.pi/2 + self._angles)
        coordinates = np.stack((self._x, self._y), axis=1)
        groups = self.group_similar_values(self._distances, 0.4)
        self.groups = groups
        avoid_obstacle_to_right = self.avoid_obstacle_to_right

        intersection_distance, intersection_points = pnt2line(coordinates, self.current_position, self.goal_point)
        candidate_groups = np.unique(groups[intersection_distance < self.robot_radius])
        minimum_length = float('inf')
        selected_group = None
        for group in candidate_groups:
            index = np.where(groups == group)
            right_index = index[0][0]  # (right = negative angles = lower/first index)
            left_index = index[0][-1]  # in perspective of robot's POV (left = positive angles = higher/last index)


            if avoid_obstacle_to_right is None:
                avoid_obstacle_to_right = intersection_distance[right_index] < intersection_distance[left_index]
                left_mid_candidate = self._calculate_middle_point(coordinates[left_index], intersection_points[left_index],
                                                                False)
                right_mid_candidate = self._calculate_middle_point(coordinates[right_index], intersection_points[right_index],
                                                                True)
                left_mid_obst_dist = self._get_closest_obstacle(coordinates, left_mid_candidate)
                right_mid_obst_dist = self._get_closest_obstacle(coordinates, right_mid_candidate)
                avoid_obstacle_to_right = right_mid_obst_dist > left_mid_obst_dist
                middle_candidate =  right_mid_candidate if avoid_obstacle_to_right else left_mid_candidate
            else:
                shortest_index = right_index if avoid_obstacle_to_right else left_index
                middle_candidate = self._calculate_middle_point(coordinates[shortest_index], intersection_points[shortest_index],
                                                                avoid_obstacle_to_right)
            distance_toward_middle_point = np.linalg.norm(middle_candidate)
            if distance_toward_middle_point < minimum_length:
                self.middle = middle_candidate
                minimum_length = distance_toward_middle_point
                selected_group = group

        # self.avoid_obstacle_to_right is None check so that this code will be run only once in a while
        if self.middle is not None and self.avoid_obstacle_to_right is None:
            avoid_obstacle_to_right = self.reassess_avoid_obstacle_to_right(coordinates)
        else:
            avoid_obstacle_to_right  =self.reassess_avoid_obstacle_to_right_caller((coordinates,), avoid_obstacle_to_right)
        self.avoid_obstacle_to_right = avoid_obstacle_to_right

        if selected_group is not None:
            index = np.where(groups == selected_group)
            right_index = index[0][0]
            distance = self._distances[right_index]
            if abs(self.distance_of_being_avoided_obstacle-distance) > self.distance_difference_threshold_for_different_obj:
                self.avoid_obstacle_to_right = None  # reset preference
            self.distance_of_being_avoided_obstacle = distance

    def reassess_avoid_obstacle_to_right(self, coordinates, avoid_obstacle_to_right=None):
        if avoid_obstacle_to_right is None:
            avoid_obstacle_to_right = self.avoid_obstacle_to_right
        if avoid_obstacle_to_right is None or self.middle is None:
            return None
        tolerance = 0.02  # arbitrary number for floating point errors

        intersection_distance, _ = self._get_closest_obstacle(coordinates, self.middle)
        mid_pnt_collides_obstacle = (intersection_distance < self.robot_radius//2 - tolerance)
        if mid_pnt_collides_obstacle.any():
            print(f"Choosen middle point ({['left', 'right'][avoid_obstacle_to_right]}) will collide "
                  f"with other obstacle. Choosing the other side...")
            avoid_obstacle_to_right = not avoid_obstacle_to_right  # choose the other way around
        return avoid_obstacle_to_right

    def _get_closest_obstacle(self, coordinates, middle):
        intersection_distance1, _ = pnt2line(coordinates, self.current_position, middle)
        intersection_distance1_min = intersection_distance1.argmin()

        intersection_distance2, _ = pnt2line(coordinates, middle, self.goal_point)
        intersection_distance2_min = intersection_distance2.argmin()
        if intersection_distance1[intersection_distance1_min] < intersection_distance2[intersection_distance2_min]:
            return  intersection_distance1[intersection_distance1_min], intersection_distance1_min
        return  intersection_distance2[intersection_distance2_min], intersection_distance2_min


    def _calculate_middle_point(self, obstacle_point, obstacle_intersection_with_start_and_goal, avoid_to_right):
        obstacle_radius = self.robot_radius
        obstacle_intersection = obstacle_intersection_with_start_and_goal

        if xor(avoid_to_right, is_ccw((0,0), self.unpacked_goal_point, obstacle_point)):
            vector = obstacle_point - obstacle_intersection
        else:
            vector = obstacle_intersection - obstacle_point

        w = np.linalg.norm(self.current_position - obstacle_intersection)
        d1 = np.linalg.norm(self.current_position - obstacle_point)
        d2 = np.sqrt(d1*d1 - obstacle_radius*obstacle_radius)
        alpha1 = np.arccos(w/d1)
        alpha2 = np.arccos(d2/d1)
        new_h = w * math.tan(alpha1 + alpha2)
        new_h = min(new_h, d1)  # at most as far as distance toward obstacle + 1
        return obstacle_intersection + vector * new_h / length(vector)

    @property
    def unpacked_goal_point(self):
        # self.goal_point is usually stored as [[x, y]], so this property will give in form of [x,y]
        return self.goal_point[0]

    def group_similar_values(self, list_of_scalar: np.ndarray, tolerance: float):
        assert len(list_of_scalar.shape) == 1
        shifted_to_right = shift(list_of_scalar, 1, 0)
        differences = list_of_scalar - shifted_to_right
        condition_data = np.abs(differences) > tolerance
        groups = self.get_group_ids(condition_data, True)
        return groups

    def get_group_ids(self, arr:np.ndarray, separator):
        """ Get group IDs based on a separator value. The separator value will be grouped with the next group
        get_group_ids(np.array([False, False, False, True, False, True, False, False]), separator=True)
        returns [0, 0, 0, 1, 1, 2, 2, 2]
        """
        # Convert the input list to a numpy array
        group_ids = np.cumsum(arr == separator)
        group_ids -= group_ids[0]
        return group_ids


    @property
    def virtual_goal_point(self):
        self._update_xy()
        if self.middle is not None and np.linalg.norm(self.middle) > 0.15:
            return self.middle
        if self.goal_point is None:
            return None
        return self.goal_point[0]



class CallLimiter:
    def __init__(self, function_to_call, delay_between_calls):  # in seconds
        self.last_call = float('-inf')
        self.delay_between_calls = delay_between_calls
        self.function_to_call = function_to_call

    def __call__(self, args, return_value_if_not_callable_yet):
        if time.time() - self.last_call < self.delay_between_calls:
            return return_value_if_not_callable_yet
        self.last_call = time.time()
        return self.function_to_call(*args)




class ObstacleAvoidancePathFinderVisualizer:
    def __init__(self, obstacleAvoidancePathFinder: ObstacleAvoidancePathFinder, fig=None, ax=None):
        self.obstacleAvoidancePathFinder = obstacleAvoidancePathFinder
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

    def update_frame(self):  # goal point is in (alpha, distance) relative to current robot's orientation
        copied_canvas = self._get_new_canvas()
        self.draw_obstacles(copied_canvas)

        goal_point = virtual_goal_point = None
        if self.obstacleAvoidancePathFinder.goal_point is not None:
            px_x, px_y = self.to_pixel(*self.obstacleAvoidancePathFinder.goal_point[0])
            goal_point = (px_x, px_y)

        virtual_goal_point = goal_point
        if self.obstacleAvoidancePathFinder.virtual_goal_point is not None:
            px_x, px_y = self.to_pixel(*self.obstacleAvoidancePathFinder.virtual_goal_point)
            virtual_goal_point = (px_x, px_y)
        if virtual_goal_point is not None:
            cv2.arrowedLine(copied_canvas, self.to_pixel(0, 0), goal_point, (50, 50, 200), 1)
            cv2.arrowedLine(copied_canvas, self.to_pixel(0, 0), virtual_goal_point, (50, 50, 200), 2)
            cv2.arrowedLine(copied_canvas, virtual_goal_point, goal_point, (50, 50, 200), 2)
            cv2.circle(copied_canvas, goal_point, 2, (0, 255, 255), 2)
            cv2.circle(copied_canvas, virtual_goal_point, 2, (255, 255, 0), 2)

        self._canvas_axes.set_data(copied_canvas)
        plt.pause(0.001)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def set_new_map(self, new_map):
        self.obstacleAvoidancePathFinder = new_map
        self.update_frame()
        return self

    def set_destinations(self, destinations):
        self.destinations = destinations

    def draw_obstacles(self, copied_canvas):
        groups = self.obstacleAvoidancePathFinder.groups
        if len(groups) == 0:
            assert len(self.obstacleAvoidancePathFinder._x) == 0
            assert len(self.obstacleAvoidancePathFinder._y) == 0
            return
        groups = groups / max(1, groups.max())
        obstacles = zip(self.obstacleAvoidancePathFinder._x, self.obstacleAvoidancePathFinder._y, groups)
        cmap = matplotlib.cm.get_cmap('Spectral')


        x_start, _ = self.to_pixel(-self.obstacleAvoidancePathFinder.robot_radius, 0)
        x_end, _ = self.to_pixel(self.obstacleAvoidancePathFinder.robot_radius, 0)
        cv2.line(copied_canvas, (x_start, 0), (x_start, self.px_size[1]), (0,0,255), 2)
        cv2.line(copied_canvas, (x_end, 0), (x_end, self.px_size[1]), (0,0,255), 2)

        for x, y, group in obstacles:
            px_x, px_y = self.to_pixel(x, y)
            cv2.circle(copied_canvas, (px_x, px_y), 2, tuple_multiply(cmap(group), 255), 1)

    def to_pixel(self, x, y):
        return to_pixel(x, y, self.start[0], self.start[1], self.w, self.h, self.px_size[0], self.px_size[1])


def tuple_multiply(tuple_input, multiplier):
    ret = list(tuple_input)
    for i in range(len(ret)):
        ret[i] = ret[i] * multiplier
    return ret


def to_pixel(x, y, start_x, start_y, actual_w, actual_h, px_w, px_h):
    return round((x - start_x)/actual_w * px_w), round((1 - (y - start_y)/actual_h) * px_h)



########################## https://stackoverflow.com/a/51240898/7069108 ##########################
def dot(v, w):
    return np.sum(v * w, axis=1)

def length(v):
    if len(v.shape) >= 2:
        return np.sqrt(np.sum(v ** 2, axis=1))
    return np.sqrt(np.sum(v ** 2))

def vector(b, e):
    return e - b

def unit(v):
    mag = length(v)
    return v / from_np_list_of_scalars(mag)

def from_np_list_of_scalars(list_of_scalars, space_dimension=2):
    return np.stack((list_of_scalars,) * space_dimension, axis=1)

def distance(p0, p1):
    return length(vector(p0, p1))

def scale(v, sc):
    return v * from_np_list_of_scalars(sc)

def add(v, w):
    return v + w

def pnt2line(pnt, start, end):
    line_vec = vector(start, end)
    pnt_vec = vector(start, pnt)
    line_len = length(line_vec)
    line_unitvec = unit(line_vec)
    pnt_vec_scaled = pnt_vec / from_np_list_of_scalars(line_len)
    t = dot(line_unitvec, pnt_vec_scaled)
    t = np.clip(t, 0.0, 1.0)
    nearest = scale(line_vec, t)
    dist = distance(nearest, pnt_vec)
    nearest = add(nearest, start)
    return dist, nearest

##################################################################################################

def is_ccw(p1, p2, p3):  # https://www.geeksforgeeks.org/orientation-3-ordered-points/
    # to find the orientation of
    # an ordered triplet (p1,p2,p3)
    # function returns the following values:
    # 0 : Collinear points
    # 1 : Clockwise points
    # 2 : Counterclockwise
    val = (float(p2[1] - p1[1]) * (p3[0] - p2[0])) - \
          (float(p2[0] - p1[0]) * (p3[1] - p2[1]))
    if (val > 0):
        # Clockwise orientation
        return False
    elif (val < 0):
        # Counterclockwise orientation
        return True
    else:
        # Collinear orientation
        return True

def xor(a, b):
    return bool(a) != bool(b)

def shift(arr, num, fill_value=np.nan):  # https://stackoverflow.com/a/42642326/7069108
    """similar to np.roll(), but shifts array instead of  rotating it"""
    result = np.empty_like(arr)
    if num > 0:
        result[:num] = fill_value
        result[num:] = arr[:-num]
    elif num < 0:
        result[num:] = fill_value
        result[:num] = arr[-num:]
    else:
        result[:] = arr
    return result