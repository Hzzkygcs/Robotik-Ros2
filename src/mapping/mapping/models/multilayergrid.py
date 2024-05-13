from __future__ import annotations
import numpy as np

def index_error_handle(x, y):
    raise IndexError(f"Index out of bound when accessing ({x},{y})")

THRESHOLD = 30  # should be the same as the one in grid_visualizer.py


# for performance issue, current optimization limits this class to only store integers between -1 and 100
class MultilayerGrid:
    def __init__(self, topleft_start: tuple, botright_end: tuple,
                 resolution_meters_per_pixel: float, initial_value=lambda: -1,
                 parent_size_multiplier:int=3, max_layers=float('inf'), on_index_error=index_error_handle):
        self.topleft_start = topleft_start
        self.botright_end = botright_end
        self.actual_width = botright_end[0] - topleft_start[0]
        self.actual_height = botright_end[1] - topleft_start[1]

        self.resolution_meters_per_pixel = resolution_meters_per_pixel  # smallest resolution
        self.initial_value = initial_value
        self.parent_size_multiplier = parent_size_multiplier
        self.layers = []
        self.max_layers = max_layers
        self.on_index_error = on_index_error
        self.__initalize_maps()

    def get_layer(self, index) -> RealGrid:
        return self.layers[index]

    def __initalize_maps(self):
        max_size = max(self.actual_width, self.actual_height)
        resolution = self.resolution_meters_per_pixel
        multiplier = self.parent_size_multiplier

        initial_value_initializer = self.initial_value
        n = 1
        next_initializer = lambda: get_counter_array_index_as_array(self.initial_value())
        while resolution < max_size and len(self.layers) < self.max_layers:
            self.layers.append(RealGrid(self.topleft_start, self.botright_end, resolution, initial_value_initializer,
                                        self.on_index_error))
            initial_value_initializer = next_initializer
            n *= multiplier*multiplier
            resolution *= multiplier

    def get(self, x, y, layer=0):
        return self.layers[layer].get(x, y)

    def set(self, x,y, value):
        old_value = self.get(x,y)
        new_value = value
        old_value_above_threshold = (old_value >= THRESHOLD)
        new_value_above_threshold = (old_value >= THRESHOLD)

        self.layers[0].set(x,y, new_value)
        for i in range(1, len(self.layers)):
            layer = self.layers[i]
            counter_array = layer.get(x,y)
            old_value_index = get_counter_array_index(old_value)
            new_value_index = get_counter_array_index(new_value)

            counter_array[old_value_index] = max(0, counter_array[old_value_index] - 1)  # possibly key not found due to precision error
            counter_array[new_value_index] = counter_array[new_value_index] + 1

    def ravel(self):
        return self.layers[0].ravel()


def get_counter_array_index(value):
    if value == -1:
        return 0
    if value < THRESHOLD:
        return 1
    return 2


def get_counter_array_index_as_array(value):
    index = get_counter_array_index(value)
    ret = np.array([0, 0, 0])
    ret[index] += 1
    return ret



class RealGrid:  # accepts real numbers
    def __init__(self, topleft_start: tuple, botright_end: tuple,
                 resolution_meters_per_pixel: float, initial_value=lambda: -1, on_index_error=index_error_handle):
        self.topleft_start = topleft_start
        self.botright_end = botright_end
        self.actual_width = botright_end[0] - topleft_start[0]
        self.actual_height = botright_end[1] - topleft_start[1]
        self.start_y_index = int(abs(self.topleft_start[1]) // resolution_meters_per_pixel)
        self.start_x_index = int(abs(self.topleft_start[0]) // resolution_meters_per_pixel)

        self.pixel_width = int(ceildiv(self.actual_width, resolution_meters_per_pixel)) + 2
        self.pixel_height = int(ceildiv(self.actual_height, resolution_meters_per_pixel)) + 2
        self.index_error_handle = index_error_handle
        self.resolution = resolution_meters_per_pixel
        self.map = IntegerGrid(self.pixel_width, self.pixel_height, self.start_x_index, self.start_y_index, initial_value,
                               on_index_error)

    def get(self, x,y):
        if not (self.topleft_start[0] < x < self.botright_end[0]) or not (self.topleft_start[1] < y < self.botright_end[1]):
            return self.index_error_handle(x, y)
        x = int(x // self.resolution)
        y = int(y // self.resolution)
        return self.map[x,y]

    def set(self, x,y, value):
        if not (self.topleft_start[0] < x < self.botright_end[0]) or not (self.topleft_start[1] < y < self.botright_end[1]):
            return self.index_error_handle(x, y)
        x = int(x // self.resolution)
        y = int(y // self.resolution)
        self.map[x,y] = value

    def ravel(self):
        return self.map.ravel()



class IntegerGrid:
    # arr[0,some_y] will be put at horizontal `start_x_index`-index
    def __init__(self, width: int, height: int, start_x_index: int, start_y_index: int, initial_value, on_index_error=index_error_handle):
        self.width = width
        self.height = height
        self.start_x_index = start_x_index
        self.start_y_index = start_y_index
        self.on_index_error = on_index_error
        self.map = [initial_value() for _ in range(self.width) for _ in range(self.height)]

    def __getitem__(self, item):
        x,y = item
        y += self.start_y_index
        x += self.start_x_index
        if not (0 <= x < self.width) or not (0 <= y < self.height):
            self.on_index_error(*item)
        index = self.width * y + x
        if not (0 <= index < len(self.map)):
            print(index, len(self.map))
        return self.map[index]

    def __setitem__(self, item, value):
        x,y = item
        y += self.start_y_index
        x += self.start_x_index
        if not (0 <= x < self.width) or not (0 <= y < self.height):
            self.on_index_error(*item)
        index = self.width * y + x
        self.map[index] = value

    def ravel(self):
        return self.map

def ceildiv(x, y):
    return -((-x)//y)


def range_2d(start, stop, step):  # like python's Range() but for 2d coords
    start_x = start[0]
    start_y = start[1]
    dx = stop[0] - start[0]
    dy = stop[1] - start[1]
    length = (dx**2 + dy**2)**0.5
    step_count = int(length // step)
    for i in range(0,step_count):
        x = start_x + dx*i/step_count
        y = start_y + dy*i/step_count
        yield i, x,y

