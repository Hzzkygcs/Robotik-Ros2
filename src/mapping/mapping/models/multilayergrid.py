from __future__ import annotations
import numpy as np

index_error_handle = lambda x,y: IndexError(f"Index out of bound when accessing ({x},{y})")

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
        next_initializer = lambda: {self.initial_value(): n}
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

        self.layers[0].set(x,y, new_value)
        for i in range(1, len(self.layers)):
            layer = self.layers[i]
            counter_dict: dict = layer.get(x,y)
            counter_dict[old_value] -= 1
            counter_dict[new_value] = counter_dict.get(new_value, 0) + 1

    def ravel(self):
        return self.layers[0].ravel()


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

        self.resolution = resolution_meters_per_pixel
        self.map = IntegerGrid(self.pixel_width, self.pixel_height, self.start_x_index, self.start_y_index, initial_value,
                               on_index_error)

    def get(self, x,y):
        x = int(x // self.resolution)
        y = int(y // self.resolution)
        return self.map[x,y]

    def set(self, x,y, value):
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
        self.map = [initial_value() for _ in range(self.width) for _ in range(self.height)]

    def __getitem__(self, item):
        x,y = item
        y += self.start_y_index
        x += self.start_x_index
        index = self.width * y + x
        return self.map[index]

    def __setitem__(self, item, value):
        x,y = item
        y += self.start_y_index
        x += self.start_x_index
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
    for i in range(1,1+step_count):
        x = start_x + dx*i/step_count
        y = start_y + dy*i/step_count
        yield i, x,y

