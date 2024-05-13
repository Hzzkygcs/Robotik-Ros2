
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

    def __initalize_maps(self):
        max_size = max(self.actual_width, self.actual_height)
        resolution = self.resolution_meters_per_pixel
        multiplier = self.parent_size_multiplier

        initial_value_initializer = self.initial_value
        n = 1
        next_initializer = lambda: {initial_value_initializer(): n}
        while resolution < max_size and len(self.layers) < self.max_layers:
            self.layers.append(RealGrid(self.topleft_start, self.botright_end, resolution, initial_value_initializer,
                                        self.on_index_error))
            initial_value_initializer = next_initializer
            n *= multiplier*multiplier
            resolution *= multiplier

    def __getitem__(self, item, layer=0):
        return self.layers[layer][item]

    def __setitem__(self, item, value):
        old_value = self[item]
        new_value = value

        self.layers[0][item] = new_value
        for i in range(1, len(self.layers)):
            layer = self.layers[i]
            counter_dict: dict = layer[item][old_value]
            counter_dict[old_value] -= 1
            counter_dict[new_value] = counter_dict.get(new_value, 0) + 1


class RealGrid:  # accepts real numbers
    def __init__(self, topleft_start: tuple, botright_end: tuple,
                 resolution_meters_per_pixel: float, initial_value=lambda: -1, on_index_error=index_error_handle):
        self.topleft_start = topleft_start
        self.botright_end = botright_end
        self.actual_width = botright_end[0] - topleft_start[0]
        self.actual_height = botright_end[1] - topleft_start[1]
        self.start_y_index = int(abs(self.topleft_start[1]) // resolution_meters_per_pixel)
        self.start_x_index = int(abs(self.topleft_start[0]) // resolution_meters_per_pixel)

        self.pixel_width = ceildiv(self.actual_width, resolution_meters_per_pixel) + 2
        self.pixel_height = ceildiv(self.actual_height, resolution_meters_per_pixel) + 2

        self.resolution = resolution_meters_per_pixel
        self.map = IntegerGrid(self.pixel_width, self.pixel_height, self.start_x_index, self.start_y_index, initial_value,
                               on_index_error)

    def __getitem__(self, item):
        x,y = item
        x = int(x // self.resolution)
        y = int(y // self.resolution)
        return self.map[y][x]

    def __setitem__(self, item, value):
        x,y = item
        x = int(x // self.resolution)
        y = int(y // self.resolution)
        self.map[y][x] = value




class IntegerGrid:
    # arr[0,some_y] will be put at horizontal `start_x_index`-index
    def __init__(self, width: int, height: int, start_x_index: int, start_y_index: int, initial_value, on_index_error=index_error_handle):
        self.width = width
        self.height = height
        self.start_x_index = start_x_index
        self.start_y_index = start_y_index
        self.map = [[initial_value() for _ in range(self.width)] for _ in range(self.height)]

    def __getitem__(self, item):
        x,y = item
        y += self.start_y_index
        x += self.start_x_index
        return self.map[y][x]

    def __setitem__(self, item, value):
        x,y = item
        y += self.start_y_index
        x += self.start_x_index
        self.map[y][x] = value

def ceildiv(x, y):
    return -((-x)//y)