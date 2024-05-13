class OccupancyGrid:
    def __init__(self, topleft_start: tuple, botright_end: tuple,
                 resolution_meters_per_pixel: float, initial_value=lambda: -1):
        self.topleft_start = topleft_start
        self.botright_end = botright_end
        self.actual_width = botright_end[0] - topleft_start[0]
        self.actual_height = botright_end[1] - topleft_start[1]
        self.pixel_width = ceildiv(self.actual_width, resolution_meters_per_pixel) + 2
        self.pixel_height = ceildiv(self.actual_height, resolution_meters_per_pixel) + 2

        self.resolution = resolution_meters_per_pixel
        self.map = Map()

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




class Map:
    # arr[0,some_y] will be put at horizontal `start_x_index`-index
    def __init__(self, width: int, height: int, start_x_index, end_x_index, initial_value):
        self.width = width
        self.height = height
        self.start_x_index = start_x_index
        self.start_y_index = start_x_index
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