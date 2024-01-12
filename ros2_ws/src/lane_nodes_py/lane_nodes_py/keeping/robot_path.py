from lane_location_interface import LaneData


def calculate_path(calculated_path: LaneData) -> list[tuple[int, int]]:
    return [(1, 1)]


def polish_path(calculated_path: list[tuple[int, int]]) -> list[tuple[int, int]]:
    return [(1, 1)]


class PathData:
    # Path to follow as a list of (x,y) coordinates
    path: list[tuple[int, int]]

    # Position of the car in the form of (x,y)
    position: tuple[int, int]

    # Direction of the car in radians
    car_direction: int

    def __init__(self, size):
        self.vehicle_size = size
        self.fresh = True

    def update(self, data):
        self.fresh = True

    def is_fresh(self):
        return self.fresh

    def set_grid(self, message_data: LaneData):
        calculated_path = calculate_path(message_data)
        polished_path = polish_path(calculated_path)
        self.set_path(polished_path)
        return

    def set_path(self, data: list[tuple[int, int]]):
        self.path = data
