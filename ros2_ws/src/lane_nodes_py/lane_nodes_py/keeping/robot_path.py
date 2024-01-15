from lane_location_interface import LaneData

from ackerman_wrapper import AckermannWrapper


def calculate_path(calculated_path: LaneData) -> list[tuple[int, int]]:
    # TODO: Implement calculation of the lane path
    print("Calculating path from lane data")
    return [(1, 1)]


def polish_path(calculated_path: list[tuple[int, int]]) -> list[tuple[int, int]]:
    # TODO: Implement the polishing of the calculated path
    print("Polishing the calculated path")
    return calculated_path


class PathData:
    # Path to follow as a list of (x,y) coordinates
    path: list[tuple[int, int]]

    # Position of the car in the form of (x,y)
    position: tuple[int, int]

    # Direction of the car in radians, with zero representing "forward" across the x-axis
    car_direction: int

    def __init__(self, size):
        self.vehicle_size = size
        self.fresh = True

    def update(self, last_message: AckermannWrapper):
        # TODO: Implement the update of the mathematical model
        print("Updating the mathematical model")
        self.fresh = False

    def is_fresh(self):
        return self.fresh

    def set_grid(self, message_data: LaneData):
        calculated_path = calculate_path(message_data)
        polished_path = polish_path(calculated_path)
        self.set_path(polished_path)
        return

    def set_path(self, data: list[tuple[int, int]]):
        self.path = data
