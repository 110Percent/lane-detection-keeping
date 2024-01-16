import math
from typing import List, Tuple

from lane_location_interface import LaneData

from ackerman_wrapper import AckermannWrapper

import numpy as np

POLYNOMIAL_DEGREE = 3

ROTATION_LIMIT = 1000

def calculate_path(calculated_path: LaneData) -> list[tuple[float, float]] | None:
    """
        Pseudo-code
        for each line in the LaneData, create a 3rd degree polynomial equation for it.
        above = lines.stream().filter(line -> line.at(0) > 0).sort(line.at(0)).findFirst()
        below = lines.stream().filter(line -> line.at(0) < 0).sort(-line.at(0)).findFirst()
        originalPointAbove = above.original()
        originalPointBelow = below.original()
        resultingPoints = (y, (above.x + below.x)/2)
    """
    directly_left = None
    directly_left_index = -1
    directly_right = None
    directly_right_index = -1
    for i in range(len(calculated_path.paths)):
        x_axis_list = []
        y_axis_list = []
        for j in range(len(calculated_path.coordinates)):
            if calculated_path.paths[i][j] != 'N':
                x_axis_list += calculated_path.coordinates[j]
                y_axis_list += calculated_path.paths[i][j]
        x_axis = np.array(x_axis_list)
        y_axis = np.array(y_axis_list)

        z = np.polyfit(x_axis, y_axis, POLYNOMIAL_DEGREE)
        if directly_left is None | (z[POLYNOMIAL_DEGREE] > 0 & z[POLYNOMIAL_DEGREE] < directly_left[POLYNOMIAL_DEGREE]):
            directly_left = z
            directly_left_index = i

        if directly_right is None | (
                z[POLYNOMIAL_DEGREE] < 0 & z[POLYNOMIAL_DEGREE] > directly_right[POLYNOMIAL_DEGREE]):
            directly_right = z
            directly_right_index = i

    if directly_right_index == -1 & directly_left_index == -1:
        print("Error: no lines were found that could be used")
        return None

    elif directly_right_index == -1:
        x_axis_list = []
        y_axis_list = []
        for j in range(len(calculated_path.coordinates)):
            if calculated_path.paths[directly_left_index][j] != 'N':
                x_axis_list += calculated_path.coordinates[j]
                y_axis_list += calculated_path.paths[directly_left_index][j]
        return list(map(lambda x, y: (x, y), x_axis_list, y_axis_list))

    elif directly_left_index == -1:
        x_axis_list = []
        y_axis_list = []
        for j in range(len(calculated_path.coordinates)):
            if calculated_path.paths[directly_right_index][j] != 'N':
                x_axis_list += calculated_path.coordinates[j]
                y_axis_list += calculated_path.paths[directly_right_index][j]
        return list(map(lambda x, y: (x, y), x_axis_list, y_axis_list))

    else:
        x_axis_list = []
        y_axis_list = []
        for j in range(len(calculated_path.coordinates)):
            if ((calculated_path.paths[directly_right_index][j] != 'N') &
                    (calculated_path.paths[directly_left_index][j] != 'N')):
                x_axis_list += calculated_path.coordinates[j]
                y_axis_list += (calculated_path.paths[directly_right_index][j] +
                                calculated_path.paths[directly_right_index][j]) / 2
        return list(map(lambda x, y: (x, y), x_axis_list, y_axis_list))


def polish_path(calculated_path: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """
    Takes a list path in the form of [(x, y)] and formats it to include 11 evenly periodic points
    """
    print("Polishing the calculated path")
    x_vals = [i[0] for i in calculated_path]
    y_vals = [i[1] for i in calculated_path]
    max_val = max(x_vals)

    x_axis = np.array(x_vals)
    y_axis = np.array(y_vals)

    degrees = np.polyfit(x_axis, y_axis, POLYNOMIAL_DEGREE)
    equation = np.poly1d(degrees)

    new_points = []
    point_distance = max_val / 10
    for i in range(-1, 11):
        x_test = (i * point_distance)
        new_points += [x_test, equation(x_test)]

    return new_points

def intersection(L1, L2):
    """
    Source: https://stackoverflow.com/a/20679579
    """

    D  = L1[0] * L2[1] - L1[1] * L2[0]
    Dx = L1[2] * L2[1] - L1[1] * L2[2]
    Dy = L1[0] * L2[2] - L1[2] * L2[0]
    if D != 0:
        x = Dx / D
        y = Dy / D
        return x,y
    else:
        return False

class PathData:
    # Path to follow as a list of (x,y) coordinates
    path: list[tuple[float, float]]

    # Position of the car in the form of (x,y)
    position: tuple[float, float]

    # Direction of the car in radians, with zero representing "forward" across the x-axis
    # Positive direction means counter-clockwise
    car_direction: int

    # Size of the vehicle in meters
    vehicle_size: float

    def __init__(self, size):
        self.vehicle_size = size
        self.fresh = True

    def update(self, last_message: AckermannWrapper, time):
        # TODO: Implement the update of the mathematical model
        """
        Step 1: Figure out the center of pivot from the size of the vehicle and the turning angle
        Step 2: Determine how far the vehicle travels
        Step 3: Determine how many radians the vehicle has moved
        Step 4: Determine vehicle's new position
        :param last_message:
        :return:
        """

        # Step 1
        fx = self.position[0]
        fy = self.position[1]
        bx = math.cos(math.pi + self.car_direction) * self.vehicle_size + fx
        by = math.sin(math.pi + self.car_direction) * self.vehicle_size + fy

        fp = self.car_direction + last_message.steering_angle - (math.pi/2)
        fm = math.tan(fp)
        fb = fy - (fm * fx)

        bp = self.car_direction - (math.pi/2)
        bm = math.tan(bp)
        bb = by - (bm * bx)

        intersection_result = intersection((-fm, 1, fb), (-bm, 1, bb))
        if intersection_result:
            # Step 2
            distance_traveled = time * last_message.speed

            radius = math.dist(intersection_result, self.position)

            if radius > ROTATION_LIMIT:
                nfx = math.cos(self.car_direction) * distance_traveled + fx
                nfy = math.sin(self.car_direction) * distance_traveled + fy
                self.position = (nfx, nfy)

            else:
                radians_traversed = distance_traveled/radius




        else:
            print("No determinant found")


        print("Updating the mathematical model")
        self.fresh = False

    def is_fresh(self):
        return self.fresh

    def set_grid(self, message_data: LaneData):
        calculated_path = calculate_path(message_data)
        polished_path = polish_path(calculated_path)
        self.set_path(polished_path)

    def set_path(self, data: list[tuple[float, float]]):
        self.path = data
        self.position = (0, 0)
        self.car_direction = 0
