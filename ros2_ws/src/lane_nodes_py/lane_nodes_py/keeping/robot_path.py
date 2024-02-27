import math
from typing import List, Tuple

from lane_nodes_py.keeping.ackerman_wrapper import AckermannWrapper

import numpy as np

import warnings

POLYNOMIAL_DEGREE = 3

ROTATION_LIMIT = 1000


# Sufficiently tested to Liam's satisfaction with manual testing
def calculate_path(calculated_path) :
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
                x_axis_list += [calculated_path.coordinates[j]]
                y_axis_list += [calculated_path.paths[i][j]]
        x_axis = np.array(x_axis_list)
        y_axis = np.array(y_axis_list)

        with warnings.catch_warnings():
            warnings.simplefilter('ignore', np.RankWarning)
            z = np.polyfit(x_axis, y_axis, POLYNOMIAL_DEGREE)

        print("Index " + str(i) + ": " + str(z))
        print("y-intercept: " + str(z[POLYNOMIAL_DEGREE]))
        if (directly_left is None and 0 <= z[POLYNOMIAL_DEGREE]) or (
                0 <= z[POLYNOMIAL_DEGREE] < directly_left[POLYNOMIAL_DEGREE]):
            print("Adding above")
            directly_left = z
            directly_left_index = i

        if (directly_right is None and 0 > z[POLYNOMIAL_DEGREE]) or (
                0 > z[POLYNOMIAL_DEGREE] > directly_right[POLYNOMIAL_DEGREE]):
            print("Adding below")
            directly_right = z
            directly_right_index = i

    print("Above index: " + str(directly_left_index))
    print("Below index: " + str(directly_right_index))

    if directly_right_index == -1 and directly_left_index == -1:
        print("Error: no lines were found that could be used")
        return None

    elif directly_right_index == -1:
        x_axis_list = []
        y_axis_list = []
        for j in range(len(calculated_path.coordinates)):
            if calculated_path.paths[directly_left_index][j] != 'N':
                x_axis_list += [calculated_path.coordinates[j]]
                y_axis_list += [calculated_path.paths[directly_left_index][j]]
        return list(map(lambda x, y: (x, y), x_axis_list, y_axis_list))

    elif directly_left_index == -1:
        x_axis_list = []
        y_axis_list = []
        for j in range(len(calculated_path.coordinates)):
            if calculated_path.paths[directly_right_index][j] != 'N':
                x_axis_list += [calculated_path.coordinates[j]]
                y_axis_list += [calculated_path.paths[directly_right_index][j]]
        return list(map(lambda x, y: (x, y), x_axis_list, y_axis_list))

    else:
        print("setting path to between the two lines")
        x_axis_list = []
        y_axis_list = []
        for j in range(len(calculated_path.coordinates)):
            if ((calculated_path.paths[directly_right_index][j] != 'N') and
                    (calculated_path.paths[directly_left_index][j] != 'N')):
                x_axis_list += [calculated_path.coordinates[j]]
                y_axis_list += [(calculated_path.paths[directly_right_index][j] +
                                 calculated_path.paths[directly_left_index][j]) / 2]
        return list(map(lambda x, y: (x, y), x_axis_list, y_axis_list))


# Sufficiently tested to Liam's satisfaction with manual testing
def polish_path(calculated_path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """
    Takes a list path in the form of [(x, y)] and formats it to include 11 evenly periodic points
    """
    print("Polishing the calculated path")
    x_vals = [i[0] for i in calculated_path]
    y_vals = [i[1] for i in calculated_path]
    max_val = max(x_vals)

    x_axis = np.array(x_vals)
    y_axis = np.array(y_vals)

    with warnings.catch_warnings():
        warnings.simplefilter('ignore', np.RankWarning)
        degrees = np.polyfit(x_axis, y_axis, POLYNOMIAL_DEGREE)
        equation = np.poly1d(degrees)

    new_points = []
    point_distance = max_val / 10
    for i in range(-1, 11):
        x_test = (i * point_distance)
        new_points += [(x_test, equation(x_test))]

    return new_points


# No plan to test since it's not my code
def intersection(L1, L2):
    """
    Source: https://stackoverflow.com/a/20679579
    """

    D = L1[0] * L2[1] - L1[1] * L2[0]
    Dx = L1[2] * L2[1] - L1[1] * L2[2]
    Dy = L1[0] * L2[2] - L1[2] * L2[0]
    if D != 0:
        x = Dx / D
        y = Dy / D
        return x, y
    else:
        return False


class PathData:
    # Path to follow as a list of (x,y) coordinates
    path: List[Tuple[float, float]]

    # Position of the car in the form of (x,y)
    position: Tuple[float, float]

    # Direction of the car in radians, with zero representing "forward" across the x-axis
    # Positive direction means counter-clockwise
    car_direction: int

    # Size of the vehicle in meters
    vehicle_size: float

    # History for a given set of data
    history: List[Tuple[float, float]]

    def __init__(self, size):
        self.vehicle_size = size
        self.fresh = True
        self.history = [(0, 0)]

    def update(self, last_message: AckermannWrapper, time):
        # TODO: Break down the method for testing
        """
        Step 1: Figure out the center of pivot from the size of the vehicle and the turning angle
        Step 2: Determine how far the vehicle travels
        Step 3: Determine how many radians the vehicle has moved
        Step 4: Determine vehicle's new position
        :param time:
        :param last_message:
        :return:
        """

        # Step 1
        fx = self.position[0]
        fy = self.position[1]
        bx = math.cos(math.pi + self.car_direction) * self.vehicle_size + fx
        by = math.sin(math.pi + self.car_direction) * self.vehicle_size + fy

        fp = self.car_direction + last_message.steering_angle - (math.pi / 2)
        fm = math.tan(fp)
        fb = fy - (fm * fx)

        bp = self.car_direction - (math.pi / 2)
        bm = math.tan(bp)
        bb = by - (bm * bx)

        intersection_result = intersection((-fm, 1, fb), (-bm, 1, bb))

        print("Rotation point: " + str(intersection_result))

        # Step 2
        distance_traveled = time * last_message.speed

        if intersection_result:
            radius = math.dist(intersection_result, self.position)

            if radius > ROTATION_LIMIT:
                nfx = math.cos(self.car_direction) * distance_traveled + fx
                nfy = math.sin(self.car_direction) * distance_traveled + fy
                self.position = (nfx, nfy)

            else:
                # Steps 3 and 4
                theta = distance_traveled / radius
                if np.cross((self.position[0] - intersection_result[0], self.position[1] - intersection_result[1]),
                            (math.cos(self.car_direction), math.sin(self.car_direction))) < 0:
                    theta = -theta
                a = intersection_result[0]
                b = intersection_result[1]
                x = self.position[0]
                y = self.position[1]
                # Source for equation: https://math.stackexchange.com/a/4434146
                nfx, nfy = a + (x - a) * math.cos(theta) - (y - b) * math.sin(theta), b + (x - a) * math.sin(theta) + (
                        y - b) * math.cos(theta)
                self.position = (nfx, nfy)
                self.car_direction = self.car_direction + theta

        else:
            nfx = math.cos(self.car_direction) * distance_traveled + fx
            nfy = math.sin(self.car_direction) * distance_traveled + fy
            self.position = (nfx, nfy)

        print("Updating the mathematical model")
        self.history += [self.position]
        self.fresh = False

    def is_fresh(self):
        return self.fresh

    def set_grid(self, message_data):
        calculated_path = calculate_path(message_data)
        polished_path = polish_path(calculated_path)
        self.set_path(polished_path)

    def set_path(self, data: List[Tuple[float, float]]):
        self.path = data
        self.position = (0, 0)
        self.car_direction = 0
        self.fresh = True
        self.history = [(0, 0)]

    # Sufficiently tested to Liam's satisfaction with manual testing
    def get_distance_to_line(self):
        # Source: https://stackoverflow.com/a/39840218

        p1 = None
        p2 = None

        # Get the closest two points to the cars position
        for point in self.path:
            if p1 is None:
                p1 = point
                continue
            if dist(point, self.position) < dist(p1, self.position):
                p2 = p1
                p1 = point
                continue
            if p2 is None or dist(point, self.position) < dist(p2, self.position):
                p2 = point
                continue

        p1 = np.array(p1)
        p2 = np.array(p2)
        p3 = np.array(self.position)
        n = p2 - p1
        v = p3 - p1
        p4 = p1 + n * (np.dot(v, n) / np.dot(n, n))
        d = dist(p4, p3)

        # This section can actually be reached but the intellisense is very highly regarded
        # noinspection PyUnreachableCode
        if np.cross((self.position[0] - p4[0], self.position[1] - p4[1]),
                    (math.cos(self.car_direction), math.sin(self.car_direction))) < 0:
            d = -d

        return d

    def get_heading_offset(self):
        p1 = None
        p2 = None

        # Get the closest two points to the cars position
        for point in self.path:
            if p1 is None:
                p1 = point
                continue
            if dist(point, self.position) < dist(p1, self.position):
                p2 = p1
                p1 = point
                continue
            if p2 is None or dist(point, self.position) < dist(p2, self.position):
                p2 = point
                continue

        p1 = np.array(p1)
        p2 = np.array(p2)

        if p1[0] > p2[0]:
            temp = p1
            p1 = p2
            p2 = temp

        angle = np.arctan((p2[1] - p1[1]) / (p2[0] - p1[0]))
        return angle - self.car_direction


# Sufficiently tested to Liam's satisfaction with manual testing
def dist(p1, p2) -> float:
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
