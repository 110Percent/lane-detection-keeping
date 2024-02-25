import math
import warnings

import numpy as np


def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)

    return [qx, qy, qz, qw]


def quaternion_to_euler(x, y, z, w):
    import math
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = math.atan2(t3, t4)

    return X, Y, Z


def dist(p1, p2) -> float:
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def in_front_arc(car, yaw, p2, angle_limit):
    x_diff = p2[0] - car[0]
    y_diff = p2[1] - car[1]

    angle = math.atan2(y_diff, x_diff)

    return abs(angle - yaw) < angle_limit


# Sufficiently tested to Liam's satisfaction with manual testing
def polish_path(calculated_path):
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
        degrees = np.polyfit(x_axis, y_axis, 3)
        equation = np.poly1d(degrees)

    new_points = []
    point_distance = max_val / 10
    for i in range(-1, 11):
        x_test = (i * point_distance)
        new_points += [(x_test, equation(x_test))]

    return new_points


def transform_points(points, current_location, yaw):
    transformed_points = []
    for point in points:
        new_x, new_y = (point[0] - current_location[0], point[1] - current_location[1])
        radians = -yaw
        rotated = (
            new_x * math.cos(radians) + new_y * math.sin(radians),
            -new_x * math.sin(radians) + new_y * math.cos(radians))
        transformed_points += [rotated]
    return transformed_points
