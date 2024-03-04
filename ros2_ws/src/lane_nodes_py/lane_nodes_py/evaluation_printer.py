# import matplotlib.pyplot as plt

from lane_nodes_py.evaluation_helper import quaternion_to_euler, dist, in_front_arc, polish_path, transform_points, euler_to_quaternion
import numpy as np

def print_measurement_diagrams(vehicle_path, waypoints):
    pass

def print_path_diagram(vehicle_path, waypoints):
    pass

def get_maximum_errors(vehicle_path, waypoints):
    # Get the closest two points to the cars position
    max_distance = None
    max_heading_error = None
    for vehicle_point in vehicle_path:
        closest = None

        current_location = (vehicle_point.pose.pose.position.x, vehicle_point.pose.pose.position.y)
        for i in range(len(waypoints)):
            waypoint = waypoints[i]
            if closest is None:
                closest = i
            elif dist(waypoint, current_location) < dist(waypoints[closest], current_location):
                closest = i
        front = None
        back = None
        if closest == 0:
            front = np.array(waypoints[1])
            back = np.array(waypoints[0])
        elif closest == (len(waypoints) - 1):
            front = np.array(waypoints[closest])
            back = np.array(waypoints[closest - 1])
        else:
            if dist(current_location, waypoints[closest - 1]) < dist(current_location, waypoints[closest + 1]):
                front = np.array(waypoints[closest])
                back = np.array(waypoints[closest - 1])
            else:
                front = np.array(waypoints[closest + 1])
                back = np.array(waypoints[closest])

        p3 = np.array(current_location)
        n = back - front
        v = p3 - front
        p4 = front + n * (np.dot(v, n) / np.dot(n, n))
        d = dist(p4, p3)

        if max_distance is None:
            max_distance = d
        elif d > max_distance:
            max_distance = d

        angle = np.arctan2((front[1] - back[1]), (front[0] - back[0]))

        direction = vehicle_point.pose.pose.orientation

        _, _, yaw = quaternion_to_euler(direction.x, direction.y, direction.z, direction.w)

        heading_error = angle - yaw

        if max_heading_error is None:
            max_heading_error = abs(heading_error)
        elif abs(heading_error) > max_heading_error:
            max_heading_error = abs(heading_error)
    return max_heading_error, max_distance




class orient():
    x: float
    y: float
    z: float
    w: float

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 0.0


class posit():
    x: float
    y: float

    def __init__(self):
        self.x = 0.0
        self.y = 0.0


class pose_obj2():
    orientation: orient
    position: posit

    def __init__(self):
        self.orientation = orient()
        self.position = posit()


class pose_obj():
    pose: pose_obj2

    def __init__(self):
        self.pose = pose_obj2()


class vehicle_path_point():
    pose: pose_obj

    def __init__(self):
        self.pose = pose_obj()

test_waypoints = [(0, 0), (1, 0), (2, 1), (3, 2), (3, 3), (3, 4), (3, 5)]

test_vehicle_points = []

point1 = vehicle_path_point()
point1.pose.pose.position.x = 0.5
point1.pose.pose.position.y = -2.0

point1.pose.pose.orientation.x=0.0
point1.pose.pose.orientation.y=0.0
point1.pose.pose.orientation.z=0.0
point1.pose.pose.orientation.w=1.0

test_vehicle_points += [point1]

point2 = vehicle_path_point()
point2.pose.pose.position.x = 3
point2.pose.pose.position.y = 4.5

point2.pose.pose.orientation.x=0.0
point2.pose.pose.orientation.y=0.0
point2.pose.pose.orientation.z=-0.24740395925452294
point2.pose.pose.orientation.w=0.9689124217106447

test_vehicle_points += [point2]
