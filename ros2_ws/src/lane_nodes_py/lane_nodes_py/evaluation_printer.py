import matplotlib.pyplot as plt

from lane_nodes_py.evaluation_helper import quaternion_to_euler, dist, euler_to_quaternion
import numpy as np


def print_measurement_diagrams(vehicle_path, waypoints):
    pass


def print_path_diagram(vehicle_path, waypoints):
    vehicle_path_points = []
    for p in vehicle_path:
        vehicle_path_points += [(p.pose.pose.position.x, p.pose.pose.position.y)]
    plt.plot(list(zip(*waypoints))[0], list(zip(*waypoints))[1])
    plt.plot(list(zip(*vehicle_path_points))[0], list(zip(*vehicle_path_points))[1])
    plt.show()


def get_maximum_errors(vehicle_path, waypoints, logga):
    # Get the closest two points to the cars position
    max_distance = None
    max_heading_error = None
    for vehicle_point in vehicle_path:
        closest = None

        current_location = (vehicle_point.pose.pose.position.x, vehicle_point.pose.pose.position.y,
                            vehicle_point.pose.pose.position.z)
        for i in range(len(waypoints)):
            waypoint = waypoints[i]
            if closest is None:
                closest = i
            elif (dist(waypoint, current_location) < dist(waypoints[closest], current_location) and
                  abs(waypoint[2] - current_location[2]) < 5):
                closest = i
        front = None
        back = None
        if closest == 0:
            front = np.array((waypoints[1][0], waypoints[1][1]))
            back = np.array((waypoints[0][0], waypoints[0][1]))
        elif closest == (len(waypoints) - 1):
            front = np.array((waypoints[closest][0], waypoints[closest][1]))
            back = np.array((waypoints[closest-1][0], waypoints[closest-1][1]))
        else:
            if dist(current_location, waypoints[closest - 1]) < dist(current_location, waypoints[closest + 1]):
                front = np.array((waypoints[closest][0], waypoints[closest][1]))
                back = np.array((waypoints[closest-1][0], waypoints[closest-1][1]))
            else:
                front = np.array((waypoints[closest+1][0], waypoints[closest+1][1]))
                back = np.array((waypoints[closest][0], waypoints[closest][1]))

        p3 = np.array((current_location[0], current_location[1]))
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
        while heading_error > np.pi:
            heading_error = heading_error - (2 * np.pi)
        while heading_error < -np.pi:
            heading_error = heading_error + (2 * np.pi)


        if max_heading_error is None:
            logga.info('New max Heading Error of ' + str(heading_error) + 'at the point of ' + str(current_location))
            logga.info('Back point: ' + str(back) + ', front point: ' + str(front) + ', Yaw' + str(yaw))
            max_heading_error = abs(heading_error)
        elif abs(heading_error) > max_heading_error:
            logga.info('New max Heading Error of ' + str(heading_error) + 'at the point of ' + str(current_location))
            logga.info('Back point: ' + str(back) + ', front point: ' + str(front) + ', Yaw' + str(yaw))
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


class stamp():
    sec: float
    nanosec: float

    def __init(self):
        self.sec = 0.0
        self.nanosec = 0.0


class heada():
    stamp: stamp

    def __init__(self):
        self.stamp = stamp()


class vehicle_path_point():
    pose: pose_obj
    header: heada

    def __init__(self):
        self.pose = pose_obj()


test_waypoints = [(0, 0), (1, 0), (2, 1), (3, 2), (3, 3), (3, 4), (3, 5), (0, 5)]

test_vehicle_points = []

point1 = vehicle_path_point()
point1.pose.pose.position.x = 0.5
point1.pose.pose.position.y = -2.0

point1.pose.pose.orientation.x = 0.0
point1.pose.pose.orientation.y = 0.0
point1.pose.pose.orientation.z = 0.0
point1.pose.pose.orientation.w = 1.0

test_vehicle_points += [point1]

point2 = vehicle_path_point()
point2.pose.pose.position.x = 3
point2.pose.pose.position.y = 4.5

point2.pose.pose.orientation.x = 0.0
point2.pose.pose.orientation.y = 0.0
point2.pose.pose.orientation.z = 0.0
point2.pose.pose.orientation.w = 1.0
test_vehicle_points += [point2]

point3 = vehicle_path_point()
point3.pose.pose.position.x = 1.5
point3.pose.pose.position.y = 5

point3.pose.pose.orientation.x = 0.0
point3.pose.pose.orientation.y = 0.0
point3.pose.pose.orientation.z = 0.0
point3.pose.pose.orientation.w = 1.0

test_vehicle_points += [point3]
