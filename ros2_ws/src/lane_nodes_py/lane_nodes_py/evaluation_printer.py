import matplotlib.pyplot as plt

from lane_nodes_py.evaluation_helper import quaternion_to_euler, dist, euler_to_quaternion
import numpy as np


class DataAnalyzer():
    lateral_error = []

    heading_error = []

    def __init__(self, vehicle_path, waypoints):

        self.vehicle_path = vehicle_path
        self.waypoints = waypoints

        self.calculate_error_over_time()

    def calculate_error_over_time(self):

        for vehicle_point in self.vehicle_path:
            closest = None

            current_location = (vehicle_point.pose.pose.position.x, vehicle_point.pose.pose.position.y,
                                vehicle_point.pose.pose.position.z)
            for i in range(len(self.waypoints)):
                waypoint = self.waypoints[i]
                if closest is None:
                    closest = i
                elif (dist(waypoint, current_location) < dist(self.waypoints[closest], current_location) and
                      abs(waypoint[2] - current_location[2]) < 5):
                    closest = i
            front = None
            back = None
            if closest == 0:
                front = np.array((self.waypoints[1][0], self.waypoints[1][1]))
                back = np.array((self.waypoints[0][0], self.waypoints[0][1]))
            elif closest == (len(self.waypoints) - 1):
                front = np.array((self.waypoints[closest][0], self.waypoints[closest][1]))
                back = np.array((self.waypoints[closest - 1][0], self.waypoints[closest - 1][1]))
            else:
                if dist(current_location, self.waypoints[closest - 1]) < dist(current_location,
                                                                              self.waypoints[closest + 1]):
                    front = np.array((self.waypoints[closest][0], self.waypoints[closest][1]))
                    back = np.array((self.waypoints[closest - 1][0], self.waypoints[closest - 1][1]))
                else:
                    front = np.array((self.waypoints[closest + 1][0], self.waypoints[closest + 1][1]))
                    back = np.array((self.waypoints[closest][0], self.waypoints[closest][1]))

            p3 = np.array((current_location[0], current_location[1]))
            n = back - front
            v = p3 - front
            p4 = front + n * (np.dot(v, n) / np.dot(n, n))
            d = dist(p4, p3)

            # TODO: Figure out a way to differentiate between which side of the waypoint the lad is on

            angle = np.arctan2((front[1] - back[1]), (front[0] - back[0]))

            direction = vehicle_point.pose.pose.orientation

            _, _, yaw = quaternion_to_euler(direction.x, direction.y, direction.z, direction.w)

            heading_error = angle - yaw
            while heading_error > np.pi:
                heading_error = heading_error - (2 * np.pi)
            while heading_error < -np.pi:
                heading_error = heading_error + (2 * np.pi)
            self.heading_error += [
                (vehicle_point.header.stamp.sec + (0.000000001 * vehicle_point.header.stamp.nanosec), heading_error)]

            self.lateral_error += [
                (vehicle_point.header.stamp.sec + (0.000000001 * vehicle_point.header.stamp.nanosec), d)]

    def print_measurement_diagrams(self, k, v):
        plt.plot(list(zip(*self.lateral_error))[0], list(zip(*self.lateral_error))[1])
        plt.title('Lateral Error with k='+str(k)+' and base velocity of '+str(v)+'m/s Over Time')
        plt.xlabel('Time(seconds)')
        plt.ylabel('Lateral Error')
        plt.show()
        plt.savefig('Lateral_Error.jpg')

        plt.plot(list(zip(*self.heading_error))[0], list(zip(*self.heading_error))[1])
        plt.title('Heading Error with k='+str(k)+' and base velocity of '+str(v)+'m/s Over Time')
        plt.xlabel('Time(seconds)')
        plt.ylabel('Heading Error')
        plt.show()
        plt.savefig('Heading_Error.jpg')


    def print_path_diagram(self):
        vehicle_path_points = []
        for p in self.vehicle_path:
            vehicle_path_points += [(p.pose.pose.position.x, p.pose.pose.position.y)]
        plt.plot(list(zip(*self.waypoints))[0], list(zip(*self.waypoints))[1], label='Waypoint Path')
        plt.plot(list(zip(*vehicle_path_points))[0], list(zip(*vehicle_path_points))[1], label='Vehicle Path')
        plt.tick_params(
            axis='both',  # changes apply to the x-axis
            which='both',  # both major and minor ticks are affected
            bottom=False,  # ticks along the bottom edge are off
            top=False,  # ticks along the top edge are off
            labelbottom=False)  # labels along the bottom edge are off
        plt.show()
        plt.savefig('Vehicle_Path.jpg')

    def get_maximum_errors(self):
        # Get the closest two points to the cars position
        heading_errors = list(zip(*self.heading_error))[1]

        lateral_errors = list(zip(*self.lateral_error))[1]

        max_distance = max(lateral_errors)

        max_heading = max(heading_errors)
        min_heading = min(heading_errors)
        max_heading_error = max_heading if (abs(min_heading) < max_heading) else abs(min_heading)

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
