import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, Odometry

from lane_nodes_py.evaluation_helper import quaternion_to_euler, dist, in_front_arc, polish_path

from lane_interfaces.msg import LaneLocation2


class EvaluationKeeping(Node):
    FREQUENCY = 15

    vehicle_path_total = []

    waypoints = []

    current_location = None
    yaw = None

    lane_pair: LaneLocation2 = None

    def __init__(self):
        super().__init__('evaluation_keeping')

        self.timer = self.create_timer(1 / self.FREQUENCY, self.publish_latest_path)
        self.waypoint_subscription = self.create_subscription(
            Path,
            '/carla/ego_vehicle/waypoints',
            self.test_callback,
            10)

        self.location_subscription = self.create_subscription(
            Odometry,
            '/carla/ego_vehicle/odometry',
            self.odometry_callback,
            10)

        self.lane_publisher_ = self.create_publisher(LaneLocation2, "lane_location_data_eval", 10)

    def publish_latest_path(self):
        self.get_logger().info('Publishing the latest generated path')
        msg = self.lane_pair
        if msg is None:
            return
        self.vehicle_path_total += [self.current_location]
        self.lane_publisher_.publish(msg)

    def test_callback(self, msg):
        # self.get_logger().info(str(msg))
        poses = msg.poses
        waypoints = []
        for pose in poses:
            waypoints += [(pose.pose.position.x, pose.pose.position.y)]

        self.get_logger().info(str(waypoints))

    def odometry_callback(self, msg):
        # self.get_logger().info(str(msg))
        self.current_location = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        direction = msg.pose.pose.orientation
        self.yaw = get_2d_direction_from_quaternion(direction.x, direction.y, direction.z, direction.w)

        points_in_range = self.find_points_in_range()

        transformed_points_to_front_view = self.transform_points(points_in_range)

        if not transformed_points_to_front_view:
            self.get_logger().info("No waypoint points found?")
            return

        created_path = polish_path(transformed_points_to_front_view)

        y_upper = [i[1] + 2 for i in created_path]
        y_lower = [i[1] - 2 for i in created_path]
        x_list = [i[0] for i in created_path]

        msg = LaneLocation2()
        msg.x_vals = x_list
        msg.y_vals1 = y_upper
        msg.y_vals2 = y_lower

        self.lane_pair = msg


    def transform_points(self, points):
        transformed_points = []
        for point in points:
            new_x,  new_y= (point[0] - self.current_location[0], point[1] - self.current_location[1])
            radians = -self.yaw
            rotated = new_x * math.cos(radians) + new_y * math.sin(radians), -new_x * math.sin(radians) + new_y * math.cos(radians)
            transformed_points += [rotated]
        return transformed_points

    def find_points_in_range(self):
        points_in_range = []
        for point in self.waypoints:
            if dist(self.current_location, point) > 20:
                continue
            if in_front_arc(self.current_location, self.yaw, point, 30):
                points_in_range += [point]
        return points_in_range


def get_2d_direction_from_quaternion(x, y, z, w):
    roll, pitch, yaw = quaternion_to_euler(x, y, z, w)

    # We kinda only care about the yaw so the other two attributes can politely f off i hate math
    return yaw


def main(args=None):
    rclpy.init(args=args)

    eval_keeping_node = EvaluationKeeping()

    rclpy.spin(eval_keeping_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    eval_keeping_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
