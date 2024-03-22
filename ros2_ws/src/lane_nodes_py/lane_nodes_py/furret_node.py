import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, Odometry


class FurretNode(Node):
    def __init__(self):
        super().__init__('keeping')

        self.timer = self.create_timer(1, self.scream)
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
        self.current_location = None

    def scream(self):
        self.get_logger().info('AAAAAA')

    def test_callback(self, msg):
        # self.get_logger().info(str(msg))
        poses = msg.poses
        points_to_follow = []
        for pose in poses:
            points_to_follow += (pose.pose.position.x, pose.pose.position.y)

        self.get_logger().info(str(points_to_follow))

    def odometry_callback(self, msg):
        # self.get_logger().info(str(msg))
        location = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        direction = msg.pose.pose.orientation
        yAW = get_2d_direction_from_quaternion(direction.x, direction.y, direction.z, direction.w)


def get_2d_direction_from_quaternion(x, y, z, w):
    t0 = 2.0 * ((w * x) + (y * z))
    t1 = 1.0 - (2.0 * ((x * x) + (y * y)))
    roll = math.atan2(t0, t1)

    t0 = math.sqrt(1.0 + 2.0 * ((w * y) + (x * z)))
    t1 = math.sqrt(1.0 - 2.0 * ((w * y) + (x * z)))
    pitch = (2 * math.atan2(t0, t1)) - (math.pi / 2)

    t0 = 2.0 * ((w * z) + (x * y))
    t1 = 1.0 - 2.0 * ((z * z) + (y * y))
    yaw = math.atan2(t0, t1)

    # We kinda only care about the yaw so the other two attributes can politely f off i hate math
    return roll, pitch, yaw

def toQuart(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return x, y, z, w

def main(args=None):
    rclpy.init(args=args)

    keeping_node = FurretNode()

    rclpy.spin(keeping_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    keeping_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
