import rclpy
from rclpy.node import Node

from lane_interfaces.msg import LaneLocation

from ackermann_msgs.msg import AckermannDrive

from ros2_ws.src.lane_nodes_py.lane_nodes_py.keeping.keeping import Keeping

from ros2_ws.src.lane_nodes_py.lane_nodes_py.keeping.pid_controller import PID

from ros2_ws.src.lane_nodes_py.lane_nodes_py.keeping.robot_path import PathData

PID_FREQUENCY = 2


class KeepingNode(Node):
    keeping = Keeping()

    pid = PID(1.0, 0, 0)

    path_grid = PathData()

    def __init__(self):
        super().__init__('keeping')

        # Create the publisher for sending movement instructions
        self.movement_publisher_ = self.create_publisher(AckermannDrive, "/carla/ego_vehicle/ackermann_cmd", 10)

        self.timer = self.create_timer(1 / PID_FREQUENCY, self.movement_output_callback)

        # Create the subscriber for receiving lane data
        self.lane_subscription = self.create_subscription(
                LaneLocation,
                'lane_location_data',
                self.lane_location_callback,
                10)
        self.lane_subscription # prevent unused variable warning

    # Callback for receiving lane data. At the moment, just echoes it down the pipeline without any further processing.
    def lane_location_callback(self, msg):
        self.get_logger().info('Received message: "%s"' % msg.temp)

    def movement_output_callback(self):
        self.get_logger().info('Publishing movement instructions')
        msg = AckermannDrive()
        msg.steering_angle = 0.0
        msg.steering_angle_velocity = 0.0
        msg.speed = 10.0
        msg.acceleration = 0.0
        msg.jerk = 0.0
        self.get_logger().info("%s" % msg)
        self.movement_publisher_.publish(msg)
        self.timer.reset()


def main(args=None):
    rclpy.init(args=args)

    keeping_node = KeepingNode()

    rclpy.spin(keeping_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    keeping_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
