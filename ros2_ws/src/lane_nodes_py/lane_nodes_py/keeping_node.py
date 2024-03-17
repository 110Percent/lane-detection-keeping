import rclpy
from rclpy.node import Node

from lane_interfaces.msg import LaneLocation, LaneLocation2

from ackermann_msgs.msg import AckermannDrive

from lane_nodes_py.keeping.keeping import Keeping

from lane_nodes_py.keeping.pid_controller import PID

from lane_nodes_py.keeping.lane_wrapper import LaneWrapper

import os

OUTPUT_FREQUENCY = 10


class KeepingNode(Node):
    keeping: Keeping

    pid = PID(1.0, 0, 0)

    def __init__(self, target_velocity, control_constant):
        super().__init__('keeping')

        self.keeping =  Keeping(1, target_velocity, control_constant)

        # Create the publisher for sending movement instructions
        self.movement_publisher_ = self.create_publisher(AckermannDrive, "/carla/ego_vehicle/ackermann_cmd", 10)

        self.timer = self.create_timer(1 / OUTPUT_FREQUENCY, self.movement_output_callback)

        # Create the subscriber for receiving lane data
        self.lane_subscription = self.create_subscription(
            LaneLocation,
            'bev_lane_location_data',
            self.lane_location_callback,
            10)

        self.eval_lane_subscription = self.create_subscription(
            LaneLocation2,
            'lane_location_data_eval',
            self.lane_location_callback_eval,
            10)

    # Callback for receiving lane data. At the moment, just echoes it down the pipeline without any further processing.
    def lane_location_callback(self, msg):
        self.get_logger().info('Received message: "%s"' % msg.temp)

    def lane_location_callback_eval(self, msg):
        lane_data: LaneWrapper = LaneWrapper()
        lane_data.paths = [msg.y_vals1, msg.y_vals2]
        lane_data.coordinates = msg.x_vals
        self.keeping.lane_location_callback(lane_data)
        self.get_logger().info('Received new lane data')

    def movement_output_callback(self):
        self.get_logger().info('Publishing movement instructions to Carla')

        ackermann_msg = self.keeping.movement_output_callback()

        msg = AckermannDrive()

        msg.steering_angle = ackermann_msg.steering_angle
        msg.steering_angle_velocity = ackermann_msg.steering_angle_velocity
        msg.speed = ackermann_msg.speed
        msg.acceleration = ackermann_msg.acceleration
        msg.jerk = ackermann_msg.jerk
        self.movement_publisher_.publish(msg)
        self.timer.reset()


def main(args=None):
    rclpy.init(args=args)

    keeping_node = KeepingNode(float(os.environ['VEHICLE_VELOCITY']), float(os.environ['CONTROL_CONSTANT']))

    rclpy.spin(keeping_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    keeping_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()