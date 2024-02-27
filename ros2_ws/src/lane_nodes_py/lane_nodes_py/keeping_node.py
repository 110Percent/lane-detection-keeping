import rclpy
from rclpy.node import Node

from lane_interfaces.msg import LaneLocation, LaneLocation2
from ackermann_msgs.msg import AckermannDrive
from lane_nodes_py.keeping.keeping import Keeping
from lane_nodes_py.keeping.lane_wrapper import LaneWrapper
import lane_nodes_py.transforms.line_transforms as line_transforms

import os

OUTPUT_FREQUENCY = 10


class KeepingNode(Node):
    # keeping = Keeping(1)

    def __init__(self, target_velocity, control_constant):
        super().__init__('keeping')

        self.keeping =  Keeping(1, target_velocity, control_constant)

        # Create the publisher for sending movement instructions
        #self.movement_publisher_ = self.create_publisher(AckermannDrive, "/carla/ego_vehicle/ackermann_cmd", 10)

        #self.timer = self.create_timer(1 / PID_FREQUENCY, self.movement_output_callback)

        # Create the subscriber for receiving lane data
        if os.environ['EVAL_MODE'] != "FULL":
            self.lane_subscription = self.create_subscription(
                LaneLocation,
                'bev_lane_location_data',
                self.lane_location_callback,
                10)
        else:
            self.eval_lane_subscription = self.create_subscription(
                LaneLocation2,
                'lane_location_data_eval',
                self.lane_location_callback_eval,
                10)

    # Callback for receiving lane data. At the moment, just echoes it down the pipeline without any further processing.
    def lane_location_callback(self, msg):
        stamp = msg.stamp
        id = msg.frame_id
        lanes = msg.lanes
        row_lengths = msg.row_lengths
        img_shape = msg.img_shape
        unflat = line_transforms.unflatten_lanes(lanes, row_lengths)
        lane_data = translator(unflat)
        self.keeping.lane_location_callback(lane_data)
        # self.get_logger().info('Received transformed lane')

    def lane_location_callback_eval(self, msg):
        lane_data: LaneWrapper = LaneWrapper()
        lane_data.paths = [msg.y_vals1, msg.y_vals2]
        lane_data.coordinates = msg.x_vals
        self.keeping.lane_location_callback(lane_data)
        # self.get_logger().info('Received new lane data')

    def movement_output_callback(self):
        # self.get_logger().info('Publishing movement instructions')

        ackermann_msg = self.keeping.movement_output_callback()

        msg = AckermannDrive()

        msg.steering_angle = ackermann_msg.steering_angle
        msg.steering_angle_velocity = ackermann_msg.steering_angle_velocity
        msg.speed = ackermann_msg.speed
        msg.acceleration = ackermann_msg.acceleration
        msg.jerk = ackermann_msg.jerk
        self.movement_publisher_.publish(msg)
        self.timer.reset()


def translator(lanedata):
    for i in range(len(lanedata)):
        print(lanedata[i])
        list1, list2 = zip(*lanedata[i])
        lanedata[i] = list(zip(*(list2, list1)))

    x_coordinates = set()
    for lane in lanedata:
        for point in lane:
            x_coordinates.add(point[0])
    x_coordinates = list(x_coordinates)
    x_coordinates.sort()

    lane_data: LaneWrapper = LaneWrapper()

    lane_data.coordinates = x_coordinates

    lanes = []
    for lane in lanedata:
        lanes += [[]]

    for i in range(len(lanedata)):
        lane_dict = dict(lanedata[i])
        print(lane_dict)
        for coordinate in x_coordinates:
            if coordinate in lane_dict:
                lanes[i] += [-lane_dict.get(coordinate)]
            else:
                lanes[i] += ['N']

    lane_data.paths = lanes

    return lane_data

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
