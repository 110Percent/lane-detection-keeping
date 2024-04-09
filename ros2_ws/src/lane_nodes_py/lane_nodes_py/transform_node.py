import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import matplotlib.pyplot as plt
import ros2_ws.src.lane_nodes_py.lane_nodes_py.transforms.line_transforms as line_transforms

from lane_interfaces.msg import LaneLocation

class TransformNode(Node):

    def __init__(self):
        super().__init__('transform')

        # Create the publisher for sending lane location data
        self.lane_publisher_ = self.create_publisher(LaneLocation, "bev_lane_location_data", 10)

        # Create the subscriber for receiving lane data
        self.lane_subscription = self.create_subscription(
            LaneLocation,
            'lane_location_data',
            self.lane_location_callback,
            10)
        
    def lane_location_callback(self, msg):
        # parse the lane location data
        stamp = msg.stamp
        id = msg.frame_id
        lanes = msg.lanes
        row_lengths = msg.row_lengths
        img_shape = msg.img_shape

        # unflatten lane data and transform to birds eye view
        unflat = line_transforms.unflatten_lanes(lanes, row_lengths)
        bev = line_transforms.lanes_to_birds_eye(unflat, img_shape)

        # flatten birds eye view to send as message
        bev_flat = line_transforms.flatten_lanes(bev)

        lane_data = LaneLocation()
        lane_data.stamp = stamp
        lane_data.frame_id = id
        lane_data.row_lengths = row_lengths
        lane_data.lanes = bev_flat
        lane_data.img_shape = img_shape

        # publish the transformed lane data
        self.lane_publisher_.publish(lane_data)
        # self.get_logger().info('Sent Transformed lane')


def main(args=None):
    rclpy.init(args=args)

    transform_node = TransformNode()

    rclpy.spin(transform_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    transform_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()