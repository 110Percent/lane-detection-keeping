import rclpy
from rclpy.node import Node

from lane_interfaces.msg import LaneLocation
import ros2_ws.src.lane_nodes_py.lane_nodes_py.transforms.line_transforms as line_transforms

from .confidence import Confidence

# ROS 2 node for the confidence system
class ConfidenceNode(Node):
    def __init__(self):
        super().__init__('confidence')
        
        # Create subscriber for lane data
        self.image_subscriber = self.create_subscription(
                LaneLocation,
                'lane_location_data',
                self.lane_callback,
                10)
        
        # Create the publisher for sending lane location data
        self.lane_publisher_ = self.create_publisher(LaneLocation, "confidence_lane_location_data", 10)

        # Create confidence object
        self.confidence = Confidence()

    # When a set of lanes are detected, process them and send only the lanes we are confident in the existence of along.
    def lane_callback(self, msg):
        stamp = msg.stamp
        id = msg.frame_id
        lanes = msg.lanes
        row_lengths = msg.row_lengths
        img_shape = msg.img_shape
        unflat = line_transforms.unflatten_lanes(lanes, row_lengths)
        confident_lanes = self.confidence.process_lanes(unflat)
        
        lane_data = LaneLocation()
        lane_data.stamp = stamp
        lane_data.frame_id = id
        lane_data.row_lengths = row_lengths
        lane_data.lanes = confident_lanes
        lane_data.img_shape = img_shape

        self.lane_publisher_.publish(lane_data)
        # self.get_logger().info('Sent Confident lanes')


def main(args=None):
    return
    rclpy.init(args=args)

    confidence_node = ConfidenceNode()

    rclpy.spin(confidence_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    confidence_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
