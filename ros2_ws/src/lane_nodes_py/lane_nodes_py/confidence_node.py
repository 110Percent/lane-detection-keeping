import rclpy
from rclpy.node import Node

from lane_interfaces.msg import LaneLocation

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

        # Create confidence object
        self.confidence = Confidence()

    
    # When a set of lanes are detected, process them and send only the lanes we are confident in the existence of along.
    def lane_callback(self, msg):
        confident_lanes = self.confidence.process_lanes(msg.lanes)
        # TODO: Publish the confident lanes 


def main(args=None):
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


        
