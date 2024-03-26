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
        unflat = self.unflatten_lanes(lanes, row_lengths)
        confident_lanes = self.confidence.process_lanes(unflat)
        
        lane_data = LaneLocation()
        lane_data.stamp = stamp
        lane_data.frame_id = id
        lane_data.row_lengths = row_lengths
        lane_data.lanes = confident_lanes
        lane_data.img_shape = img_shape

        self.lane_publisher_.publish(lane_data)
        # self.get_logger().info('Sent Confident lanes')
    
    def flatten_lanes(self, lanes):
        flattened = []
        for row in lanes:
            for point in row:
                flattened.append(point[0])
                flattened.append(point[1])
        return flattened
    
    def unflatten_lanes(self, lanes, row_lengths):
        unflat = []
        for length in row_lengths:
            flat_lane = lanes[:length * 2]
            del lanes[:length * 2]
            x_vals = []
            y_vals = []
            for i, val in enumerate(flat_lane):
                if i % 2 == 0:
                    x_vals.append(val)
                else:
                    y_vals.append(val)
            unflat.append(list(zip(x_vals, y_vals)))
        # self.get_logger().info('parsed lanes: ' + str(unflat))
        # for i, lane in enumerate(unflat):
        #     self.get_logger().info('lane ' + str(i) + ': length=' + str(len(lane)) + ' points=' + str(lane))
        return unflat


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
