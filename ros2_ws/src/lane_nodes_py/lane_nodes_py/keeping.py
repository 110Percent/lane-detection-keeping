import rclpy
from rclpy.node import Node

from lane_interfaces.msg import LaneLocation, MovementInstruction


class Keeping(Node):

    def __init__(self):
        super().__init__('keeping')
        
        # Create the publisher for sending movement instructions
        self.movement_publisher_ = self.create_publisher(MovementInstruction, "movement_instructions", 10)
        
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
        movement_msg = MovementInstruction()
        movement_msg.temp = msg.temp
        self.movement_publisher_.publish(movement_msg)

def main(args=None):
    rclpy.init(args=args)

    keeping = Keeping()

    rclpy.spin(keeping)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    keeping.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
