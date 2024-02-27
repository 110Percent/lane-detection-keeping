import rclpy
from rclpy.node import Node

from lane_interfaces.msg import LaneLocation

PID_FREQUENCY = 2


class KeepingNode(Node):
    # keeping = Keeping(1)

    # pid = PID(1.0, 0, 0)

    def __init__(self):
        super().__init__('keeping')

        # Create the publisher for sending movement instructions
        #self.movement_publisher_ = self.create_publisher(AckermannDrive, "/carla/ego_vehicle/ackermann_cmd", 10)

        #self.timer = self.create_timer(1 / PID_FREQUENCY, self.movement_output_callback)

        # Create the subscriber for receiving lane data
        self.lane_subscription = self.create_subscription(
            LaneLocation,
            'lane_location_data',
            self.lane_location_callback,
            10)
        
    def lane_location_callback(self, msg):
        stamp = msg.stamp
        id = msg.frame_id
        lanes = msg.lanes
        row_lengths = msg.row_lengths

        self.get_logger().info('msg: ' + str(msg))

        unflat = self.unflatten_lanes(lanes, row_lengths)



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
        for i, lane in enumerate(unflat):
            self.get_logger().info('lane ' + str(i) + ': length=' + str(len(lane)) + ' points=' + str(lane))


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