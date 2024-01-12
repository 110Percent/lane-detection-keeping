import rclpy
from rclpy.node import Node

from lane_interfaces.msg import LaneLocation, MovementInstruction

from ackermann_msgs.msg import AckermannDrive

from .keeping import Keeping

from .pid_controller import PID

from .robot_path import PathData

PID_FREQUENCY = 2


class KeepingNode(Node):
    Keeping = Keeping()

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
        self.lane_subscription  # prevent unused variable warning
        self.last_message = AckermannDrive()
        self.last_message.steering_angle = 0
        self.last_message.steering_angle_velocity = 0.0
        self.last_message.speed = 0.0
        self.last_message.acceleration = 0.0
        self.last_message.jerk = 0.0

    # Callback for receiving lane data. At the moment, just echoes it down the pipeline without any further processing.
    def lane_location_callback(self, msg):
        self.get_logger().info('Received message: "%s"' % msg.temp)
        self.path_grid.set_grid(msg)

    def movement_output_callback(self):
        self.get_logger().info('Publishing movement instructions')

        # If the data is not fresh, meaning we're using old data, use the last message sent to update our
        # "expected" model
        if not self.path_grid.is_fresh():
            self.path_grid.update(self.last_message)

        # Calculate our error using the grid data
        error = self.calculate_error(self.path_grid)

        # Throw the error into the PID controller
        output = self.pid.update(error)

        # Use the output of the PID controller with the grid data to determine the next control
        msg = self.generateAckermannControls(output, self.path_grid)

        self.get_logger().info("%s" % msg)
        self.movement_publisher_.publish(msg)
        self.timer.reset()

    def calculate_error(self, path_grid: PathData):
        return 0

    def generate_ackerman_control(self, output, path_grid: PathData):
        msg = AckermannDrive()
        msg.steering_angle = 1.22
        msg.steering_angle_velocity = 0.0
        msg.speed = 10.0
        msg.acceleration = 0.0
        msg.jerk = 0.0
        self.last_message = msg
        return msg


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
