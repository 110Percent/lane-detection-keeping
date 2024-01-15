from ackerman_wrapper import AckermannWrapper

from pid_controller import PID

from robot_path import PathData

from lane_location_interface import LaneData


class Keeping:

    pid = PID(1.0, 0, 0)

    def __init__(self, size: float):

        # Initialize the mathematical model
        self.path_grid = PathData(size)

        # Set a default last message for the ackerman drive, defaulting to remaining stationary
        self.last_message = AckermannWrapper()
        self.last_message.steering_angle = 0
        self.last_message.steering_angle_velocity = 0.0
        self.last_message.speed = 0.0
        self.last_message.acceleration = 0.0
        self.last_message.jerk = 0.0

    # Callback for receiving lane data. At the moment, just echoes it down the pipeline without any further processing.
    def lane_location_callback(self, msg: LaneData):
        print('Received message: "%s"' % msg)
        self.path_grid.set_grid(msg)

    def movement_output_callback(self):
        # Logs for the log god
        print('Publishing movement instructions')

        print('Exiting for testing purposes')

        # If the data is not fresh, meaning we're using old data, use the last message sent to update our
        # "expected" model
        if not self.path_grid.is_fresh():
            print('Grid path is not fresh')
            self.path_grid.update(self.last_message)

        # Calculate our error using the grid data
        error = self.calculate_error(self.path_grid)

        # Throw the error into the PID controller
        output = self.pid.update(error)

        # Use the output of the PID controller with the grid data to determine the next control
        return self.generate_ackerman_control(output, self.path_grid)

    def calculate_error(self, path_grid: PathData):
        # TODO: Implement the calculation of the error
        return 0

    def generate_ackerman_control(self, output, path_grid: PathData):
        """Generates default controls, but in practice will incorporate the output, and the mathematical model"""
        # TODO: Actually implement a proper calculation of controls
        msg = AckermannWrapper()
        msg.steering_angle = 1.22
        msg.steering_angle_velocity = 0.0
        msg.speed = 10.0
        msg.acceleration = 0.0
        msg.jerk = 0.0
        self.last_message = msg
        return msg
