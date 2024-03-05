import time

import numpy as np

from lane_nodes_py.keeping.ackerman_wrapper import AckermannWrapper

from lane_nodes_py.keeping.pid_controller import PID

from lane_nodes_py.keeping.robot_path import PathData

from typing import Tuple, List


class Keeping:
    pid = PID(1.0, 0, 0)

    def __init__(self, size: float):
        # Initialize the mathematical model
        self.path_grid = PathData(size)

        # Set a default last message for the ackerman drive, defaulting to remaining stationary
        self.last_message = AckermannWrapper()
        self.last_message.steering_angle = 0
        self.last_message.steering_angle_velocity = 0.0
        self.last_message.speed = 1.0
        self.last_message.acceleration = 0.0
        self.last_message.jerk = 0.

        # Set a default time
        self.last_time = time.time()

        # Set a stanley constant
        self.k = 1

    # Callback for receiving lane data. At the moment, just echoes it down the pipeline without any further processing.
    def lane_location_callback(self, msg):
        print('Received message: "%s"' % msg)
        self.path_grid.set_grid(msg)

    def movement_output_callback(self):
        # Logs for the log god
        if self.path_grid.path is None:
            return AckermannWrapper()
        # If the data is not fresh, meaning we're using old data, use the last message sent to update our
        # "expected" model
        current_time = time.time()
        if not self.path_grid.is_fresh():
            self.path_grid.update(self.last_message, current_time - self.last_time)

        self.last_time = current_time

        self.path_grid.fresh = False

        # Calculate our cross track and heading error using the grid data
        (cross_track_error, heading_error) = self.calculate_error()

        # Calculate the velocity of the vehicle
        v = self.last_message.speed

        # Figure out the new heading from the error
        new_heading = self.calculate_heading(cross_track_error, heading_error, v, self.path_grid)

        # Use the output of the PID controller with the grid data to determine the next control
        return self.generate_ackerman_control(new_heading, self.path_grid)

    def calculate_error(self) -> Tuple[float, float]:
        return self.path_grid.get_distance_to_line(), self.path_grid.get_heading_offset()

    def calculate_heading(self, cross_error, heading_error, velocity, path_grid: PathData):
        # TODO: Add a null checker for velocity
        cross_heading = np.arctan((self.k * cross_error) / velocity)
        return cross_heading + heading_error

    def generate_ackerman_control(self, output, path_grid: PathData):
        msg = AckermannWrapper()
        msg.steering_angle = output
        msg.steering_angle_velocity = 0.0
        msg.speed = 8.0
        msg.acceleration = 0.0
        msg.jerk = 0.0
        self.last_message = msg
        return msg


def test():
    print(str(time.time()))





