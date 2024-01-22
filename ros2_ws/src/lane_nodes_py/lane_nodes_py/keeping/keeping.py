import time

import numpy as np

from ackerman_wrapper import AckermannWrapper

from pid_controller import PID

from robot_path import PathData

from lane_location_interface import LaneData

from ischedule import schedule, run_loop

import matplotlib.pyplot as plt


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
    def lane_location_callback(self, msg: LaneData):
        print('Received message: "%s"' % msg)
        self.path_grid.set_grid(msg)

    def movement_output_callback(self):
        # Logs for the log god
        print('Publishing movement instructions')

        # If the data is not fresh, meaning we're using old data, use the last message sent to update our
        # "expected" model
        current_time = time.time()
        if not self.path_grid.is_fresh():
            print('Grid path is not fresh')
            self.path_grid.update(self.last_message, current_time - self.last_time)

        self.last_time = current_time

        self.path_grid.fresh = False

        # Calculate our cross track and heading error using the grid data
        (cross_track_error, heading_error) = self.calculate_error()

        # Calculate the velocity of the vehicle
        v = self.last_message.speed

        # Figure out the new heading from the error
        new_heading = self.calculate_heading(cross_track_error, heading_error, v, self.path_grid)

        print("Vehicle position in grid: " + str(self.path_grid.position))
        print("Cross track error: " + str(cross_track_error))
        print("Heading error: " + str(heading_error))
        print("Steering Yaw: " + str(new_heading))

        # Use the output of the PID controller with the grid data to determine the next control
        return self.generate_ackerman_control(new_heading, self.path_grid)

    def calculate_error(self) -> tuple[float, float]:
        return self.path_grid.get_distance_to_line(), self.path_grid.get_heading_offset()

    def calculate_heading(self, cross_error, heading_error, velocity, path_grid: PathData):
        # TODO: Add a null checker for velocity
        cross_heading = np.arctan((self.k * cross_error) / velocity)
        return cross_heading + heading_error

    def generate_ackerman_control(self, output, path_grid: PathData):
        msg = AckermannWrapper()
        msg.steering_angle = output
        msg.steering_angle_velocity = 0.0
        msg.speed = 1.0
        msg.acceleration = 0.0
        msg.jerk = 0.0
        self.last_message = msg
        return msg


def test():
    print(str(time.time()))


def main():
    keeping = Keeping(1)
    path: list[tuple[float, float]] = perabola()
    keeping.path_grid.set_path(path)
    schedule(keeping.movement_output_callback, interval=0.1)
    run_loop(return_after=10)
    print("Vehicle History: " + str(keeping.path_grid.history))
    x1, y1 = zip(*path)
    x2, y2 = zip(*keeping.path_grid.history)
    plt.plot(x1, y1, c='#4CAF50', label="Desired Path")
    plt.plot(x2, y2, color='r', ls=':', label="Vehicle Path")
    plt.ylabel('y')
    plt.xlabel('x')
    plt.legend(loc='upper right')
    plt.show()


def sinusodal_curve():
    path: list[tuple[float, float]] = []
    for i in range(-1, 110):
        path += [(i / 10, -np.cos(0.62831853071 * (i / 10)) + 1)]
    return path


def perabola():
    path: list[tuple[float, float]] = []
    for i in range(-1, 110):
        path += [(i / 10, (-1/2)*((i/10) - 5)**2 + 5)]
    return path


if __name__ == '__main__':
    main()
