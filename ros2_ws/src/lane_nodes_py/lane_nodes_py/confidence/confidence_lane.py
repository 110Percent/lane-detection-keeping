# TODO: Use numpy arrays for the lists to allow for much more efficient calculations on them
import numpy as np

NUM_PAST_LANES = 3          # Number of iterations to retain lane data for.
INITIAL_CONFIDENCE = 0.3    # The initial confidence value for the lane.
POLY_DEGREE = 3
HORIZON_POSITION = 0.5
NUM_POINTS = 50

# Class used to store a lane, its past values, and the confidence in the lane.
class ConfidenceLane(object):
    # Constructor for a ConfidenceLane. Takes the line it should follow as a parameter 
    def __init__(self, initial_line) -> None:
        # Initialize the average of the past known lanes
        self.average_lane = None
        # Initialize the x-average of the average lane
        self.x_average = None
        # Initialize the list of past lanes (stored as numpy arrays containing polynomial coefficients) to a python list of length NUM_PAST_LANES
        self.lane_data = [None] * NUM_PAST_LANES
        # Float from 0.0 to 1.0 representing the current confidence in this lane
        self.confidence = INITIAL_CONFIDENCE

        self.add_line(initial_line)

    # Takes a list of tuples and returns polynomial coefficients that fit the given data
    @staticmethod
    def create_polynomial(data):
        # Rotate the data 90 degrees about the point (0.5, 0.5), right in the center of the range of acceptable values.
        # This is required to accurately model the distance of the line.
        points = np.array(data)
        pivot = (0.5, 0.5)
        # Translate points so they can be rotated about the origin
        translated_points = points - pivot
        
        # Rotate the points 90 degrees about the origin
        angle_rad = np.rad2deg(90)
        rotation_matrix = np.array([[np.cos(angle_rad), -np.sin(angle_rad)],
                                    [np.sin(angle_rad), np.cos(angle_rad)]])
        rotated_translated_points = np.dot(translated_points, rotation_matrix)

        # Translate the points so that they are in their final positions
        rotated_points = rotated_translated_points + pivot


        # Split the coordinates into separate numpy arrays
        x_coordinates = rotated_points[:, 0]
        y_coordinates = rotated_points[:, 1]

        # Fit a polynomial to the data
        degree = POLY_DEGREE
        poly_coeffs = np.polyfit(x_coordinates, y_coordinates, degree)

        return poly_coeffs
    

    # Compute a set of points along the given line within the range 0 to 1. Return a list of tuples containing all the points.
    @staticmethod
    def get_polynomial_points(poly_coeffs):
        # create the set of x-values to get points for
        x_values = np.arange(0, HORIZON_POSITION, HORIZON_POSITION/NUM_POINTS)

        # evaluate the polynomial at all the desired x-values
        y_values = np.polyval(poly_coeffs, x_values)

        return np.column_stack((x_values, y_values))

    # Compute the average x value of a line
    @staticmethod
    def get_line_x_average(line):
        definite_integral = np.trapz(line, dx=0.02)       
        return definite_integral / HORIZON_POSITION
        
    # Compute the average of each point on the currently stored lines, and return the average line
    def calculate_average_line(self):
        return np.mean(np.array([self.lane_data]), axis=0)
    
    # Decrease confidence according to the chosen mathematical function (linear for now, look into other options)
    def decrease_confidence(self):
        self.confidence -= 0.1
        if (self.confidence < 0.0):
            self.confidence = 0.0

    # Increase confidence according to the chosen mathematical function (linear for now, look into other options)
    def increase_confidence(self):
        self.confidence += 0.1
    
    # Return the most recent known position of lane
    def get_last_lane(self):
        return self.lane_data[0]

    # Add new line to the lane data and remove the oldest. Line must be passed as a numpy array.
    # Also update the list of averages.
    def add_line(self, new_line_points):
        # Update the list of stored lanes
        self.lane_data.insert(0, new_line_points)
        self.lane_data.pop()

        # Recalculate the average values
        self.average_lane = self.calculate_average_line()
        self.x_average = self.get_line_x_average(self.average_lane)

    # Calculate the average distance of points between a line and this lane. Used to determine what lane, if any, a line corresponds to.
    def lane_distance(self, test_line):
        # For each point on the line, calculate the distance between it and this lane.
        return np.nanmean(np.array([self.get_last_lane(), test_line]), axis=0)
