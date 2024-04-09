# TODO: Use numpy arrays for the lists to allow for much more efficient calculations on them
import numpy as np

NUM_PAST_LANES = 3          # Number of iterations to retain lane data for.
INITIAL_CONFIDENCE = 0.0    # The initial confidence value for the lane.
LINE_MAX_LENGTH = 640       # The maximum length of a line in the lane data. TODO: Tune this

# Class used to store a lane, its past values, and the confidence in the lane.
class ConfidenceLane(object):
    # Constructor for a ConfidenceLane. Takes the line it should follow as a parameter 
    def __init__(self, initial_line) -> None:
        # Initialize the list of past lanes to a numpy array of length NUM_PAST_LANES
        self.lane_data = [None, None, None]
        # Float from 0.0 to 1.0 representing the current confidence in this lane
        self.confidence = INITIAL_CONFIDENCE
        # The index of the current line in the list of past lanes. Using an index allows us to avoid shifting items around in the arrya, which is time-consuming.
    
    # Create numpy array from new line. Index corresponds to the y-value, value is the x-value. y-values without an x-value are NaN
    @staticmethod
    def create_numpy_array(line):
        result = np.full(LINE_MAX_LENGTH, np.NaN)
        for x, y in line:
            result[y] = x

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
    def add_line(self, new_line):
        self.lane_data.insert(0, new_line)
        self.lane_data.pop()


    # Calculate the average distance of points between a line and this lane. Used to determine what lane, if any, a line corresponds to.
    def lane_distance(self, test_line):
        # For each point on the line, calculate the distance between it and this lane.
        return np.nanmean(np.array([self.get_last_lane(), test_line]), axis=0)
