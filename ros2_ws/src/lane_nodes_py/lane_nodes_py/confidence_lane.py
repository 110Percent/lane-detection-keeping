# TODO: Use numpy arrays for the lists to allow for much more efficient calculations on them

NUM_PAST_LANES = 3          # Number of iterations to retain lane data for.
INITIAL_CONFIDENCE = 0.0    # The initial confidence value for the lane.

# Class used to store a lane, its past values, and the confidence in the lane.
class ConfidenceLane(object):
    # Constructor for a ConfidenceLane. Takes the line it should follow as a parameter 
    def __init__(self, initial_line) -> None:
        self.lane_data = initial_line    # This should be a list of length NUM_PAST_LANES of the past few known positions of the line. When new data is received, insert it at the start and remove the last one.
        self.confidence = INITIAL_CONFIDENCE   # This should be a float, from 0.0 to 1.0, containing the current confidence level of the lane.
    
    # Decrease confidence according to the chosen mathematical function (linear for now, look into other options)
    def decrease_confidence(self):
        self.confidence -= 0.1
        if (self.confidence < 0.0):
            self.confidence = 0.0

    # Increase confidence according to the chosen mathematical function (linear for now, look into other options)
    def increase_confidence(self):
        self.confidence += 0.1

    # Calculate average position of lane across all stored data
    def get_smoothed_lane(self):
        # If the past lines are of different lengths, decide what to do (discard excess length, ignore the shorter lines for these points, etc)
        # Calculate the average of the lane across all stored lines.
        pass

    # Add new line to the lane data and remove the oldest
    def add_line(self):
        # Add the new line to the start of the lane list
        # Shift existing lines to the right, discarding the oldest one.
        pass

    # Calculate the average distance of points between a line and this lane. Used to determine what lane, if any, a line corresponds to.
    def lane_distance(self):
        # For each point on the line, calculate the distance between it and this lane.
        # Return the average distance.
        pass
