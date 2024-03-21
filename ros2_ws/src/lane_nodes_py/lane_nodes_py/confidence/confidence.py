from numpy import who
from .confidence_lane import ConfidenceLane

LANE_DISTANCE_THRESHOLD = 20     # The maximum distance that a line can be from a lane and still be considered part of it. TODO: Tune this value
LANE_REMOVAL_THRESHOLD = 0.05     # The minimum confidence for a lane to not be removed
LANE_CUTOFF_THRESHOLD = 0.50    # The minimum confidence for a lane to be passed along to the rest of the system

# For each set of lines received as input, one iteration
class Confidence(object):
    
    def __init__(self):
        self.confidence_lanes = []
        pass

    # Take a list of lines and identify the which lane it corresponds to
    def process_lanes(self, lanes):
        updated_lanes = []
        for lane in lanes:
            #   Find the existing lane closest to the line
            min_distance = 100000000000     # Initialize this to a ridiculous number
            candidate_lane = None
            poly_coeffs = ConfidenceLane.create_polynomial(lane)
            lane_points = ConfidenceLane.get_polynomial_points(poly_coeffs)
            x_average = ConfidenceLane.get_line_x_average(lane)

            for conf_lane in self.confidence_lanes:
                
                average_distance = abs(conf_lane.x_average - x_average)
                if (average_distance < min_distance):
                    min_distance = average_distance
                    candidate_lane = conf_lane
            
            # If there are no known lanes and no lines are detected
            if (candidate_lane is None):
                return []

            #   If the line is close enough (below some threshold), then add it to the lane.
            if (min_distance < LANE_DISTANCE_THRESHOLD):
                candidate_lane.add_line(lane_points)
            else:   # If the closest lane is still too far, initialize a new lane with this line.     
                self.confidence_lanes.append(ConfidenceLane(lane_points))
                
        # Increase confidence in lanes that were found, decrease it in those that were not.
        for conf_lane in self.confidence_lanes:
            if (conf_lane in updated_lanes):
                conf_lane.increase_confidence()
            else:
                conf_lane.decrease_confidence()

        result = []
        # Remove lanes with confidence below a certain threshold, and gather lanes with high enough confidence
        for conf_lane in self.confidence_lanes:
            if (conf_lane.confidence <= LANE_REMOVAL_THRESHOLD):
                self.confidence_lanes.remove(conf_lane)
            elif(conf_lane.confidence >= LANE_CUTOFF_THRESHOLD):
                result.append(conf_lane.get_last_lane())

        # Return a list of all lanes with confidence above a certain threshold
        return result


        

    
