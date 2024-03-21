# For each set of lines received as input, one iteration

# For each iteration, loop over the lines and do the following for each:
#   Find the lane closest to the line
#   If the line is close enough (below some threshold), then add it to the lane
#   If the closest lane is still too far, initialize a new lane with this line.
# After looping over the lines, do the following:
#   Update the confidence of the known lanes. Will have to keep track of which lanes had a line added.
#   Remove lanes with too low a confidence (maybe 0, maybe a bit more if a non-linear function is being used.)
#   Pass lanes with a high enough confidence on to the transformation subsystem

import cv2

class Confidence(object):
    
    def __init__(self):
        pass

    def process_image(self, image):
        cv2.imshow('image', image)
        cv2.waitKey(1)
