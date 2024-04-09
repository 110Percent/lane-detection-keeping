import numpy as np
import cv2
import matplotlib.pyplot as plt


def lanes_to_birds_eye(lanes, img_shape):
    '''Transform first person lane markings into birds eye view coordinates.

    The following is derived from: 
    https://github.com/JonathanCMitchell/Advanced-Lane-Line-Detection/blob/master/Perspective_transform.ipynb
    Some code and methods and taken directly from the implementation by JonathanCMitchell
    
    Details of the some of the steps performed are further explained at the repo above.

    @param lanes: list of lists of tuples, each list of tuples represents a line
    @param img_shape: tuple of the shape (height, width) of the source image the lanes were detected from

    @return: list of lists of tuples, each list of tuples represents a lane in birds eye view
    '''
    # The values of the following constants were determined through trial and error when running
    # the system. Calibration steps should likely be implemented to better tune these values.
    TOP_TUNING_CONSTANT = 40
    BOTTOM_TUNING_CONSTANT = 40
    TRANSFORM_SCALING_FACTOR = 25


    Lhs = np.zeros((2, 2), dtype = np.float32)
    Rhs = np.zeros((2, 1), dtype = np.float32)
    x_max = 0
    x_min = 2555

    # get values for calculating vanishing point
    for line in lanes:
        x_list = line[::2]
        y_list = line[1::2]
        for point1, point2 in zip(x_list, y_list):
            x1, y1 = point1
            x2, y2 = point2

            x1 = int(x1 *img_shape[1])
            x2 = int(x2 *img_shape[1])
            y1 = int(y1 *img_shape[0])
            y2 = int(y2 *img_shape[0])

            # Find the norm (the distances between the two points)
            normal = np.array([[-(y2-y1)], [x2-x1]], dtype = np.float32) # question about this implementation
            normal = normal / np.linalg.norm(normal)
            
            pt = np.array([[x1], [y1]], dtype = np.float32)
            outer = np.matmul(normal, normal.T)
            
            Lhs += outer
            Rhs += np.matmul(outer, pt) #use matmul for matrix multiply and not dot product
            
            x_iter_max = max(x1, x2)
            x_iter_min = min(x1, x2)
            x_max = max(x_max, x_iter_max)
            x_min = min(x_min, x_iter_min)

    width = x_max - x_min
    # calculate vanishing point 
    vp = np.matmul(np.linalg.inv(Lhs), Rhs)

    def find_pt_inline(p1, p2, y):
        """
        Here we use point-slope formula in order to find a point that is present on the line
        that passes through our vanishing point (vp). 
        input: points p1, p2, and y. They come is as tuples [x, y]
        We then use the point-slope formula: y - b = m(x - a)
        y: y-coordinate of desired point on the line
        x: x-coordinate of desired point on the line
        m: slope
        b: y-coordinate of p1
        a: x-coodrinate of p1
        x = p1x + (1/m)(y - p1y)
        """
        m_inv = (p2[0] - p1[0]) / float(p2[1] - p1[1])
        Δy = (y - p1[1])
        x = p1[0] + m_inv * Δy
        return [x, y]

    top = vp[1] + TOP_TUNING_CONSTANT
    bottom = img_shape[1] - BOTTOM_TUNING_CONSTANT

    width = 600

    print('vp[0]', vp[0], type(vp[0][0]))
    print('vp[1]', vp[1], type(vp[1][0]))

    # calculate src points for generating the homography matrix
    p1 = [vp[0] - width/2, top]
    p2 = [vp[0] + width/2, top]
    p3 = find_pt_inline(p2, vp, bottom)
    p4 = find_pt_inline(p1, vp, bottom)

    src_pts = np.array([p1, p2, p3, p4], dtype=object)
    dst_pts = np.float32([[0, 0], [img_shape[0], 0],
                    [img_shape[0], img_shape[1]],
                    [0, img_shape[1]]])
    src_pts = np.float32(src_pts)

    # the homography matrix
    M = cv2.getPerspectiveTransform(src_pts, dst_pts)

    # this implementation for transforming the points is inefficient and could likely be improved
    lines = []
    for line in lanes:
        points = [(x*img_shape[1], y*img_shape[0]) for x,y in line]
        t_points = []
        for point in points:
            t_point = cv2.perspectiveTransform(np.float32([[[point[0],point[1]]]]), M)
            # translate the coordinates from image formatting to a cartesian grid with the origin at 
            # the bottom center of the image. The scaling factor is attempts to scale the dimensions
            # to a lane size of about 3 meters (minimum size for Ontario) and should be better calibrated.
            t_point[0][0][0] = (t_point[0][0][0]  - img_shape[0] / 2) / TRANSFORM_SCALING_FACTOR
            t_point[0][0][1] = (-1*(t_point[0][0][1]  - img_shape[1])) / TRANSFORM_SCALING_FACTOR
            t_points.append(t_point[0][0])
        lines.append(t_points)
    return lines


def flatten_lanes(lanes):
    '''Flatten the list of lane markings into a single dimensional array for messaging.
    
    @param lanes: a list of lists of tuples, each list of tuples represents a line
    @return a flat list of tuples to be published to a ROS topic
    '''
    flattened = []
    for row in lanes:
        for point in row:
            flattened.append(float(point[0]))
            flattened.append(float(point[1]))
    return flattened


def unflatten_lanes(lanes, row_lengths):
    '''Unflatten the flat list of lane markings into a list of lists of tuples, each list of tuples represents a line.

    @param lanes: a list of lists of tuples, each list of tuples represents a line
    @param row_lengths: a list of integers representing the number of points in each lane
    @return a list of lists of tuples, each list of tuples represents a line
    '''
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
    return unflat
