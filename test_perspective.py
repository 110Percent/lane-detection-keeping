# The following is derived from https://github.com/JonathanCMitchell/Advanced-Lane-Line-Detection/blob/master/Perspective_transform.ipynb

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import matplotlib.gridspec as gridspec
import cv2
import numpy as np
import pickle as pickle
import glob
import lane

ORIGINAL_SIZE = 695, 1263
UNWARPED_SIZE = 695, 1263


example_images = ['./test_images/straight_lines1.jpg', './test_images/straight_lines2.jpg']
roi_points = np.array([[0, ORIGINAL_SIZE[1] - 50],
                        [ORIGINAL_SIZE[0], ORIGINAL_SIZE[1] - 50],
                        [ORIGINAL_SIZE[0]//2, ORIGINAL_SIZE[1]//2+50]], np.int32)
print('roi_points: ', roi_points)
# Find the region of interest
roi = np.zeros((720, 1280), np.uint8)
print('roi: ', roi.shape)
cv2.fillPoly(roi, [roi_points], 1)


img = mpimg.imread('ikea.png')

plt.imshow(img)

print('img: ', img.shape)

low_thresh = 100
high_thresh = 200

Lhs = np.zeros((2, 2), dtype = np.float32)
Rhs = np.zeros((2, 1), dtype = np.float32)
x_max = 0
x_min = 2555

lanes = lane.lanes

# print('lanes[0]: ', lanes[0])
for line in lanes:
    x_list = line[::2]
    y_list = line[1::2]
    for point1, point2 in zip(x_list, y_list):
        x1, y1 = point1
        x2, y2 = point2

        x1 = int(x1 *ORIGINAL_SIZE[1])
        x2 = int(x2 *ORIGINAL_SIZE[1])
        y1 = int(y1 *ORIGINAL_SIZE[0])
        y2 = int(y2 *ORIGINAL_SIZE[0])

        # print(x1, y1)
        # print(x2, y2)
        # Find the norm (the distances between the two points)
        normal = np.array([[-(y2-y1)], [x2-x1]], dtype = np.float32) # question about this implementation
        normal = normal / np.linalg.norm(normal)
        
        pt = np.array([[x1], [y1]], dtype = np.float32)
        
        outer = np.matmul(normal, normal.T)
        
        Lhs += outer
        Rhs += np.matmul(outer, pt) #use matmul for matrix multiply and not dot product

        cv2.line(img, (x1, y1), (x2, y2), (255, 0, 0), thickness = 1)
        
        x_iter_max = max(x1, x2)
        x_iter_min = min(x1, x2)
        x_max = max(x_max, x_iter_max)
        x_min = min(x_min, x_iter_min)

width = x_max - x_min
print('width: ', width, 'x_max: ', x_max, 'x_min: ', x_min)

vp = np.matmul(np.linalg.inv(Lhs), Rhs)

print('vp: ', vp)
plt.plot(vp[0], vp[1], 'c^')
plt.imshow(img)
plt.title('Vanishing Point')

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

top = vp[1] + 65
bottom = ORIGINAL_SIZE[1] - 40

width = 500

print('vp[0]', vp[0], type(vp[0][0]))
print('vp[1]', vp[1], type(vp[1][0]))

p1 = [vp[0] - width/2, top]
p2 = [vp[0] + width/2, top]
p3 = find_pt_inline(p2, vp, bottom)
p4 = find_pt_inline(p1, vp, bottom)

print('******\n', p1, p2, p3, p4, '\n**********************')
print(len(p1), len(p2), len(p3), len(p4))

src_pts = np.array([p1, p2, p3, p4], dtype=object)

dst_pts = np.float32([[0, 0], [UNWARPED_SIZE[0], 0],
                        [UNWARPED_SIZE[0], UNWARPED_SIZE[1]],
                        [0, UNWARPED_SIZE[1]]])

cv2.polylines(img, [src_pts.astype(np.int32)],True, (0,200,100), thickness=5)
plt.plot(p1[0], p1[1], 'r+')
plt.plot(p2[0], p2[1], 'c^')
plt.plot(p3[0], p3[1], 'r^')
plt.plot(p4[0], p4[1], 'g^')
plt.title('Trapezoid For Perspective Transform')
plt.imshow(img)

# H is the homography matrix

src_pts = np.float32(src_pts)
M_inv = cv2.getPerspectiveTransform(dst_pts, src_pts)
M = cv2.getPerspectiveTransform(src_pts, dst_pts)
warped = cv2.warpPerspective(img, M, UNWARPED_SIZE)
# warped_lines = cv2.warpPerspective(lanes, M, UNWARPED_SIZE)
plt.clf()
# plt.imshow(warped)

for line in lanes:
    warped_line = cv2.warpPerspective(np.array(line), M, UNWARPED_SIZE)
    print('warped line: ', warped_line.shape)
    print('warped line: ', warped_line)
    x_list = warped_line[::2]
    y_list = warped_line[1::2]
    for point1, point2 in zip(x_list, y_list):
        x1, y1 = point1
        x2, y2 = point2
        
        x1 = int(x1 *ORIGINAL_SIZE[1])
        x2 = int(x2 *ORIGINAL_SIZE[1])
        y1 = int(y1 *ORIGINAL_SIZE[0])
        y2 = int(y2 *ORIGINAL_SIZE[0])
        cv2.line(warped, (x1, y1), (x1, y1), (255, 0, 0), thickness = 5)

plt.imshow(warped)