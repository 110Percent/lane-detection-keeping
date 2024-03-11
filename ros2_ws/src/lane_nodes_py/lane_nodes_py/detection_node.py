from datetime import datetime

import rclpy
import torch
import numpy as np

from clrnet.utils.config import Config

from rclpy.node import Node

from lane_interfaces.msg import LaneLocation

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

import cv2

from .detection import Detection

CONFIG = '/opt/clrnet/configs/clrnet/clr_resnet18_tusimple.py'
MODEL = '/opt/clrnet/models/tusimple_r18.pth'
IMAGE_DIR = '/imgs/in'
OUTPUT_DIR = '/imgs/out'

cv_bridge = CvBridge()

class DetectionNode(Node):

    def __init__(self):
        super().__init__('detection')

        cfg = Config.fromfile(CONFIG)
        cfg.show = False
        cfg.savedir = OUTPUT_DIR
        cfg.load_from = MODEL
        cfg.ori_img_w = 800
        cfg.ori_img_h = 600
        cfg.cut_height = 320
        self.detection = Detection(cfg)

        # Create the publisher for sending lane location data
        self.lane_publisher_ = self.create_publisher(LaneLocation, "lane_location_data", 10)

        # Create the subscriber for receiving images 
        self.image_subscription = self.create_subscription(
                Image,
                "raw_input_images",
                self.image_callback,
                10)
        self.image_subscription # prevent unused variable warning

    # Callback for receiving image data. At the moment, just echoes the received message, which is a string.
    def image_callback(self, msg):

        cv_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        #self.get_logger().info('Received image')
        detected = self.detection.run_raw(cv_image)

        lanes = detected['lanes']
        bev_lanes = self.lanes_to_birds_eye(lanes, cv_image)
        flat_lanes = self.flatten_lanes(bev_lanes)
        
        # self.get_logger().info('len lanes:' + str(len(lanes)))
        # self.get_logger().info('lanes:' + str(lanes))

        # self.get_logger().info('len flat:' + str(len(flat_lanes)))

        # self.get_logger().info('lane1:' + str(lanes[0]))
        # self.get_logger().info('lane2:' + str(lanes[1]))

        lane_date = LaneLocation()
        lane_date.stamp = str(datetime.now())
        lane_date.frame_id = "FILENAME"
        lane_date.row_lengths = [len(lane.points) for lane in lanes]
        lane_date.lanes = flat_lanes

        # self.get_logger().info('Sent: ' + str(lane_date))
        self.lane_publisher_.publish(lane_date)
    
    def flatten_lanes(self, lanes):
        flattened = []
        for row in lanes:
            for point in row:
                # self.get_logger().info('flattend=' + str(flattened))
                # self.get_logger().info('point=' + str(point[0]) + ',' + str(point[1]))
                # self.get_logger().info('point=' + str(type(point)))
                flattened.append(point[0])
                flattened.append(point[1])
        return flattened

    def lanes_to_birds_eye(self, lanes, img):
        ORIGINAL_SIZE = img.shape[0], img.shape[1]
        UNWARPED_SIZE = ORIGINAL_SIZE
        low_thresh = 100
        high_thresh = 200

        Lhs = np.zeros((2, 2), dtype = np.float32)
        Rhs = np.zeros((2, 1), dtype = np.float32)
        x_max = 0
        x_min = 2555

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

                cv2.line(np.int32(img), (x1, y1), (x2, y2), (255, 0, 0), thickness = 1)
                
                x_iter_max = max(x1, x2)
                x_iter_min = min(x1, x2)
                x_max = max(x_max, x_iter_max)
                x_min = min(x_min, x_iter_min)

        width = x_max - x_min
        print('width: ', width, 'x_max: ', x_max, 'x_min: ', x_min)

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

        top = vp[1] + 40
        bottom = ORIGINAL_SIZE[1] - 40

        width = 600

        print('vp[0]', vp[0], type(vp[0][0]))
        print('vp[1]', vp[1], type(vp[1][0]))

        p1 = [vp[0] - width/2, top]
        p2 = [vp[0] + width/2, top]
        p3 = find_pt_inline(p2, vp, bottom)
        p4 = find_pt_inline(p1, vp, bottom)

        src_pts = np.array([p1, p2, p3, p4], dtype=object)
        dst_pts = np.float32([[0, 0], [UNWARPED_SIZE[0], 0],
                        [UNWARPED_SIZE[0], UNWARPED_SIZE[1]],
                        [0, UNWARPED_SIZE[1]]])

        src_pts = np.float32(src_pts)
        M_inv = cv2.getPerspectiveTransform(dst_pts, src_pts)
        M = cv2.getPerspectiveTransform(src_pts, dst_pts)
        print('M.shape=',M.shape)
        print('M',M)
        warped = cv2.warpPerspective(img, M, UNWARPED_SIZE)
        print('warped.shape=',warped.shape)
        print('warped: ', warped)

        # this is very gross and I don't like it either, but it works for now, the formatting of the input is difficult :(
        lines = []
        for line in lanes:
            points = [(x*ORIGINAL_SIZE[1], y*ORIGINAL_SIZE[0]) for x,y in line]
            t_points = []
            for point in points:
                t_point = cv2.perspectiveTransform(np.float32([[[point[0],point[1]]]]), M)
                print(t_point)
                t_points.append(t_point[0][0])
            lines.append(t_points)

        print(t_points)
        for line in lines:
            cv2.polylines(warped, [np.array(line).astype(np.int32)], False, (255,0,0), thickness=10)


def main(args=None):
    rclpy.init(args=args)

    detection_node = DetectionNode()

    rclpy.spin(detection_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
       # when the garbage collector destroys the node object)
    detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
