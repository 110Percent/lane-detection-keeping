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
        self.get_logger().info('img type' + str(type(cv_image)))
        self.get_logger().info('img shape:' + str(cv_image.shape))

        #self.get_logger().info('Received image')
        detected = self.detection.run_raw(cv_image)

        lanes = detected['lanes']
        self.get_logger().info('type lanes[0]: ' + str(type(lanes[0])))
        # self.get_logger().info('lanes:' + str(lanes))
        # bev_lanes = self.lanes_to_birds_eye(lanes, cv_image)
        flat_lanes = self.flatten_lanes(lanes)
        
        # self.get_logger().info('len lanes:' + str(len(lanes)))
        # self.get_logger().info('lanes:' + str(lanes))

        # self.get_logger().info('len flat:' + str(len(flat_lanes)))

        # self.get_logger().info('lane1:' + str(lanes[0]))
        # self.get_logger().info('lane2:' + str(lanes[1]))

        lane_data = LaneLocation()
        lane_data.stamp = str(datetime.now())
        lane_data.frame_id = "FILENAME"
        lane_data.row_lengths = [len(lane.points) for lane in lanes]
        lane_data.lanes = flat_lanes
        lane_data.img_shape = [cv_image.shape[0], cv_image.shape[1]]

        # self.get_logger().info('Sent: ' + str(lane_date))
        self.lane_publisher_.publish(lane_data)
    
    def flatten_lanes(self, lanes):
        flattened = []
        for row in lanes:
            for point in row:
                flattened.append(point[0])
                flattened.append(point[1])
        return flattened




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
