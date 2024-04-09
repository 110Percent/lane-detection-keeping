from datetime import datetime

import rclpy
import torch
import numpy as np

from clrnet.utils.config import Config
from rclpy.node import Node
from lane_interfaces.msg import LaneLocation
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import ros2_ws.src.lane_nodes_py.lane_nodes_py.transforms.line_transforms as line_transforms
import cv2

from .detection.detection import Detection

# These paths are hardcoded for now, but should be moved to a config file or something similar.
# We use /opt/clrnet because that's where CLRNet is installed in the Docker container.
CONFIG = "/opt/clrnet/configs/clrnet/clr_resnet18_tusimple.py"
MODEL = "/opt/clrnet/models/tusimple_r18.pth"
# IMAGE_DIR and OUTPUT_DIR don't do anything since we're getting images from ROS, but I'm just writing comments as we
# wrap up the project and don't want to end up breaking anything. Probably safe to remove them though.
IMAGE_DIR = "/imgs/in"
OUTPUT_DIR = "/imgs/out"

cv_bridge = CvBridge()


class DetectionNode(Node):
    def __init__(self):
        super().__init__("detection")

        cfg = Config.fromfile(CONFIG)
        cfg.show = False
        cfg.savedir = OUTPUT_DIR
        cfg.load_from = MODEL
        cfg.ori_img_w = 800
        cfg.ori_img_h = 600
        cfg.cut_height = 320
        self.detection = Detection(cfg)

        # Create the publisher for sending lane location data
        self.lane_publisher_ = self.create_publisher(
            LaneLocation, "lane_location_data", 10
        )

        # Create the subscriber for receiving images
        self.image_subscription = self.create_subscription(
            Image, "raw_input_images", self.image_callback, 10
        )
        self.image_subscription  # prevent unused variable warning

    # Callback for when an image is received. Performs lane detection and sends the results to the lane location publisher.
    def image_callback(self, msg):
        # NOTE: There isn't any mutex on this, meaning too many images sent in rapid succession could crash the node by overloading the GPU.
        # Something to keep in mind for the future that should probably be addressed.
        cv_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")

        # Actual detection magic happens here, see detection.py
        detected = self.detection.run_raw(cv_image)

        lanes = detected["lanes"]

        if lanes is None:
            return
        if len(lanes) < 1:
            return

        # bev_lanes = self.lanes_to_birds_eye(lanes, cv_image)
        flat_lanes = line_transforms.flatten_lanes(lanes)

        lane_data = LaneLocation()
        lane_data.stamp = str(datetime.now())
        lane_data.frame_id = "FILENAME"  # Is this supposed to be constant?
        lane_data.row_lengths = [len(lane.points) for lane in lanes]
        lane_data.lanes = flat_lanes
        lane_data.img_shape = [cv_image.shape[0], cv_image.shape[1]]

        self.lane_publisher_.publish(lane_data)


def main(args=None):
    rclpy.init(args=args)

    detection_node = DetectionNode()

    rclpy.spin(detection_node)

    detection_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
