import datetime

import rclpy
import torch


from clrnet.utils.config import Config

from rclpy.node import Node

from lane_interfaces.msg import LaneLocation

from sensor_msgs.msg import Image, Header

from cv_bridge import CvBridge, CvBridgeError

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
        cfg.show = True
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
                'image',
                self.image_callback,
                10)
        self.image_subscription # prevent unused variable warning

    # Callback for receiving image data. At the moment, just echoes the received message, which is a string.
    def image_callback(self, msg):

        cv_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        self.get_logger().info('Received image')
        lanes = self.detection.run_raw(cv_image)

        msg = LaneLocation()
        header = Header()
        header.stamp = datetime.now()
        header.frame_id = "FILENAME"
        header.seq = 0

        msg.header = header
        msg.lanes = lanes

        self.lane_publisher_.publish(msg)


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
