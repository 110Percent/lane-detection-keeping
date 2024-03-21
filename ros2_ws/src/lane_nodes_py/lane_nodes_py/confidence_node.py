import rclpy
from rclpy.node import node

from sensor_msgs.msg import Image

from cv_bridge import CvBridge
from .confidence import Confidence
import cv2

class ConfidenceNode(Node):
    def __init__(self):
        super().__init__('confidence')
        
        self.image_subscriber = self.create_subscription(
                Image,
                '/video_frames',
                self.image_callback,
                10)

        self.bridge = CvBridge()
        self.confidence = Confidence()

    def image_callback(self, msg):
        self.confidence.process_image(self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8"))

        
