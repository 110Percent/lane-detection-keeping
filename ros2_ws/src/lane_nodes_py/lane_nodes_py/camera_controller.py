import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Image

import numpy as np

import cv2

from cv_bridge import CvBridge, CvBridgeError


cv_bridge = CvBridge()

class CameraController(Node):

    def __init__(self):
        super().__init__('camera_controller')
        
        # Create the publisher for sending image data
        # self.image_publisher_ = self.create_publisher(Image, "image", 10)
        
        # Create the subscriber for testing the ROS2 boilerplate. 
        self.test_subscription = self.create_subscription(
                Image,
                '/carla/ego_vehicle/rgb_front/image',
                self.test_callback,
                10)
        self.test_subscription # prevent unused variable warning

    # Callback for testing that the ROS2 boilerplate works. In reality, this node will receive input from a camera.
    def test_callback(self, msg):
        # self.get_logger().info('Received message: "%s"' % msg)
        cv_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        cv_image = cv2.Canny(cv_image, 100, 200)
        cv2.imshow("test", cv_image)
        cv2.waitKey(1)
        # self.image_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    camera_controller = CameraController()

    rclpy.spin(camera_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
