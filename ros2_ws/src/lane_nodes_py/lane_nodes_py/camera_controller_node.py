import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Image

import numpy as np

import cv2

from cv_bridge import CvBridge, CvBridgeError

from .camera_controller import CameraController


cv_bridge = CvBridge()

class CameraControllerNode(Node):

    def __init__(self):
        super().__init__('camera_controller')

        self.camera_controller = CameraController()
        
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
        
        cv_image = camera_controller.canny(cv_image)
        self.camera_controller.show_image(cv_image)

        # self.image_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    camera_controller_node = CameraControllerNode()

    rclpy.spin(camera_controller_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
