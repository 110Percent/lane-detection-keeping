import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import os
import cv2

filedir = os.path.dirname(__file__)
target_directory = os.path.join(filedir, 'test_videos')
video_filepath = os.path.join(target_directory, 'carleton-drive.mp4')

# Outputs all the images
class ImageGenerationNode(Node):
    def __init__(self):
        super().__init__('image generation')

        # Create the publisher for sending frames to 
        self.image_publisher_ = self.create_publisher(Image, "/video_frames", 10)
        
        # Create the cap for the video
        self.cap = cv2.VideoCapture(video_filepath)
        # Create the OpenCV-ROS bridge
        self.bridge = CvBridge()


    # Go through the video, outputting each frame as a sensor_msgs/msg/Image message to the /video_frames topic
    def run(self):
        while(self.cap.isOpened()):
            ret, frame = self.cap.read()
            if ret:
                self.image_publisher.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            else:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

        self.cap.release()

def main(args=None):
    rclpy.init(args=args)

    image_generation_node = ImageGenerationNode()

    rclpy.spin(image_generation_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_generation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
