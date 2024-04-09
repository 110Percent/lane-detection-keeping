import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from lane_interfaces.msg import LaneLocation
import ros2_ws.src.lane_nodes_py.lane_nodes_py.transforms.line_transforms as line_transforms


cv_bridge = CvBridge()
WINDOW_NAME = "Lane Detection and Keeping"
FRAME_TIME = 0.05


# ROS 2 node for the HUD
class DisplayNode(Node):
    def __init__(self):
        super().__init__('display')
        self.base_image = None
        self.lane_coords = None
        self.frame_contents = None
        self.base_image_sub = self.create_subscription(Image, 'raw_input_images', self.background_callback, 10)
        self.lane_location_sub = self.create_subscription(LaneLocation, 'lane_location_data', self.lane_location_callback, 10)
        self.timer = self.create_timer(FRAME_TIME, self.update_frame_callback)

    def background_callback(self, msg):
        # self.get_logger().info('Received image')
        self.base_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")

    def lane_location_callback(self, msg):
        # self.get_logger().info('Received lane location data')
        self.lane_coords = line_transforms.unflatten_lanes(msg.lanes, msg.row_lengths, msg.img_shape)

    def update_frame_callback(self):
        if self.base_image is None or self.lane_coords is None:
            return
        # self.get_logger().info('Updating frame')
        self.frame_contents = self.base_image.copy()
        for lane in self.lane_coords:
            for i in range(1, len(lane)):
                cv2.line(self.frame_contents, lane[i - 1], lane[i], (0, 255, 0), 2)
        cv2.imshow(WINDOW_NAME, self.frame_contents)
        cv2.waitKey(60)


def main(args=None):
    rclpy.init(args=args)

    display_node = DisplayNode()

    rclpy.spin(display_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    display_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
