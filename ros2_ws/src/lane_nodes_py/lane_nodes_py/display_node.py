import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from lane_interfaces.msg import LaneLocation


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
        self.get_logger().info('Received image')
        self.base_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")

    def lane_location_callback(self, msg):
        self.get_logger().info('Received lane location data')
        self.lane_coords = self.unflatten_lanes(msg.lanes, msg.row_lengths, msg.img_shape)


    def unflatten_lanes(self, lanes, row_lengths, img_shape):
        unflat = []
        for length in row_lengths:
            flat_lane = lanes[:length * 2]
            del lanes[:length * 2]
            x_vals = []
            y_vals = []
            for i, val in enumerate(flat_lane):
                if i % 2 == 0:
                    x_vals.append(int(val * img_shape[1]))
                else:
                    y_vals.append(int(val * img_shape[0]))
            unflat.append(list(zip(x_vals, y_vals)))
        # self.get_logger().info('parsed lanes: ' + str(unflat))
        # for i, lane in enumerate(unflat):
        #     self.get_logger().info('lane ' + str(i) + ': length=' + str(len(lane)) + ' points=' + str(lane))
        return unflat

    def update_frame_callback(self):
        if self.base_image is None or self.lane_coords is None:
            return
        self.get_logger().info('Updating frame')
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
