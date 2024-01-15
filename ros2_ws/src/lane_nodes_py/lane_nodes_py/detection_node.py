import rclpy

from clrnet.utils.config import Config

from rclpy.node import Node

from lane_interfaces.msg import LaneLocation, Image

from .detection import Detection

CONFIG = '/opt/clrnet/configs/clrnet/clr_resnet18_tusimple.py'
MODEL = '/opt/clrnet/models/tusimple_r18.pth'
IMAGE_DIR = '/imgs/in'
OUTPUT_DIR = '/imgs/out'

class DetectionNode(Node):

    def __init__(self):
        super().__init__('detection')

        cfg = Config.fromfile(CONFIG)
        cfg.show = False
        cfg.savedir = OUTPUT_DIR
        cfg.load_from = MODEL
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

        # TODO: call detect.run() on the received image

        self.get_logger().info('Received message: "%s"' % msg.temp)
        lane_msg = LaneLocation()
        lane_msg.temp = msg.temp
        self.lane_publisher_.publish(lane_msg)

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
