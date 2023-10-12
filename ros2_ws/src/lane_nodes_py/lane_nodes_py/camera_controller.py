import rclpy

from rclpy.node import Node

from lane_interfaces.msg import Image



class CameraController(Node):

    def __init__(self):
        super().__init__('camera_controller')
        
        # Create the publisher for sending image data
        self.image_publisher_ = self.create_publisher(Image, "image", 10)
        
        # Create the subscriber for testing the ROS2 boilerplate. 
        self.test_subscription = self.create_subscription(
                Image,
                'test_topic',
                self.test_callback,
                10)
        self.test_subscription # prevent unused variable warning

    # Callback for testing that the ROS2 boilerplate works. In reality, this node will receive input from a camera.
    def test_callback(self, msg):
        self.get_logger().info('Received message: "%s"' % msg.temp)
        self.image_publisher_.publish(msg)

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
