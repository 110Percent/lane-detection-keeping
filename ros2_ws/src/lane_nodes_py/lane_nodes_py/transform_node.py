import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import matplotlib.pyplot as plt

from lane_interfaces.msg import LaneLocation

PID_FREQUENCY = 2


class TransformNode(Node):

    def __init__(self):
        super().__init__('transform')

        # Create the publisher for sending lane location data
        self.lane_publisher_ = self.create_publisher(LaneLocation, "bev_lane_location_data", 10)

        # Create the subscriber for receiving lane data
        self.lane_subscription = self.create_subscription(
            LaneLocation,
            'lane_location_data',
            self.lane_location_callback,
            10)
        
    def lane_location_callback(self, msg):
        stamp = msg.stamp
        id = msg.frame_id
        lanes = msg.lanes
        row_lengths = msg.row_lengths
        img_shape = msg.img_shape

        unflat = self.unflatten_lanes(lanes, row_lengths)

        # self.get_logger().info('unflat: ' + str(unflat))

        bev = self.lanes_to_birds_eye(unflat, img_shape)

        # self.get_logger().info('bev: ' + str(bev))

        bev_flat = self.flatten_lanes(bev)

        # self.get_logger().info('bev_flat: ' + str(bev_flat))

        lane_data = LaneLocation()
        lane_data.stamp = stamp
        lane_data.frame_id = id
        lane_data.row_lengths = row_lengths
        lane_data.lanes = bev_flat
        lane_data.img_shape = img_shape

        self.lane_publisher_.publish(lane_data)
        self.get_logger().info('Sent Transformed lane')

    def lanes_to_birds_eye(self, lanes, img_shape):
        Lhs = np.zeros((2, 2), dtype = np.float32)
        Rhs = np.zeros((2, 1), dtype = np.float32)
        x_max = 0
        x_min = 2555

        # self.get_logger().info('lanes:' + str(lanes))
        for line in lanes:
            x_list = line[::2]
            y_list = line[1::2]
            for point1, point2 in zip(x_list, y_list):
                x1, y1 = point1
                x2, y2 = point2

                x1 = int(x1 *img_shape[1])
                x2 = int(x2 *img_shape[1])
                y1 = int(y1 *img_shape[0])
                y2 = int(y2 *img_shape[0])

                # print(x1, y1)
                # print(x2, y2)
                # Find the norm (the distances between the two points)
                normal = np.array([[-(y2-y1)], [x2-x1]], dtype = np.float32) # question about this implementation
                normal = normal / np.linalg.norm(normal)
                
                pt = np.array([[x1], [y1]], dtype = np.float32)
                
                outer = np.matmul(normal, normal.T)
                
                Lhs += outer
                Rhs += np.matmul(outer, pt) #use matmul for matrix multiply and not dot product
                
                x_iter_max = max(x1, x2)
                x_iter_min = min(x1, x2)
                x_max = max(x_max, x_iter_max)
                x_min = min(x_min, x_iter_min)

        width = x_max - x_min
        print('width: ', width, 'x_max: ', x_max, 'x_min: ', x_min)

        vp = np.matmul(np.linalg.inv(Lhs), Rhs)

        def find_pt_inline(p1, p2, y):
            """
            Here we use point-slope formula in order to find a point that is present on the line
            that passes through our vanishing point (vp). 
            input: points p1, p2, and y. They come is as tuples [x, y]
            We then use the point-slope formula: y - b = m(x - a)
            y: y-coordinate of desired point on the line
            x: x-coordinate of desired point on the line
            m: slope
            b: y-coordinate of p1
            a: x-coodrinate of p1
            x = p1x + (1/m)(y - p1y)
            """
            m_inv = (p2[0] - p1[0]) / float(p2[1] - p1[1])
            Δy = (y - p1[1])
            x = p1[0] + m_inv * Δy
            return [x, y]

        top = vp[1] + 40
        bottom = img_shape[1] - 40

        width = 600

        print('vp[0]', vp[0], type(vp[0][0]))
        print('vp[1]', vp[1], type(vp[1][0]))

        p1 = [vp[0] - width/2, top]
        p2 = [vp[0] + width/2, top]
        p3 = find_pt_inline(p2, vp, bottom)
        p4 = find_pt_inline(p1, vp, bottom)

        src_pts = np.array([p1, p2, p3, p4], dtype=object)
        dst_pts = np.float32([[0, 0], [img_shape[0], 0],
                        [img_shape[0], img_shape[1]],
                        [0, img_shape[1]]])

        src_pts = np.float32(src_pts)
        M = cv2.getPerspectiveTransform(src_pts, dst_pts)

        # this is very gross and I don't like it either, but it works for now, the formatting of the input is difficult :(
        lines = []
        for line in lanes:
            points = [(x*img_shape[1], y*img_shape[0]) for x,y in line]
            t_points = []
            for point in points:
                t_point = cv2.perspectiveTransform(np.float32([[[point[0],point[1]]]]), M)
                t_point[0][0][0] = (t_point[0][0][0]  - img_shape[0] / 2) * 20
                t_point[0][0][1] = -1*(t_point[0][0][1]  - img_shape[1])
                t_points.append(t_point[0][0])
            lines.append(t_points)
        
        x_points = []
        y_points = []
        plt.clf()
        # cv_image = cv2.imread("/opt/lane-capstone/src/lane_nodes_py/lane_nodes_py/test_images/drawn_lines.png", cv2.IMREAD_COLOR)
        for line in lines:
            for point in line:
                x_points.append(point[0])
                y_points.append(point[1])
            # cv2.polylines(cv_image, [np.array(line).astype(np.int32)], False, (255,0,0), thickness=5)
            plt.plot(x_points, y_points)
            x_points = []
            y_points = []
        plt.savefig("/opt/lane-capstone/src/lane_nodes_py/lane_nodes_py/test_images/figure.png")
        cv_image = cv2.imread("/opt/lane-capstone/src/lane_nodes_py/lane_nodes_py/test_images/figure.png", cv2.IMREAD_COLOR)
        # self.get_logger().info('***********************show lines before**********')
        fig = plt.figure()
        cv2.imshow("lines on dumb image", cv_image)
        cv2.waitKey(300)
        # plt.show()
        self.get_logger().info('***********************show lines after**********')
        return lines

    def flatten_lanes(self, lanes):
        flattened = []
        for row in lanes:
            for point in row:
                flattened.append(float(point[0]))
                flattened.append(float(point[1]))
        return flattened

    def unflatten_lanes(self, lanes, row_lengths):
        unflat = []
        for length in row_lengths:
            flat_lane = lanes[:length * 2]
            del lanes[:length * 2]
            x_vals = []
            y_vals = []
            for i, val in enumerate(flat_lane):
                if i % 2 == 0:
                    x_vals.append(val)
                else:
                    y_vals.append(val)
            unflat.append(list(zip(x_vals, y_vals)))
        # self.get_logger().info('parsed lanes: ' + str(unflat))
        # for i, lane in enumerate(unflat):
        #     self.get_logger().info('lane ' + str(i) + ': length=' + str(len(lane)) + ' points=' + str(lane))
        return unflat


def main(args=None):
    rclpy.init(args=args)

    transform_node = TransformNode()

    rclpy.spin(transform_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    transform_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()