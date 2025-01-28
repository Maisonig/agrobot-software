import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from image_occupancy import image_to_occupancy


class ImageToOccupancy(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.mainTimer = self.create_timer(timer_period_sec=0.1,
                                           callback=self.main_timer_callback)

        self.ColorImageSubscription = self.create_subscription(msg_type=Image,
                                                               topic="camera/color",
                                                               callback=self.color_image_callback,
                                                               qos_profile=10)

        self.DepthImageSubscription = self.create_subscription(msg_type=Image,
                                                               topic="camera/depth",
                                                               callback=self.depth_image_callback,
                                                               qos_profile=10)

        self.OccupancyGridPublisher = self.create_publisher(msg_type=OccupancyGrid,
                                                            topic="/map",
                                                            qos_profile=10)

        self.CvBridge = CvBridge()

        self.ColorImage = Image()
        self.DepthImage = Image()
        self.OutputGrid = OccupancyGrid()

    def main_timer_callback(self):

        try:
            color_image = self.CvBridge.imgmsg_to_cv2(self.ColorImage)
            depth_image = self.CvBridge.imgmsg_to_cv2(self.DepthImage)
            occupancy = image_to_occupancy(color_image, depth_image)[:200, :]

            msg = OccupancyGrid()
            msg.header.frame_id = "base_link"
            msg.header.stamp = self.get_clock().now().to_msg()

            msg.info.width = occupancy.shape[0]
            msg.info.height = occupancy.shape[1]
            msg.info.resolution = 0.1

            arr = occupancy.flatten('F')
            for element in arr:
                if element == 0:
                    msg.data.append(100)
                elif element == 255:
                    msg.data.append(0)
                else:
                    msg.data.append(-1)

            self.OccupancyGridPublisher.publish(msg)
        except CvBridgeError as e:
            print(e)

    def color_image_callback(self, msg: Image):
        self.ColorImage = msg

    def depth_image_callback(self, msg: Image):
        self.DepthImage = msg


def main(args=None):
    rclpy.init(args=args)
    node = ImageToOccupancy("image_to_occupancy_node")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
