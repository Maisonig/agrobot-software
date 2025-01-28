import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid


class ImageToOccupancyNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name=node_name)
        self.mainTimer = self.create_timer(timer_period_sec=0.1,
                                           callback=self.main_timer_callback)

        self.ColorImageSubscription = self.create_subscription(msg_type=Image,
                                                               topic="/camera/depth/rect_raw",
                                                               callback=self.color_image_callback,
                                                               qos_profile=10)

        self.DepthImageSubscription = self.create_subscription(msg_type=Image,
                                                               topic="/camera/color/rect_raw",
                                                               callback=self.depth_image_callback,
                                                               qos_profile=10)

        self.OccupancyGridPublisher = self.create_publisher(msg_type=OccupancyGrid,
                                                            topic="/map",
                                                            qos_profile=10)

        self.ColorImage = Image()
        self.DepthImage = Image()
        self.OutputGrid = OccupancyGrid()

    def main_timer_callback(self):
        msg = OccupancyGrid()

        pass

        self.OccupancyGridPublisher.publish(msg)

    def color_image_callback(self, msg: Image):
        self.ColorImage = msg

    def depth_image_callback(self, msg: Image):
        self.DepthImage = msg


def main(args=None):
    rclpy.init(args=args)
    node = ImageToOccupancyNode("image_to_occupancy_node")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
