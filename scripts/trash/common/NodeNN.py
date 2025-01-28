import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_broadcaster import TransformBroadcaster
import cv2


image = cv2.imread("agro_platform/occupancy_grid.png")
image = image[150:450, 100:520, :]
image = cv2.flip(image, 1)
image = cv2.resize(image, (80, 80))


class DemoNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer2 = self.create_timer(0.1, self.timer_callback2)
        self.pub = self.create_publisher(OccupancyGrid, 'map1', 10)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = image.shape[0] / 2 / 10
        t.transform.translation.y = image.shape[0] / 2 / 10
        t.transform.translation.z = 0.
        t.transform.rotation.x = 0.
        t.transform.rotation.y = 0.
        t.transform.rotation.z = 0.
        t.transform.rotation.w = 1.
        self.tf_broadcaster.sendTransform(t)

    def timer_callback(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        print(t.transform.translation.x)
        self.tf_broadcaster.sendTransform(t)

        o = OccupancyGrid()
        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id = "odom"
        img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        w = img.shape[0]
        h = img.shape[1]
        img = img.ravel()
        img = np.array(img, dtype=int)
        img = list(img)
        arr = []
        for i in img:
            if i < 126:
                arr.append(100)
            elif 126 <= i <= 128:
                arr.append(-1)
            else:
                arr.append(0)

        o.data = arr
        o.info = MapMetaData()
        o.info.height = h
        o.info.width = w
        o.info.resolution = 0.1

        self.pub.publish(o)

    def timer_callback2(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_footprint'
        t.child_frame_id = 'camera_link'
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = DemoNode('DemoNode')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
