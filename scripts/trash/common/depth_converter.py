import time
from math import sqrt, atan2, degrees

import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy, read_points


class DepthConverter(Node):
    """
    Node to convert depth camera PointCloud2 data to LaserScan data
    """

    def __init__(self, node_name):
        super().__init__(node_name)

        # Declare class variables for ros2 parameter names
        self.PointCloudParameterName = 'point_cloud_topic_name'
        self.LaserScanParameterName = 'laser_scan_topic_name'

        # Declare class variables for ros2 parameters
        self.PointCloudTopicName = '/camera/depth/color/points'
        self.LaserScanTopicName = '/scan'

        # Declare ros2 parameters in ecosystem
        self.declare_parameter(self.PointCloudParameterName, self.PointCloudTopicName)
        self.declare_parameter(self.LaserScanParameterName, self.LaserScanTopicName)

        # Declare class variables for serialized PC2 and LS messages
        self.LaserScanMessage = None
        self.PointCloud2Message = None

        # Create subscription for getting PointCloud2 data
        self.PointCloudSubscription = self.create_subscription(
            PointCloud2,
            self.get_parameter(self.PointCloudParameterName).get_parameter_value().string_value,
            self.process_callback,
            10)

        # Create timer for main loop
        self.TimerPeriod = 0.1
        self.MainTimer = self.create_timer(self.TimerPeriod, self.process_timer)

        # Create publisher for LaserScan message
        self.LaserScanPublisher = self.create_publisher(
            LaserScan,
            self.get_parameter(self.LaserScanParameterName).get_parameter_value().string_value,
            10
        )

    def process_callback(self, msg):
        self.PointCloud2Message = msg
        self.pointcloud_to_laserscan(self.PointCloud2Message)

    def process_timer(self):
        pass

    def check_parameters(self):
        pass

    def pointcloud_to_laserscan(self, cloud):

        t1 = time.time()
        # Вычитать все значения точек в формате (x, y, z) в виде ndarray
        cloud = read_points_numpy(cloud)[:, :3]
        # Определение всех параметров
        height_min = 0.
        height_max = 2.
        range_min = 0.4
        range_max = 10.
        angle_min = -0.7
        angle_max = 0.7
        angle_increment = 0.03
        # Сепарировать точки разных осей по разным массивам
        x_values = cloud[:, 0]
        y_values = cloud[:, 1]
        z_values = cloud[:, 2]
        # Отсеять точки с аномальными значениями глубины
        indexes3 = np.argwhere(z_values >= range_max)
        y_values = np.delete(y_values, indexes3)
        x_values = np.delete(x_values, indexes3)
        z_values = np.delete(z_values, indexes3)
        # Отсеять точки не проходящие по высоте
        indexes1 = np.argwhere(y_values >= height_max)
        indexes2 = np.argwhere(y_values <= height_min)
        indexes = np.concatenate((indexes1, indexes2), 0)
        y_values = np.delete(y_values, indexes)
        x_values = np.delete(x_values, indexes)
        z_values = np.delete(z_values, indexes)

        angles = []
        angle = angle_min
        while angle <= angle_max + angle_increment:
            angles.append(angle)
            angle += angle_increment
        ranges = [[range_max] for i in angles]

        for x, y, z in zip(x_values, y_values, z_values):
            depth = sqrt(x ** 2 + z ** 2)
            if range_min <= depth <= range_max:
                angle = atan2(x, z)
                if angle_min <= angle <= angle_max:
                    index, angle = min(enumerate(angles), key=lambda th: abs(th[1] - angle))
                    ranges[index].append(depth)

        new_ranges = []
        for range_row in ranges:
            new_ranges.append(min(range_row))

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.angle_min = angle_min
        msg.angle_max = angle_max
        msg.angle_increment = angle_increment
        msg.range_min = range_min
        msg.range_max = range_max
        msg.ranges = new_ranges
        self.LaserScanPublisher.publish(msg)

        print(f"Затраченное время n: {time.time() - t1}")

    def pointcloud_to_laserscan_working(self, cloud):
        """
        Это работает, но медленно, 1.8 сек на 1 итерацию
        :param cloud:
        :return:
        """

        t1 = time.time()

        cloud = read_points_numpy(cloud)[:, :3]

        range_min = 0.4
        range_max = 10.
        angle_min = -1.
        angle_max = 1.
        angle_increment = 0.1

        x_values = cloud[:, 0]
        y_values = cloud[:, 1]
        z_values = cloud[:, 2]

        # x_min_index = x_values.index(min(x_values))

        # x_raw_values = cloud[:, 0]
        # y_raw_values = cloud[:, 1]
        # z_raw_values = cloud[:, 2]

        # x_values, y_values, z_values = zip(*[(x_raw_values, y_raw_values, z_raw_values)
        #                                      for x_raw_values, y_raw_values, z_raw_values in
        #                                      sorted(zip(x_raw_values, y_raw_values, z_raw_values))])

        angles = []
        angle = angle_min
        while angle < angle_max:
            angles.append(angle)
            angle += angle_increment

        ranges = [[range_max] for i in angles]

        for x, y, z in zip(x_values, y_values, z_values):
            depth = sqrt(x ** 2 + z ** 2)
            if range_min <= depth <= range_max:
                angle = atan2(x, z)
                if angle_min <= angle <= angle_max:
                    index, angle = min(enumerate(angles), key=lambda th: abs(th[1] - angle))
                    ranges[index].append(depth)

        new_ranges = []
        for range_row in ranges:
            new_ranges.append(min(range_row))

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.angle_min = angle_min
        msg.angle_max = angle_max
        msg.angle_increment = angle_increment
        msg.range_min = range_min
        msg.range_max = range_max
        msg.ranges = new_ranges
        self.LaserScanPublisher.publish(msg)

        print(f"Затраченное время n: {time.time() - t1}")


def main(args=None):
    rclpy.init(args=args)
    node = DepthConverter('depth_converter_node')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
