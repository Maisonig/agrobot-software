#!/usr/bin/env python3

# Рабочий код для преобразования данных лидара скан и точек распознанных линий в сетку занятости рос2

import cv2
import time
import math
import rclpy
import numpy as np
import ros2_numpy as rnp

from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2, LaserScan, Image
from geometry_msgs.msg import TransformStamped, Polygon
from tf2_py import TransformException
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener

UNKNOWN_CELL = -1
FREE_CELL = 0
OBSTACLE_CELL = 100

focalX = 1451
focalY = 2312
centerX = 640
centerY = 360
cameraDefaultIntrinsics = [[3056.9775390625, 0.0, 1870.62060546875],
                           [0.0, 3056.9775390625, 1087.102294921875],
                           [0.0, 0.0, 1.0]]


def transform_image_coord(u, depth):
    dmx = np.sqrt((centerX - u) ** 2 + focalX ** 2)
    y = depth * (centerX - u) / dmx
    x = depth
    return x, y


def polar_to_decart(rho, phi):
    return rho * np.cos(phi), rho * np.sin(phi)


def decart_to_polar(x, y):
    return np.sqrt(x ** 2 + y ** 2), np.arctan2(y, x)


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk
    q = np.empty((4,))
    q[0] = cj * sc - sj * cs
    q[1] = cj * ss + sj * cc
    q[2] = cj * cs - sj * sc
    q[3] = cj * cc + sj * ss
    return q


class LocalMapping(Node):

    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.declare_parameter('frequency', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('front_detection_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('back_detection_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('front_depth_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('back_depth_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('front_scan_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('back_scan_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('odom_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('grid_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('grid_frame', rclpy.Parameter.Type.STRING)
        self.declare_parameter('robot_frame', rclpy.Parameter.Type.STRING)
        self.declare_parameter('front_camera_frame', rclpy.Parameter.Type.STRING)
        self.declare_parameter('back_camera_frame', rclpy.Parameter.Type.STRING)
        self.declare_parameter('grid_size', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('grid_resolution', rclpy.Parameter.Type.DOUBLE)

        params = [
            rclpy.Parameter('frequency', rclpy.Parameter.Type.INTEGER, 30),
            rclpy.Parameter('front_detection_topic', rclpy.Parameter.Type.STRING, '/agro_bot/front_camera/detection'),
            rclpy.Parameter('back_detection_topic', rclpy.Parameter.Type.STRING, '/agro_bot/back_camera/detection'),
            rclpy.Parameter('front_depth_topic', rclpy.Parameter.Type.STRING, '/agro_bot/front_camera/image/depth'),
            rclpy.Parameter('back_depth_topic', rclpy.Parameter.Type.STRING, '/agro_bot/back_camera/image/depth'),
            rclpy.Parameter('front_scan_topic', rclpy.Parameter.Type.STRING, '/agro_bot/front_camera/scan'),
            rclpy.Parameter('back_scan_topic', rclpy.Parameter.Type.STRING, '/agro_bot/back_camera/scan'),
            rclpy.Parameter('odom_topic', rclpy.Parameter.Type.STRING, '/agro_bot/odom'),
            rclpy.Parameter('grid_topic', rclpy.Parameter.Type.STRING, '/agro_bot/local_map'),
            rclpy.Parameter('grid_frame', rclpy.Parameter.Type.STRING, 'odom'),
            rclpy.Parameter('robot_frame', rclpy.Parameter.Type.STRING, 'base_footprint'),
            rclpy.Parameter('front_camera_frame', rclpy.Parameter.Type.STRING, 'front_camera_link'),
            rclpy.Parameter('back_camera_frame', rclpy.Parameter.Type.STRING, 'back_camera_link'),
            rclpy.Parameter('grid_size', rclpy.Parameter.Type.INTEGER, 26),
            rclpy.Parameter('grid_resolution', rclpy.Parameter.Type.DOUBLE, 0.1),

        ]

        self.set_parameters(params)

        frequency = self.get_parameter('frequency').get_parameter_value().integer_value
        front_detection_topic = self.get_parameter('front_detection_topic').get_parameter_value().string_value
        back_detection_topic = self.get_parameter('back_detection_topic').get_parameter_value().string_value
        front_depth_topic = self.get_parameter('front_depth_topic').get_parameter_value().string_value
        back_depth_topic = self.get_parameter('back_depth_topic').get_parameter_value().string_value
        front_scan_topic = self.get_parameter('front_scan_topic').get_parameter_value().string_value
        back_scan_topic = self.get_parameter('back_scan_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        grid_topic = self.get_parameter('grid_topic').get_parameter_value().string_value
        self.grid_frame = self.get_parameter('grid_frame').get_parameter_value().string_value
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.front_camera_frame = self.get_parameter('front_camera_frame').get_parameter_value().string_value
        self.back_camera_frame = self.get_parameter('back_camera_frame').get_parameter_value().string_value
        self.grid_size = self.get_parameter('grid_size').get_parameter_value().integer_value
        self.grid_resolution = self.get_parameter('grid_resolution').get_parameter_value().double_value

        self.frontDetectionMsg = Polygon()
        self.frontDetectionSub = self.create_subscription(Polygon, front_detection_topic, self.front_detection_callback, 10)
        self.backDetectionMsg = Polygon()
        self.backDetectionSub = self.create_subscription(Polygon, back_detection_topic, self.back_detection_callback, 10)
        self.frontDepthMsg = Image()
        self.frontDepthSub = self.create_subscription(Image, front_depth_topic, self.front_depth_callback, 10)
        self.backDepthMsg = Image()
        self.backDepthSub = self.create_subscription(Image, back_depth_topic, self.back_depth_callback, 10)

        self.frontScanMsg = LaserScan()
        self.frontScanSub = self.create_subscription(LaserScan, front_scan_topic, self.front_scan_callback, 10)
        self.backScanMsg = LaserScan()
        self.backScanSub = self.create_subscription(LaserScan, back_scan_topic, self.back_scan_callback, 10)
        self.odomMsg = Odometry()
        self.odomSub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

        self.gridData = np.zeros(
            (int(self.grid_size / self.grid_resolution), int(self.grid_size / self.grid_resolution)),
            np.uint8) + 127
        self.gridPub = self.create_publisher(OccupancyGrid, grid_topic, 10)
        self.mainTimer = self.create_timer(1 / frequency, self.main_timer)

        self.tfBroadcaster = StaticTransformBroadcaster(self)
        self.refreshGrid = True

        self.front_camera_x = 1.5
        self.front_camera_y = -0.9
        self.front_camera_a = 0.

        self.back_camera_x = -1.5
        self.back_camera_y = -0.9
        self.back_camera_a = 3.1415

    def front_detection_callback(self, msg):
        self.frontDetectionMsg = msg

    def back_detection_callback(self, msg):
        self.backDetectionMsg = msg

    def front_depth_callback(self, msg):
        self.frontDepthMsg = msg

    def back_depth_callback(self, msg):
        self.backDepthMsg = msg

    def front_scan_callback(self, msg):
        self.frontScanMsg = msg

    def back_scan_callback(self, msg):
        self.backScanMsg = msg

    def odom_callback(self, msg):
        self.odomMsg = msg

    def remap_grid_coord(self, x, y):
        x = x + self.odomMsg.pose.pose.position.x
        y = y + self.odomMsg.pose.pose.position.y
        return int(x), int(y)

    def set_scan(self, scan_msg, pose_x, pose_y, pose_a):
        if scan_msg != LaserScan():
            pose_a += scan_msg.angle_min
            angles = [pose_a + i * scan_msg.angle_increment for i in range(len(scan_msg.ranges))]
            grid_center_x = self.gridData.shape[0] / 2
            grid_center_y = self.gridData.shape[1] / 2
            base_x = int(grid_center_x + pose_x / self.grid_resolution)
            base_y = int(grid_center_y - pose_y / self.grid_resolution)
            for rho, phi in zip(scan_msg.ranges, angles):
                if rho != 0.:
                    x, y = polar_to_decart(rho / self.grid_resolution, phi)
                    x = x + base_x
                    y = y + base_y
                    x, y = self.remap_grid_coord(x, y)
                    cv2.line(self.gridData, [base_x, base_y], [x, y], [255], 1)
                    self.gridData[y, x] = 0
                else:
                    rho = scan_msg.range_max
                    x, y = polar_to_decart(rho / self.grid_resolution, phi)
                    x = x + base_x
                    y = y + base_y
                    x, y = self.remap_grid_coord(x, y)
                    cv2.line(self.gridData, [base_x, base_y], [x, y], [255], 1)

    def set_detection(self, poly_msg, depth_msg, pose_x, pose_y):
        grid_center_x = self.gridData.shape[0] / 2
        grid_center_y = self.gridData.shape[1] / 2
        base_x = int(grid_center_x + pose_x / self.grid_resolution)
        base_y = int(grid_center_y - pose_y / self.grid_resolution)
        if poly_msg != Polygon() and depth_msg != Image():
            points = poly_msg.points
            depth = rnp.numpify(depth_msg)
            for i in range(len(points)):
                if np.isnan(points[i].x):
                    break
            left_line = points[:i]
            right_line = points[i + 1:-2]
            p1 = left_line[0]
            for point in left_line[1:]:
                try:
                    x0, y0 = transform_image_coord(p1.x, depth[int(p1.y), int(p1.x)])
                    x, y = transform_image_coord(point.x, depth[int(point.y), int(point.x)])
                    x0 = int(x0 / self.grid_resolution + base_x)
                    y0 = int(y0 / self.grid_resolution + base_y)
                    x = int(x / self.grid_resolution + base_x)
                    y = int(y / self.grid_resolution + base_y)
                    self.gridData = cv2.line(self.gridData, [x0, y0], [x, y], [126], 1)
                    p1 = point
                except (IndexError, OverflowError):
                    pass
            if right_line.__len__() > 0:
                p1 = right_line[0]
                for point in right_line[1:]:
                    try:
                        x0, y0 = transform_image_coord(p1.x, depth[int(p1.y), int(p1.x)])
                        x, y = transform_image_coord(point.x, depth[int(point.y), int(point.x)])
                        x0 = int(x0 / self.grid_resolution + base_x)
                        y0 = int(y0 / self.grid_resolution + base_y)
                        x = int(x / self.grid_resolution + base_x)
                        y = int(y / self.grid_resolution + base_y)
                        self.gridData = cv2.line(self.gridData, [x0, y0], [x, y], [126], 1)
                        p1 = point
                    except (IndexError, OverflowError):
                        pass

    def set_back_detection(self, poly_msg, depth_msg, pose_x, pose_y):
        grid_center_x = self.gridData.shape[0] / 2
        grid_center_y = self.gridData.shape[1] / 2
        base_x = int(grid_center_x + pose_x / self.grid_resolution)
        base_y = int(grid_center_y - pose_y / self.grid_resolution)
        if poly_msg != Polygon() and depth_msg != Image():
            points = poly_msg.points
            depth = rnp.numpify(depth_msg)
            for i in range(len(points)):
                if np.isnan(points[i].x):
                    break
            left_line = points[:i]
            right_line = points[i + 1:-2]
            p1 = left_line[0]
            for point in left_line[1:]:
                try:
                    x0, y0 = transform_image_coord(p1.x, depth[int(p1.y), int(p1.x)])
                    x0 = -x0
                    y0 = -y0
                    x, y = transform_image_coord(point.x, depth[int(point.y), int(point.x)])
                    x = -x
                    y = -y
                    x0 = int(x0 / self.grid_resolution + base_x)
                    y0 = int(y0 / self.grid_resolution + base_y)
                    x = int(x / self.grid_resolution + base_x)
                    y = int(y / self.grid_resolution + base_y)
                    self.gridData = cv2.line(self.gridData, [x0, y0], [x, y], [126], 1)
                    p1 = point
                except (IndexError, OverflowError):
                    pass
            if right_line.__len__() > 0:
                p1 = right_line[0]
                for point in right_line[1:]:
                    try:
                        x0, y0 = transform_image_coord(p1.x, depth[int(p1.y), int(p1.x)])
                        x0 = -x0
                        y0 = -y0
                        x, y = transform_image_coord(point.x, depth[int(point.y), int(point.x)])
                        x = -x
                        y = -y
                        x0 = int(x0 / self.grid_resolution + base_x)
                        y0 = int(y0 / self.grid_resolution + base_y)
                        x = int(x / self.grid_resolution + base_x)
                        y = int(y / self.grid_resolution + base_y)
                        self.gridData = cv2.line(self.gridData, [x0, y0], [x, y], [126], 1)
                        p1 = point
                    except (IndexError, OverflowError):
                        pass

    def main_timer(self):
        self.set_scan(self.frontScanMsg, self.front_camera_x, self.front_camera_y, self.front_camera_a)
        self.set_scan(self.backScanMsg, self.back_camera_x, self.back_camera_y, self.back_camera_a)
        self.set_detection(self.frontDetectionMsg, self.frontDepthMsg, self.front_camera_x, self.front_camera_y)
        self.set_back_detection(self.backDetectionMsg, self.backDepthMsg, self.back_camera_x, self.back_camera_y)

        grid = np.array(self.gridData, np.int8)
        grid[grid == 0] = OBSTACLE_CELL
        grid[grid == -1] = FREE_CELL
        grid[grid == 127] = UNKNOWN_CELL
        msg = rnp.msgify(OccupancyGrid, grid)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.robot_frame
        msg.info.resolution = self.grid_resolution
        msg.info.width = int(self.grid_size / self.grid_resolution)
        msg.info.height = int(self.grid_size / self.grid_resolution)
        msg.info.origin.position.x = - msg.info.width / 2 * self.grid_resolution
        msg.info.origin.position.y = - msg.info.height / 2 * self.grid_resolution
        self.gridPub.publish(msg)

        self.gridData = np.zeros(
            (int(self.grid_size / self.grid_resolution), int(self.grid_size / self.grid_resolution)),
            np.uint8) + 127


def main():
    rclpy.init()
    node = LocalMapping("local_mapping")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
