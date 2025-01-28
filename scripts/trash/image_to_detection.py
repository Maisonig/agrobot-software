#!/usr/bin/env python3
import math

import cv2
import rclpy
import numpy as np
import ros2_numpy as rnp
from numpy.polynomial import Polynomial

from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Polygon, Point, Point32


def get_hsv(img):
    img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    return img


def get_color_segmented(img):
    lower_range = np.array((12, 25, 25))  # lower range of green color in HSV
    upper_range = np.array((86, 255, 255))  # upper range of green color in HSV
    msk = cv2.inRange(img, lower_range, upper_range)
    img = cv2.bitwise_and(img, img, mask=msk)
    return img, msk


def get_noiseless(img):
    img = cv2.medianBlur(img, 5)
    kernel = np.ones((5, 5), np.uint8)
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
    img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
    return img


def get_max_pooled(img):
    w, h = img.shape[1], img.shape[0]
    img = cv2.resize(img, (int(w / 7), int(h / 7)))
    img = cv2.resize(img, (w, h))
    img[img > 0] = 255
    return img


def get_rows_nodes(img):
    h = 10
    slices = int(img.shape[0] / h) + 1
    slices_coord = [[h * i, h * i + h] for i in range(slices)]
    node_img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    g = []
    j = slices
    for j in range(slices):
        row = img[slices_coord[j][0]:slices_coord[j][1], :]
        contours, hierarchy = cv2.findContours(row, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = list(contours)
        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            x, y, width, height = cv2.boundingRect(contours[i])
            x, y = int(x + width / 2), int((h * j + y) + height / 2)
            if area > 100:
                cv2.circle(node_img, [x, y], 5, (255, 255, 0), -1)
                g.append(Dot(x, y))
        j -= 1
    return node_img, g


def get_row_path(img, g):
    ps = []
    start_height = 20
    starts = []
    for node in g:
        if node.y > img.shape[0] - start_height:
            starts.append(node)
    if len(starts) == 0:
        return img, None
    start = starts[0]
    for node in starts[1:]:
        if start.x + 200 > node.x > start.x - 200:
            starts.remove(node)
    if starts is None:
        return img, None
    if len(starts) == 1:
        return img, None
    if len(starts) == 2:
        if starts[1].x < starts[0].x:
            starts = list(reversed(starts))
    if len(starts) > 2:
        left = Dot(0, 0)
        right = Dot(img.shape[1], 0)
        for node in starts:
            if left.x < node.x < img.shape[1] / 2:
                left = node
        for node in starts:
            if left.x + 250 < node.x:
                if right.x > node.x > img.shape[1] / 2:
                    if node.x < right.x:
                        right = node
        starts = [left, right]
        for node in starts:
            node.marked = True
        if right.x == img.shape[1]:
            return img, None

    min_dst = img.shape[1]
    for start in starts:
        p = []
        while start.y > img.shape[0] / 2:
            cur = start
            for node in g:
                if node.marked is False:
                    if node.y < start.y:
                        if start.x - 100 < node.x < start.x + 100:
                            dst = math.dist([node.x, node. y], [start.x, start.y])
                            if dst < min_dst:
                                min_dst = dst
                                cur = node
            if cur == start:
                break
            cv2.line(img, [start.x, start.y], [cur.x, cur.y], (127, 127, 0), 2)
            start = cur
            start.marked = True
            p.append(start)
            cv2.circle(img, [start.x, start.y], 5, (255, 0, 0), 2)
            min_dst = img.shape[1]
        ps.append(p)

    return img, ps


def get_paths_straights(img, ps):
    ss = []
    for p in ps:
        if len(p) > 4:
            x_1 = p[1].x
            y_1 = p[1].y
            x_2 = p[-1].x
            y_2 = p[-1].y
            node1 = Dot(x_1, y_1)
            node2 = Dot(x_2, y_2)
            s = Straight(node1.x, node1.y, node2.x, node2.y)
            y_1 = img.shape[0] - 1
            x_1 = int(s.calc_x(y_1))
            y_2 = int(img.shape[0] / 4 * 3)
            x_2 = int(s.calc_x(y_2))
            s = Straight(x_1, y_1, x_2, y_2)
            cv2.line(img, (x_1, y_1), (x_2, y_2), (139, 0, 0), 9)
            cv2.line(img, (x_1, y_1), (x_2, y_2), (255, 0, 0), 5)
            ss.append(s)
    return img, ss


class Dot(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.marked = False


class Straight(object):
    def __init__(self, x_1, y_1, x_2, y_2):
        self.x1 = x_1
        self.y1 = y_1
        self.x2 = x_2
        self.y2 = y_2

        self.k = 0.
        self.b = 0.
        self.get_canonical_equation_coefficients()

        self.accuracy = 0.

    def get_canonical_equation_coefficients(self):
        if self.x1 == self.x2:
            self.x1 = self.x1 - 0.001
        if self.y1 == self.y2:
            self.y1 = self.y1 - 0.001
        self.k = (self.y1 - self.y2) / (self.x1 - self.x2)
        self.b = self.y2 - self.k * self.x2

    def calc_y(self, x):
        return self.k * x + self.b

    def calc_x(self, y):
        return (y - self.b) / self.k


class ImageToDetection(Node):

    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.declare_parameter('image_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('depth_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('detection_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('frequency', rclpy.Parameter.Type.INTEGER)

        params = [
            rclpy.Parameter('image_topic', rclpy.Parameter.Type.STRING, '/agro_bot/front_camera/image/color'),
            rclpy.Parameter('depth_topic', rclpy.Parameter.Type.STRING, '/agro_bot/front_camera/image/depth'),
            rclpy.Parameter('detection_topic', rclpy.Parameter.Type.STRING, '/poly'),
            rclpy.Parameter('frequency', rclpy.Parameter.Type.INTEGER, 30),
        ]

        # self.set_parameters(params)

        self.isSim = self.get_parameter('use_sim_time').get_parameter_value().bool_value

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        poly_topic = self.get_parameter('detection_topic').get_parameter_value().string_value
        frequency = self.get_parameter('frequency').get_parameter_value().integer_value

        self.imageMsg = Image()
        self.imageSub = self.create_subscription(Image,
                                                 image_topic,
                                                 self.image_callback,
                                                 10)
        self.depthMsg = Image()
        self.depthSub = self.create_subscription(Image,
                                                 depth_topic,
                                                 self.depth_callback,
                                                 10)
        self.polyData = []
        self.polyPub = self.create_publisher(Polygon,
                                             poly_topic,
                                             10)
        self.polyTimer = self.create_timer(1 / frequency,
                                           self.timer_callback)

    def image_callback(self, msg):
        self.imageMsg = msg

    def depth_callback(self, msg):
        self.depthMsg = msg

    def detect_row(self, image, depth):
        source = np.copy(image)
        image = get_hsv(image)
        image, mask = get_color_segmented(image)
        mask = get_noiseless(mask)
        mask = get_max_pooled(mask)
        mask, graph = get_rows_nodes(mask)
        if len(graph) != 0:
            mask, paths = get_row_path(mask, graph)
            if paths is not None:
                # source, straights = get_paths_straights(source, paths)
                self.polyData = []
                for path in paths:
                    for dot in path:
                        self.polyData.append(Point32(x=float(dot.x), y=float(dot.y), z=0.))
                    self.polyData.append(Point32(x=np.nan, y=np.nan, z=np.nan))
                # cv2.imshow('Image', source)
                # cv2.waitKey(0)
            else:
                pass
                # self.get_logger().info("No rows have been found")
        else:
            pass
            # self.get_logger().info("No plants have been found")

    def timer_callback(self):
        # start_time = time.time()
        try:
            image_data = rnp.numpify(self.imageMsg)
            depth_data = rnp.numpify(self.depthMsg)
        except TypeError:
            image_data = None
            depth_data = None
        if image_data is not None and depth_data is not None:
            self.detect_row(image_data, depth_data)
            msg = Polygon()
            msg.points = self.polyData
            self.polyPub.publish(msg)

        # print(1.0 / (time.time() - start_time))


def main():
    rclpy.init()
    node = ImageToDetection("image_to_detection")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
