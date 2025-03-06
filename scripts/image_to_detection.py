#!/usr/bin/env python3
import math

import cv2
import rclpy
import numpy as np
import ros2_numpy as rnp

from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Polygon, Point32


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
            start = cur
            start.marked = True
            p.append(start)
            min_dst = img.shape[1]
        ps.append(p)

    return img, ps


def approx_curve(x, y):
    a = np.polyfit(np.log(x), y, 1)
    y = a[0] * np.log(x) + a[1]
    return x, y


class Dot(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.marked = False


class ImageToDetection(Node):
    """
    Класс ноды, отвечающий за сегментацию и распознавание линий рядков сельскохозяйственных культур на изображении с
    камеры.

    Реализованы методы:
        * сегментации по зеленому цвету;
        * выявления ключевых точек (точек центра каждого растения) с помощью функции поиска замкнутых контуров на
          черно-белом бинарном изображении путем отсеивания контуров с малой площадью;
        * построения графа по обнаруженным точкам;
        * обнаружения самых первых двух точек центральных линий рядков слева и справа с последующим поиском пути
          алгоритмом Дейкстры к ближайшим точкам впереди, то есть поиска всех растений на линии одного ряда;
        * аппроксимации двух обнаруженных линий рядов в плавные кривые второго порядка.

    Основные параметры класса для среды ROS2:
        * субскрайбер на топик входного изображения цвета;
        * паблишер топика полигона (массива точек) распознанных линий;
            ! В одном полигоне содержится информация о точках двух распознанных линий. Они разделены точкой с
            координатами NAN.
        * таймер для выполнения обработки изображений и отправки полигонов.

    Основные параметры ROS2:
        * image_topic - название топика изображения цвета;
        * detection_topic - название топика полигона распознанных линий рядов;
        * frequency - частота работы таймера.
    """

    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.declare_parameter('image_topic', '/agro_bot/front_camera/image/color')
        self.declare_parameter('detection_topic', '/agro_bot/front_camera/detection')
        self.declare_parameter('frequency', 30)

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        poly_topic = self.get_parameter('detection_topic').get_parameter_value().string_value
        frequency = self.get_parameter('frequency').get_parameter_value().integer_value

        self.imageMsg = Image()
        self.imageSub = self.create_subscription(Image,
                                                 image_topic,
                                                 self.image_callback,
                                                 10)
        self.polyData = []
        self.polyPub = self.create_publisher(Polygon,
                                             poly_topic,
                                             10)
        self.polyTimer = self.create_timer(1 / frequency,
                                           self.timer_callback)

    def image_callback(self, msg):
        self.imageMsg = msg

    def detect_row(self, image):
        self.polyData = []

        image = get_hsv(image)
        image, mask = get_color_segmented(image)
        mask = get_noiseless(mask)
        mask = get_max_pooled(mask)
        mask, graph = get_rows_nodes(mask)
        if len(graph) != 0:
            img, paths = get_row_path(mask, graph)
            if paths is not None:
                for path in paths:
                    if path:
                        x = []
                        y = []
                        for dot in path:
                            x.append(dot.x)
                            y.append(dot.y)
                        x, y = approx_curve(x, y)
                        for u, v in zip(x, y):
                            self.polyData.append(Point32(x=float(u), y=float(v), z=0.))
                            # image = cv2.circle(image, [int(u), int(v)], 5, (255, 255, 255), -1)
                        self.polyData.append(Point32(x=np.nan, y=np.nan, z=np.nan))

    def timer_callback(self):
        try:
            image_data = rnp.numpify(self.imageMsg)
        except TypeError:
            image_data = None
        if image_data is not None:
            self.detect_row(image_data)
            msg = Polygon()
            msg.points = self.polyData
            self.polyPub.publish(msg)


def main():
    rclpy.init()
    node = ImageToDetection("image_to_detection")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
