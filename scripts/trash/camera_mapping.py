#!/usr/bin/env python3

import cv2
import rclpy
import cv_bridge
import numpy as np
import matplotlib.pyplot as plt

from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from cv_bridge import CvBridgeError
# from keras.layers import MaxPooling2D, AveragePooling2D
from nav_msgs.msg import OccupancyGrid


RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)


class CameraMapper(Node):

    def __init__(self, node_name):
        super().__init__(node_name)

        qos_profile = QoSProfile(depth=10)
        self.create_subscription(Image,
                                 "/agro_bot/image/depth/remapped",
                                 self.depth_callback,
                                 qos_profile)
        self.create_subscription(Image,
                                 "/agro_bot/image/color/remapped",
                                 self.color_callback,
                                 qos_profile)
        self.gridPub = self.create_publisher(OccupancyGrid,
                                             "/agro_bot/camera_map",
                                             qos_profile)

        self.stateTimer = self.create_timer(0.066, self.timer_callback)
        self.colorMsg = np.zeros((100, 100))
        self.depthMsg = np.zeros((100, 100))

        self.cvBridge = cv_bridge.CvBridge()

    def timer_callback(self):
        try:
            image = cv2.imread("../corn_rows.jpg")
            image = cv2.resize(image, (1280, 720))
            gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # # Перспектива изображения
            # real_pts = np.float32([
            #     [770, 700],
            #     [840, 700],
            #     [840, 710],
            #     [770, 710],
            #
            # ])
            # img_pts = np.float32([
            #     [400, 500],
            #     [855, 500],
            #     [958, 719],
            #     [300, 719],
            #
            # ])
            # h, status = cv2.findHomography(img_pts, real_pts)
            # grid = cv2.warpPerspective(image, h, [1600, 710])
            # # Вырез области интереса из изображения
            # image = grid[grid.shape[0] - 300:, int(grid.shape[1] / 2 - 200):int(grid.shape[1] / 2 + 200)]
            # gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            # Преобразование изображения в формат HSV
            hsv_img = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
            # Сегментация изображения по зеленому цвету
            lower_range = np.array((12, 25, 25))  # lower range of green color in HSV
            upper_range = np.array((86, 255, 255))  # upper range of green color in HSV
            # Получение бинарной маски изображения
            mask = cv2.inRange(hsv_img, lower_range, upper_range)
            gray_segmented_image = cv2.bitwise_and(gray_image, gray_image, mask=mask)
            color_segmented_image = cv2.bitwise_and(image, image, mask=mask)
            # Фильтрация маски изображения
            # mask = cv2.medianBlur(mask, 3)
            _t, thresholded_image = cv2.threshold(gray_segmented_image, 127, 255, cv2.THRESH_BINARY)

            kernel = np.ones((7, 7), np.uint8)
            noiseless_image = cv2.morphologyEx(thresholded_image, cv2.MORPH_CLOSE, kernel)
            noiseless_image = cv2.morphologyEx(noiseless_image, cv2.MORPH_OPEN, kernel)
            # Разрезание изображения на горизонтальные полосы высотой h
            # tensor_image = noiseless_image.reshape(1, noiseless_image.shape[0], noiseless_image.shape[1], 1)
            # max_pooling = MaxPooling2D(pool_size=(11, 11))
            # average_pooling = AveragePooling2D((15, 15))
            # pooled_image = average_pooling(tensor_image)[0, :, :]
            # pooled_image = pooled_image.numpy()
            # pooled_image = cv2.resize(pooled_image, (1280, 720))
            pooled_image = cv2.resize(noiseless_image, (85, 45))
            pooled_image = cv2.resize(pooled_image, (1280, 720))
            pooled_image[pooled_image > 0] = 255

            h = 50
            slices = int(pooled_image.shape[0] / 50) - 1
            slices_coord = [[h * i, h * i + h] for i in range(slices)]
            rgb_pooled_image = cv2.cvtColor(pooled_image, cv2.COLOR_GRAY2RGB)
            for j in range(slices):
                row = pooled_image[slices_coord[j][0]:slices_coord[j][1], :]
                # rgb_pooled_image = cv2.line(rgb_pooled_image, [0, slices_coord[j][1]], [rgb_pooled_image.shape[1], slices_coord[j][1]], GREEN, 2)

                # canny = cv2.Canny(row, 50, 150)
                contours, hierarchy = cv2.findContours(row, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                contours = list(contours)
                drawing = np.zeros((row.shape[0], row.shape[1], 3), dtype=np.uint8)

                for i in range(len(contours)):
                    x, y, w, height = cv2.boundingRect(contours[i])
                    max_area = w * height
                    area = cv2.contourArea(contours[i])
                    x1, y1 = int(x + w / 2), int(y + h * j)
                    color = (255, 255, 255)
                    cv2.drawContours(drawing, contours, i, RED, 2, cv2.LINE_8, hierarchy, 0)
                    cv2.circle(rgb_pooled_image, [x1, y1], 5, (255, 0, 0), -1)
            cv2.imshow("image", rgb_pooled_image)
            cv2.waitKey(0)
            # plt.imshow(mask)
            # plt.show()
        except cv2.error as e:
            print(e)

    def depth_callback(self, msg):
        # self.depthMsg = self.cvBridge.imgmsg_to_cv2(msg)
        pass

    def color_callback(self, msg):
        # self.colorMsg = self.cvBridge.imgmsg_to_cv2(msg)
        pass


def main():
    rclpy.init()
    node = CameraMapper("depth")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
