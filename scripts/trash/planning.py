#!/usr/bin/env python3

import cv2
import time
import rclpy
import numpy as np
import ros2_numpy as rnp
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid


def polar_to_decart(rho, phi):
    return rho * np.cos(phi), rho * np.sin(phi)


def decart_to_polar(x, y):
    return np.sqrt(x ** 2 + y ** 2), np.arctan2(y, x)


class Planner(Node):
    UNKNOWN_CELL = -1
    FREE_CELL = 0
    OBSTACLE_CELL = 100

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.create_subscription(LaserScan,
                                 '/agro_bot/scan/remapped',
                                 self.lidar_callback,
                                 10)
        self.occupancyPub = self.create_publisher(OccupancyGrid,
                                                  '/agro_bot/local_map',
                                                  10)
        self.occupancyTimer = self.create_timer(0.066, self.occupancy_timer_callback)

        self.localMap = np.zeros((self.MAP_SIZE, self.MAP_SIZE), np.int8) + self.UNKNOWN_CELL
        self.sensorInfo = {
            'lidar': [[self.CAR_CENTER[0], self.CAR_CENTER[1], np.pi, 640], []],
        }

        self.n = 0
        self.sum_fps = 0

    def draw_ray_line(self, rho, phi, x0, y0, min_rho=1, max_rho=56):
        if min_rho < rho:
            try:
                x, y = polar_to_decart(rho, phi)
                x, y = int(x), int(y)
            except (ValueError, OverflowError):
                x, y = polar_to_decart(max_rho, phi)
                x, y = int(x), int(y)
            x_, y_ = polar_to_decart(min_rho, phi)
            x1, y1 = int(x0 + x_), int(y0 + y_)
            x2, y2 = x0 + x, y0 + y
            cv2.line(self.localMap, [x1, y1], [x2, y2], [self.FREE_CELL], 1)
            if not np.isnan(rho):
                try:
                    self.localMap[y2, x2] = self.OBSTACLE_CELL
                except IndexError:
                    pass

    def occupancy_timer_callback(self):
        start_time = time.time()

        self.localMap = np.zeros((self.MAP_SIZE, self.MAP_SIZE), np.int8) + self.UNKNOWN_CELL

        x, y = self.sensorInfo.get('lidar')[0][:2]
        for rho, phi in self.sensorInfo['lidar'][1]:
            self.draw_ray_line(rho, phi, x, y, 2, 1600)

        self.send_occupancy()
        print(1.0 / (time.time() - start_time))
        self.sum_fps += 1.0 / (time.time() - start_time)
        self.n += 1
        if self.n == 1500:
            print(self.sum_fps / self.n)
            self.n = 0
            self.sum_fps = 0

    def send_occupancy(self):
        self.localMap = np.array(self.localMap, np.int8)
        grid = rnp.msgify(OccupancyGrid, self.localMap)
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'base_footprint'
        grid.info.resolution = 0.1
        grid.info.origin.position.x = -self.CAR_CENTER[0] / 10
        grid.info.origin.position.y = -self.CAR_CENTER[1] / 10
        self.occupancyPub.publish(grid)

    def lidar_callback(self, msg):
        try:
            start = self.sensorInfo['lidar'][0][2]
            increment = len(msg.ranges) / self.sensorInfo['lidar'][0][3]
            angle_increment = 2 * np.pi / self.sensorInfo['lidar'][0][3]
            arr = [(msg.ranges[int(i * increment)] * 10, start + angle_increment * i)
                   for i in range(self.sensorInfo['lidar'][0][3])]
            self.sensorInfo['lidar'][1] = arr
        except IndexError:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = Planner("planner")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
