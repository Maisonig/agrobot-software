#!/usr/bin/env python3

import cv2
import time
import rclpy
import imutils
import numpy as np
import ros2_numpy as rnp

from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from math import asin, atan2, degrees
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry


def polar_to_decart(rho, phi):
    return rho * np.cos(phi), rho * np.sin(phi)


def decart_to_polar(x, y):
    return np.sqrt(x ** 2 + y ** 2), np.arctan2(y, x)


def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    x_ = degrees(atan2(t0, t1))
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    y_ = degrees(asin(t2))
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    z_ = degrees(atan2(t3, t4))
    return x_, y_, z_


def get_combined_image(glob, loc):
    mask = np.copy(loc)
    mask[mask == 0] = 255
    mask[mask == 127] = 0
    fg = cv2.bitwise_or(loc, loc, mask=mask)
    mask = cv2.bitwise_not(mask)
    bk = cv2.bitwise_or(glob, glob, mask=mask)
    return cv2.bitwise_or(fg, bk)


def rotate_uint8_grid(grid, phi):
    o_grid = np.copy(grid)
    o_grid[o_grid != 0] = 1
    o_grid[o_grid == 0] = 255
    o_grid[o_grid == 1] = 0
    o_rotated = imutils.rotate(o_grid, phi)
    o_rotated[o_rotated <= 127] = 0
    o_rotated[o_rotated > 127] = 128

    f_grid = np.copy(grid)
    f_grid[f_grid != 255] = 0
    f_rotated = imutils.rotate(f_grid, angle=phi)
    f_rotated[f_rotated <= 127] = 0
    f_rotated[f_rotated > 127] = 255
    rotated = o_rotated + f_rotated
    rotated[rotated == 0] = 127
    rotated[rotated == 128] = 0
    return rotated


class GlobalMapper(Node):
    UNKNOWN_CELL = -1
    FREE_CELL = 0
    OBSTACLE_CELL = 100

    MAP_SIZE = 1500

    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.localMapSub = self.create_subscription(OccupancyGrid,
                                                    '/agro_bot/local_map',
                                                    self.local_map_callback,
                                                    10)
        self.odomSub = self.create_subscription(Odometry,
                                                '/odometry/filtered',
                                                self.odom_callback,
                                                10)
        self.clockSub = self.create_subscription(Clock,
                                                 '/gz/clock',
                                                 self.clock_callback,
                                                 10)
        self.globalMapPub = self.create_publisher(OccupancyGrid,
                                                  '/agro_bot/global_map',
                                                  10)

        self.localOccupancyTimer = self.create_timer(0.066, self.occupancy_timer_callback)
        self.globalOccupancyTimer = self.create_timer(5, self.publish_global_map)

        self.odomData = Odometry()
        self.globalMap = np.zeros((self.MAP_SIZE, self.MAP_SIZE), np.uint8) + 127
        self.localMap = None
        self.clockMsg = Clock()

        self.transformBroadcaster = TransformBroadcaster(self, qos=10)
        self.bTrans = TransformStamped()
        self.bTrans.header.frame_id = 'odom'
        self.bTrans.child_frame_id = 'base_link'

        self.n = 0
        self.sum_fps = 0

    def occupancy_timer_callback(self):
        self.publish_map_frame()
        try:
            start_time = time.time()
            width = self.localMap.shape[0] / 2
            x_offset = int(self.odomData.pose.pose.position.x * 10 + (self.MAP_SIZE / 2) - width)
            y_offset = int(self.odomData.pose.pose.position.y * 10 + (self.MAP_SIZE / 2) - width)
            # x_offset = int(- (self.MAP_SIZE / 2) * 0.1 + self.odomData.pose.pose.position.x * 10)
            # y_offset = int(- (self.MAP_SIZE / 2) * 0.1 + self.odomData.pose.pose.position.y * 10)
            x = self.odomData.pose.pose.orientation.x
            y = self.odomData.pose.pose.orientation.y
            z = self.odomData.pose.pose.orientation.z
            w = self.odomData.pose.pose.orientation.w
            a, b, c = quaternion_to_euler(x, y, z, w)

            rotated = rotate_uint8_grid(self.localMap, -c)

            y1, y2 = y_offset, y_offset + rotated.shape[0]
            x1, x2 = x_offset, x_offset + rotated.shape[1]

            g = self.globalMap[y1:y2, x1:x2]
            g = get_combined_image(g, rotated)
            self.globalMap[y1:y2, x1:x2] = g
            self.publish_global_map()

            print(1.0 / (time.time() - start_time))
            # print(self.get_clock().now().to_msg().nanosec / 1000000000)
            self.sum_fps += 1.0 / (time.time() - start_time)
            self.n += 1
            if self.n == 150:
                # print(self.sum_fps / self.n)
                self.n = 0
                self.sum_fps = 0
        except Exception as e:
            print(e)

    def publish_global_map(self):
        oc_array = np.copy(self.globalMap)
        oc_array[oc_array < 127] = 100
        oc_array[oc_array > 127] = 0
        oc_array[oc_array == 127] = 255
        oc_array = np.array(oc_array, np.int8)
        grid = rnp.msgify(OccupancyGrid, oc_array)
        grid.header.stamp = self.clockMsg.clock
        grid.header.frame_id = 'odom'
        grid.info.resolution = .1
        grid.info.origin.position.x = - (self.MAP_SIZE / 2) * 0.1
        grid.info.origin.position.y = - (self.MAP_SIZE / 2) * 0.1
        self.globalMapPub.publish(grid)

    def publish_map_frame(self):
        self.bTrans.header.stamp = self.clockMsg.clock
        self.bTrans.transform.translation.x = self.odomData.pose.pose.position.x
        self.bTrans.transform.translation.y = self.odomData.pose.pose.position.y
        self.bTrans.transform.translation.z = self.odomData.pose.pose.position.z
        self.bTrans.transform.rotation.x = self.odomData.pose.pose.orientation.x
        self.bTrans.transform.rotation.y = self.odomData.pose.pose.orientation.y
        self.bTrans.transform.rotation.z = self.odomData.pose.pose.orientation.z
        self.bTrans.transform.rotation.w = self.odomData.pose.pose.orientation.w

        self.transformBroadcaster.sendTransform(self.bTrans)

    def local_map_callback(self, msg):
        g = np.array(rnp.numpify(msg), np.uint8)
        g[g == 255] = 127
        g[g == 0] = 255
        g[g == 100] = 0
        self.localMap = g

    def odom_callback(self, msg):
        self.odomData = msg
        print("got odom")

    def clock_callback(self, msg):
        self.clockMsg = msg


def main(args=None):
    rclpy.init(args=args)
    node = GlobalMapper("mapper")
    rclpy.spin(node)
    cv2.imwrite('../global_map.jpg', node.globalMap)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
