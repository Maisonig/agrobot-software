#!/usr/bin/env python3
import math

import cv2
import imutils
import numpy as np
import rclpy
import ros2_numpy as rnp
from nav_msgs.msg import OccupancyGrid, Odometry

from rclpy.node import Node
from shapely import geometry
from geometry_msgs.msg import Polygon, TransformStamped
from tf2_ros import TransformBroadcaster

MAP_SIZE = 500
FIELD_EMPTY = True

NON_FIELD = 0
FIELD = 1

PASS = 0
SPRAYING = 1
SAMPLING = 2

IDLE = 0
VECTOR3_FOLLOWING = 2
ROW_FOLLOWING = 3

ACKERMANN = 0
HOLONOMIC = 1


def polar_to_decart(rho, phi):
    return rho * np.cos(phi), rho * np.sin(phi)


def decart_to_polar(x, y):
    return np.sqrt(x ** 2 + y ** 2), np.arctan2(y, x)


def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    x_ = math.degrees(math.atan2(t0, t1))
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    y_ = math.degrees(math.asin(t2))
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    z_ = math.degrees(math.atan2(t3, t4))
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


def is_contains(p, poly):
    p = geometry.Point(p[0], p[1])
    poly = geometry.Polygon(poly)
    return poly.contains(p)


class Planner(object):

    def __init__(self):
        self.mode = IDLE
        self.task = PASS
        self.kinematics = ACKERMANN
        self.temp_environment = NON_FIELD
        self.environment = NON_FIELD

    def set_movement_parameters(self):
        # Если находимся на поле
        if self.environment == FIELD:
            # Если агротехнологическая задача отсутсвует
            if self.task == PASS:
                # Если поле пустое
                if FIELD_EMPTY:
                    # Движемся со всенаправленной кинематикой строим траекторию к конечной точке
                    self.mode = VECTOR3_FOLLOWING
                    self.kinematics = HOLONOMIC
                # Если поле имеет растительность
                if not FIELD_EMPTY:
                    # Движемся вдоль рядков с кинематикой аккермана
                    self.mode = ROW_FOLLOWING
                    self.kinematics = ACKERMANN
            # Если задача опрыскивание
            if self.task == SPRAYING:
                # Движемся вдоль рядков с кинематикой аккермана
                self.mode = ROW_FOLLOWING
                self.kinematics = ACKERMANN
            # Если задача взятие проб почвы
            if self.task == SAMPLING:
                # Движемся со всенаправленной кинематикой строим траекторию к конечной точке
                self.mode = VECTOR3_FOLLOWING
                self.kinematics = HOLONOMIC
        # Если находимся вне поля
        elif self.environment == NON_FIELD:
            # Движемся со всенаправленной кинематикой строим траекторию к конечной точке
            self.mode = VECTOR3_FOLLOWING
            self.kinematics = HOLONOMIC

    def set_environment(self, cur_pos, field_poly, detections=None):
        if field_poly is None:
            self.environment = NON_FIELD
            self.temp_environment = NON_FIELD
            return None
        contact = is_contains(cur_pos, field_poly)
        detections = False if FIELD_EMPTY else detections
        if contact or detections:
            if self.temp_environment == NON_FIELD:
                self.temp_environment = FIELD
            else:
                self.environment = FIELD
        else:
            if self.temp_environment == FIELD:
                self.temp_environment = NON_FIELD
            else:
                self.environment = NON_FIELD
        self.set_movement_parameters()

    def plan(self, grid, cur_pos, end_pos, ):
        if self.mode == VECTOR3_FOLLOWING:
            pass

    def path_to_vector3(self, cur_pos, end_pos):
        pass


plan = Planner()
plan.set_environment((30, 30, 30), [(0, 0), (0, 50), (50, 50), (50, 0)])


class PathPlanning(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.tfPublisher = TransformBroadcaster(self)
        self.localMapSub = self.create_subscription(OccupancyGrid,
                                                    '/agro_bot/local_map',
                                                    self.local_map_callback,
                                                    10)
        self.odomSub = self.create_subscription(Odometry,
                                                '/agro_bot/odom',
                                                self.odom_callback,
                                                10)
        self.globalMapPub = self.create_publisher(OccupancyGrid,
                                                  '/agro_bot/global_map',
                                                  10)

        self.mainTimer = self.create_timer(0.066, self.timer_callback)

        self.odomData = Odometry()

        self.globalMapSize = 1000
        self.gridRes = 0.1

        self.globalMap = np.zeros((self.globalMapSize, self.globalMapSize), np.uint8) + 127
        self.localMapMsg = OccupancyGrid()
        self.localMap = None

    def timer_callback(self):
        self.refresh_global_map()

    def refresh_global_map(self):
        try:
            x_offset = int(self.odomData.pose.pose.position.x / self.gridRes + (self.globalMapSize / 2) -
                           (self.localMapMsg.info.width / 2))
            y_offset = int(self.odomData.pose.pose.position.y / self.gridRes + (self.globalMapSize / 2) -
                           (self.localMapMsg.info.height / 2))
            a, b, c = quaternion_to_euler(self.odomData.pose.pose.orientation.x,
                                          self.odomData.pose.pose.orientation.y,
                                          self.odomData.pose.pose.orientation.z,
                                          self.odomData.pose.pose.orientation.w)
            rotated = rotate_uint8_grid(self.localMap, -c)
            y1, y2 = y_offset, y_offset + rotated.shape[0]
            x1, x2 = x_offset, x_offset + rotated.shape[1]
            g = self.globalMap[y1:y2, x1:x2]
            g = get_combined_image(g, rotated)
            self.globalMap[y1:y2, x1:x2] = g
            self.publish_tf()
            self.publish_global_map()
        except Exception as e:
            print(e)

    def publish_global_map(self):
        oc_array = np.copy(self.globalMap)
        oc_array = cv2.resize(oc_array, [int(self.globalMapSize), int(self.globalMapSize)])
        oc_array[oc_array < 127] = 100
        oc_array[oc_array > 127] = 0
        oc_array[oc_array == 127] = 255
        oc_array = np.array(oc_array, np.int8)
        grid = rnp.msgify(OccupancyGrid, oc_array)
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'map'
        grid.info.resolution = self.gridRes
        grid.info.origin.position.x = -self.globalMapSize / (2 / self.gridRes)
        grid.info.origin.position.y = -self.globalMapSize / (2 / self.gridRes)
        self.globalMapPub.publish(grid)

    def publish_tf(self):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_link'
        msg.transform.translation.x = self.odomData.pose.pose.position.x
        msg.transform.translation.y = self.odomData.pose.pose.position.y
        msg.transform.translation.z = self.odomData.pose.pose.position.z + 0.6
        msg.transform.rotation = self.odomData.pose.pose.orientation
        self.tfPublisher.sendTransform(msg)

    def local_map_callback(self, msg):
        g = np.array(rnp.numpify(msg), np.uint8)
        g[g == 255] = 127
        g[g == 0] = 255
        g[g == 100] = 0
        self.localMap = g
        self.localMapMsg = msg

    def odom_callback(self, msg):
        self.odomData = msg


def main():
    rclpy.init()
    node = PathPlanning("path_planning")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    pass
