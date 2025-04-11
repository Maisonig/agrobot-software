#!/usr/bin/env python3

import time
import rclpy
import rclpy.qos
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Imu
from math import sin, cos, sqrt, atan2, asin, atan
from geometry_msgs.msg import Quaternion, Vector3

qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                  history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                  depth=1)


def polar_to_decart(rho: float, phi: float):
    return rho * cos(phi), rho * sin(phi)


def decart_to_polar(x: float, y: float):
    return sqrt(x ** 2 + y ** 2), atan2(y, x)


def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    qx = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
    qy = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2)
    qz = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2)
    qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
    return qx, qy, qz, qw


def quaternion_to_euler(qx: float, qy: float, qz: float, qw: float):
    t0 = +2.0 * (qw * qx + qy * qz)
    t1 = +1.0 - 2.0 * (qx * qx + qy * qy)
    roll = atan2(t0, t1)
    t2 = +2.0 * (qw * qy - qz * qx)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = asin(t2)
    t3 = +2.0 * (qw * qz + qx * qy)
    t4 = +1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = atan2(t3, t4)
    return roll, pitch, yaw


def rotate_vector3(x, y, z, roll, pitch, yaw):
    c = [[cos(yaw) * cos(pitch), -cos(roll) * cos(pitch) * sin(yaw) + sin(roll) * sin(pitch),
          sin(roll) * cos(pitch) * sin(yaw) + cos(roll) * sin(pitch)],
         [sin(yaw), cos(roll) * cos(yaw), -sin(roll) * cos(yaw)],
         [-cos(yaw) * sin(pitch), cos(roll) * sin(pitch) * sin(yaw) + sin(roll) * cos(pitch),
          -sin(roll) * sin(pitch) * sin(yaw) + cos(roll) * cos(pitch)]]
    np_c = np.array(c)
    np_xyz = np.array([x, y, z])
    xyz_g = np.matmul(np_c, np_xyz)
    return xyz_g[0], xyz_g[1], xyz_g[2]


class ImuNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.imuSub = self.create_subscription(Imu,
                                                '/imu',
                                                self.imu_callback,
                                                qos_policy)
        self.imuPub = self.create_publisher(Imu, '/imu/filtered', 10)
        self.create_timer(0.066, self.timer_callback)
        self.imuMsg = Imu()
        self.imuMsg.header.frame_id = 'base_link'
        self.time = time.time()

        self.eulerOrientation = Vector3()

        self.calibX = 0.
        self.calibY = 0.
        self.calibZ = 0.
        self.calibRoll = 0.
        self.calibPitch = 0.
        self.calibYaw = 0.
        self.isAccel = False
        self.isGyro = False
        self.isCalib = False

    def timer_callback(self):
        if self.isGyro and self.isAccel and self.isCalib:
            self.calibX = self.imuMsg.linear_acceleration.x
            self.calibY = self.imuMsg.linear_acceleration.y
            self.calibZ = self.imuMsg.linear_acceleration.z
            self.calibRoll = self.eulerOrientation.x
            self.calibPitch = self.eulerOrientation.y
            self.calibYaw = self.eulerOrientation.z
            self.isCalib = False

        if (self.calibX + 0.1 > self.imuMsg.linear_acceleration.x > self.calibX - 0.1
                and self.calibY + 0.1 > self.imuMsg.linear_acceleration.y > self.calibY - 0.1
                and self.calibZ + 0.1 > self.imuMsg.linear_acceleration.z > self.calibZ - 0.1):
            self.eulerOrientation.x = self.calibRoll
            self.eulerOrientation.z = self.calibYaw

        self.imuMsg.header.stamp = self.get_clock().now().to_msg()
        self.calc_orientation()
        self.remove_gravitation()
        self.calc_accel_orientation(self.eulerOrientation.x, self.eulerOrientation.y, self.eulerOrientation.z)

        y = self.imuMsg.linear_acceleration.z
        z = self.imuMsg.linear_acceleration.y
        self.imuMsg.linear_acceleration.y = z
        self.imuMsg.linear_acceleration.z = y

        y = self.imuMsg.angular_velocity.z
        z = self.imuMsg.angular_velocity.y
        self.imuMsg.angular_velocity.y = z
        self.imuMsg.angular_velocity.z = y

        qx, qy, qz, qw = euler_to_quaternion(self.eulerOrientation.x, self.eulerOrientation.z, self.eulerOrientation.y)
        self.imuMsg.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        self.imuPub.publish(self.imuMsg)
        self.time = time.time()

    def calc_orientation(self):
        # Угловые скорости, находящиеся в диапазоне принимаем как погрешность ввиду дрейфа значений гироскопа
        # Фильтрация дрейфа, жертвуя точностью
        range_ = 0.015
        dt = time.time() - self.time
        if not (range_ > self.imuMsg.angular_velocity.x > -range_):
            self.eulerOrientation.x = self.eulerOrientation.x + self.imuMsg.angular_velocity.x * dt
        if not (range_ > self.imuMsg.angular_velocity.y > -range_):
            self.eulerOrientation.y = self.eulerOrientation.y + self.imuMsg.angular_velocity.y * dt
        if not (range_ > self.imuMsg.angular_velocity.z > -range_):
            self.eulerOrientation.z = self.eulerOrientation.z + self.imuMsg.angular_velocity.z * dt

    def calc_accel_orientation(self, roll, pitch, yaw):
        c = [[cos(yaw) * cos(pitch), -cos(roll) * cos(pitch) * sin(yaw) + sin(roll) * sin(pitch),
              sin(roll) * cos(pitch) * sin(yaw) + cos(roll) * sin(pitch)],
             [sin(yaw), cos(roll) * cos(yaw), -sin(roll) * cos(yaw)],
             [-cos(yaw) * sin(pitch), cos(roll) * sin(pitch) * sin(yaw) + sin(roll) * cos(pitch),
              -sin(roll) * sin(pitch) * sin(yaw) + cos(roll) * cos(pitch)]]
        self.eulerOrientation.x = atan(-c[2][0] / c[0][0])
        self.eulerOrientation.y = asin(c[1][0])
        self.eulerOrientation.z = atan(-c[1][2] / c[1][1])

    def remove_gravitation(self):
        # Устранение ускорения свободного падения в показаниях акселерометра
        # Поворот кажущихся ускорений акселерометра
        x, y, z = rotate_vector3(self.imuMsg.linear_acceleration.x,
                                 self.imuMsg.linear_acceleration.y,
                                 self.imuMsg.linear_acceleration.z,
                                 self.eulerOrientation.x,
                                 self.eulerOrientation.y,
                                 self.eulerOrientation.z)
        # print(x, y, z)
        x = x - self.calibX
        y = y - self.calibY
        z = z - self.calibZ
        print(x, y, z)
        # if x > 0.3 or x < -0.3:
        #     self.imuMsg.linear_acceleration.x = x
        # else:
        #     self.imuMsg.linear_acceleration.x = 0.
        # if y > 0.3 or y < -0.3:
        #     self.imuMsg.linear_acceleration.y = y
        # else:
        #     self.imuMsg.linear_acceleration.y = 0.
        # if z > 0.3 or z < -0.3:
        #     self.imuMsg.linear_acceleration.z = z
        # else:
        #     self.imuMsg.linear_acceleration.z = 0.

    def calibrate(self):
        self.isAccel, self.isGyro = True, True

    def imu_callback(self, msg):
        self.imuMsg.angular_velocity = msg.angular_velocity
        self.imuMsg.linear_acceleration = msg.linear_acceleration
        if not self.isGyro and not self.isAccel:
            self.isCalib = True
            self.isGyro = True
            self.isAccel = True


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode("imu")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
