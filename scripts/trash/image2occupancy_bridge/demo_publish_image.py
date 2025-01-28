import os
import time
import cv2
import rclpy
import numpy as np
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from tf2_ros import TransformStamped


cap = cv2.VideoCapture("media/Jul_7_2022_17_55_24_depth_car_cameras.mp4")
num = 0

x = -10.
y = -10.
yaw = 0.
nullTime = time.time()


def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.

    Input
      :param roll: The roll (rotation around x-axis) angle in radians.
      :param pitch: The pitch (rotation around y-axis) angle in radians.
      :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
      :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)

    return [qx, qy, qz, qw]

class DemoImagePublisher(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.timer = self.create_timer(0.1, self.callback)
        self.cv_bridge = CvBridge()
        self.c_pub = self.create_publisher(Image, 'camera/color', 10)
        self.d_pub = self.create_publisher(Image, 'camera/depth', 10)

        self.transform = TransformStamped()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

    def callback(self):
        global cap, x, y, yaw

        self.transform.header.frame_id = "map"
        self.transform.header.stamp = self.get_clock().now().to_msg()
        self.transform.child_frame_id = "odom"
        self.transform.transform.translation.x = x
        self.transform.transform.translation.y = y
        q = get_quaternion_from_euler(0., 0., yaw)
        self.transform.transform.rotation.x = q[0]
        self.transform.transform.rotation.y = q[1]
        self.transform.transform.rotation.z = q[2]
        self.transform.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(self.transform)

        static = TransformStamped()
        static.header.frame_id = "odom"
        static.header.stamp = self.get_clock().now().to_msg()
        static.child_frame_id = "base_link"
        static.transform.translation.x = -22.5
        static.transform.translation.y = -10.
        q = get_quaternion_from_euler(0., 0., 0.)
        static.transform.rotation.x = q[0]
        static.transform.rotation.y = q[1]
        static.transform.rotation.z = q[2]
        static.transform.rotation.w = q[3]

        self.static_tf_broadcaster.sendTransform(static)

        current_time = time.time()
        t = current_time - nullTime
        if t > 42:
            x += 0.25
            if 45 < t < 46:
                yaw += 0.01

        if cap.isOpened():
            ret, frame = cap.read()
            color_image = frame[:, 0:640]
            color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            depth_image = frame[:, 640:]

            color = self.cv_bridge.cv2_to_imgmsg(color_image)
            depth = self.cv_bridge.cv2_to_imgmsg(depth_image)

            self.c_pub.publish(color)
            self.d_pub.publish(depth)


def main(args=None):
    rclpy.init(args=args)
    node = DemoImagePublisher("demo_image_publisher")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
