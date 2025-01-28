#!/usr/bin/env python3

import cv2
import rclpy
import cv_bridge
import numpy as np
from cv_bridge import CvBridgeError
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image


class Depth(Node):

    def __init__(self, node_name):
        super().__init__(node_name)

        qos_profile = QoSProfile(depth=10)
        self.create_subscription(Image,
                                 "/agro_bot/image/depth/remapped",
                                 self.callback,
                                 qos_profile)
        self.create_subscription(Image,
                                 "/agro_bot/image/color/remapped",
                                 self.color_callback,
                                 qos_profile)
        self.stateTimer = self.create_timer(0.066, self.timer_callback)
        self.imgMsg = Image()
        self.cvBridge = cv_bridge.CvBridge()

    def timer_callback(self):
        try:
            img = self.cvBridge.imgmsg_to_cv2(self.imgMsg)
            img = img * 32
            img[img == np.inf] = 255
            img[img == - np.inf] = 0
            img = np.array(img, np.uint8)
            print(img[127, 127])
            cv2.imshow("img", img)
            cv2.waitKey(1)
        except CvBridgeError:
            pass

    def callback(self, msg):
        self.imgMsg = msg

    def color_callback(self, msg):
        msg = self.cvBridge.imgmsg_to_cv2(msg)
        print(np.max(msg))


def main():
    rclpy.init()
    node = Depth("depth")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
