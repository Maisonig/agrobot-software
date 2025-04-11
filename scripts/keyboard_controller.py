#!/usr/bin/env python3

import rclpy
from pynput import keyboard
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist


class KeyboardController(Node):

    def __init__(self, node_name):
        super().__init__(node_name)

        qos_profile = QoSProfile(depth=10)
        self.pub = self.create_publisher(Twist,
                                         '/cmd_vel',
                                         qos_profile)

        self.x_, self.y_, self.z_ = 0., 0., 0.

        with keyboard.Listener(on_press=self.on_press_callback) as self.listener:
            self.listener.join()

    def on_press_callback(self, key):
        try:
            if key.char == '8':
                self.x_ += 0.05
            if key.char == '2':
                self.x_ -= 0.05
            if key.char == '4':
                self.y_ += 0.05
            if key.char == '6':
                self.y_ -= 0.05
            if key.char == '9':
                self.z_ += 0.05
            if key.char == '7':
                self.z_ -= 0.05
            if key.char == '0':
                self.x_, self.y_, self.z_ = 0., 0., 0.
        except AttributeError:
            pass

        if self.x_ >= 1.:
            self.x_ = 1.
        if self.x_ <= -1.:
            self.x_ = -1.
        if self.y_ >= 1.:
            self.y_ = 1.
        if self.y_ <= -1.:
            self.y_ = -1.
        if self.z_ >= 1.:
            self.z_ = 1.
        if self.z_ <= -1.:
            self.z_ = -1.

        msg = Twist()
        msg.linear.x = self.x_
        msg.linear.y = self.y_
        msg.angular.z = self.z_
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = KeyboardController("keyboard_controller")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
