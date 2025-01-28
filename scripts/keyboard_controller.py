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
                                         'cmd_vel',
                                         qos_profile)

        self.speed = 0.
        self.steering = 0.

        with keyboard.Listener(on_press=self.on_press_callback) as self.listener:
            self.listener.join()

    def on_press_callback(self, key):
        try:
            if key == keyboard.Key.up:
                self.speed += 0.1
            if key == keyboard.Key.down:
                self.speed += -0.1
            if key == keyboard.Key.left:
                self.steering += 0.1
            if key == keyboard.Key.right:
                self.steering += -0.1
            print('alphanumeric key {0} pressed'.format(
                key.char))
        except AttributeError:
            print('special key {0} pressed'.format(
                key))
        if self.speed >= 1.:
            self.speed = 1.
        if self.speed <= -1.:
            self.speed = -1.
        if self.steering >= 1.:
            self.steering = 1.
        if self.steering <= -1:
            self.steering = -1.
        msg = Twist()
        msg.linear.x = self.speed
        msg.angular.z = self.steering
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = KeyboardController("keyboard_controller")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
