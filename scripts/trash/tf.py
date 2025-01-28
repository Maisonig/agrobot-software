#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, TransformStamped


class Frame(Node):

    def __init__(self, node_name):
        super().__init__(node_name)

        qos_profile = QoSProfile(depth=10)
        self.transformBroadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.stateTimer = self.create_timer(0.066, self.timer_callback)
        self.odomMsg = Odometry()
        self.create_subscription(Odometry,
                                 'odometry/filtered',
                                 self.odom_callback,
                                 qos_profile)

        self.bTrans = TransformStamped()
        self.bTrans.header.frame_id = 'map'
        self.bTrans.child_frame_id = 'odom'
        self.bTrans.transform.translation.z = 0.57

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        self.bTrans.header.stamp = now

        self.bTrans.transform.translation.x = self.odomMsg.pose.pose.position.x
        self.bTrans.transform.translation.y = self.odomMsg.pose.pose.position.y
        self.bTrans.transform.translation.z = self.odomMsg.pose.pose.position.z

        self.transformBroadcaster.sendTransform(self.bTrans)

    def odom_callback(self, msg):
        self.odomMsg = msg


def main():
    rclpy.init()
    node = Frame("joints")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
