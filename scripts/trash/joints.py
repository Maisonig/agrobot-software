#!/usr/bin/env python3

import rclpy
from math import sin, cos
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion
from tf2_ros import TransformBroadcaster, TransformStamped


def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
    qy = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2)
    qz = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2)
    qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


class JointStatePublisher(Node):

    def __init__(self, node_name):
        super().__init__(node_name)

        qos_profile = QoSProfile(depth=10)
        self.jointPublisher = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.transformBroadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.stateTimer = self.create_timer(0.066, self.joint_timer_callback)

        self.baseLinkTransform = TransformStamped()
        self.baseLinkTransform.header.frame_id = 'base_footprint'
        self.baseLinkTransform.child_frame_id = 'base_link'
        self.baseLinkTransform.transform.translation.z = 0.57
        self.jointsState = JointState()

    def joint_timer_callback(self):
        try:
            now = self.get_clock().now().to_msg()
            self.jointsState.header.stamp = now
            self.jointsState.name = ['front_left_steering_joint',
                                     'front_right_steering_joint',
                                     'back_left_steering_joint',
                                     'back_right_steering_joint',
                                     'front_left_wheel_joint',
                                     'front_right_wheel_joint',
                                     'back_left_wheel_joint',
                                     'back_right_wheel_joint',
                                     ]
            self.jointsState.position = [0., 0., 0., 0., 0., 0., 0., 0.]
            self.baseLinkTransform.header.stamp = now

            self.jointPublisher.publish(self.jointsState)
            self.transformBroadcaster.sendTransform(self.baseLinkTransform)

        except KeyboardInterrupt:
            pass


def main():
    rclpy.init()
    node = JointStatePublisher("joints")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
