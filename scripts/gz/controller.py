#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


WHEEL_RADIUS = 0.4


class GzController(Node):

    def __init__(self, node_name):
        super().__init__(node_name)

        qos_profile = QoSProfile(depth=10)
        self.create_subscription(Twist,
                                 '/cmd_vel',
                                 self.cmd_vel_callback,
                                 qos_profile)
        self.cmdVelMsg = Twist()

        self.frs = self.create_publisher(Float64,
                                         '/agro_bot/cmd_vel/steering/front_right_steering',
                                         qos_profile)
        self.fls = self.create_publisher(Float64,
                                         '/agro_bot/cmd_vel/steering/front_left_steering',
                                         qos_profile)
        self.brs = self.create_publisher(Float64,
                                         '/agro_bot/cmd_vel/steering/back_right_steering',
                                         qos_profile)
        self.bls = self.create_publisher(Float64,
                                         '/agro_bot/cmd_vel/steering/back_left_steering',
                                         qos_profile)
        self.frw = self.create_publisher(Float64,
                                         '/agro_bot/cmd_vel/wheel/front_right_wheel',
                                         qos_profile)
        self.flw = self.create_publisher(Float64,
                                         '/agro_bot/cmd_vel/wheel/front_left_wheel',
                                         qos_profile)
        self.brw = self.create_publisher(Float64,
                                         '/agro_bot/cmd_vel/wheel/back_right_wheel',
                                         qos_profile)
        self.blw = self.create_publisher(Float64,
                                         '/agro_bot/cmd_vel/wheel/back_left_wheel',
                                         qos_profile)

        self.mainTimer = self.create_timer(0.066, self.timer_callback)

    def timer_callback(self):
        speed = Float64()
        speed.data = self.cmdVelMsg.linear.x
        steering = Float64()
        steering.data = self.cmdVelMsg.angular.z

        if steering.data >= 1.:
            steering.data = 1.
        elif steering.data <= -1.:
            steering.data = -1.
        if speed.data >= 1.:
            speed.data = 1.
        elif speed.data <= -1.:
            speed.data = -1.

        speed.data = speed.data / WHEEL_RADIUS

        for st in [self.frs, self.fls, self.brs, self.bls]:
            st.publish(steering)
        for sp in [self.frw, self.flw, self.brw, self.blw]:
            sp.publish(speed)

    def cmd_vel_callback(self, msg):
        self.cmdVelMsg = msg


def main():
    rclpy.init()
    node = GzController("gz_controller")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
