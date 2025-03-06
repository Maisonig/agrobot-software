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
                                         '/agro_bot/cmd_vel/steering/rear_right_steering',
                                         qos_profile)
        self.bls = self.create_publisher(Float64,
                                         '/agro_bot/cmd_vel/steering/rear_left_steering',
                                         qos_profile)
        self.fllw = self.create_publisher(Float64,
                                         '/agro_bot/cmd_vel/wheel/front_left_left_wheel',
                                         qos_profile)
        self.flrw = self.create_publisher(Float64,
                                         '/agro_bot/cmd_vel/wheel/front_left_right_wheel',
                                         qos_profile)
        self.frlw = self.create_publisher(Float64,
                                         '/agro_bot/cmd_vel/wheel/front_right_left_wheel',
                                         qos_profile)
        self.frrw = self.create_publisher(Float64,
                                         '/agro_bot/cmd_vel/wheel/front_right_right_wheel',
                                         qos_profile)
        self.rllw = self.create_publisher(Float64,
                                         '/agro_bot/cmd_vel/wheel/rear_left_left_wheel',
                                         qos_profile)
        self.rlrw = self.create_publisher(Float64,
                                         '/agro_bot/cmd_vel/wheel/rear_left_right_wheel',
                                         qos_profile)
        self.rrlw = self.create_publisher(Float64,
                                         '/agro_bot/cmd_vel/wheel/rear_right_left_wheel',
                                         qos_profile)
        self.rrrw = self.create_publisher(Float64,
                                         '/agro_bot/cmd_vel/wheel/rear_right_right_wheel',
                                         qos_profile)

        self.mainTimer = self.create_timer(0.066, self.timer_callback)

    def timer_callback(self):
        speed = Float64()
        speed.data = self.cmdVelMsg.linear.x
        steering = Float64()
        steering.data = self.cmdVelMsg.angular.z

        if steering.data >= 6.28:
            steering.data = 6.28
        elif steering.data <= -6.28:
            steering.data = -6.28
        if speed.data >= 3.:
            speed.data = 3.
        elif speed.data <= -3.:
            speed.data = -3.

        speed.data = speed.data / WHEEL_RADIUS

        for st in [self.frs, self.fls, self.brs, self.bls]:
            st.publish(steering)
        for sp in [self.fllw, self.flrw, self.frlw, self.frrw, self.rllw, self.rlrw, self.rrlw, self.rrrw]:
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
