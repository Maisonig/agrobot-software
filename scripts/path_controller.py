#!/usr/bin/env python3

import math
import time
import rclpy

from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from rcl_interfaces.srv import GetParameters
from geometry_msgs.msg import Twist, PoseStamped, Pose

from util.utils import decart_to_polar, euler_from_quaternion


class PathController(Node):

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.declare_parameter('frequency', 30)

        # In this section, the topics that the node subscribes to get odometry and a path
        self.declare_parameter('path_topic', '/path')
        self.declare_parameter('odom_topic', '/odom')

        # The topic to publish goal velocities 'Twist'
        self.declare_parameter('cmd_topic', '/cmd_vel')

        # Average linear and angular velocities
        self.declare_parameter('average_linear_speed', .5)
        self.declare_parameter('average_angular_speed', .4)

        # The node uses a service 'GetParameters' from which it gets the ROS2 <path_state> parameter.
        # If the parameter value is True, then the controller publishes Twist in the <cmd_topic> topic.
        self.declare_parameter('path_state_service_node', 'path_mapping')

        frequency = self.get_parameter('frequency').get_parameter_value().integer_value
        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        path_state_service = self.get_parameter('path_state_service_node').get_parameter_value().string_value

        self.averageLinearSpeed = self.get_parameter('average_linear_speed').get_parameter_value().double_value
        self.averageAngularSpeed = self.get_parameter('average_angular_speed').get_parameter_value().double_value

        # Client to get path state and finder type from other node
        # Robot moves only if path state is True
        # Finder type determines the movement kinematic
        self.cli = self.create_client(GetParameters, f'/{path_state_service}/get_parameters')
        self.req = GetParameters.Request()
        # Timer for client requesting
        self.service_timer = self.create_timer(0.33, self.service_timer_callback)
        self.pathState = False
        self.finderType = ''
        # First request must be after path mapping node initialization for time synchronization
        time.sleep(5)
        self.req.names = ['path_state', 'finder_type']
        self.future = self.cli.call_async(self.req)

        # Subscribers and publishers
        self.pathSub = self.create_subscription(Path,
                                                path_topic,
                                                self.path_callback,
                                                10)

        self.odomSub = self.create_subscription(Odometry,
                                                odom_topic,
                                                self.odom_callback,
                                                10)
        self.cmdPub = self.create_publisher(Twist,
                                            cmd_topic,
                                            10)

        # Main timer for control
        self.mainTimer = self.create_timer(1 / frequency, self.timer_callback)

        self.pathData = Path()
        self.odomData = Odometry()

    def path_callback(self, msg):
        self.pathData = msg

    def odom_callback(self, msg):
        self.odomData = msg

    def service_timer_callback(self):
        if not self.cli.wait_for_service(timeout_sec=2.0):
            self.pathState = False
            self.finderType = ''
            self.get_logger().info('service not available, waiting again...')
        else:
            if self.future.done():
                self.pathState = self.future.result().values[0].bool_value
                self.finderType = self.future.result().values[1].string_value
                self.future.set_result(None)
                self.future = self.cli.call_async(self.req)

    def timer_callback(self):
        # If path is available
        if self.pathState:
            pose = self.odomData.pose.pose
            path = self.pathData.poses
            # Check current pathfinding algorithm and choose controller
            if self.finderType == 'astar':
                x_speed, y_speed, z_speed = self.control_astar_path(pose, path)
            elif self.finderType == 'hybrid_astar':
                x_speed, y_speed, z_speed = self.control_hybrid_astar_path(pose, path)
            elif self.finderType == 'omni_hybrid_astar':
                x_speed, y_speed, z_speed = self.control_omni_hybrid_astar_path(pose, path)
            else:
                self.get_logger().error("Unknown finder type! Stopping path control")
                x_speed, y_speed, z_speed = 0., 0., 0.
        else:
            x_speed, y_speed, z_speed = 0., 0., 0.
        # Publish velocities: linear -> x, y; angular -> z
        self.publish_twist(x_speed, y_speed, z_speed)

    def publish_twist(self, x, y, z):
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.angular.z = z
        self.cmdPub.publish(msg)

    def simple_control(self, pose: Pose, path: list[PoseStamped]):
        robot_x, robot_y = pose.position.x, pose.position.y
        robot_angle = euler_from_quaternion(pose.orientation.x,
                                            pose.orientation.y,
                                            pose.orientation.z,
                                            pose.orientation.w)[2]
        goal_x, goal_y = path[1].pose.position.x, path[1].pose.position.y
        steer = decart_to_polar(goal_x - robot_x, goal_y - robot_y)[1]
        x = self.averageLinearSpeed
        z = steer - robot_angle
        print(f"ANGLE TO TURN: {math.degrees(steer)}")
        print(f"ANGLE RESULT: {math.degrees(z)}")
        return x, 0., z

    def control_astar_path(self, pose: Pose, path: list[PoseStamped]):
        robot_x, robot_y = pose.position.x, pose.position.y
        robot_angle = euler_from_quaternion(pose.orientation.x,
                                            pose.orientation.y,
                                            pose.orientation.z,
                                            pose.orientation.w)[2]
        path_length = len(path)
        if path_length >= 2:
            goal_x, goal_y = path[1].pose.position.x, path[1].pose.position.y
            x, y = (goal_x - robot_x), (goal_y - robot_y)
            x_ = x * math.cos(robot_angle) + y * math.sin(robot_angle)
            y_ = -x * math.sin(robot_angle) + y * math.cos(robot_angle)
            max_s = max([abs(x), abs(y)])
            k = self.averageLinearSpeed / max_s

            x_ = x_ * k
            y_ = y_ * k

            path_angle = euler_from_quaternion(path[0].pose.orientation.z,
                                               path[0].pose.orientation.y,
                                               path[0].pose.orientation.w,
                                               path[0].pose.orientation.x)[2]
            if path_angle < 0:
                path_angle += 6.28
            if robot_angle < 0:
                robot_angle += 6.28
            delta_angle = path_angle - robot_angle
            ranged = delta_angle - int(delta_angle / math.pi) * 2 * math.pi
            if ranged != 0:
                # z_ = self.averageAngularSpeed * ranged / abs(ranged) * 0.1
                z_ = 0.
            else:
                z_ = 0.
        else:
            path_angle = euler_from_quaternion(path[0].pose.orientation.z,
                                               path[0].pose.orientation.y,
                                               path[0].pose.orientation.w,
                                               path[0].pose.orientation.x)[2]
            if path_angle < 0:
                path_angle += 6.28
            if robot_angle < 0:
                robot_angle += 6.28
            delta_angle = path_angle - robot_angle
            ranged = delta_angle - int(delta_angle / math.pi) * 2 * math.pi

            x_ = 0.
            y_ = 0.
            z_ = self.averageAngularSpeed * ranged / abs(ranged)
        return x_, y_, z_

    def control_hybrid_astar_path(self, pose: Pose, path: list[PoseStamped]):
        robot_x, robot_y = pose.position.x, pose.position.y
        robot_angle = euler_from_quaternion(pose.orientation.x,
                                            pose.orientation.y,
                                            pose.orientation.z,
                                            pose.orientation.w)[2]
        if robot_angle < 0:
            robot_angle += 6.28
        path_length = len(path)
        if path_length >= 2:
            goal_x, goal_y = path[1].pose.position.x, path[1].pose.position.y
            goal_th = euler_from_quaternion(path[1].pose.orientation.w,
                                            path[1].pose.orientation.y,
                                            path[1].pose.orientation.z,
                                            path[1].pose.orientation.x)[0]
            if goal_th < 0:
                goal_th += 6.28
            delta_angle = goal_th - robot_angle
            ranged = delta_angle - int(delta_angle / math.pi) * 2 * math.pi
            delta_path = math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
            dt = delta_path / self.averageLinearSpeed
            # a = 1
            # x1, y1 = polar_to_decart(a, ranged)
            # path_angle = decart_to_polar(goal_x - robot_x, goal_y - robot_y)[1]
            # x2, y2 = polar_to_decart(a, path_angle)
            #
            # xr1, yr1 = rotate_point((x1, y1), ranged)
            # xr2, yr2 = rotate_point((x2, y2), ranged)
            # if xr1 / xr2 < 0:
            #     x_ = -self.averageLinearSpeed
            #     z_ = abs(ranged) * ranged / abs(ranged)
            # else:

            x_ = self.averageLinearSpeed
            y_ = 0.
            z_ = abs(ranged) * ranged / abs(ranged)
            z_ = z_ / dt

        else:
            path_angle = euler_from_quaternion(path[0].pose.orientation.z,
                                               path[0].pose.orientation.y,
                                               path[0].pose.orientation.w,
                                               path[0].pose.orientation.x)[2]
            if path_angle < 0:
                path_angle += 6.28
            delta_angle = path_angle - robot_angle
            ranged = delta_angle - int(delta_angle / math.pi) * 2 * math.pi
            # if abs(delta_angle) > self.averageAngularSpeed:

            x_ = 0.
            y_ = 0.
            z_ = self.averageAngularSpeed * ranged / abs(ranged)
        return x_, y_, z_

    def control_omni_hybrid_astar_path(self, pose: Pose, path: list[PoseStamped]):
        return 0., 0., 0.

def main():
    rclpy.init()
    node = PathController("path_controller")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
