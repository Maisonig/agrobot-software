#!/usr/bin/env python3
import time
import rclpy

from shapely import Point, Polygon
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from util.parallel import find_start_cords, find_parallel_trajectory
from util.utils import quaternion_from_euler
from nav_msgs.msg import Path


class WaypointFollower(Node):

    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.declare_parameter('frequency', 30)
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('waypoints', [0., 0., 0., -13., 0., 0., -13., 2.5, 0., 0., 2.5, 0., 0., 0., 0.])
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('start_following', True)

        goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        frequency = self.get_parameter('frequency').get_parameter_value().integer_value
        points = self.get_parameter('waypoints').get_parameter_value().double_array_value
        self.mapFrame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.isStart = False
        self.waypoints = []
        j = 0
        for i in range(int(len(points) / 3)):
            self.waypoints.append([points[j], points[j + 1], points[j + 2]])
            j += 3


        # Timer for client requesting
        self.cli = self.create_client(GetParameters, '/path_mapping/get_parameters')
        self.req = GetParameters.Request()
        self.service_timer = self.create_timer(0.33, self.service_timer_callback)
        self.pathState = True
        # First request must be after path mapping node initialization for time synchronization
        time.sleep(5)
        self.req.names = ['path_state']
        self.future = self.cli.call_async(self.req)

        self.trajPub = self.create_publisher(Path, 'trajectory', 10)

        self.goalPub = self.create_publisher(PoseStamped,
                                             goal_topic,
                                             10)

        self.mainTimer = self.create_timer(1, self.timer_callback)

        self.goalData = PoseStamped()

        self.pubFlg = False

        # Пример полигона (задается в метрах)
        polygon_coords = [
            (0, -1.5),
            (-13, 0),
            (-13, 25),
            (0, 25),
        ]

        # Текущее положение техники

        polygon = Polygon(polygon_coords)
        current_position = Point(0, 0)
        lines_angle = 3.14
        lines_width = 3

        if not polygon.is_valid:
            print("Некорректный полигон.")
            exit()

        offset_x, offset_y, offset_th, parallel_dir = find_start_cords(polygon, current_position, lines_angle,
                                                                       lines_width)
        self.traj = find_parallel_trajectory(polygon, offset_x, offset_y, offset_th, parallel_dir, lines_width)


        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.mapFrame
        self.waypoints = []
        self.traj = self.traj.coords
        for p in self.traj:
            ps = PoseStamped()
            ps.header.frame_id = self.mapFrame
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x = float(p[0])
            ps.pose.position.y = float(p[1])
            # q = quaternion_from_euler(0., 0., p[2])
            # ps.pose.orientation.x = q[0]
            # ps.pose.orientation.y = q[1]
            # ps.pose.orientation.z = q[2]
            # ps.pose.orientation.w = q[3]
            msg.poses.append(ps)
            self.waypoints.append((p[0], p[1], 0.))
        self.trajPub.publish(msg)

    def goal_callback(self, msg):
        self.goalData = msg

    def service_timer_callback(self):
        if not self.cli.wait_for_service(timeout_sec=2.0):
            self.pathState = False
            self.get_logger().info('service not available, waiting again...')
        else:
            if self.future.done():
                self.pathState = self.future.result().values[0].bool_value
                self.future.set_result(None)
                self.future = self.cli.call_async(self.req)

    def timer_callback(self):

        if self.isStart:
            if not self.pubFlg:
                self.goalData.header.stamp = self.get_clock().now().to_msg()
                self.goalData.header.frame_id = self.mapFrame
                self.goalData.pose.position.x = self.waypoints[0][0]
                self.goalData.pose.position.y = self.waypoints[0][1]
                q = quaternion_from_euler(0., 0., self.waypoints[0][2])
                self.goalData.pose.orientation.x = q[1]
                self.goalData.pose.orientation.y = q[2]
                self.goalData.pose.orientation.z = q[3]
                self.goalData.pose.orientation.w = q[0]
                self.goalPub.publish(self.goalData)
                self.pubFlg = True
            else:
                if not self.pathState:
                    self.get_logger().info("Goal reached!")
                    self.pubFlg = False
                    self.waypoints.pop(0)
                    if len(self.waypoints) == 0:
                        self.get_logger().info("All waypoints reached!")
                        self.destroy_node()
                        exit()
        else:
            self.isStart = self.get_parameter('start_following').get_parameter_value().bool_value
            if self.isStart:
                self.get_logger().info('Starting waypoint following')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower("waypoint_follower")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()