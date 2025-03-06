import os

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_path = get_package_share_directory('agro_bot')
    launch_path = os.path.join(package_path, 'launch')
    param_file = os.path.join(get_package_share_directory("agro_bot"), 'config', 'agro_bot_sim_nav.yaml')
    use_sim_time = True

    action_simulation_launch = IncludeLaunchDescription(
        launch_description_source=os.path.join(launch_path, 'gz.launch.py')
    )

    action_robot_localization_ekf_launch = IncludeLaunchDescription(
        launch_description_source=os.path.join(launch_path, 'robot_localization_ekf.launch.py')
    )

    action_rviz_launch = IncludeLaunchDescription(
        launch_description_source=os.path.join(launch_path, 'rviz.launch.py')
    )

    action_front_cloud_to_scan = Node(
        package='agro_bot',
        executable='cloud_to_scan.py',
        name='front_cloud_to_scan',
        parameters=[param_file, {'use_sim_time': use_sim_time}]
    )

    action_rear_cloud_to_scan = Node(
        package='agro_bot',
        executable='cloud_to_scan.py',
        name='rear_cloud_to_scan',
        parameters=[param_file, {'use_sim_time': use_sim_time}]
    )

    action_left_cloud_to_scan = Node(
        package='agro_bot',
        executable='cloud_to_scan.py',
        name='left_cloud_to_scan',
        parameters=[param_file, {'use_sim_time': use_sim_time}]
    )

    action_right_cloud_to_scan = Node(
        package='agro_bot',
        executable='cloud_to_scan.py',
        name='right_cloud_to_scan',
        parameters=[param_file, {'use_sim_time': use_sim_time}]
    )

    action_path_mapping = Node(
        package='agro_bot',
        executable='path_mapping.py',
        name='path_mapping',
        parameters=[param_file, {'use_sim_time': use_sim_time}]
    )

    action_path_controller = Node(
        package='agro_bot',
        executable='path_controller.py',
        name='path_controller',
        parameters=[param_file, {'use_sim_time': use_sim_time}]
    )

    action_movement_control = Node(
        package='agro_bot',
        executable='movement_controller',
        name='controller',
    )

    action_waypoint_follower = Node(
        package='agro_bot',
        executable='waypoint_follower.py',
        name='waypoint_follower',
        parameters=[param_file, {'use_sim_time': use_sim_time}]
    )

    ld = LaunchDescription()
    ld.add_action(action_simulation_launch)
    ld.add_action(action_rviz_launch)

    # ld.add_action(action_front_cloud_to_scan)
    # ld.add_action(action_rear_cloud_to_scan)
    # ld.add_action(action_left_cloud_to_scan)
    # ld.add_action(action_right_cloud_to_scan)

    ld.add_action(action_path_mapping)

    ld.add_action(action_robot_localization_ekf_launch)
    ld.add_action(action_path_controller)
    ld.add_action(action_movement_control)
    # ld.add_action(action_waypoint_follower)
    return ld
