import os

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_path = get_package_share_directory('agro_bot')
    launch_path = os.path.join(package_path, 'launch')
    param_file = os.path.join(get_package_share_directory("agro_bot"), 'config', 'agro_bot_sim.yaml')

    action_simulation_launch = IncludeLaunchDescription(
        launch_description_source=os.path.join(launch_path, 'gz.launch.py')
    )

    action_robot_localization_ekf_launch = IncludeLaunchDescription(
        launch_description_source=os.path.join(launch_path, 'robot_localization_ekf.launch.py')
    )

    action_rviz_launch = IncludeLaunchDescription(
        launch_description_source=os.path.join(launch_path, 'rviz.launch.py')
    )

    action_keyboard_controller = Node(
        package='agro_bot',
        executable='keyboard_controller.py'
    )

    action_front_cloud_to_scan = Node(
        package='agro_bot',
        executable='cloud_to_scan.py',
        name='front_cloud_to_scan',
        parameters=[param_file]
    )

    action_back_cloud_to_scan = Node(
        package='agro_bot',
        executable='cloud_to_scan.py',
        name='back_cloud_to_scan',
        parameters=[param_file]
    )

    action_front_image_to_detection = Node(
        package='agro_bot',
        executable='image_to_detection.py',
        name='front_image_to_detection',
        parameters=[param_file]
    )

    action_back_image_to_detection = Node(
        package='agro_bot',
        executable='image_to_detection.py',
        name='back_image_to_detection',
        parameters=[param_file]
    )

    action_local_mapping = Node(
        package='agro_bot',
        executable='local_mapping.py',
        name='local_mapping',
        parameters=[param_file]
    )

    ld = LaunchDescription()
    ld.add_action(action_simulation_launch)
    ld.add_action(action_rviz_launch)
    ld.add_action(action_robot_localization_ekf_launch)
    ld.add_action(action_keyboard_controller)
    ld.add_action(action_front_cloud_to_scan)
    ld.add_action(action_back_cloud_to_scan)
    ld.add_action(action_front_image_to_detection)
    ld.add_action(action_back_image_to_detection)
    ld.add_action(action_local_mapping)
    return ld
