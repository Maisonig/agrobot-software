import os

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_path = get_package_share_directory('agro_bot')
    launch_path = os.path.join(package_path, 'launch')

    action_simulation_launch = IncludeLaunchDescription(
        launch_description_source=os.path.join(launch_path, 'gz.launch.py')
    )

    action_rviz_launch = IncludeLaunchDescription(
        launch_description_source=os.path.join(launch_path, 'rviz.launch.py')
    )

    action_keyboard_controller = Node(
        package='agro_bot',
        executable='keyboard_controller.py'
    )

    action_movement_controller = Node(
        package='agro_bot',
        executable='sim_movement_controller',
        name='movement_controller',
    )

    ld = LaunchDescription()
    ld.add_action(action_simulation_launch)
    ld.add_action(action_rviz_launch)
    ld.add_action(action_keyboard_controller)
    ld.add_action(action_movement_controller)
    return ld
