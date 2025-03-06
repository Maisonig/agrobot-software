import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    action_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory("agro_bot"), 'config', 'robot_localization_ekf.yaml'),
                    {'use_sim_time': True},],
    )

    ld = LaunchDescription()
    ld.add_action(action_ekf_node)
    return ld
