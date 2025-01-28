import os
import yaml

from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_path = get_package_share_directory('agro_bot')
    param_file = os.path.join(get_package_share_directory("agro_bot"), 'config', 'launch.yaml')

    with open(param_file, 'r') as param:
        configuration = yaml.safe_load(param)
        model_name = configuration["launch"]["ros__parameters"]["rviz_model_name"]
        print(model_name)

    urdf_file = os.path.join(package_path, 'model', model_name)

    with open(urdf_file, 'r') as urdf_file:
        robot_desc = urdf_file.read()

    action_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    action_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d' + os.path.join(package_path, 'rviz', 'agro_bot.rviz')]
    )

    ld = LaunchDescription()
    ld.add_action(action_robot_state_publisher)
    ld.add_action(action_rviz_node)
    return ld
