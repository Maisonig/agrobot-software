import os
import yaml

from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_path = get_package_share_directory('agro_bot')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    param_file = os.path.join(get_package_share_directory("agro_bot"), 'config', 'launch.yaml')

    with open(param_file, 'r') as param:
        configuration = yaml.safe_load(param)
        model_name = configuration["launch"]["ros__parameters"]["gz_model_name"]

    action_gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            package_path,
            'model',
            model_name
        ])}.items(),
    )

    action_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(package_path, 'config', 'gz_remapings.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    action_gz_controller = Node(
        package='agro_bot',
        executable='controller.py'
    )

    ld = LaunchDescription()
    ld.add_action(action_gz_sim)
    ld.add_action(action_bridge)
    ld.add_action(action_gz_controller)
    return ld
