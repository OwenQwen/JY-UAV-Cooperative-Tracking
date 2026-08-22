from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_arg = DeclareLaunchArgument('use_target_sim', default_value='true')
    use_target_sim = LaunchConfiguration('use_target_sim')

    config_path = os.path.join(
        get_package_share_directory('uav_tracking_control'),
        'config',
        'mission.yaml',
    )

    mission_node = Node(
        package='uav_tracking_control',
        executable='mission_node.py',
        output='screen',
        parameters=[config_path]
    )

    target_sim_node = Node(
        package='uav_tracking_control',
        executable='target_simulator.py',
        output='screen',
        condition=IfCondition(use_target_sim)
    )

    return LaunchDescription([use_sim_arg, mission_node, target_sim_node])
