from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('ahtovik_gazebo')
    world = os.path.join(pkg, 'worlds', 'sar_world.sdf')
    return LaunchDescription([
        ExecuteProcess(cmd=['gazebo', '--verbose', world], output='screen'),
        Node(package='ahtovik_gazebo', executable='telemetry_publisher_node', output='screen'),
        Node(package='ahtovik_gazebo', executable='task_allocator_node', output='screen'),
        Node(package='ahtovik_gazebo', executable='failure_injector_node', output='screen'),
    ])
