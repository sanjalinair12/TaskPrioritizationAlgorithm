from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='ahtovik_gazebo', executable='telemetry_publisher_node', name='telemetry_publisher', output='screen'),
        Node(package='ahtovik_gazebo', executable='task_allocator_node', name='task_allocator', output='screen'),
        Node(package='ahtovik_gazebo', executable='failure_injector_node', name='failure_injector', output='screen'),
    ])
