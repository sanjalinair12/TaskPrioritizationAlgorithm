import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    pkg_share_dir = get_package_share_directory('robot_allocator')
    world_file = os.path.join(pkg_share_dir, 'worlds', 'dynamic_arena.world')
    
    # 1. Start Gazebo Server and Client
    gazebo_server = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen'
    )

    # 2. Spawn Robot Models
    # (Note: This is simplified. You would typically use the `spawn_entity.py` 
    # node from the gazebo_ros package for each model, specifying its URDF/SDF path 
    # and initial position.)

    # 3. Launch your allocator node (set to run once the environment is up)
    allocator_node = Node(
        package='robot_allocator',
        executable='allocator',
        name='allocator_node',
        output='screen',
    )
    
    return LaunchDescription([
        gazebo_server,
        # Add a delay here if needed, or wait for Gazebo service (best practice)
        allocator_node, 
    ])