from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('ahtovik_gazebo')
    world = os.path.join(pkg, 'worlds', 'ahtovik_30_robot_motion_world.sdf')
    return LaunchDescription([
        ExecuteProcess(cmd=['gazebo', '--verbose', world], output='screen'),
        Node(package='ahtovik_gazebo', executable='ahtovik_motion_demo', name='ahtovik_motion_demo', output='screen')
    ])
