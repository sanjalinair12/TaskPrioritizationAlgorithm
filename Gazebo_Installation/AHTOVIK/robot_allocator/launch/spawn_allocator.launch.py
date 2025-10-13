import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Function to run XACRO conversion and spawn a model
def spawn_model(name, model_file, initial_pose_xyz):
    pkg_share_dir = get_package_share_directory('robot_allocator')
    urdf_file = os.path.join(pkg_share_dir, 'urdf', model_file)

    # 1. Convert XACRO to URDF (needed for all XACRO files)
    xacro_command = ExecuteProcess(
        cmd=['xacro', urdf_file, '-o', '/tmp/' + name + '.urdf'],
        output='screen'
    )

    # 2. Spawn the model in Gazebo
    spawn_command = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', name,
                   '-file', '/tmp/' + name + '.urdf',
                   '-x', initial_pose_xyz[0],
                   '-y', initial_pose_xyz[1],
                   '-z', initial_pose_xyz[2],
                   '-unpause'],
        output='screen'
    )
    return [xacro_command, spawn_command]

def generate_launch_description():
    pkg_share_dir = get_package_share_directory('robot_allocator')
    world_file = os.path.join(pkg_share_dir, 'worlds', 'dynamic_arena.world')
    
    # 1. Launch Gazebo with your custom world
    # This requires the ROS 2 Gazebo package to be installed (gazebo_ros)
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file}.items(),
    )

    # 2. Spawn the Heterogeneous Robots
    # NOTE: The names here must match the model names in your Excel file for the allocator node to find them!
    
    # Drone (e.g., drone_A) - Spawn slightly above ground
    drone_spawn = spawn_model('drone_A', 'drone.urdf.xacro', ['-2.0', '0.0', '1.0'])
    
    # Ground Lifter (e.g., ground_robot_1) - Spawn on the ground
    ground_robot_spawn = spawn_model('ground_robot_1', 'ground_robot.urdf.xacro', ['-3.0', '0.0', '0.1'])
    
    # Snakebot (e.g., snakebot_alpha) - Spawn on the ground
    snakebot_spawn = spawn_model('snakebot_alpha', 'snakebot.urdf.xacro', ['-4.0', '0.0', '0.05'])
    
    # 3. Launch the Allocator Node
    allocator_node = Node(
        package='robot_allocator',
        executable='allocator',
        name='allocator_node',
        output='screen',
        # Set the log level to INFO for detailed output
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        gazebo_server,
        *drone_spawn,
        *ground_robot_spawn,
        *snakebot_spawn,
        allocator_node,
    ])
