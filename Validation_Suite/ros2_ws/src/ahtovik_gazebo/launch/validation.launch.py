from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('algorithm', default_value='ahtovik'),
        DeclareLaunchArgument('v', default_value='0.5'),
        DeclareLaunchArgument('threshold', default_value='0.10'),
        DeclareLaunchArgument('noise_level', default_value='0.0'),
        DeclareLaunchArgument('run_id', default_value='0'),
        DeclareLaunchArgument('experiment_id', default_value='manual'),
        DeclareLaunchArgument('seed', default_value='42'),
        DeclareLaunchArgument('num_drones', default_value='3'),
        DeclareLaunchArgument('num_ground', default_value='4'),
        DeclareLaunchArgument('num_creepers', default_value='3'),
        DeclareLaunchArgument('fail_robot', default_value='uav_0'),
        DeclareLaunchArgument('fail_time', default_value='12.0'),
        DeclareLaunchArgument('failure_enabled', default_value='true'),
        DeclareLaunchArgument('duration_s', default_value='30'),
        DeclareLaunchArgument('output_dir', default_value='~/ahtovik_validation_results'),
        DeclareLaunchArgument('cbba_max_rounds', default_value='20'),
        DeclareLaunchArgument('cbba_bundle_size', default_value='3'),
    ]

    telemetry = Node(
        package='ahtovik_gazebo', executable='telemetry_publisher_node', name='telemetry_publisher', output='screen',
        parameters=[{
            'noise_level': LaunchConfiguration('noise_level'),
            'seed': LaunchConfiguration('seed'),
            'num_drones': LaunchConfiguration('num_drones'),
            'num_ground': LaunchConfiguration('num_ground'),
            'num_creepers': LaunchConfiguration('num_creepers'),
        }]
    )
    allocator = Node(
        package='ahtovik_gazebo', executable='task_allocator_node', name='task_allocator', output='screen',
        parameters=[{
            'algorithm': LaunchConfiguration('algorithm'),
            'v': LaunchConfiguration('v'),
            'threshold': LaunchConfiguration('threshold'),
            'noise_level': LaunchConfiguration('noise_level'),
            'run_id': LaunchConfiguration('run_id'),
            'experiment_id': LaunchConfiguration('experiment_id'),
            'cbba_max_rounds': LaunchConfiguration('cbba_max_rounds'),
            'cbba_bundle_size': LaunchConfiguration('cbba_bundle_size'),
        }]
    )
    failure = Node(
        package='ahtovik_gazebo', executable='failure_injector_node', name='failure_injector', output='screen',
        parameters=[{
            'fail_robot': LaunchConfiguration('fail_robot'),
            'fail_time': LaunchConfiguration('fail_time'),
            'enabled': LaunchConfiguration('failure_enabled'),
        }]
    )
    logger = Node(
        package='ahtovik_gazebo', executable='experiment_logger_node', name='experiment_logger', output='screen',
        parameters=[{
            'output_dir': LaunchConfiguration('output_dir'),
            'experiment_id': LaunchConfiguration('experiment_id'),
            'run_id': LaunchConfiguration('run_id'),
        }]
    )
    return LaunchDescription(args + [telemetry, allocator, failure, logger])
