from setuptools import setup
from glob import glob
import os

package_name = 'ahtovik_gazebo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models/uav'), glob('models/uav/*')),
        (os.path.join('share', package_name, 'models/ugv'), glob('models/ugv/*')),
        (os.path.join('share', package_name, 'models/creeper_robot'), glob('models/creeper_robot/*')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Anjali Santhosh',
    maintainer_email='sanjaliasn@gmail.com',
    description='AhToVik ROS 2 Gazebo demo',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'telemetry_publisher_node = ahtovik_gazebo.telemetry_publisher_node:main',
            'task_allocator_node = ahtovik_gazebo.task_allocator_node:main',
            'failure_injector_node = ahtovik_gazebo.failure_injector_node:main',
            'experiment_logger_node = ahtovik_gazebo.experiment_logger_node:main',
        ],
    },
)
