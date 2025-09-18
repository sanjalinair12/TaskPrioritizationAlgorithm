from setuptools import setup

package_name = 'AHTOVIK'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/spawn_robots.launch.py', 'launch/allocator_node.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/drone.urdf.xacro', 'urdf/ground_robot.urdf.xacro', 'urdf/creeper_robot.urdf.xacro']),
        ('share/' + package_name, ['robot_criteria_populated.xlsx']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AHTOVIK',
    maintainer_email='sanjaliasn@gmail.com',
    description='Heterogeneous Multi-robot task allocation with Gazebo integration',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'allocator_node = AHTOVIK.allocator_node:main'
        ],
    },
)
