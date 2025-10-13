from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'robot_allocator'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),  # 👈 no "where='src'" since your code is already here
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf.xacro')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'data'), glob('data/*.xlsx')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Robot allocator node using AHP–TOPSIS–VIKOR hybrid decision logic',
    license='Apache License 2.0',
    entry_points={
    'console_scripts': [
        'allocator = robot_allocator.allocator_node:main',
    ],
},

)
