from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'robot_allocator'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf.xacro')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anjali',
    maintainer_email='sanjaliasn@gmail.com',
    description='Heterogeneous Robot team formation and Task allocation package using AHP–TOPSIS–VIKOR hybrid logic.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'allocator = robot_allocator.allocator_node:main',  # must match function name
        ],
    },
)
