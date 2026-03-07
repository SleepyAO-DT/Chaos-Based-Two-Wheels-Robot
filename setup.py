from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lidar_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.*'))),
        (os.path.join('share', package_name, 'urdf/sensors'), glob(os.path.join('urdf/sensors', '*.*'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xta',
    maintainer_email='xta@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'classic_vfh_controller = lidar_robot.auto:main',
            'path_controller = lidar_robot.path:main',
        ],
    },
)
