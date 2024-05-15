from setuptools import find_packages, setup
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from glob import glob

package_name = 'p3at_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joshua',
    maintainer_email='joshua@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imageRecognitionNode = p3at_vision.imageRecognitionNode:main',
            'digitRecognitionNode = p3at_vision.digitRecognitionNode:main'
        ],
    },
)
