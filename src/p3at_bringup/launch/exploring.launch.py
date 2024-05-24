from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="p3at_cpp_nodes",
                executable="goal_client"
            ),
            Node(
                package="p3at_cpp_nodes",
                executable="explore",
                parameters=[
                    {
                        "range" : 1.0,
                        "time_duration" : 0.1,
                        "time_scale" : 1.0
                    }
                ]
            )
        ]
    )
