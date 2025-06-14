import launch
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return launch.LaunchDescription(
        [
            Node(
                package="p3at_vision",
                executable="imageRecognitionNode",
            ),

            Node(
                package="p3at_vision",
                executable="digitRecognitionNode",
            ),

            Node(
                package="p3at_vision",
                executable="marker_node",
            ),
            Node(
                package="p3at_vision",
                executable="obstacle_detector",
            ),
        
            # launch.actions.ExecuteProcess(
            #     cmd=['ros2', 'bag', 'record', '-o', 'SavedImages', '/photos_taken'],
            #     output='screen'
            # )
        ]
    )

