import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    sick_scan_prefix = get_package_share_directory("sick_scan_xd")
    launch_file = "sick_tim_7xxS.launch"
    launch_file_path = os.path.join(sick_scan_prefix, "launch", launch_file)
    config_file = os.path.join(get_package_share_directory("p3at_bringup"), "config", "lidar.yaml")
    # argument = [launch_file_path]

    launch_lidar = Node(
        package="sick_scan_xd",
        executable="sick_generic_caller",
        name="lidar",
        parameters=[
            config_file
        ],
        remappings=[
            ("cloud", "/lidar/cloud"),
            ("/sick_tim_7xxS/scan", "/lidar/scan")
        ],
        output="screen"
    )

    return LaunchDescription([
        launch_lidar,
        # Node(
        #     package="p3at_cpp_nodes",
        #     executable="test_lidar"
        # )
    ])