from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    joy_control_params = os.path.join(get_package_share_directory("p3at_controller"), "config", "joy_control.yaml")
    return LaunchDescription(
        [
            Node(
                package="joy_linux",
                executable="joy_linux_node",
                parameters=[
                    {
                        "autorepeat_rate": 50.0,
                        "deadzone": 0.01,
                        "coalesce_interval": 0.02,
                    }
                ],
            ),
            Node(
                package="teleop_twist_joy",
                executable="teleop_node",
                name="teleop_twist_joy_node",
                remappings=[
                    ('/cmd_vel', '/joy_cmd_vel')
                ],
                parameters=[
                    os.path.join(
                        get_package_share_directory("p3at_controller"), "config", "controller.yaml"
                    )
                ],
            ),
            Node(
                package="p3at_cpp_nodes",
                executable="joy_control",
                name="joy_control_node",
                parameters=[joy_control_params]
            ),
            Node(
                package="p3at_cpp_nodes",
                executable="odom_node2"
            )
        ]
    )
