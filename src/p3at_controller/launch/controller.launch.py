from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    emer_params = os.path.join(get_package_share_directory("p3at_controller"), "config", "emergency.yaml")
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
                # remappings=[
                #     ('/cmd_vel', '/joy_cmd_vel')
                # ],
                parameters=[
                    os.path.join(
                        get_package_share_directory("p3at_controller"), "config/controller.yaml"
                    )
                ],
            ),
            Node(
                package="p3at_cpp_nodes",
                executable="joy_input",
                name="joy_input_node",
            ),
            Node(
                package="p3at_cpp_nodes",
                executable="emergency_stop",
                name="emergency_node",
                parameters=[emer_params]
            )
        ]
    )
