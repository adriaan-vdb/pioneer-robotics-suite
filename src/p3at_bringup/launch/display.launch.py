from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    model_arg = DeclareLaunchArgument(
            "model",
            default_value=os.path.join(get_package_share_directory("p3at_bringup"), "urdf", "pioneer.urdf.xacro"),
            description= "Absolute path to urdf file."
    )

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration("model")]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(get_package_share_directory("p3at_bringup"), "rviz", "display2.rviz")],
    )

    overlay_node1 = Node(
        package='rviz_2d_overlay_plugins',
        executable='string_to_overlay_text',
        name='string_to_overlay_text_1',
        output='screen',
        parameters=[
            {"string_topic": "gear"},
            {"fg_color": "b"}, # colors can be: r,g,b,w,k,p,y (red,green,blue,white,black,pink,yellow)
            {"bg_color": "w"},
        ],
    )

    overlay_node2 = Node(
        package='rviz_2d_overlay_plugins',
        executable='string_to_overlay_text',
        name='string_to_overlay_text_2',
        output='screen',
        parameters=[
            {"string_topic": "safety"},
            {"fg_color": "b"}, # colors can be: r,g,b,w,k,p,y (red,green,blue,white,black,pink,yellow)
            {"bg_color": "w"},
        ],
    )

    overlay_node3 = Node(
        package='rviz_2d_overlay_plugins',
        executable='string_to_overlay_text',
        name='string_to_overlay_text_3',
        output='screen',
        parameters=[
            {"string_topic": "pose_text"},
            {"fg_color": "b"}, # colors can be: r,g,b,w,k,p,y (red,green,blue,white,black,pink,yellow)
            {"bg_color": "w"},
        ],
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        # joint_state_publisher_node,
        rviz2_node,
        overlay_node1,
        overlay_node2,
        overlay_node3,
        Node(
            package="p3at_vision",
            executable="gui"
        )
    ])