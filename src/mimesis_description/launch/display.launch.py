"""Show the Mimesis arm in RViz.

ros2 launch mimesis_description display.launch.py              # sliders move the model
ros2 launch mimesis_description display.launch.py jsp:=none    # follow /joint_states
                                                               # from the real arm's driver
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package = FindPackageShare("mimesis_description")
    jsp = LaunchConfiguration("jsp")

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                PathJoinSubstitution([package, "urdf", "mimesis.urdf.xacro"]),
                " prefix:=",
                LaunchConfiguration("prefix"),
            ]
        ),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "jsp",
                default_value="gui",
                choices=["gui", "headless", "none"],
                description="Joint states: sliders (gui), all zeros (headless), "
                "or none when a driver publishes /joint_states",
            ),
            DeclareLaunchArgument(
                "prefix", default_value="", description="Link name prefix"
            ),
            DeclareLaunchArgument(
                "rviz", default_value="true", description="Start RViz"
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                condition=IfCondition(PythonExpression(["'", jsp, "' == 'gui'"])),
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                condition=IfCondition(PythonExpression(["'", jsp, "' == 'headless'"])),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    PathJoinSubstitution([package, "rviz", "display.rviz"]),
                ],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
