from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg = FindPackageShare("gp12_simulation")

    robot_description = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([
                pkg,
                "urdf",
                "gp12_gripper.xacro"
            ])
        ]),
        value_type=str
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description
        }],
        output="screen"
    )

    joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        rviz
    ])