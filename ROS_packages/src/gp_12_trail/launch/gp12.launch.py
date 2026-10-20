from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    pkg_gp12 = FindPackageShare('gp_12_trail')

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            pkg_gp12,
            'urdf',
            'gp12.xacro',
        ]),
    ])

    rviz_config = PathJoinSubstitution([
        pkg_gp12,
        'rviz',
        'gp12.rviz',
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(robot_description_content, value_type=str),
        }],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if needed',
        ),
        robot_state_publisher,
        joint_state_publisher,
        rviz_node,
    ])
