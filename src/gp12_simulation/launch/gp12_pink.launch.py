from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    udp_port = LaunchConfiguration('udp_port', default='5555')

    pkg_gp12_simulation = FindPackageShare('gp12_simulation')

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([pkg_gp12_simulation, 'urdf', 'gp12.xacro']),
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(robot_description_content, value_type=str),
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    urdf_path = PathJoinSubstitution([pkg_gp12_simulation, 'urdf', 'gp12.urdf'])

    pink_ik_node = Node(
        package='gp12_simulation',
        executable='pink_ik_node.py',
        name='pink_ik_node',
        output='screen',
        parameters=[{
            'urdf_path': urdf_path,
            'udp_port': udp_port,
            'base_frame': 'base_link',
            'ee_frame': 'tool0',
            'publish_topic': '/joint_states',
            'rate_hz': 100.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('udp_port', default_value='5555'),
        robot_state_publisher,
        rviz_node,
        pink_ik_node,
    ])

