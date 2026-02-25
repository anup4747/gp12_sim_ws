from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Paths
    pkg_gp12_support = FindPackageShare('motoman_gp12_support')
    pkg_gz_sim = FindPackageShare('ros_gz_sim')
    pkg_gp12_gazebo = FindPackageShare('gp12_gazebo')

    # Robot description (load official xacro)
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([pkg_gp12_gazebo, 'urdf', 'gp12.xacro'])
    ])

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_gz_sim, 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={'gz_args': ' -r -v 4 empty.sdf'}.items()
    )
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description_content}]
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'motoman_gp12',
                   '-x', '0', '-y', '0', '-z', '0.1'],
        output='screen'
    )

    # ros2_control node (loads controllers)
    controller_manager_node = Node(
    package='controller_manager',
    executable='ros2_control_node',
    parameters=[
        {'robot_description': ParameterValue(robot_description_content, value_type=str)},
        PathJoinSubstitution([pkg_gp12_gazebo, 'config', 'gp12_controllers.yaml'])
    ],
    output='screen'
    )

    # Load controllers (broadcaster + trajectory)
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    load_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
    )

    # Bridge (for clock, etc. if needed)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity,
        controller_manager_node,
        load_joint_state_broadcaster,
        load_arm_controller,
        bridge,
    ])
