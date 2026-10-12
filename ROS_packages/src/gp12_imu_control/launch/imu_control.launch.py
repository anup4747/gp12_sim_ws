import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    udp_port_arg = DeclareLaunchArgument(
        'udp_port',
        default_value='5555',
        description='UDP port for HyperIMU Android app connection'
    )

    hyperimu_bridge_node = Node(
        package='gp12_imu_control',
        executable='hyperimu_bridge',
        name='hyperimu_bridge',
        output='screen',
        parameters=[{
            'udp_port': LaunchConfiguration('udp_port')
        }]
    )

    imu_to_moveit_node = Node(
        package='gp12_imu_control',
        executable='imu_to_moveit',
        name='imu_to_moveit',
        output='screen'
    )

    return LaunchDescription([
        udp_port_arg,
        hyperimu_bridge_node,
        imu_to_moveit_node
    ])
