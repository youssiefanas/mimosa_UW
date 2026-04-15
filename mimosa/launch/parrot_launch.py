from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('mimosa')

    viz_arg = DeclareLaunchArgument('viz', default_value='false')

    mimosa_node = Node(
        package='mimosa',
        executable='mimosa_node',
        name='mimosa_node',
        output='screen',
        parameters=[{
            'config_path': os.path.join(pkg_share, 'config', 'parrot', 'params.yaml'),
            'use_sim_time': True,
        }],
        remappings=[
            ('~/imu/manager/imu_in', '/imu/data'),
            ('~/odometry/manager/odometry_in', '/odometry'),
            ('~/dvl/manager/dvl_in', '/dvl/data'),
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'mimosa.rviz')],
        condition=IfCondition(LaunchConfiguration('viz')),
    )

    return LaunchDescription([
        viz_arg,
        mimosa_node,
        rviz_node,
    ])
