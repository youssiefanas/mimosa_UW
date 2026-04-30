"""
BlueROV2 (recorded bag) replay launch.

Plays a ROS 2 bag through the mimosa_rosbag executable + the in-package
nortek_imu_bridge so that an offline BlueROV2 + Nortek Nucleus dataset is
processed identically to the live `bluerov2_launch.py` pipeline.

Usage:
    ros2 launch mimosa bluerov2_rosbag_launch.py bag_name:=/path/to/bag
    ros2 launch mimosa bluerov2_rosbag_launch.py bag_name:=/path/to/bags/'*' s:=2.5

Args:
    bag_name : single bag dir, or shell-glob pattern across multiple bags
               (the * is expanded inside mimosa_rosbag, not the shell).
    s        : seconds to skip from the start of each bag (default 0.0).
    viz      : true to launch RViz (default false).

The mimosa node calls rclcpp::shutdown() when the bag(s) finish, and the
on_exit handler tears the rest of the launch tree down — so the launch
returns immediately after the last bag, making this safe to chain in a
batch script.

Trajectory output (TUM): see `logs_directory` in bluerov2/params.yaml,
e.g. /tmp/mimosa_bluerov2/graph_manager_odometry.tum.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mimosa')

    bag_name_arg = DeclareLaunchArgument('bag_name')
    s_arg = DeclareLaunchArgument('s', default_value='0.0')
    viz_arg = DeclareLaunchArgument('viz', default_value='false')

    # Convert interfaces/msg/IMU (from the bagged /nucleus_node/imu_packets)
    # into sensor_msgs/msg/Imu on /nortek/imu, which is what mimosa_rosbag
    # subscribes to.
    nortek_imu_bridge = Node(
        package='mimosa',
        executable='nortek_imu_bridge',
        name='nortek_imu_bridge',
        output='screen',
        remappings=[
            ('~/imu_in',  '/nucleus_node/imu_packets'),
            ('~/imu_out', '/nortek/imu'),
        ],
    )

    mimosa_node = Node(
        package='mimosa',
        executable='mimosa_rosbag',
        name='mimosa_node',
        output='screen',
        parameters=[{
            'config_path': os.path.join(pkg_share, 'config', 'bluerov2', 'params.yaml'),
            'bag_name': LaunchConfiguration('bag_name'),
            's': LaunchConfiguration('s'),
        }],
        remappings=[
            ('~/imu/manager/imu_in',           '/nortek/imu'),
            ('~/dvl/manager/dvl_in',           '/nucleus_node/bottom_track_packets'),
            ('~/depth/manager/depth_in',       '/nucleus_node/bottom_track_packets'),
            ('~/odometry/manager/odometry_in', '/visual_odom_node/odometry'),
        ],
        # When the bag(s) finish, mimosa_rosbag shuts down — tear the whole
        # launch tree down with it so batch scripts see a clean exit.
        on_exit=Shutdown(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'mimosa.rviz')],
        condition=IfCondition(LaunchConfiguration('viz')),
    )

    return LaunchDescription([
        bag_name_arg,
        s_arg,
        viz_arg,
        nortek_imu_bridge,
        mimosa_node,
        rviz_node,
    ])
