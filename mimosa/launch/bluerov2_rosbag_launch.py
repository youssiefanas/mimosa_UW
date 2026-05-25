"""
BlueROV2 (recorded bag) replay launch.

Plays a ROS 2 bag through the mimosa_rosbag executable so that an offline
BlueROV2 + Nortek Nucleus dataset is processed identically to the live
`bluerov2_launch.py` pipeline.

Usage:
    ros2 launch mimosa bluerov2_rosbag_launch.py bag_name:=/path/to/bag
    ros2 launch mimosa bluerov2_rosbag_launch.py bag_name:=/path/to/bags/'*' s:=2.5

Args:
    bag_name             : single bag dir, or shell-glob pattern across
                           multiple bags (the * is expanded inside
                           mimosa_rosbag, not the shell).
    s                    : seconds to skip from the start (default 0.0).
    viz                  : true to launch RViz (default false).
    config_override      : optional YAML path; deep-merged on top of the
                           base bluerov2 params (same pattern as
                           eval_params_rosbag_launch_bluerov_imu.py).
    logs_directory       : optional override for the merged config's
                           logs_directory field — lets per-variant runs
                           write to distinct folders without producing a
                           full variant config file.

Trajectory output (TUM): see `logs_directory` in bluerov2/params.yaml,
e.g. /tmp/mimosa_bluerov2/graph_manager_odometry.tum.
"""

import os
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _deep_merge(base, override):
    if not isinstance(base, dict) or not isinstance(override, dict):
        return override
    out = dict(base)
    for key, val in override.items():
        out[key] = _deep_merge(out.get(key), val) if key in out else val
    return out


def _resolve_config_path(context, base_path):
    override_path = LaunchConfiguration('config_override').perform(context)
    logs_directory = LaunchConfiguration('logs_directory').perform(context)

    with open(base_path) as f:
        merged = yaml.safe_load(f) or {}
    if override_path:
        with open(override_path) as f:
            override = yaml.safe_load(f) or {}
        merged = _deep_merge(merged, override)
    if logs_directory:
        # Ensure trailing slash — mimosa concatenates without it.
        merged['logs_directory'] = logs_directory.rstrip('/') + '/'
        os.makedirs(merged['logs_directory'], exist_ok=True)

    if not override_path and not logs_directory:
        return base_path

    # The merged file is written to /tmp, so any relative paths inside the
    # config (notably lidar.sensor_json = "../os_dummy.json") would resolve
    # against /tmp and fail. Normalize them against the base config's
    # directory so they still point at the in-tree dummy file.
    base_dir = os.path.dirname(os.path.abspath(base_path))
    lidar = merged.get('lidar')
    if isinstance(lidar, dict) and 'sensor_json' in lidar:
        sensor_json = lidar['sensor_json']
        if isinstance(sensor_json, str) and not os.path.isabs(sensor_json):
            lidar['sensor_json'] = os.path.normpath(os.path.join(base_dir, sensor_json))

    fd, merged_path = tempfile.mkstemp(prefix='mimosa_bluerov2_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(merged, f, sort_keys=False)
    return merged_path


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('mimosa')
    base_config = os.path.join(pkg_share, 'config', 'bluerov2', 'nortek_imu_params.yaml')
    config_path = _resolve_config_path(context, base_config)

    mimosa_node = Node(
        package='mimosa',
        executable='mimosa_rosbag',
        name='mimosa_node',
        output='screen',
        parameters=[{
            'config_path': config_path,
            'bag_name': LaunchConfiguration('bag_name'),
            's': LaunchConfiguration('s'),
            # mimosa_rosbag reads the bag directly (no ROS pubsub for sensor
            # data), so the live nortek_imu_bridge can't sit in the data path.
            # This parameter tells mimosa_rosbag to also pull interfaces/msg/IMU
            # off the raw Nortek topic and convert it inline.
            # 'nortek_raw_imu_topic': '/nucleus_node/imu_packets',
        }],
        remappings=[
            ('~/imu/manager/imu_in',           '/nortek/imu'),
            ('~/dvl/manager/dvl_in',           '/nucleus_node/bottom_track_packets'),
            ('~/depth/manager/depth_in',       '/nucleus_node/bottom_track_packets'),
            ('~/odometry/manager/odometry_in', '/visual_odom/odometry_in'),
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

    return [mimosa_node, rviz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag_name'),
        DeclareLaunchArgument('s', default_value='0.0'),
        DeclareLaunchArgument('viz', default_value='false'),
        DeclareLaunchArgument(
            'config_override',
            default_value='',
            description='Optional YAML path deep-merged on top of the base bluerov2 params.',
        ),
        DeclareLaunchArgument(
            'logs_directory',
            default_value='',
            description='Optional logs_directory override (post-merge). Useful when running '
                        'multiple variants from the same override file.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
