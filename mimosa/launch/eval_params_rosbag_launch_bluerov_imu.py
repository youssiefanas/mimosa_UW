import os
import tempfile

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _deep_merge(base, override):
    if not isinstance(base, dict) or not isinstance(override, dict):
        return override
    out = dict(base)
    for key, val in override.items():
        out[key] = _deep_merge(out.get(key), val) if key in out else val
    return out


def _resolve_config_path(context, base_path):
    override_path = LaunchConfiguration('config_override').perform(context)
    if not override_path:
        return base_path

    with open(base_path) as f:
        base = yaml.safe_load(f) or {}
    with open(override_path) as f:
        override = yaml.safe_load(f) or {}
    merged = _deep_merge(base, override)

    base_dir = os.path.dirname(os.path.abspath(base_path))
    lidar = merged.get('lidar')
    if isinstance(lidar, dict) and 'sensor_json' in lidar:
        sensor_json = lidar['sensor_json']
        if isinstance(sensor_json, str) and not os.path.isabs(sensor_json):
            lidar['sensor_json'] = os.path.normpath(os.path.join(base_dir, sensor_json))

    fd, merged_path = tempfile.mkstemp(prefix='mimosa_hornbill_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(merged, f, sort_keys=False)
    return merged_path


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('mimosa')
    base_config = os.path.join(pkg_share, 'config', 'bluerov2', 'bluerov_imu_params.yaml')
    config_path = _resolve_config_path(context, base_config)
    bag_output = LaunchConfiguration('bag_output').perform(context)

    mimosa_node = Node(
        package='mimosa',
        executable='mimosa_rosbag',
        name='mimosa_node',
        output='screen',
        parameters=[{
            'config_path': config_path,
            'bag_name': LaunchConfiguration('bag_name'),
            's': LaunchConfiguration('s'),
        }],
        remappings=[
            ('~/imu/manager/imu_in',           '/bluerov2/imu/data'),
            ('~/dvl/manager/dvl_in',           '/nucleus_node/bottom_track_packets'),
            ('~/depth/manager/depth_in',       '/nucleus_node/bottom_track_packets'),
            ('~/odometry/manager/odometry_in', '/visual_odom_node/odometry'),
        ],
        on_exit=Shutdown(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'mimosa.rviz')],
        condition=IfCondition(LaunchConfiguration('viz')),
    )

    recorder_cmd = [
        'ros2', 'bag', 'record',
        '-o', bag_output,
        '/mimosa_node/graph/debug',
        '/mimosa_node/graph/odometry',
        '/mimosa_node/imu/manager/odometry',
    ]
    recorder = ExecuteProcess(cmd=recorder_cmd, output='screen')

    return [mimosa_node, rviz_node, recorder]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag_name'),
        DeclareLaunchArgument('s', default_value='0.0'),
        DeclareLaunchArgument('viz', default_value='false'),
        DeclareLaunchArgument(
            'config_override',
            default_value='',
            description='Optional path to a YAML deep-merged on top of the base hornbill params.',
        ),
        DeclareLaunchArgument(
            'bag_output',
            default_value='mimosa_results',
            description='Output directory for ros2 bag record.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
