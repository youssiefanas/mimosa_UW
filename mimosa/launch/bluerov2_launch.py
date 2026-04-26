"""
BlueROV2 (real hardware) + Nortek Nucleus DVL + mimosa launch file.

Hardware topic flow
-------------------
Nortek Nucleus (nucleus_driver_ros2 / nucleus_node):
  /nucleus_node/bottom_track_packets    interfaces/BottomTrack      (DVL bottom track)

BlueROV2 autopilot bridge:
  /bluerov2/imu/data                    sensor_msgs/Imu             (filtered, with orientation)
  /bluerov2/imu/data_raw                sensor_msgs/Imu             (raw accel + gyro)
  /bluerov2/imu/mag                     sensor_msgs/MagneticField
  /bluerov2/imu/static_pressure         sensor_msgs/FluidPressure   (IMU-internal baro)
  /bluerov2/imu/diff_pressure           sensor_msgs/FluidPressure
  /bluerov2/imu/temperature_imu         sensor_msgs/Temperature
  /bluerov2/imu/temperature_baro        sensor_msgs/Temperature
  /bluerov2/pressure                    sensor_msgs/FluidPressure   (external depth sensor, Pa)
  /bluerov2/depth                       std_msgs/Float64            (derived depth, m)
  /bluerov2/camera/image/compressed     sensor_msgs/CompressedImage

Mimosa is wired to:
  IMU      ← /bluerov2/imu/data_raw   (raw — mimosa runs its own attitude estimation)
  DVL      ← /nucleus_node/bottom_track_packets  (Nortek BottomTrack, dvl_type=1)
  Depth    ← /bluerov2/pressure       (FluidPressure, converted to depth inside mimosa)
  Odometry ← /visual_odom_node/odometry (only if `vo` arg is true)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mimosa_pkg = get_package_share_directory('mimosa')

    # ── Arguments ────────────────────────────────────────────────────────────
    viz_arg = DeclareLaunchArgument(
        'viz', default_value='false',
        description='Launch RViz2 with mimosa visualisation')

    vo_arg = DeclareLaunchArgument(
        'vo', default_value='false',
        description='Enable DVP_Underwater_SK visual odometry from the down camera')

    # ── 1. (optional) image_transport republish: CompressedImage → raw Image ─
    image_republisher = Node(
        package='image_transport',
        executable='republish',
        name='image_republisher',
        arguments=['compressed', 'raw'],
        remappings=[
            ('in/compressed', '/bluerov2/camera/image/compressed'),
            ('out', '/bluerov2/camera/image_raw'),
        ],
        condition=IfCondition(LaunchConfiguration('vo')),
        output='screen',
    )

    # ── 2. (optional) DVP_Underwater_SK visual odometry ──────────────────────
    # Configure dv_slam for the BlueROV2 camera (topic / intrinsics) in its
    # own dataset yaml, similar to dv_slam/config/datasets/oceansim.yaml.
    vo_node = Node(
        package='dv_slam',
        executable='vo_node',
        name='visual_odom_node',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('dv_slam'), 'config', 'dv_slam.yaml'),
            os.path.join(get_package_share_directory('dv_slam'), 'config', 'datasets', 'bluerov2.yaml'),
        ],
        condition=IfCondition(LaunchConfiguration('vo')),
    )

    # ── 3. mimosa state estimator ────────────────────────────────────────────
    mimosa_params = os.path.join(mimosa_pkg, 'config', 'bluerov2', 'params.yaml')
    qos_overrides = os.path.join(mimosa_pkg, 'config', 'bluerov2', 'qos_overrides.yaml')

    mimosa_node = Node(
        package='mimosa',
        executable='mimosa_node',
        name='mimosa_node',
        output='screen',
        parameters=[
            qos_overrides,
            {
                'config_path': mimosa_params,
                'use_sim_time': False,
            },
        ],
        remappings=[
            ('~/imu/manager/imu_in',           '/bluerov2/imu/data_raw'),
            ('~/dvl/manager/dvl_in',           '/nucleus_node/bottom_track_packets'),
            ('~/depth/manager/depth_in',       '/bluerov2/pressure'),
            ('~/odometry/manager/odometry_in', '/visual_odom_node/odometry'),
        ],
    )

    # ── 4. RViz (optional) ───────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(mimosa_pkg, 'rviz', 'mimosa.rviz')],
        condition=IfCondition(LaunchConfiguration('viz')),
    )

    return LaunchDescription([
        viz_arg,
        # vo_arg,
        # image_republisher,
        # vo_node,
        mimosa_node,
        # rviz_node,
    ])
