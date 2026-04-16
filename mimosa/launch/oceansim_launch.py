"""
OceanSim + DVP_Underwater_SK + mimosa integration launch file.

Topic flow
----------
IsaacSim / OceanSim:
  /oceansim/robot/uw_img          CompressedImage  (20 Hz)  single down-looking camera
  /oceansim/robot/imu             sensor_msgs/Imu  (200 Hz)
  dvl/velocity                    TwistStamped     (~10 Hz, published by DVL node inside sim)
  /clock                          Clock

This launch file adds:
  image_republisher               CompressedImage  →  sensor_msgs/Image
                                  /oceansim/robot/uw_img  →  /oceansim/camera/image_raw

  oceansim_dvl_bridge             TwistStamped  →  waterlinked_a50_ros_driver/DVL
                                  dvl/velocity  →  /dvl/data

  DVP_Underwater_SK (vo_node)     Image  →  nav_msgs/Odometry
                                  /oceansim/camera/image_raw  →  /visual_odom_node/odometry

  mimosa_node                     IMU + DVL + Odometry  →  fused navigation state
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
    dv_slam_pkg = get_package_share_directory('dv_slam')

    # ── Arguments ────────────────────────────────────────────────────────────
    viz_arg = DeclareLaunchArgument(
        'viz', default_value='false',
        description='Launch RViz2 with mimosa visualisation')

    # ── 1. image_transport republish: CompressedImage → raw Image ────────────
    # OceanSim publishes CompressedImage directly (not through image_transport),
    # so we remap in/compressed to the actual topic name.
    image_republisher = Node(
        package='image_transport',
        executable='republish',
        name='image_republisher',
        arguments=['compressed', 'raw'],
        remappings=[
            ('in/compressed', '/oceansim/robot/downcamleft'),
            ('out', '/oceansim/camera/image_raw'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ── 2. DVL bridge: OceanSim TwistStamped → waterlinked DVL msg ───────────
    dvl_bridge = Node(
        package='waterlinked_a50_ros_driver',
        executable='oceansim_bridge.py',
        name='oceansim_dvl_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'velocity_topic': 'dvl/velocity',
            'transducers_topic': 'dvl/transducers',
            'dvl_topic': '/dvl/data',
        }],
    )

    # ── 3. DVP_Underwater_SK visual odometry ─────────────────────────────────
    dv_slam_config   = os.path.join(dv_slam_pkg, 'config', 'dv_slam.yaml')
    oceansim_dataset = os.path.join(dv_slam_pkg, 'config', 'datasets', 'oceansim.yaml')

    vo_node = Node(
        package='dv_slam',
        executable='vo_node',
        name='visual_odom_node',
        output='screen',
        parameters=[
            dv_slam_config,
            oceansim_dataset,
            {'use_sim_time': True},
        ],
        # image_topic is set inside oceansim.yaml (/oceansim/camera/image_raw)
    )

    # ── 4. mimosa state estimator ─────────────────────────────────────────────
    mimosa_params = os.path.join(mimosa_pkg, 'config', 'oceansim', 'params.yaml')

    mimosa_node = Node(
        package='mimosa',
        executable='mimosa_node',
        name='mimosa_node',
        output='screen',
        parameters=[{
            'config_path': mimosa_params,
            'use_sim_time': True,
        }],
        remappings=[
            # IMU: OceanSim publishes directly on this topic
            ('~/imu/manager/imu_in',       '/oceansim/robot/imu'),
            # DVL: converted by oceansim_dvl_bridge
            ('~/dvl/manager/dvl_in',       '/dvl/data'),
            # Pressure → depth prior
            ('~/depth/manager/depth_in',   '/barometer/pressure'),
            # Odometry: published by DVP_Underwater_SK vo_node
            ('~/odometry/manager/odometry_in', '/odometry'),
        ],
    )

    # ── 5. RViz (optional) ────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(mimosa_pkg, 'rviz', 'mimosa.rviz')],
        condition=IfCondition(LaunchConfiguration('viz')),
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        # viz_arg,
        # image_republisher,
        dvl_bridge,
        # vo_node,
        mimosa_node,
        # rviz_node,
    ])
