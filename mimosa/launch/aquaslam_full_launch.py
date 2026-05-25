
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
        'viz', default_value='true',
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
            ('out', '/odometry'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )


    # ── 3. DVP_Underwater_SK visual odometry ─────────────────────────────────
    dv_slam_config   = os.path.join(dv_slam_pkg, 'config', 'dv_slam.yaml')
    oceansim_dataset = os.path.join(dv_slam_pkg, 'config', 'datasets', 'aquaslam.yaml')

    vo_node = Node(
        package='dv_slam',
        executable='vo_node',
        name='visual_odom_node',
        # output='screen',
        parameters=[
            dv_slam_config,
            oceansim_dataset,
            {
                'use_sim_time': True,
                # Publish under the same fixed frame mimosa uses, so both
                # trajectories appear in the same RViz tree.
                'frame_id': 'mimosa_map',
                'child_frame_id': 'vo_camera',
                'publish_tf': True,
            },
        ],
    )

    # ── 4b. Static TFs for mimosa sensor frames not auto-broadcast ───────────
    # mimosa::dvl::Manager broadcasts mimosa_body → mimosa_dvl on its first
    # message, but the IMU manager (no sensor_frame concept — IMU is body) and
    # the odometry manager (currently disabled) never broadcast theirs. Publish
    # them here so RViz shows the full sensor triad.
    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_mimosa_body_to_imu',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'mimosa_body',
            '--child-frame-id', 'mimosa_imu',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # T_B_S for odometry from params_qr.yaml (oceansim).
    # [x, y, z, qx, qy, qz, qw] = [0, 0, 0, 0.7071068, -0.7071068, 0, 1]
    # Note: the qw=1 in that param is non-normalised; gtsam normalises on load.
    # Here we use the matching normalised quaternion.
    static_tf_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_mimosa_body_to_odometry',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0.5', '--qy', '-0.5', '--qz', '0', '--qw', '0.7071068',
            '--frame-id', 'mimosa_body',
            '--child-frame-id', 'mimosa_odometry',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # ── 4. mimosa state estimator ─────────────────────────────────────────────
    mimosa_params = os.path.join(mimosa_pkg, 'config', 'parrot', 'params.yaml')

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
            ('~/imu/manager/imu_in', '/imu/data'),
            ('~/odometry/manager/odometry_in', '/visual_odom/odometry_in'),
            ('~/dvl/manager/dvl_in', '/dvl/data'),
            # ('~/depth/manager/depth_in',   '/barometer/pressure'),

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
        viz_arg,
        # image_republisher,
        vo_node,
        static_tf_imu,
        static_tf_odom,
        mimosa_node,
        rviz_node,
    ])
