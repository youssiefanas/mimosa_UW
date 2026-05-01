# mimosa

![License: MIT](https://img.shields.io/badge/License-BSD-green.svg)
![ROS Version](https://img.shields.io/badge/ROS-Noetic-blue)
![ROS Version](https://img.shields.io/badge/ROS2-Jazzy-blue)

This package implements a tightly-coupled multi-modal fusion framework. It currently supports fusing LiDAR (Geometric, Photometric), Radar, any Odometry, DVL, pressure/depth, and IMU to provide robust state estimation in challenging environments. The framework is designed to be modular and easily extensible to add new sensors.

## Working Description

mimosa maintains a sliding window factor graph to fuse factors generated from multiple sensors. On arrival of a new measurement (a pointcloud from the LiDAR or Radar or an odometry message from an external odometry source like VIO), a new state is "declared" in the graph and connected with a preintegrated IMU factor. Then the measurement gets processed by the corresponding sensor manager to generate a new factor(s). This factor(s) is(are) then added to the graph and the graph is optimized. The optimized state is then published on `mimosa_node/graph/odometry` topic as a `nav_msgs/Odometry` message.

### IMU Factor

The IMU factor is based on GTSAM's provided `PreintegratedIMUFactor` but modified to also have gravity as a state in the preintegration. This is due to the fact that we consider the initial orientation of the IMU to be the map frame (whereas GTSAM assumes the map frame to be gravity aligned).

### LiDAR Factor

There are two types of LiDAR factors implemented:

1. Geometric Factor: This factor uses point-to-plane scan-to-map ICP residuals to constrain the LiDAR pose. The correspondences are found using a k-d tree and the residuals are computed using the point-to-plane distance.
2. Photometric Factor (Implemented only for Ouster LiDARs): This factor uses photometric error of patches in the intensity image to constrain the LiDAR pose.

### Radar Factor

The radar factor provides a single factor per pointcloud that utilizes the radial speed residuals from the radar measurements.

### Odometry Factor

Consecutive odometry measurements (e.g., from a VIO system) are used to create relative pose factors (Between factors) between the corresponding states in the graph.

### DVL Factor

The DVL factor constrains the body-frame linear velocity using a bottom-track measurement from a Doppler Velocity Log. It is a 3-key factor tying `X(k)` (pose), `V(k)` (world-frame velocity) and `B(k)` (IMU bias, for gyroscope compensation of the lever arm), and models the sensor-frame measurement with full lever-arm compensation:

```
v_S = R_S_B * ( R_B_W * v_W + (omega_B - b_g) x t_B_S )
```

where `t_B_S` is the lever arm from the body origin to the DVL transducer expressed in the body frame, `omega_B` is the body-frame angular velocity sampled from the IMU buffer around the DVL timestamp, and `b_g` is the estimated gyroscope bias. The factor analytically provides the Jacobians w.r.t. all three keys. Per-axis noise sigmas are derived from the DVL's reported figure-of-merit (`fom_scale * fom`), and measurements with FOM above `max_fom` or speed above `max_velocity` are rejected.

Two DVL drivers are supported via `dvl.manager.dvl_type` in the config: `0 = Waterlinked A50` (shared-FOM) and `1 = Nortek BottomTrack` (per-axis FOM, uses the message's `system_timestamp` as the stamp since it has no `std_msgs/Header`).

If `dvl.manager.use_to_init` is `true`, the manager also provides a body-frame velocity hint when declaring the first measurement. The graph manager rotates this hint to world via the IMU-estimated attitude and warm-starts `V(0)` so the solver does not have to pull the state away from a zero initial guess against a tight velocity prior. Widening `graph.manager.smoother.initial_velocity_sigma` lets the DVL measurement dominate the initial velocity estimate.

### Depth Factor

The depth factor is a unary prior on `X(k)` that constrains the world-frame z-component of the sensor position using a `sensor_msgs/FluidPressure` reading. It follows the measurement model:

```
residual = (T_W_B * t_B_S).z() - measured_z
```

where `t_B_S` is the lever arm of the pressure sensor in the body frame (taken from `depth.T_B_S`), and the Jacobian w.r.t. the pose tangent space is the z-row of the 3x6 Jacobian returned by `Pose3::transformFrom`. Only the z-component of `T_W_B` is observed — roll, pitch, yaw and the xy-position are unaffected by this factor.

The manager converts raw pressure to a signed z via the hydrostatic equation:

```
depth_below_surface = (fluid_pressure - surface_pressure - pressure_offset_pa) / (fluid_density * g)
measured_z          = -depth_below_surface        # world z is up
```

`pressure_offset_pa` is a sensor-specific bias measured once on deck: record the raw pressure reading in air, subtract `surface_pressure`, paste the result into the YAML. Leave it at `0.0` if your sensor is already calibrated against `surface_pressure` (the typical case for absolute sensors with a known datasheet offset, or gauge sensors with `surface_pressure: 0.0`). Because the offset is static, the depth factor reports the *true* signed z from the first message, even when the vehicle starts already submerged — `getInitZHint()` propagates this to the graph manager so `X(0).z()` initializes to the actual depth.

The relevant config fields are `sigma_depth_m`, `fluid_density` (use `1025.0` for seawater, `1000.0` for freshwater), `surface_pressure` (Pa), `gravity_magnitude`, and `pressure_offset_pa`. `depth.manager.use_to_init` should be left `false`: depth alone cannot initialize a 6-DOF pose, so it should only refine an init produced by another sensor.

## Setup

This package supports both **ROS1 (Noetic)** and **ROS2 (Jazzy)**. The same codebase builds on both via a `ros_interface.hpp` abstraction layer.

### Common Setup

```bash
  # System wide dependencies
  sudo apt install python3-catkin-tools \
  libgoogle-glog-dev \
  libspdlog-dev

  # Create a workspace and clone the code
  mkdir your_ws/src && cd your_ws/src

  # Local dependencies
  git clone git@github.com:ntnu-arl/config_utilities.git -b dev/mimosa
  git clone git@github.com:ntnu-arl/gtsam.git -b feature/imu_factor_with_gravity
  git clone git@github.com:ntnu-arl/gtsam_points.git -b minimal_updated

  # get this code
  git clone git@github.com:ntnu-arl/mimosa.git
```

### ROS Version Specific Setup

#### ROS1 (Noetic)

These instructions assume that `ros-noetic-desktop-full` is installed on your Ubuntu 20.04 system.

  ```bash
  cd your_ws

  catkin config -DCMAKE_BUILD_TYPE=Release -DGTSAM_POSE3_EXPMAP=ON -DGTSAM_ROT3_EXPMAP=ON -DGTSAM_USE_QUATERNIONS=ON -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_WITH_TBB=OFF
  catkin build mimosa
  ```

#### ROS2 (Jazzy)

These instructions assume that `ros-jazzy-desktop` is installed on your Ubuntu 24.04 system.

  ```bash
  cd your_ws

  colcon build --packages-up-to mimosa --cmake-args -DCMAKE_BUILD_TYPE=Release -DGTSAM_POSE3_EXPMAP=ON -DGTSAM_ROT3_EXPMAP=ON -DGTSAM_USE_QUATERNIONS=ON -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_WITH_TBB=OFF
  ```

## Usage

This package can be run either online (`mimosa_node`) or offline using a rosbag (`mimosa_rosbag`). The online version is for deployment on a robot, while the offline version is for testing and debugging. Both versions are identical in terms of functionality since they use the same callbacks. The offline version just allows you to read a rosbag and process the callbacks directly instead of receiving them over the network.

ROS1 launch files use the `.launch` XML format. ROS2 launch files use the `_launch.py` Python format.

### Examples

#### LiDAR-Radar-IMU Fusion

##### [Unified Autonomy Stack Datasets](https://huggingface.co/datasets/ntnu-arl/unified_autonomy_stack_datasets)

Download any rosbag from [https://huggingface.co/datasets/ntnu-arl/unified_autonomy_stack_datasets](https://huggingface.co/datasets/ntnu-arl/unified_autonomy_stack_datasets). There are two variants of robots in this dataset: one which has an Ouster LiDAR and the other which has a Unipilot module (with a Robosense Airy LiDAR). As per the rosbag that you downloaded, you will need to build the [ouster_ros](https://github.com/ouster-lidar/ouster-ros) or [rslidar_sdk](https://github.com/ntnu-arl/rslidar_sdk/tree/develop) package respectively to be able to convert the raw LiDAR packets to pointclouds. Please follow the instructions in the respective repositories to build the drivers. Note that in the case of rslidar_sdk you will need to modify the config file to set the `lidar_type` as `RSAIRY`, the `common/msg_source` as `2` (for packet message comes from ros or ros2), `send_packet_ros` as `false` and `send_point_cloud_ros` as `true`. You can find a working config file [here](https://github.com/ntnu-arl/rslidar_sdk/blob/d02b59cc43d027e4bdaa1ee0762d36e77129ecc7/config/config.yaml).

The bags should be replayed with `--clock` option and the global `/use_sim_time` parameter set to true. To do this, a simple way is to create a launch file called `simcore.launch` with the following content:

```xml
<launch>
  <param name="/use_sim_time" value="true"/>
</launch>
```

Then you can launch the system as:

**ROS1:**
```bash
# Terminal 1
roslaunch simcore.launch

# Terminal 2
roslaunch ouster_ros replay.launch
# or
roslaunch rslidar_sdk start.launch

# Terminal 3
roslaunch mimosa hornbill.launch
# or
roslaunch mimosa magpie.launch

# Terminal 4
rosbag play --clock /path/to/your/rosbag.bag
```

**ROS2:**
```bash
# Terminal 1 - Launch mimosa (example with parrot config)
ros2 launch mimosa parrot_launch.py

# Terminal 2 - Replay the bag
ros2 bag play /path/to/your/ros2bag --clock
```

#### LiDAR(Photometric-Geometric)-IMU Fusion

##### [ENWIDE Dataset](https://projects.asl.ethz.ch/datasets/enwide)

Download the dataset from [https://projects.asl.ethz.ch/datasets/enwide](https://projects.asl.ethz.ch/datasets/enwide).
After downloading any of the sequences in the dataset, you can run the following command to launch the example:

**ROS1:**
```bash
roslaunch mimosa enwide_rosbag.launch bag_name:="/path/to/your/rosbag.bag" viz:="true"
```

##### [Newer College Multi-Camera Dataset](https://ori-drs.github.io/newer-college-dataset/multi-cam/)

Download the dataset from [https://ori-drs.github.io/newer-college-dataset/download/](https://ori-drs.github.io/newer-college-dataset/download/).
After downloading any of the sequences in the dataset, you can run the following command to launch the example:

**ROS1:**
```bash
roslaunch mimosa newer_college_rosbag.launch bag_name:="/path/to/your/rosbag.bag" viz:="true"
```

#### Offline Rosbag Processing (ROS2)

The `mimosa_rosbag` node reads a ROS2 bag directly and processes messages without needing `ros2 bag play`. This is the ROS2 equivalent of the `*_rosbag.launch` files:

```bash
ros2 launch mimosa parrot_rosbag_launch.py bag_name:="/path/to/your/ros2bag" viz:="true"
```

##### Complete Dataset Example

There is also a script [dataset_evaluation.py](src/mimosa/scripts/dataset_evaluation.py) that can be used to run the complete dataset evaluation. This script will run the `mimosa_rosbag` node on all the sequences in the dataset and save the results in a folder. Before running this script make sure to set the correct `dataset_path` and `results_directory`. It is assumed that the dataset is in the following format:

```bash
dataset_path/
├── sequence_1
│   ├── xxxxxsequence_1.bag
|   ├── gt-sequence_1.csv
├── sequence_2
│   ├── xxxxxsequence_2.bag
|   ├── gt-sequence_2.csv
├── sequence_3
│   ├── xxxxxsequence_3.bag
|   ├── gt-sequence_3.csv
├── sequence_4
│   ├── xxxxxsequence_4.bag
|   ├── gt-sequence_4.csv
```

```bash
python dataset_evaluation.py
```

### On your own data

To run on your own data, you need to set up the following:

1. Take the most relevant launch file (`.launch` for ROS1, `_launch.py` for ROS2)
2. Modify the remapping of the input topics to your sensor topics in the launch file
3. Modify parameters in the corresponding config file
   1. e.g. for velodyne the `point_skip_factor` must be 1
   2. Set your transforms for `T_B_S` for each of the sensors you are using. This is the transform from the Sensor frame to the Body frame (i.e. pose of Sensor in the Body frame).
4. Launch your launch file

For ROS2, only the `parrot` configuration currently has launch files (`parrot_launch.py`, `parrot_rosbag_launch.py`). These can be used as templates to create launch files for other configurations.

## License

This project is licensed under the BSD-3-Clause License - see the [LICENSE](LICENSE) file for details.

## Citing

If you use this work in your research, please cite the relevant publications:

```bibtex
@misc{perception_pglio,
    title = {{PG}-{LIO}: {Photometric}-{Geometric} fusion for {Robust} {LiDAR}-{Inertial} {Odometry}},
    shorttitle = {{PG}-{LIO}},
    url = {http://arxiv.org/abs/2506.18583},
    doi = {10.48550/arXiv.2506.18583},
    publisher = {arXiv},
    author = {Khedekar, Nikhil and Alexis, Kostas},
    month = jun,
    year = {2025},
    note = {arXiv:2506.18583 [cs]},
    keywords = {Computer Science - Robotics},
}

@inproceedings{perception_dlrio,
    title = {Degradation {Resilient} {LiDAR}-{Radar}-{Inertial} {Odometry}},
    url = {https://ieeexplore.ieee.org/document/10611444},
    doi = {10.1109/ICRA57147.2024.10611444},
    booktitle = {2024 {IEEE} {International} {Conference} on {Robotics} and {Automation} ({ICRA})},
    author = {Nissov, Morten and Khedekar, Nikhil and Alexis, Kostas},
    month = may,
    year = {2024},
    keywords = {Degradation, Estimation, Laser radar, Odometry, Prevention and mitigation, Robot sensing systems, Sensors},
    pages = {8587--8594},
}

@ARTICLE{perception_jplRadar,
    author={Nissov, Morten and Edlund, Jeffrey A. and Spieler, Patrick and Padgett, Curtis and Alexis, Kostas and Khattak, Shehryar},
    journal={IEEE Robotics and Automation Letters},
    title={Robust High-Speed State Estimation for Off-Road Navigation Using Radar Velocity Factors},
    year={2024},
    volume={9},
    number={12},
    pages={11146-11153},
    keywords={Radar;Sensors;Velocity measurement;Radar measurements;Laser radar;Odometry;State estimation;Robustness;Robot sensing systems;Field robots;localization;sensor fusion},
    doi={10.1109/lra.2024.3486189}
}
```

## Questions

You can open an issue or contact us for any questions:

- [Nikhil Khedekar](mailto:nikhil.v.khedekar@ntnu.no)
- [Morten Nissov](mailto:morten.nissov@ntnu.no)
- [Kostas Alexis](mailto:konstantinos.alexis@ntnu.no)
