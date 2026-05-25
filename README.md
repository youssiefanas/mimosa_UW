# mimosa

![License: MIT](https://img.shields.io/badge/License-BSD-green.svg)
![ROS Version](https://img.shields.io/badge/ROS-Noetic-blue)
![ROS Version](https://img.shields.io/badge/ROS2-Jazzy-blue)

This package implements a tightly-coupled multi-modal fusion framework. It currently supports fusing LiDAR (Geometric, Photometric), Radar, any Odometry, DVL, pressure/depth, and IMU to provide robust state estimation in challenging environments. The framework is designed to be modular and easily extensible to add new sensors.

## Table of Contents

- [How it works (in one paragraph)](#how-it-works-in-one-paragraph)
- [Sensor factors](#sensor-factors)
  - [IMU](#imu)
  - [LiDAR](#lidar)
  - [Radar](#radar)
  - [External odometry](#external-odometry)
  - [DVL](#dvl)
  - [Depth (pressure)](#depth-pressure)
- [Setup](#setup)
- [Usage — general datasets](#usage--general-datasets)
- [Usage — underwater (BlueROV2)](#usage--underwater-bluerov2)
- [License](#license)
- [Citing](#citing)
- [Questions](#questions)

## How it works

When a new measurement arrives (a LiDAR/radar scan, a VIO pose, a DVL or pressure reading), mimosa declares a new state in the graph, links it to the previous state with a preintegrated IMU factor, hands the measurement to the corresponding sensor manager to build the appropriate factor(s), optimises the sliding-window graph, and publishes the optimised state on `mimosa_node/graph/odometry` (`nav_msgs/Odometry`). All sensors share the same graph — there is no per-sensor filter being fused after the fact.

## Sensor factors

### IMU

The IMU factor is based on GTSAM's provided `PreintegratedIMUFactor` but modified to also have gravity as a state in the preintegration. This is due to the fact that we consider the initial orientation of the IMU to be the map frame (whereas GTSAM assumes the map frame to be gravity aligned).


### LiDAR Factor

There are two types of LiDAR factors implemented:

1. Geometric Factor: This factor uses point-to-plane scan-to-map ICP residuals to constrain the LiDAR pose. The correspondences are found using a k-d tree and the residuals are computed using the point-to-plane distance.
2. Photometric Factor (Implemented only for Ouster LiDARs): This factor uses photometric error of patches in the intensity image to constrain the LiDAR pose.

### Radar Factor

The radar factor provides a single factor per pointcloud that utilizes the radial speed residuals from the radar measurements.

### Odometry Factor

Consecutive odometry measurements (e.g., from a VIO system) are used to create relative pose factors (Between factors) between the corresponding states in the graph.

### DVL

The DVL factor uses bottom-track velocity from a Doppler Velocity Log to constrain the vehicle's linear velocity. 
Per-axis measurement noise is scaled from the DVL's reported figure-of-merit, and outlier readings are gated out.

See [Usage — underwater (BlueROV2)](#usage--underwater-bluerov2) for driver selection (Waterlinked vs Nortek) and the relevant config knobs.

### Depth (pressure)

The depth factor constrains **only the world-frame z** of the vehicle pose from a pressure reading — roll, pitch, yaw, and xy are unaffected. Raw pressure is converted to a signed z via the hydrostatic equation:

```
z = -(pressure - surface_pressure - pressure_offset_pa) / (fluid_density * g)
```

See [Usage — underwater (BlueROV2)](#usage--underwater-bluerov2) for pressure-source selection (dedicated `FluidPressure` topic vs Nortek Nucleus packet) and the calibration recipe for `pressure_offset_pa`.

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
## Usage — underwater (BlueROV2)

mimosa runs unmodified underwater by adding the **DVL** and **depth** factors described above. The BlueROV2 configs in `config/bluerov2/` are the canonical examples and come in two flavours depending on the IMU you use.

### Sensor stack

| Role           | Hardware                                                    | ROS message                                        |
| -------------- | ----------------------------------------------------------- | -------------------------------------------------- |
| IMU + pressure | Pixhawk autopilot (BlueROV IMU) *or* Nortek Nucleus internal IMU | `sensor_msgs/Imu`, `sensor_msgs/FluidPressure`     |
| DVL bottom-track | **Waterlinked A50** *or* **Nortek Nucleus 1000**          | `waterlinked_a50_ros_driver/DVL` or `interfaces/BottomTrack` |
| Depth (pressure) | Bar30 on Pixhawk *or* pressure field of the Nucleus packet | `sensor_msgs/FluidPressure` or `interfaces/BottomTrack` |

mimosa selects between the two DVL message types via `dvl.manager.dvl_type` (`0 = Waterlinked`, `1 = Nortek`), and between the two depth sources via `depth.depth_source` (`0 = FluidPressure topic`, `1 = Nortek BottomTrack`).

### Dependencies

In addition to the [Common setup](#common-setup) above, clone the DVL driver(s) you actually use into your workspace `src/`:

#### Waterlinked A50

mimosa is compiled against a ROS 2 port of the Waterlinked A50 driver that publishes a custom `waterlinked_a50_ros_driver/DVL` message. Use this fork:

```bash
cd your_ws/src
git clone https://github.com/youssiefanas/dvl_a50_ros_driver_ros2.git
```

> The upstream Waterlinked driver at https://github.com/waterlinked/waterlinked_dvl publishes `marine_acoustic_msgs/Dvl` and is **not** a drop-in replacement — mimosa's [ros_interface.hpp](mimosa/include/mimosa/ros_interface.hpp) is hard-coded to the custom message type.

#### Nortek Nucleus (DVL + IMU + pressure)

The Nortek Nucleus ships its own ROS 2 driver, which provides the `interfaces` package mimosa builds against, plus the `nucleus_node` that publishes the bottom-track and IMU streams.

```bash
cd your_ws/src
git clone https://github.com/NortekSupport/nucleus_driver.git
# (the ROS2 packages live under nucleus_driver/ros2/)
```

> **Recommended tweak.** The upstream `nucleus_node.py` advertises all 9 packet topics with the default `RELIABLE` QoS (depth 100). At the Nucleus' native ~100 Hz this fills the queue under any subscriber stall and back-pressures the publisher. For mimosa we override every publisher to `BEST_EFFORT` (depth 2, `KEEP_LAST`) so the latest sample always wins:

mimosa also ships a tiny `nortek_imu_bridge` executable (built automatically on ROS 2) that re-publishes the Nucleus' `interfaces/msg/IMU` as a standard `sensor_msgs/msg/Imu` so the rest of the pipeline is sensor-agnostic.

> If you only use the Waterlinked DVL with a Pixhawk IMU you do **not** need the Nucleus driver, but the `interfaces` dependency is currently mandatory because mimosa is compiled against its message definitions. Cloning `nucleus_driver` alongside is the simplest fix.

### Launch files

Two ready-to-run ROS 2 launch files live in `mimosa/launch/`, plus matching rosbag-replay variants:

| Launch file                              | IMU source              | DVL    | Use for                          |
| ---------------------------------------- | ----------------------- | ------ | -------------------------------- |
| `bluerov2_launch_bluerov_imu.py`         | Pixhawk (BlueROV IMU)   | Either | Live BlueROV2 with autopilot IMU |
| `bluerov2_launch_nortek_imu.py`          | Nortek Nucleus IMU      | Nortek | Live BlueROV2 with Nucleus       |
| `bluerov_imu_rosbag_launch.py`           | Pixhawk (from bag)      | Either | Replay BlueROV-IMU recordings    |
| `bluerov2_rosbag_launch.py`              | from bag                | from bag | Generic BlueROV2 bag replay     |

Matching YAML configs:

- `config/bluerov2/bluerov_imu_params.yaml` — Pixhawk IMU + Bar30 depth.
- `config/bluerov2/nortek_imu_params.yaml`  — Nucleus IMU + Nucleus pressure (`depth_source: 1`).
- `config/bluerov2/params.yaml`             — generic / starting-point config.

### Quick start (Nortek Nucleus stack)

```bash
# Live
ros2 launch mimosa bluerov2_launch_nortek_imu.py

# Replay a recorded bag
ros2 launch mimosa bluerov_imu_rosbag_launch.py bag_name:=/path/to/bag
```

### Tuning checklist for a new BlueROV2

1. **Extrinsics.** Set `T_B_S` for the DVL, the IMU and the pressure sensor in the active YAML — these are pose-of-sensor-in-body and must reflect your actual mount.
2. **Water column.** `depth.manager.fluid_density: 1025.0` for seawater, `1000.0` for freshwater.
3. **Pressure offset.** With the vehicle dry on deck, record the raw pressure reading. Subtract `surface_pressure` from it and paste the result into `depth.manager.pressure_offset_pa`. Leave at `0.0` if your sensor is already calibrated.
4. **DVL gating.** `dvl.manager.max_fom` rejects noisy bottom-track readings (e.g. when leaving the bottom). `max_velocity` rejects spurious large speeds.


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
