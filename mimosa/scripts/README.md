# MIMOSA Evaluation Scripts

Scripts for running the MIMOSA pipeline on recorded ROS 2 bags, extracting trajectories and camera frames, and evaluating estimator accuracy against ground truth.

The expected workflow is:

1. **Process** a batch of bags through MIMOSA to produce odometry trajectories — [`batch_process.sh`](#batch_processsh).
2. **Extract** ground-truth and estimator trajectories (and optionally camera frames) from each bag into TUM-format files — [`eval/batch_extract.sh`](#evalbatch_extractsh), [`eval/batch_extract_frames.sh`](#evalbatch_extract_framessh).
3. **Evaluate** the resulting TUM trajectories against ground truth — [`eval/evaluate_bag.py`](#evalevaluate_bagpy) (primary), [`evaluate.sh`](#evaluatesh), [`eval/evaluate_tum_trajectories.py`](#evalevaluate_tum_trajectoriespy).

## Table of Contents

- [1. Running MIMOSA on bags](#1-running-mimosa-on-bags)
  - [`batch_process.sh`](#batch_processsh)
- [2. Extracting data from bags](#2-extracting-data-from-bags)
  - [`eval/batch_extract.sh`](#evalbatch_extractsh)
  - [`eval/tag_gt_to_tum.py`](#evaltag_gt_to_tumpy)
  - [`eval/batch_extract_frames.sh`](#evalbatch_extract_framessh)
  - [`eval/extract_frames.py`](#evalextract_framespy)
- [3. Trajectory evaluation](#3-trajectory-evaluation)
  - [`eval/evaluate_bag.py`](#evalevaluate_bagpy)
  - [`evaluate.sh`](#evaluatesh)
  - [`eval/evaluate_tum_trajectories.py`](#evalevaluate_tum_trajectoriespy)
- [4. Parameter sweeps](#4-parameter-sweeps)
  - [`param_change_evaluation/run_evaluation.sh`](#param_change_evaluationrun_evaluationsh)

---

## 1. Running MIMOSA on bags

### `batch_process.sh`

Runs the MIMOSA `bluerov2_rosbag_launch.py` launch file (or a launch file you specify) on every ROS 2 bag found under a base directory, then copies MIMOSA's output trajectory (`graph_manager_odometry.tum`) next to each bag for downstream evaluation.

For each immediate subdirectory of `<base_dir>`, the script:
- Looks for a ROS 2 bag (a folder containing `metadata.yaml`), optionally nested under `<bag_subpath>`.
- Deletes any stale `graph_manager_odometry.tum` from the configured `logs_directory` (read out of `nortek_imu_params.yaml`) so a failed launch can't be misattributed to the new bag.
- Runs `ros2 launch mimosa <launch_file> bag_name:=<bag_path>`.
- Copies the produced trajectory to either `<output_dir>/<subfolder>/mimosa.tum` or, if `output_dir` is unset, `<bag_dir>/../estimate.tum`.

**Arguments:**
- `<base_dir>` — Directory whose subfolders each contain a ROS 2 bag.
- `[bag_subpath]` *(optional)* — Path inside each subfolder where the bag dir actually lives. Default `""` (the subfolder itself is the bag).
- `[launch_file]` *(optional)* — Launch file to invoke per bag. Default `bluerov2_rosbag_launch.py`.
- `[output_dir]` *(optional)* — Directory to collect trajectories into.

**Example:**
```bash
./src/mimosa/mimosa/scripts/batch_process.sh \
  /mnt/F68C0A428C09FE3D/Academics/MIR/NTNU/Thesis/bluerov_rosbags/rosbags \
  "" \
  bluerov2_rosbag_launch.py \
  /mnt/F68C0A428C09FE3D/Academics/MIR/NTNU/Thesis/bluerov_rosbags/rosbags/rosbags_traj_eval
```

---

## 2. Extracting data from bags

### `eval/batch_extract.sh`

Loops over every `run_*` bag directory under a parent folder and extracts three TUM files into each bag:

| File         | Source                                  | Extractor                                 |
| ------------ | --------------------------------------- | ----------------------------------------- |
| `gt.tum`     | ChArUco board detections in `/cam1/...` | [`tag_gt_to_tum.py`](#evaltag_gt_to_tumpy) |
| `mavros.tum` | `/bluerov2/local_position/odom`         | `mavros_to_tum.py`                        |
| `mimosa.tum` | `/mimosa_node/graph/odometry`           | `odom_to_tum.py`                          |

Existing non-empty TUM files are skipped, so re-running only fills in what's missing.

**Arguments:**
- `<parent_dir>` — Parent directory containing the run folders.
- `[glob]` *(optional)* — Pattern to match run folders. Default `run_*`.

**Example:**
```bash
./src/mimosa/mimosa/scripts/eval/batch_extract.sh \
  /mnt/56F01D0DF01CF4C9/Thesis_MSc_data/19_05/good_data 'run_*'
```

### `eval/tag_gt_to_tum.py`

Produces the ground-truth `gt.tum` used by every evaluation script in this directory. Reads `sensor_msgs/CompressedImage` from a bag, detects a ChArUco board (or a single AprilTag) in each frame, and writes the body pose in the tag world frame as a TUM trajectory:

```
T_world_body = inv(T_cam_world) @ inv(T_body_cam)
```

`T_body_cam` is hard-coded at the top of the script (`APRILTAG_T_BODY_CAM` and `CHARUCO_T_BODY_CAM`) — edit those constants when the camera mount changes. ChArUco board geometry (`CHARUCO_SQUARES_X/Y`, `CHARUCO_SQUARE_LENGTH`, `CHARUCO_MARKER_LENGTH`, `CHARUCO_DICT_NAME`) is hard-coded the same way. Camera intrinsics default to the `dv_slam` dataset YAML so this script and the VO pipeline share one source of truth, and can be overridden with `--camera-yaml`.

**Arguments:**
- `bag` — Input ROS 2 bag directory.
- `out` — Output TUM file.
- `--tag-type {apriltag,charuco}` — Default `apriltag`.
- `--camera-yaml` — Camera intrinsics YAML. Default `dv_slam/config/datasets/bluerov.yaml`.
- `--every-n` — Process every Nth frame (default `1`).

**Example:**
```bash
python3 src/mimosa/mimosa/scripts/eval/tag_gt_to_tum.py \
  /mnt/.../run_20260519_141318 \
  /mnt/.../run_20260519_141318/gt.tum \
  --tag-type charuco
```

> Normally invoked indirectly via `batch_extract.sh`; run it directly only for one-off re-extractions or when tuning detector settings.

### `eval/batch_extract_frames.sh`

Loops over every `run_*` bag and dumps representative camera frames into `<bag>/frames/` by calling [`extract_frames.py`](#evalextract_framespy) on each. Skips bags whose `frames/` directory already has content unless `--force` is given.

**Arguments:**
- `<parent_dir>` — Parent directory containing the run folders.
- `[glob]` *(optional)* — Pattern to match run folders. Default `run_*`.
- `[--force]` *(optional)* — Overwrite existing frames.

**Example:**
```bash
./src/mimosa/mimosa/scripts/eval/batch_extract_frames.sh \
  /mnt/56F01D0DF01CF4C9/Thesis_MSc_data/19_05/good_data 'run_*' --force
```

### `eval/extract_frames.py`

Extracts three representative frames per camera topic from a single bag — at **15 s after start**, **40 % of the bag duration**, and **90 % of the bag duration**. For each (topic, timestamp) target it keeps the closest message and writes it as `.jpg`/`.png` based on the `CompressedImage.format`. Also writes a `timestamps.txt` index alongside the images.

**Arguments:**
- `bag` — Input ROS 2 bag directory.
- `outdir` — Output directory for the images.
- `--topics` — Camera topics to extract. Default `/cam1/image_raw/compressed /cam2/image_raw/compressed`.

**Example:**
```bash
python3 src/mimosa/mimosa/scripts/eval/extract_frames.py \
  /mnt/.../run_20260519_141318 \
  /mnt/.../run_20260519_141318/frames
```

> Normally invoked indirectly via `batch_extract_frames.sh`.

---

## 3. Trajectory evaluation

### `eval/evaluate_bag.py`

**Primary evaluation pipeline for the thesis/OCEANS paper.** End-to-end evaluation of one bag's trajectories (`gt.tum`, `mimosa.tum`, `mavros.tum`). Renames `mimosa` → **MUSE-K** and `mavros` → **EKF** in all output.

Pipeline:
1. **Crop** all trajectories to a shared `[+start_s, −end_s]` window of the GT time range.
2. **Align** each estimator to GT — first run `evo_ape` with `--align_origin` (configurable to `-a` Umeyama at the top of the file) purely to extract the Sim(3) alignment matrix from the resulting zip.
3. **Apply** that Sim(3) to the estimator and add a constant body-frame rotation correction (evo's position-only alignment leaves a constant orientation bias). Save the result as `<name>_aligned.tum`.
4. **Re-run** `evo_ape` and `evo_rpe` (no alignment this time) on the aligned TUMs to get honest post-alignment APE/RPE metrics.
5. **Compute** extra metrics evo doesn't report directly: per-DOF translation error and per-axis Euler drift with linear-slope (`deg/1000s`, `accum_drift_deg`).
6. **Persist** per-sample APE arrays as `<name>_ape.npz` so cross-experiment plots don't have to re-invoke evo.
7. **Plot** a 6-DOF comparison (relative to GT[0]) and per-estimator APE-over-time figures (PNG + PDF).
8. **Write** `summary.txt` (human-readable) and `metrics.json` (machine-readable) under `<bag_dir>/evaluation/`.

**Arguments:**
- `<bag_dir>` — Directory containing `gt.tum` and the estimator TUM files.
- `--crop START_S END_S` — Seconds dropped from start and end of the GT time range. Default `0.0 0.0`.
- `--gt-file` — GT filename. Default `gt.tum`.
- `--estimators` — Estimator base names to evaluate. Default `mimosa mavros`.
- `--rpe-delta` — `evo_rpe -d` value. Default `10.0`.
- `--rpe-unit {f,d,r,m}` — `evo_rpe -u` value. Default `f`.
- `--evo-venv` — Path to a Python venv with `evo` installed. Default `~/evo`.
- `--output-dir` — Output directory. Default `<bag_dir>/evaluation`.

**Output (under `<bag_dir>/evaluation/`):**
- `cropped/`, `aligned/` — intermediate TUM trajectories.
- `evo/` — raw `evo` zips for the pre- and post-alignment passes.
- `plots/` — `6dof_comparison.{png,pdf}`, `<name>_ape_error.{png,pdf}`.
- `<name>_ape.npz` — per-sample APE arrays for downstream plotting.
- `summary.txt`, `metrics.json`.

**Example:**
```bash
python3 src/mimosa/mimosa/scripts/eval/evaluate_bag.py \
  /mnt/.../run_20260519_141318 \
  --crop 5 5 --estimators mimosa mavros
```

### `evaluate.sh`

Thin Bash wrapper around the `evo` CLI for **multi-estimator comparison in a single folder** (e.g. LiDAR/visual baselines vs Leica GT). For each estimator file in the folder it runs four evo commands and writes the result zips into `<folder>/evo_eval/`:

| Metric                         | Command                                                          |
| ------------------------------ | ---------------------------------------------------------------- |
| Pairwise RPE (translation)     | `evo_rpe ... -r trans_part --all_pairs -a -d 10 -u m`            |
| ATE (default SE(3) pose error) | `evo_ape ... -a`                                                 |
| ATE (rotation, deg)            | `evo_ape ... --pose_relation angle_deg -a`                       |
| ATE (translation only)         | `evo_ape ... --pose_relation trans_part -a`                      |

After processing all estimators it aggregates each metric across estimators with `evo_res ... --save_table <metric>_summary.csv`.

Setting `force_recompute=false` near the top of the script skips estimators whose result zip already exists.

**Arguments:**
- `[folder_path]` *(optional)* — Folder containing the GT and estimator TUMs. Has a hard-coded default at the top of the script.
- `[gt_file_name]` *(optional)* — GT filename in that folder. Default `leica_result.tum`.
- `[estimate_file_names...]` *(optional)* — Estimator filenames to evaluate. Default is a built-in list (`lri.tum`, `lri_4dof.tum`, `li.tum`, `ri.tum`, `fast_livo_2.tum`, …).

**Example:**
```bash
./src/mimosa/mimosa/scripts/evaluate.sh \
  /path/to/folder leica_result.tum lri.tum li.tum ri.tum
```

> Use this when you want raw evo zips/CSVs across many baseline estimators. For the MUSE-K vs EKF comparison with plots and aggregated metrics, use [`evaluate_bag.py`](#evalevaluate_bagpy) instead.

### `eval/evaluate_tum_trajectories.py`

Older custom Python evaluator. Loads a GT and several estimator TUMs (filenames and display names listed in a YAML config), normalises each trajectory to start at the origin, then interpolates estimators onto GT timestamps and computes ATE plus per-DOF (X/Y/Z/Roll/Pitch/Yaw) error stats.

Optional per-estimator `time_offset` (or `auto` to align first samples) compensates for clock skew between GT and estimator topics.

**Arguments:**
- `--config` — Path to evaluation config YAML (lists `gt`, `estimators[]`, `folder_path`, `output_path`).
- `--folder` *(optional)* — Override `folder_path`.
- `--output` *(optional)* — Override `output_path`.

**Output (in `output_path`):**
- `6dof_comparison.{png,pdf}` — overlaid GT and estimator trajectories.
- `<estimator>_ate_error.{png,pdf}` — ATE over time per estimator.
- `evaluation_summary.txt` — ATE + per-DOF stats.

**Example:**
```bash
./src/mimosa/mimosa/scripts/eval/evaluate_tum_trajectories.py \
  --config src/mimosa/mimosa/scripts/eval/eval_tum_config.yaml
```

> This evaluator does **not** perform Sim(3) alignment — it assumes the trajectories are already in a common frame (e.g. both reported in the MAVROS world frame). For evaluations that need alignment, prefer [`evaluate_bag.py`](#evalevaluate_bagpy).

---

## 4. Parameter sweeps

### `param_change_evaluation/run_evaluation.sh`

Replays a single bag through MIMOSA once per YAML config-override in a sweep directory, so you can compare the effect of different tunings on the same recording. For each `*.yaml` file under `OVERRIDES_DIR` the script:

- Wipes the streaming `graph_manager_odometry.tum` from MIMOSA's `logs_directory` (read out of `nortek_imu_params.yaml`) so a failed run can't be misattributed.
- Runs `ros2 launch mimosa eval_params_rosbag_launch_bluerov_imu.py bag_name:=<BAG_PATH> config_override:=<override_file> bag_output:=<RESULTS_DIR>/<experiment>_results`.
- Copies the produced trajectory to `<RESULTS_DIR>/trajectories/<experiment>.tum`.
- Skips experiments whose `<experiment>_results` directory already exists, unless `force_recompute=true`.

All inputs are configured by editing the top of the script — there are no CLI arguments:

| Variable           | Purpose                                                                 |
| ------------------ | ----------------------------------------------------------------------- |
| `BAG_PATH`         | The `.mcap` bag to replay.                                              |
| `force_recompute`  | `true` to overwrite existing experiment output directories.             |
| `EXPERIMENT_TYPE`  | Label for the sweep; becomes a subfolder under `RESULTS_DIR`.           |
| `OVERRIDES_DIR`    | Directory of YAML overrides; one experiment per file.                   |

**Output:**
- `data/param_change_evaluation_results/<bag_name>/<EXPERIMENT_TYPE>/<experiment>_results/` — output bag per experiment.
- `data/param_change_evaluation_results/<bag_name>/<EXPERIMENT_TYPE>/trajectories/<experiment>.tum` — MIMOSA trajectory per experiment.

**Example:**
```bash
cd ~/jazzy_ws/src/mimosa/mimosa/scripts/param_change_evaluation
./run_evaluation.sh
```

> The trajectories produced here can be fed to [`evaluate.sh`](#evaluatesh) (treating each `<experiment>.tum` as a separate "estimator" in one folder) to compare parameter settings against a common ground truth.
