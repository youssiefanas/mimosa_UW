#!/bin/bash
set -u

# Configuration
# BAG_PATH="/home/cyanide/rosbags/nk4/lab/bags/fyllingsdal/day_2/500m_7_fast_actually_615/processed/sensors_only_with_clouds.bag"
# BAG_PATH="/mnt/F68C0A428C09FE3D/Academics/MIR/NTNU/Thesis/bluerov_rosbags/rosbags/run_20260429_173203/run_20260429_173203_0.mcap"
# BAG_PATH="/mnt/F68C0A428C09FE3D/Academics/MIR/NTNU/Thesis/bluerov_rosbags/rosbags/run_20260429_174159/run_20260429_174159_0.mcap"
BAG_PATH="/mnt/F68C0A428C09FE3D/Academics/MIR/NTNU/Thesis/bluerov_rosbags/rosbags/run_20260429_174555/run_20260429_174555_0.mcap"

force_recompute=false

BAG_NAME="$(basename "$BAG_PATH" .mcap)"
echo "Evaluating on bag: $BAG_NAME"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MIMOSA_DIR="$(realpath "$SCRIPT_DIR/../..")"
EXPERIMENT_TYPE="bluerov2_imu"
OVERRIDES_DIR="$SCRIPT_DIR/bluerov_param_eval"
RESULTS_DIR="$MIMOSA_DIR/data/param_change_evaluation_results/$BAG_NAME/$EXPERIMENT_TYPE"
TRAJ_DIR="$RESULTS_DIR/trajectories"


mkdir -p "$RESULTS_DIR" "$TRAJ_DIR"

# Resolve the streaming TUM path mimosa writes to (logs_directory in the base
# bluerov2 params). It always writes to <logs_directory>/graph_manager_odometry.tum;
# we'll copy that to TRAJ_DIR/<experiment_name>.tum after each run.
# PARAMS_YAML="$(ros2 pkg prefix mimosa)/share/mimosa/config/bluerov2/params.yaml"
PARAMS_YAML="$(ros2 pkg prefix mimosa)/share/mimosa/config/bluerov2/bluerov_imu_params.yaml"

if [[ ! -f "$PARAMS_YAML" ]]; then
    echo "Error: mimosa params not found at $PARAMS_YAML"
    echo "       Did you source the workspace?  source install/setup.bash"
    exit 1
fi
LOGS_DIR="$(grep -E '^logs_directory:' "$PARAMS_YAML" | head -n1 | awk '{print $2}')"
TUM_FILE="${LOGS_DIR%/}/graph_manager_odometry.tum"

if [[ ! -d "$OVERRIDES_DIR" ]]; then
    echo "Error: Overrides directory not found: $OVERRIDES_DIR"
    exit 1
fi

shopt -s nullglob
yaml_files=("$OVERRIDES_DIR"/*.yaml)
shopt -u nullglob
if [[ ${#yaml_files[@]} -eq 0 ]]; then
    echo "No YAML files found in $OVERRIDES_DIR"
    exit 1
fi

echo "Found ${#yaml_files[@]} override file(s)"
echo "=========================================="

for override_file in "${yaml_files[@]}"; do
    experiment_name=$(basename "$override_file" .yaml)
    bag_output="$RESULTS_DIR/${experiment_name}_results"
    traj_output="$TRAJ_DIR/${experiment_name}.tum"

    echo ""
    echo "Running experiment: $experiment_name"
    echo "Override file: $override_file"
    echo "Output bag: $bag_output"
    echo "Output traj: $traj_output"
    echo "------------------------------------------"

    if [[ -d "$bag_output" ]]; then
        if [[ "$force_recompute" = false ]]; then
            echo "[INFO] Skipping $experiment_name - output already exists at $bag_output"
            continue
        fi
        echo "[INFO] Removing existing output at $bag_output"
        rm -rf "$bag_output"
    fi

    # Wipe stale trajectory so a launch failure can't get attributed to this run.
    rm -f "$TUM_FILE"

    # ros2 launch mimosa hornbill_rosbag_launch.py \
    ros2 launch mimosa eval_params_rosbag_launch_bluerov_imu.py \
        bag_name:="$BAG_PATH" \
        config_override:="$override_file" \
        bag_output:="$bag_output"

    exit_status=$?
    if [[ $exit_status -ne 0 ]]; then
        echo "Warning: Experiment $experiment_name exited with status $exit_status"
    fi

    if [[ -d "$bag_output" ]]; then
        echo "Results saved to: $bag_output"
    else
        echo "Warning: Expected output directory $bag_output was not created"
    fi

    if [[ -f "$TUM_FILE" ]]; then
        cp "$TUM_FILE" "$traj_output"
        echo "Trajectory saved to: $traj_output"
    else
        echo "Warning: No trajectory written at $TUM_FILE"
    fi

    echo "Finished: $experiment_name"
    echo "=========================================="

    sleep 2
done

echo ""
echo "All experiments complete!"
echo "Results saved in: $RESULTS_DIR"
