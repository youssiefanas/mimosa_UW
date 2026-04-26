#!/bin/bash
set -u

# Configuration
# BAG_NAME="/home/cyanide/rosbags/nk4/lab/bags/fyllingsdal/day_2/500m_7_fast_actually_615/processed/sensors_only_with_clouds.bag"
BAG_NAME="/home/cyanide/rosbags/nk4/lab/bags/2025_11_20_runehamar/hornbill/flight8/processed/sensors_only_with_clouds.bag"

force_recompute=false

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MIMOSA_DIR="$(realpath "$SCRIPT_DIR/../..")"
OVERRIDES_DIR="$SCRIPT_DIR/config_overrides"
RESULTS_DIR="$MIMOSA_DIR/data/param_change_evaluation_results"

mkdir -p "$RESULTS_DIR"

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

    echo ""
    echo "Running experiment: $experiment_name"
    echo "Override file: $override_file"
    echo "Output bag: $bag_output"
    echo "------------------------------------------"

    if [[ -d "$bag_output" ]]; then
        if [[ "$force_recompute" = false ]]; then
            echo "[INFO] Skipping $experiment_name - output already exists at $bag_output"
            continue
        fi
        echo "[INFO] Removing existing output at $bag_output"
        rm -rf "$bag_output"
    fi

    ros2 launch mimosa hornbill_rosbag_launch.py \
        bag_name:="$BAG_NAME" \
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

    echo "Finished: $experiment_name"
    echo "=========================================="

    sleep 2
done

echo ""
echo "All experiments complete!"
echo "Results saved in: $RESULTS_DIR"
