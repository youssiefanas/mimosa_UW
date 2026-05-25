#!/bin/bash
# Batch-process every BlueROV2 ROS 2 bag under a base directory through mimosa.
#
# For each immediate subdirectory of <base_dir>, finds a ROS 2 bag (a folder
# containing metadata.yaml) and runs the bluerov2_rosbag_launch on it. After
# each run, the trajectory file produced by mimosa is copied next to the bag
# for downstream evaluation (e.g. with evo_*).
#
# Usage:
#   batch_process.sh <base_dir> [bag_subpath] [launch_file] [output_dir]
#
#   base_dir     Directory whose subfolders each contain a ROS 2 bag.
#   bag_subpath  Optional path inside each subfolder where the bag dir lives.
#                Defaults to "" (the subfolder itself is the bag).
#                Example: "processed/sensors_only_with_clouds" if your bags
#                are nested at <base_dir>/<run>/processed/sensors_only_with_clouds/
#   launch_file  Launch file to invoke per bag.
#                Defaults to "bluerov2_rosbag_launch.py".
#   output_dir   Optional directory to collect trajectories into, named
#                <subfolder>.tum. If a destination already exists, the new
#                file is saved as <subfolder>_<YYYYMMDD_HHMMSS>.tum so prior
#                runs are preserved. If unset, falls back to writing
#                <bag_dir>/../estimate.tum (legacy behavior).

set -euo pipefail

if [ $# -lt 1 ]; then
    echo "Usage: $0 <base_dir> [bag_subpath] [launch_file] [output_dir]"
    exit 1
fi

BASE_DIR="$1"
BAG_SUBPATH="${2:-}"
LAUNCH_FILE="${3:-bluerov2_rosbag_launch.py}"
OUTPUT_DIR="${4:-}"

if [ ! -d "$BASE_DIR" ]; then
    echo "Error: base_dir does not exist: $BASE_DIR"
    exit 1
fi

if [ -n "$OUTPUT_DIR" ]; then
    mkdir -p "$OUTPUT_DIR"
fi

# Resolve mimosa's logs_directory by reading the bluerov2 params file.
MIMOSA_PARAMS_DIR="$(ros2 pkg prefix mimosa)/share/mimosa/config/bluerov2"
PARAMS_YAML="${MIMOSA_PARAMS_DIR}/nortek_imu_params.yaml"
if [ ! -f "$PARAMS_YAML" ]; then
    echo "Error: mimosa params not found at $PARAMS_YAML"
    echo "       Did you source the workspace?  source install/setup.bash"
    exit 1
fi
LOGS_DIR="$(grep -E '^logs_directory:' "$PARAMS_YAML" | head -n1 | awk '{print $2}')"
TUM_FILE="${LOGS_DIR%/}/graph_manager_odometry.tum"

echo "base_dir:    $BASE_DIR"
echo "bag_subpath: ${BAG_SUBPATH:-(subfolder is the bag itself)}"
echo "launch:      $LAUNCH_FILE"
echo "logs_dir:    $LOGS_DIR"
echo "tum_file:    $TUM_FILE"
echo "output_dir:  ${OUTPUT_DIR:-(none, writing next to each bag)}"
echo

shopt -s nullglob
for folder in "$BASE_DIR"/*/; do
    folder_name="$(basename "$folder")"

    if [ -n "$BAG_SUBPATH" ]; then
        bag_path="${folder%/}/${BAG_SUBPATH}"
    else
        bag_path="${folder%/}"
    fi

    if [ ! -f "${bag_path}/metadata.yaml" ]; then
        echo "[SKIP] $folder_name — no ROS 2 bag at ${bag_path} (missing metadata.yaml)"
        continue
    fi

    echo "[RUN ] $folder_name"

    # Wipe stale trajectory so a launch failure doesn't get attributed to this bag.
    rm -f "$TUM_FILE"

    if ros2 launch mimosa "$LAUNCH_FILE" bag_name:="$bag_path"; then
        if [ -f "$TUM_FILE" ]; then
            if [ -n "$OUTPUT_DIR" ]; then
                bag_out_dir="${OUTPUT_DIR%/}/${folder_name}"
                mkdir -p "$bag_out_dir"
                dest="${bag_out_dir}/mimosa.tum"
            else
                dest="${bag_path%/}/../estimate.tum"
            fi
            cp "$TUM_FILE" "$dest"
            echo "[OK  ] $folder_name -> $dest"
        else
            echo "[WARN] $folder_name — launch finished but no trajectory at $TUM_FILE"
        fi
    else
        echo "[FAIL] $folder_name — launch returned non-zero"
    fi
    echo "---"
done
