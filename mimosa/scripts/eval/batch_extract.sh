#!/usr/bin/env bash
# Extract gt.tum / mavros.tum / mimosa.tum into every run_* bag dir under
# the given parent directory.
#
# Usage:
#   batch_extract.sh <parent_dir> [glob]
#   batch_extract.sh /mnt/.../19_05
#   batch_extract.sh /mnt/.../19_05 'run_2026*'

set -euo pipefail

PARENT="${1:-}"
GLOB="${2:-run_*}"
if [[ -z "$PARENT" || ! -d "$PARENT" ]]; then
  echo "usage: $0 <parent_dir> [glob]"; exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

CHARUCO_TOPIC="/cam1/image_raw/compressed"
MAVROS_TOPIC="/bluerov2/local_position/odom"
MIMOSA_TOPIC="/mimosa_node/graph/odometry"

shopt -s nullglob
runs=("$PARENT"/$GLOB)
shopt -u nullglob

if [[ ${#runs[@]} -eq 0 ]]; then
  echo "[error] no bags matched ${PARENT}/${GLOB}"; exit 1
fi

echo "[info] processing ${#runs[@]} bag(s) under $PARENT"

for bag in "${runs[@]}"; do
  [[ -d "$bag" ]] || continue
  [[ -f "$bag/metadata.yaml" ]] || { echo "[skip] $bag has no metadata.yaml"; continue; }
  name="$(basename "$bag")"
  echo
  echo "=============================="
  echo "[bag] $name"
  echo "=============================="

  # --- ChArUco GT ---
  if [[ -s "$bag/gt.tum" ]]; then
    echo "[skip] gt.tum already exists"
  else
    echo "[step] gt.tum (ChArUco)"
    python3 "$SCRIPT_DIR/tag_gt_to_tum.py" "$bag" "$bag/gt.tum" \
        --tag-type charuco || echo "[warn] gt extraction failed for $name"
  fi

  # --- MAVROS EKF ---
  if [[ -s "$bag/mavros.tum" ]]; then
    echo "[skip] mavros.tum already exists"
  else
    echo "[step] mavros.tum"
    python3 "$SCRIPT_DIR/mavros_to_tum.py" "$bag" "$bag/mavros.tum" \
        --topic "$MAVROS_TOPIC" || echo "[warn] mavros extraction failed for $name"
  fi

  # --- MIMOSA odometry ---
  if [[ -s "$bag/mimosa.tum" ]]; then
    echo "[skip] mimosa.tum already exists"
  else
    echo "[step] mimosa.tum"
    python3 "$SCRIPT_DIR/odom_to_tum.py" "$bag" "$bag/mimosa.tum" \
        --topic "$MIMOSA_TOPIC" || echo "[warn] mimosa extraction failed for $name"
  fi
done

echo
echo "[done] extracted all bags under $PARENT"
