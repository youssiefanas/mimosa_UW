#!/bin/bash

folder_path="/home/cyanide/rosbags/nk4/lab/bags/fyllingsdal/day_2/results/500m_7_fast"
gt_file_name="leica_result.tum"

estimate_file_names=(
  "lri.tum"
  "lri_4dof.tum"
  "lri_4dof_avar_imu_params.tum"
  "li.tum"
  "ri.tum"
  "ri_avar_imu_params.tum"
  "fast_livo_2.tum"
)

force_recompute=false

# Check if folder exists
if [[ ! -d "$folder_path" ]]; then
  echo "[ERROR] Directory does not exist: $folder_path"
  exit 1
fi

gt_file="$folder_path/$gt_file_name"

# Check if ground truth file exists
if [[ ! -f "$gt_file" ]]; then
  echo "[ERROR] Ground truth file $gt_file does not exist"
  exit 1
fi

for estimate_file_name in "${estimate_file_names[@]}"; do
  estimate_file="$folder_path/$estimate_file_name"

  # Check if estimate file exists
  if [[ ! -f "$estimate_file" ]]; then
    echo "[WARNING] Skipping $estimate_file - file does not exist"
    continue
  fi

  # Extract the base name without extension for --save_results
  base_name="${estimate_file_name%.tum}"

  rpe_zip="$folder_path/rpe_trans_$base_name.zip"
  rpe_plot="$folder_path/rpe_trans_$base_name"
  if [[ -f "$rpe_zip" && "$force_recompute" = false ]]; then
    echo "[INFO] Skipping RPE for $estimate_file - results already exist at $rpe_zip"
  else
    evo_rpe tum -r trans_part --all_pairs -a -d 10 -u m "$gt_file" "$estimate_file" --save_results "$rpe_zip" --save_plot "$rpe_plot" --no_warnings
  fi

  ape_zip="$folder_path/ape_trans_$base_name.zip"
  ape_plot="$folder_path/ape_trans_$base_name"
  if [[ -f "$ape_zip" && "$force_recompute" = false ]]; then
    echo "[INFO] Skipping ATE for $estimate_file - results already exist at $ape_zip"
  else
    evo_ape tum -r trans_part -a "$gt_file" "$estimate_file" --save_results "$ape_zip" --save_plot "$ape_plot" --no_warnings
  fi

done

# Aggregate summary tables across all estimates
rpe_zips=("$folder_path"/rpe_trans_*.zip)
if [[ -e "${rpe_zips[0]}" ]]; then
  evo_res "${rpe_zips[@]}" --save_table "$folder_path/rpe_summary.csv" --no_warnings
fi

ape_zips=("$folder_path"/ape_trans_*.zip)
if [[ -e "${ape_zips[0]}" ]]; then
  evo_res "${ape_zips[@]}" --save_table "$folder_path/ape_summary.csv" --no_warnings
fi
