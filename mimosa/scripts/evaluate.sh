#!/bin/bash

folder_path="${1:-/home/cyanide/rosbags/nk4/lab/bags/fyllingsdal/day_2/results/500m_7_fast}"
gt_file_name="${2:-leica_result.tum}"

if [[ $# -gt 2 ]]; then
  shift 2
  estimate_file_names=("$@")
else
  estimate_file_names=(
    "lri.tum"
    "lri_4dof.tum"
    "lri_4dof_avar_imu_params.tum"
    "li.tum"
    "ri.tum"
    "ri_avar_imu_params.tum"
    "fast_livo_2.tum"
  )
fi
force_recompute=true
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

  eval_dir="$folder_path/evo_eval"
  mkdir -p "$eval_dir"

  rpe_zip="$eval_dir/rpe_trans_$base_name.zip"
  rpe_plot="$eval_dir/rpe_trans_$base_name"
  if [[ -f "$rpe_zip" && "$force_recompute" = false ]]; then
    echo "[INFO] Skipping RPE for $estimate_file - results already exist at $rpe_zip"
  else
    evo_rpe tum -r trans_part --all_pairs -a -d 10 -u m "$gt_file" "$estimate_file" --save_results "$rpe_zip" --save_plot "$rpe_plot" --no_warnings
  fi

  # 1. As it is (default with alignment). the SE(3) pose error combining translation and rotation into a single norm
  ape_1_zip="$eval_dir/ape_default_$base_name.zip"
  ape_1_plot="$eval_dir/ape_default_$base_name"
  if [[ -f "$ape_1_zip" && "$force_recompute" = false ]]; then
    echo "[INFO] Skipping ATE (default) for $estimate_file - results already exist at $ape_1_zip"
  else
    evo_ape tum -a "$gt_file" "$estimate_file" --save_results "$ape_1_zip" --save_plot "$ape_1_plot" --no_warnings
  fi

  # 2. Angle degrees with alignment
  ape_2_zip="$eval_dir/ape_angle_$base_name.zip"
  ape_2_plot="$eval_dir/ape_angle_$base_name"
  if [[ -f "$ape_2_zip" && "$force_recompute" = false ]]; then
    echo "[INFO] Skipping ATE (angle) for $estimate_file - results already exist at $ape_2_zip"
  else
    evo_ape tum --pose_relation angle_deg -a "$gt_file" "$estimate_file" --save_results "$ape_2_zip" --save_plot "$ape_2_plot" --no_warnings
  fi

  # 3. Trans part without alignment. This command restricts the error to the translational component
  ape_3_zip="$eval_dir/ape_trans_$base_name.zip"
  ape_3_plot="$eval_dir/ape_trans_$base_name"
  if [[ -f "$ape_3_zip" && "$force_recompute" = false ]]; then
    echo "[INFO] Skipping ATE (trans part) for $estimate_file - results already exist at $ape_3_zip"
  else
    evo_ape tum --pose_relation trans_part -a "$gt_file" "$estimate_file" --save_results "$ape_3_zip" --save_plot "$ape_3_plot" --no_warnings
  fi

done

eval_dir="$folder_path/evo_eval"

# Aggregate summary tables across all estimates
rpe_zips=("$eval_dir"/rpe_trans_*.zip)
if [[ -e "${rpe_zips[0]}" ]]; then
  evo_res "${rpe_zips[@]}" --save_table "$eval_dir/rpe_summary.csv" --no_warnings
fi

ape_1_zips=("$eval_dir"/ape_default_*.zip)
if [[ -e "${ape_1_zips[0]}" ]]; then
  evo_res "${ape_1_zips[@]}" --save_table "$eval_dir/ape_default_summary.csv" --no_warnings
fi

ape_2_zips=("$eval_dir"/ape_angle_*.zip)
if [[ -e "${ape_2_zips[0]}" ]]; then
  evo_res "${ape_2_zips[@]}" --save_table "$eval_dir/ape_angle_summary.csv" --no_warnings
fi

ape_3_zips=("$eval_dir"/ape_trans_*.zip)
if [[ -e "${ape_3_zips[0]}" ]]; then
  evo_res "${ape_3_zips[@]}" --save_table "$eval_dir/ape_trans_summary.csv" --no_warnings
fi
