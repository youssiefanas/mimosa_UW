#!/usr/bin/env python3

import os
import argparse
import yaml
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import interp1d
from scipy.spatial.transform import Slerp


log_lines = []
def log(*args, **kwargs):
    msg = " ".join(map(str, args))
    print(msg)
    log_lines.append(msg)

def load_tum(filename):
    """Load a TUM trajectory file. Returns Nx8 array (t, tx, ty, tz, qx, qy, qz, qw)."""
    return np.loadtxt(filename)

def find_time_offset(gt_data, est_data):
    """
    Find optimal time offset to add to est_data[:, 0] to align with gt_data[:, 0].
    Uses cross-correlation of velocity magnitudes.
    """
    t_gt = gt_data[:, 0]
    p_gt = gt_data[:, 1:4]
    
    t_est = est_data[:, 0]
    p_est = est_data[:, 1:4]
    
    # Calculate velocities
    v_gt = np.linalg.norm(np.diff(p_gt, axis=0), axis=1) / np.diff(t_gt)
    v_est = np.linalg.norm(np.diff(p_est, axis=0), axis=1) / np.diff(t_est)
    
    t_gt_mid = t_gt[:-1] + np.diff(t_gt)/2
    t_est_mid = t_est[:-1] + np.diff(t_est)/2
    
    # Rough alignment (align starts)
    rough_offset = t_gt_mid[0] - t_est_mid[0]
    t_est_shifted = t_est_mid + rough_offset
    
    # Create common grid
    dt = 0.01  # 100 Hz
    t_min = max(t_gt_mid[0], t_est_shifted[0])
    t_max = min(t_gt_mid[-1], t_est_shifted[-1])
    
    if t_max <= t_min + 1.0:
        return rough_offset
        
    grid = np.arange(t_min, t_max, dt)
    
    v_gt_interp = np.interp(grid, t_gt_mid, v_gt)
    v_est_interp = np.interp(grid, t_est_shifted, v_est)
    
    v_gt_interp -= np.mean(v_gt_interp)
    v_est_interp -= np.mean(v_est_interp)
    
    if np.std(v_gt_interp) < 1e-6 or np.std(v_est_interp) < 1e-6:
        return rough_offset
        
    correlation = np.correlate(v_gt_interp, v_est_interp, mode='full')
    lags = np.arange(-len(v_est_interp) + 1, len(v_gt_interp))
    best_lag = lags[np.argmax(correlation)]
    
    fine_offset = best_lag * dt
    
    return rough_offset + fine_offset


def transform_trajectory_to_origin(data, name="Trajectory"):
    """
    Transform the trajectory such that its first pose is at the origin (0,0,0)
    with Identity rotation.
    Returns the transformed data (Nx8).
    Prints the initial rotation matrix.
    """
    if len(data) == 0:
        return data
        
    p0 = data[0, 1:4]
    q0 = data[0, 4:8]
    r0 = R.from_quat(q0)
    
    r0_inv = r0.inv()
    p0_inv = - r0_inv.apply(p0)
    
    log(f"[{name}] Initial Rotation Matrix (World to Pose0):")
    log(r0_inv.as_matrix())
    log("-" * 40)
    
    transformed_data = np.copy(data)
    
    p_all = data[:, 1:4]
    q_all = data[:, 4:8]
    
    p_new = r0_inv.apply(p_all) + p0_inv
    
    r_all = R.from_quat(q_all)
    r_new = r0_inv * r_all
    q_new = r_new.as_quat()
    
    transformed_data[:, 1:4] = p_new
    transformed_data[:, 4:8] = q_new
    
    return transformed_data

def interpolate_trajectory(t_target, t_source, p_source, q_source):
    """
    Interpolate source trajectory (t_source, p_source, q_source) to match t_target.
    Returns:
    - t_valid: valid target timestamps (within source bounds)
    - p_interp: interpolated positions
    - q_interp: interpolated quaternions
    - valid_idx: boolean mask of valid timestamps in t_target
    """
    valid_idx = (t_target >= t_source[0]) & (t_target <= t_source[-1])
    t_valid = t_target[valid_idx]
    
    if len(t_valid) < 2:
        return t_valid, np.empty((0, 3)), np.empty((0, 4)), valid_idx
    
    # Interpolate position
    interp_pos = interp1d(t_source, p_source, axis=0)
    p_interp = interp_pos(t_valid)
    
    # Interpolate rotation
    slerp = Slerp(t_source, R.from_quat(q_source))
    r_interp = slerp(t_valid)
    q_interp = r_interp.as_quat()
    
    return t_valid, p_interp, q_interp, valid_idx

def plot_6dof(gt_data, estimators_data, output_dir):
    """Plot 6 DOF comparison of trajectories."""
    fig, axs = plt.subplots(6, 1, figsize=(12, 18), sharex=True)
    
    # Ground Truth
    t_gt = gt_data[:, 0]
    t_gt_rel = t_gt - t_gt[0] # relative time for plotting
    
    p_gt = gt_data[:, 1:4]
    r_gt = R.from_quat(gt_data[:, 4:8]).as_euler('xyz', degrees=True)
    
    labels = ['X [m]', 'Y [m]', 'Z [m]', 'Roll [deg]', 'Pitch [deg]', 'Yaw [deg]']
    gt_vals = [p_gt[:,0], p_gt[:,1], p_gt[:,2], r_gt[:,0], r_gt[:,1], r_gt[:,2]]
    
    for i in range(6):
        axs[i].plot(t_gt_rel, gt_vals[i], 'k-', label='Ground Truth', linewidth=2)
        axs[i].set_ylabel(labels[i])
        axs[i].grid(True, linestyle='--', alpha=0.6)
        
    colors = plt.cm.tab10.colors

    for idx, est in enumerate(estimators_data):
        name = est['name']
        data = est['data']
        # Use relative time aligned with GT start time (assuming synchronized timestamps)
        t_est_rel = data[:, 0] - t_gt[0]
        p_est = data[:, 1:4]
        r_est = R.from_quat(data[:, 4:8]).as_euler('xyz', degrees=True)

        est_vals = [p_est[:,0], p_est[:,1], p_est[:,2], r_est[:,0], r_est[:,1], r_est[:,2]]
        color = est.get('color') or colors[idx % len(colors)]
        
        for i in range(6):
            axs[i].plot(t_est_rel, est_vals[i], label=name, color=color, linewidth=1.5, alpha=0.8)
            
    axs[5].set_xlabel('Time [s]')
    axs[0].legend(loc='upper right', bbox_to_anchor=(1.0, 1.25), ncol=3)
    
    plt.suptitle("6 DOF Trajectory Comparison", fontsize=16, y=0.96)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    
    # Ensure output dir exists
    os.makedirs(output_dir, exist_ok=True)
    
    plt.savefig(os.path.join(output_dir, '6dof_comparison.png'), dpi=300)
    plt.savefig(os.path.join(output_dir, '6dof_comparison.pdf'))
    plt.close()
    log("Saved 6 DOF comparison plots.")

def compute_and_plot_error(gt_data, est_data, est_name, output_dir, est_color=None):
    """Plot ATE error over time and compute all DOFs."""
    t_gt = gt_data[:, 0]
    p_gt = gt_data[:, 1:4]
    q_gt = gt_data[:, 4:8]
    
    t_est = est_data[:, 0]
    p_est = est_data[:, 1:4]
    q_est = est_data[:, 4:8]
    
    t_valid, p_interp, q_interp, valid_idx = interpolate_trajectory(t_gt, t_est, p_est, q_est)
    
    if len(t_valid) == 0:
        log(f"Warning: No overlapping timestamps found between GT and {est_name}.")
        return None
        
    p_gt_valid = p_gt[valid_idx]
    q_gt_valid = q_gt[valid_idx]
    
    # ATE translation
    ate_error = np.linalg.norm(p_interp - p_gt_valid, axis=1)
    
    ate_metrics = {
        'rmse': np.sqrt(np.mean(ate_error**2)),
        'mean': np.mean(ate_error),
        'median': np.median(ate_error),
        'std': np.std(ate_error)
    }
    
    # Per-DOF Error
    r_gt_valid_euler = R.from_quat(q_gt_valid).as_euler('xyz', degrees=True)
    r_interp_euler = R.from_quat(q_interp).as_euler('xyz', degrees=True)
    
    euler_error = r_interp_euler - r_gt_valid_euler
    # Normalize angular error to [-180, 180]
    euler_error = (euler_error + 180) % 360 - 180
    
    dof_errors = {
        'X': p_interp[:, 0] - p_gt_valid[:, 0],
        'Y': p_interp[:, 1] - p_gt_valid[:, 1],
        'Z': p_interp[:, 2] - p_gt_valid[:, 2],
        'Roll': euler_error[:, 0],
        'Pitch': euler_error[:, 1],
        'Yaw': euler_error[:, 2]
    }
    
    dof_metrics = {}
    for dof, err in dof_errors.items():
        dof_metrics[dof] = {
            'rmse': np.sqrt(np.mean(err**2)),
            'mean': np.mean(err),
            'std': np.std(err),
        }
        
    mean_err = ate_metrics['mean']
    median_err = ate_metrics['median']
    rmse = ate_metrics['rmse']
    std_err = ate_metrics['std']
    error = ate_error
    
    fig, ax = plt.subplots(figsize=(10, 5))
    
    # Plot relative time
    t_plot = t_valid - t_gt[0]
    
    ax.plot(t_plot, error, label=f'{est_name} ATE', color=est_color or 'C0', alpha=0.6, linewidth=1.5)
    
    ax.axhline(mean_err, color='r', linestyle='--', label=f'Mean: {mean_err:.4f} m')
    ax.axhline(median_err, color='g', linestyle='-.', label=f'Median: {median_err:.4f} m')
    ax.axhline(rmse, color='b', linestyle=':', label=f'RMSE: {rmse:.4f} m')
    
    ax.fill_between(t_plot, np.maximum(0, mean_err - std_err), mean_err + std_err, 
                    color='r', alpha=0.2, label=f'Std: ±{std_err:.4f} m')
    
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Absolute Translation Error [m]')
    ax.set_title(f'Translation Error over Time - {est_name}')
    ax.grid(True, linestyle='--', alpha=0.6)
    
    # Place legend outside or neatly inside
    ax.legend(loc='upper right', bbox_to_anchor=(1.25, 1.0))
    
    plt.tight_layout()
    filename_base = est_name.lower().replace(' ', '_')
    plt.savefig(os.path.join(output_dir, f'{filename_base}_ate_error.png'), dpi=300, bbox_inches='tight')
    plt.savefig(os.path.join(output_dir, f'{filename_base}_ate_error.pdf'), bbox_inches='tight')
    plt.close()
    log(f"Saved error plot for {est_name} (RMSE: {rmse:.4f}m).")
    return {'ate': ate_metrics, 'dof': dof_metrics}

def main():
    parser = argparse.ArgumentParser(description="Evaluate TUM trajectories.")
    parser.add_argument('--config', type=str, required=True, help="Path to evaluation config YAML.")
    parser.add_argument('--folder', type=str, default=None,
                        help="Override folder_path from the config (where the TUM files live).")
    parser.add_argument('--output', type=str, default=None,
                        help="Override output_path from the config (where plots go).")
    args = parser.parse_args()

    with open(args.config, 'r') as f:
        config = yaml.safe_load(f)

    folder_path = args.folder or config.get('folder_path', '.')
    output_path = args.output or config.get('output_path', folder_path)
    gt_filename = config.get('gt')
    estimators = config.get('estimators', [])
    
    if not gt_filename:
        log("Error: Ground truth file 'gt' not specified in config.")
        return
        
    gt_path = os.path.join(folder_path, gt_filename)
    if not os.path.exists(gt_path):
        log(f"Error: GT file not found at {gt_path}")
        return
        
    log(f"\nLoading GT: {gt_path}")
    gt_data = load_tum(gt_path)
    log(f"  -> Found {len(gt_data)} poses.")
    if len(gt_data) > 0:
        log(f"  -> Time range: {gt_data[0,0]:.3f} to {gt_data[-1,0]:.3f} (Duration: {gt_data[-1,0] - gt_data[0,0]:.3f}s)")
        
    gt_data_transformed = transform_trajectory_to_origin(gt_data, name="Ground Truth")
    
    estimators_data = []
    for est in estimators:
        est_name = est.get('name', 'Unknown')
        est_filename = est.get('filename')
        if not est_filename:
            log(f"Warning: No filename for estimator '{est_name}'. Skipping.")
            continue
            
        est_path = os.path.join(folder_path, est_filename)
        if not os.path.exists(est_path):
            log(f"Warning: File not found for '{est_name}' at {est_path}. Skipping.")
            continue
            
        log(f"\nLoading Estimator: {est_path}")
        est_data = load_tum(est_path)
        log(f"  -> Found {len(est_data)} poses.")
        
        # Apply optional time offset
        time_offset_config = est.get('time_offset', 0.0)
        
        if time_offset_config == 'auto':
            if len(est_data) > 0 and len(gt_data) > 0:
                # Strictly align the start times so all trajectories start together
                time_offset = gt_data[0, 0] - est_data[0, 0]
                log(f"  -> Auto-calculated optimal time_offset (aligned starts): {time_offset:.3f}s")
            else:
                time_offset = 0.0
        else:
            time_offset = float(time_offset_config)
            
        if time_offset != 0.0:
            if time_offset_config != 'auto':
                log(f"  -> Applying manual time_offset: {time_offset:.3f}s")
            est_data[:, 0] += time_offset
            
        if len(est_data) > 0:
            log(f"  -> Time range: {est_data[0,0]:.3f} to {est_data[-1,0]:.3f} (Duration: {est_data[-1,0] - est_data[0,0]:.3f}s)")
            if len(gt_data) > 0:
                log(f"  -> Offset from GT start: {est_data[0,0] - gt_data[0,0]:.3f}s")
                
        est_data_transformed = transform_trajectory_to_origin(est_data, name=est_name)
        
        estimators_data.append({
            'name': est_name,
            'data': est_data_transformed,
            'color': est.get('color'),
        })
        
    if not estimators_data:
        log("No estimators loaded. Exiting.")
        return
        
    log("\nGenerating 6-DOF comparison plot...")
    plot_6dof(gt_data_transformed, estimators_data, output_path)
    
    log("\nGenerating per-estimator error plots...")
    colors = plt.cm.tab10.colors
    results = {}
    for idx, est in enumerate(estimators_data):
        est_color = est.get('color') or colors[idx % len(colors)]
        metrics = compute_and_plot_error(gt_data_transformed, est['data'], est['name'], output_path, est_color=est_color)
        if metrics:
            results[est['name']] = metrics

    log("\n" + "="*40)
    log("EVALUATION METRICS SUMMARY")
    log("="*40)
    for est_name, metrics in results.items():
        log(f"\n--- {est_name} ---")
        ate = metrics['ate']
        log(f"ATE (Translation Error):")
        log(f"  RMSE:   {ate['rmse']:.4f} m")
        log(f"  Mean:   {ate['mean']:.4f} m")
        log(f"  Std:    {ate['std']:.4f} m")
        log(f"  Median: {ate['median']:.4f} m")
        
        log(f"\nPer-DOF Errors:")
        for dof in ['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw']:
            d_met = metrics['dof'][dof]
            unit = "deg" if dof in ['Roll', 'Pitch', 'Yaw'] else "m"
            log(f"  {dof}:")
            log(f"    RMSE: {d_met['rmse']:.4f} {unit}")
            log(f"    Mean: {d_met['mean']:.4f} {unit}")
            log(f"    Std:  {d_met['std']:.4f} {unit}")
            
    # Save text log
    summary_path = os.path.join(output_path, "evaluation_summary.txt")
    with open(summary_path, "w") as f:
        f.write("\n".join(log_lines) + "\n")
    print(f"\nSaved evaluation summary text to {summary_path}")

if __name__ == '__main__':
    main()
