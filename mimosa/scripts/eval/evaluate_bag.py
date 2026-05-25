#!/usr/bin/env python3
"""End-to-end evaluation pipeline for a single bag's trajectories.

Inputs (in <bag_dir>):
    gt.tum, mimosa.tum, mavros.tum     (TUM-format trajectories)

Steps:
    1. Load + crop trajectories by [--crop START_S END_S] (seconds dropped
       from start/end of GT's time range, applied to all trajectories).
    2. Run evo_ape (with -a) on cropped TUMs to get the Umeyama Sim(3)
       alignment matrix that brings the estimator onto GT.
    3. Apply that matrix to the cropped estimator (positions + orientations).
       Then compute a constant body-frame rotation correction (evo's -a only
       aligns positions, so orientations keep a constant frame bias unless
       we add this step). Save the result as <name>_aligned.tum.
    4. Re-run evo_ape (without -a) + evo_rpe on the aligned TUMs to get the
       *post-alignment* APE/RPE metrics.
    5. Plot:
         - 6dof_comparison.{png,pdf}            (GT[0]-relative frame)
         - <name>_ape_error.{png,pdf}           (APE-translation over time)
    6. Compute angular drift (linear slope of per-axis Euler error vs time)
       and persist per-sample APE arrays to <name>_ape.npz so downstream
       cross-experiment plots can read clean arrays without re-running evo.
    7. Write summary.txt and metrics.json under <bag_dir>/evaluation/.

Naming:
    mimosa.tum -> "MUSE-K"
    mavros.tum -> "EKF"

Usage:
    evaluate_bag.py /mnt/.../run_20260519_141318
    evaluate_bag.py /mnt/.../run_... --crop 5 5
"""

import argparse
import json
import os
import subprocess
import sys
import zipfile

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from evaluate_tum_trajectories import load_tum

ESTIMATOR_CFG = {
    'mimosa': {'display': 'MUSE-K', 'color': '#ff7f0e'},
    'mavros': {'display': 'EKF',    'color': "#1f3ab4"},
}
GT_COLOR = 'black'

# evo alignment flag, used by every evo_ape / evo_rpe invocation in this script.
# Choose:
#   '-a'              → Umeyama SE(3) alignment over all matched samples.
#                       Right for trajectories with real motion.
#   '--align_origin'  → First-pose alignment (rigid T mapping est[0] → gt[0]).
#                       Right for nearly-stationary station-keeping bags where
#                       Umeyama is degenerate and can produce an axis-swap.
EVO_ALIGN_FLAG = '--align_origin'


# --------------------------------------------------------------------- IO

def save_tum(data, path):
    np.savetxt(path, data, fmt='%.9f',
               header='timestamp tx ty tz qx qy qz qw', comments='# ')


def crop_tum(data, t_start, t_end):
    m = (data[:, 0] >= t_start) & (data[:, 0] <= t_end)
    return data[m]


# --------------------------------------------------------------------- evo

def evo_env(venv_path):
    """Return an os.environ-like dict with venv activated."""
    venv = os.path.expanduser(venv_path)
    env = os.environ.copy()
    env['VIRTUAL_ENV'] = venv
    env['PATH'] = f'{venv}/bin:' + env.get('PATH', '')
    env.pop('PYTHONHOME', None)
    return env, f'{venv}/bin'


def run_evo(args_list, env, log_prefix='evo'):
    print(f'  $ {" ".join(args_list)}')
    r = subprocess.run(args_list, env=env, check=False,
                       capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout)
        print(r.stderr, file=sys.stderr)
        raise RuntimeError(f'{log_prefix} failed ({r.returncode})')


def evo_ape(env, venv_bin, gt, est, out_zip, pose_relation=None, align=True):
    cmd = [f'{venv_bin}/evo_ape', 'tum']
    if align:
        cmd.append(EVO_ALIGN_FLAG)
    if pose_relation:
        cmd.extend(['--pose_relation', pose_relation])
    cmd += [gt, est, '--save_results', out_zip, '--no_warnings']
    run_evo(cmd, env, 'evo_ape')


def evo_rpe(env, venv_bin, gt, est, out_zip, delta=10, unit='f', align=True):
    cmd = [f'{venv_bin}/evo_rpe', 'tum',
           '-r', 'trans_part',
           '-d', str(delta), '-u', unit]
    if align:
        cmd.append(EVO_ALIGN_FLAG)
    cmd += [gt, est, '--save_results', out_zip, '--no_warnings']
    run_evo(cmd, env, 'evo_rpe')


def read_evo_zip(zip_path):
    """Return dict with 'stats' (from stats.json) and 'alignment' (4x4 or None)."""
    out = {'stats': None, 'alignment': None}
    with zipfile.ZipFile(zip_path) as z:
        names = z.namelist()
        if 'stats.json' in names:
            with z.open('stats.json') as f:
                out['stats'] = json.load(f)
        if 'alignment_transformation_sim3.npy' in names:
            with z.open('alignment_transformation_sim3.npy') as f:
                out['alignment'] = np.load(f)
    return out


# ------------------------------------------------------------ alignment

def decompose_sim3(T):
    M = T[:3, :3]
    t = T[:3, 3]
    s = float(np.cbrt(np.linalg.det(M)))
    return M / s, t, s


def apply_sim3_to_tum(data, T):
    rot, t, s = decompose_sim3(T)
    out = data.copy()
    out[:, 1:4] = s * (data[:, 1:4] @ rot.T) + t
    out[:, 4:8] = (R.from_matrix(rot) * R.from_quat(data[:, 4:8])).as_quat()
    return out


def _nearest_match(t_ref, t_query):
    """For each t_query, return index into t_ref of the closest sample."""
    j = np.searchsorted(t_ref, t_query)
    j = np.clip(j, 1, len(t_ref) - 1)
    j_prev = j - 1
    use_prev = np.abs(t_ref[j_prev] - t_query) < np.abs(t_ref[j] - t_query)
    j[use_prev] = j_prev[use_prev]
    return j


def estimate_body_rotation_offset(gt, est, max_dt=0.02):
    """Constant right-multiplied rotation that makes est best-match gt
    after evo's position-only alignment."""
    j = _nearest_match(gt[:, 0], est[:, 0])
    mask = np.abs(gt[j, 0] - est[:, 0]) <= max_dt
    if mask.sum() < 5:
        raise RuntimeError(f'not enough matched poses ({mask.sum()})')
    r_gt = R.from_quat(gt[j[mask], 4:8])
    r_est = R.from_quat(est[mask, 4:8])
    return (r_est.inv() * r_gt).mean()


def apply_right_rotation_to_tum(data, r_right):
    out = data.copy()
    out[:, 4:8] = (R.from_quat(data[:, 4:8]) * r_right).as_quat()
    return out


def shift_to_gt_origin(gt, est_list):
    """Apply GT[0] -> origin/identity to gt and each estimator (same transform
    on all, so alignment is preserved). Used only for the 6-DOF plot."""
    p0 = gt[0, 1:4]
    q0 = gt[0, 4:8]
    r0_inv = R.from_quat(q0).inv()
    p0_inv = -r0_inv.apply(p0)

    def shift(d):
        out = d.copy()
        out[:, 1:4] = r0_inv.apply(d[:, 1:4]) + p0_inv
        out[:, 4:8] = (r0_inv * R.from_quat(d[:, 4:8])).as_quat()
        return out

    return shift(gt), [shift(d) for d in est_list]


# ------------------------------------------------------------- metrics

def per_axis_drift(gt, est, max_dt=0.05):
    """Per-axis Euler error stats + linear-trend drift (deg / 1000s).
    Computed in the GT[0]-relative frame so ranges match the 6-DOF plot."""
    j = _nearest_match(gt[:, 0], est[:, 0])
    mask = np.abs(gt[j, 0] - est[:, 0]) <= max_dt
    r0_inv = R.from_quat(gt[0, 4:8]).inv()
    e_gt = (r0_inv * R.from_quat(gt[j[mask], 4:8])).as_euler('xyz', degrees=True)
    e_est = (r0_inv * R.from_quat(est[mask, 4:8])).as_euler('xyz', degrees=True)
    diff = (e_est - e_gt + 180) % 360 - 180
    t_rel = est[mask, 0] - est[mask, 0][0]
    out = {}
    for k, ax in enumerate(['Roll', 'Pitch', 'Yaw']):
        slope = float(np.polyfit(t_rel, diff[:, k], 1)[0])
        out[ax] = {
            'gt_range_deg':      float(e_gt[:, k].max() - e_gt[:, k].min()),
            'est_range_deg':     float(e_est[:, k].max() - e_est[:, k].min()),
            'err_rmse_deg':      float(np.sqrt(np.mean(diff[:, k] ** 2))),
            'err_mean_deg':      float(np.mean(diff[:, k])),
            'err_std_deg':       float(np.std(diff[:, k])),
            'drift_deg_per_1000s': float(slope * 1000.0),
            'accum_drift_deg':   float(slope * t_rel[-1]),
        }
    return out


def compute_ape_arrays(gt, est, max_dt=0.05):
    """Per-sample APE arrays for downstream cross-experiment plots
    (long-duration drift, H2 degradation panel, error-distribution box plots).

    Computed on the *aligned* trajectory. Per-axis rotation error uses the
    GT[0]-relative frame so it matches what `per_axis_drift` and the 6-DOF
    plot report. The total rotation error is axis-angle magnitude of
    R_gt^-1 * R_est (frame-invariant).

    Returns dict of numpy arrays:
        t           — time (s) relative to GT[0] (matches plot x-axis)
        t_abs       — absolute timestamps
        err_trans_m — Euclidean translation error per sample (m)
        err_rot_deg — total rotation error per sample (deg, axis-angle)
        err_xyz_m   — Nx3 per-axis translation error (m)
        err_rpy_deg — Nx3 per-axis Euler error in GT[0] frame (deg)
    """
    j = _nearest_match(gt[:, 0], est[:, 0])
    mask = np.abs(gt[j, 0] - est[:, 0]) <= max_dt

    t_abs = est[mask, 0]
    t_rel = t_abs - gt[0, 0]

    p_gt = gt[j[mask], 1:4]
    p_est = est[mask, 1:4]
    err_xyz = p_est - p_gt
    err_trans = np.linalg.norm(err_xyz, axis=1)

    r_gt = R.from_quat(gt[j[mask], 4:8])
    r_est = R.from_quat(est[mask, 4:8])

    err_rot = np.degrees((r_gt.inv() * r_est).magnitude())

    r0_inv = R.from_quat(gt[0, 4:8]).inv()
    e_gt = (r0_inv * r_gt).as_euler('xyz', degrees=True)
    e_est = (r0_inv * r_est).as_euler('xyz', degrees=True)
    err_rpy = (e_est - e_gt + 180) % 360 - 180

    return {
        't':           t_rel,
        't_abs':       t_abs,
        'err_trans_m': err_trans,
        'err_rot_deg': err_rot,
        'err_xyz_m':   err_xyz,
        'err_rpy_deg': err_rpy,
    }


def per_dof_translation_error(gt, est, max_dt=0.05):
    j = _nearest_match(gt[:, 0], est[:, 0])
    mask = np.abs(gt[j, 0] - est[:, 0]) <= max_dt
    p_gt = gt[j[mask], 1:4]
    p_est = est[mask, 1:4]
    diff = p_est - p_gt
    norm = np.linalg.norm(diff, axis=1)
    out = {'ATE_m': {
        'rmse': float(np.sqrt(np.mean(norm ** 2))),
        'mean': float(np.mean(norm)),
        'median': float(np.median(norm)),
        'std': float(np.std(norm)),
        'min': float(np.min(norm)),
        'max': float(np.max(norm)),
    }}
    for k, ax in enumerate(['X', 'Y', 'Z']):
        out[ax] = {
            'rmse': float(np.sqrt(np.mean(diff[:, k] ** 2))),
            'mean': float(np.mean(diff[:, k])),
            'std':  float(np.std(diff[:, k])),
        }
    return out


# --------------------------------------------------------------- plots

def plot_6dof_relative(gt_data, estimators_data, output_dir, t_ref=None):
    """6-DOF comparison where every trace is shown as deviation from its own
    first pose, so all traces start at y=0 at t=0. This makes the rotational
    and translational *dynamics* directly comparable; the residual offset at
    t=0 (already known from the body-frame correction step) is suppressed."""
    fig, axs = plt.subplots(6, 1, figsize=(12, 18), sharex=True)

    def extract(data):
        p = data[:, 1:4] - data[0, 1:4]
        e = R.from_quat(data[:, 4:8]).as_euler('xyz', degrees=True)
        e = e - e[0]
        return p, e

    if t_ref is None:
        t_ref = gt_data[0, 0]

    labels = ['X [m]', 'Y [m]', 'Z [m]', 'Roll [deg]', 'Pitch [deg]', 'Yaw [deg]']

    p_gt, e_gt = extract(gt_data)
    gt_vals = [p_gt[:, 0], p_gt[:, 1], p_gt[:, 2],
               e_gt[:, 0], e_gt[:, 1], e_gt[:, 2]]
    t_gt_rel = gt_data[:, 0] - t_ref

    for i in range(6):
        axs[i].axhline(0, color='gray', linewidth=0.5, alpha=0.4, zorder=0)
        axs[i].plot(t_gt_rel, gt_vals[i], 'k-', label='Ground Truth',
                    linewidth=2)
        axs[i].set_ylabel(labels[i])
        axs[i].grid(True, linestyle='--', alpha=0.6)

    for est in estimators_data:
        p_e, e_e = extract(est['data'])
        est_vals = [p_e[:, 0], p_e[:, 1], p_e[:, 2],
                    e_e[:, 0], e_e[:, 1], e_e[:, 2]]
        t_e_rel = est['data'][:, 0] - t_ref
        for i in range(6):
            axs[i].plot(t_e_rel, est_vals[i], label=est['name'],
                        color=est.get('color'), linewidth=1.5, alpha=0.85)

    axs[5].set_xlabel('Time [s]')
    axs[0].legend(loc='upper right', bbox_to_anchor=(1.0, 1.25), ncol=3)
    plt.suptitle('6-DOF Trajectory Comparison (relative to initial pose)',
                 fontsize=16, y=0.96)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])

    os.makedirs(output_dir, exist_ok=True)
    fig.savefig(os.path.join(output_dir, '6dof_comparison.png'), dpi=300)
    fig.savefig(os.path.join(output_dir, '6dof_comparison.pdf'))
    plt.close(fig)


def plot_ape_over_time(gt, est, display_name, color, out_dir, max_dt=0.05):
    j = _nearest_match(gt[:, 0], est[:, 0])
    mask = np.abs(gt[j, 0] - est[:, 0]) <= max_dt
    err = np.linalg.norm(est[mask, 1:4] - gt[j[mask], 1:4], axis=1)
    t = est[mask, 0] - gt[0, 0]

    rmse = float(np.sqrt(np.mean(err ** 2)))
    mean = float(np.mean(err))
    median = float(np.median(err))
    std = float(np.std(err))

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(t, err, color=color, alpha=0.75, linewidth=1.4,
            label=f'{display_name} APE')
    ax.axhline(mean,   color='r', linestyle='--', label=f'Mean: {mean:.4f} m')
    ax.axhline(median, color='g', linestyle='-.', label=f'Median: {median:.4f} m')
    ax.axhline(rmse,   color='b', linestyle=':',  label=f'RMSE: {rmse:.4f} m')
    ax.fill_between(t, np.maximum(0, mean - std), mean + std,
                    color='r', alpha=0.15, label=f'Std: ±{std:.4f} m')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Absolute Translation Error [m]')
    ax.set_title(f'APE (translation) over time — {display_name}')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='upper right', bbox_to_anchor=(1.25, 1.0))
    plt.tight_layout()

    base = display_name.lower().replace(' ', '_')
    fig.savefig(os.path.join(out_dir, f'{base}_ape_error.png'), dpi=300,
                bbox_inches='tight')
    fig.savefig(os.path.join(out_dir, f'{base}_ape_error.pdf'),
                bbox_inches='tight')
    plt.close(fig)


# --------------------------------------------------------------- main

def fmt_initial_R(d, label, out_lines):
    r0_inv = R.from_quat(d[0, 4:8]).inv()
    out_lines.append(f'[{label}] Initial Rotation Matrix (World to Pose0):')
    for row in r0_inv.as_matrix():
        out_lines.append('  [{:+.6e} {:+.6e} {:+.6e}]'.format(*row))
    out_lines.append('-' * 40)


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument('bag_dir')
    p.add_argument('--crop', nargs=2, type=float, default=[0.0, 0.0],
                   metavar=('START_S', 'END_S'),
                   help='seconds dropped from start and end of GT time range')
    p.add_argument('--gt-file', default='gt.tum')
    p.add_argument('--estimators', nargs='+', default=['mimosa', 'mavros'])
    p.add_argument('--evo-venv', default='~/evo')
    p.add_argument('--rpe-delta', type=float, default=10.0,
                   help='evo_rpe -d argument (default 10)')
    p.add_argument('--rpe-unit', default='f', choices=['f', 'd', 'r', 'm'],
                   help='evo_rpe -u argument (f=frames, d=deg, r=rad, m=meters; default f)')
    p.add_argument('--output-dir', default=None,
                   help='default: <bag_dir>/evaluation')
    args = p.parse_args()

    bag = args.bag_dir
    out = args.output_dir or os.path.join(bag, 'evaluation')
    dirs = {k: os.path.join(out, k) for k in ('cropped', 'aligned', 'evo', 'plots')}
    for d in dirs.values():
        os.makedirs(d, exist_ok=True)

    env, venv_bin = evo_env(args.evo_venv)

    log = []
    def say(*a):
        msg = ' '.join(str(x) for x in a)
        print(msg)
        log.append(msg)

    # === 1. Load + crop =========================================
    gt_path = os.path.join(bag, args.gt_file)
    say(f'Loading GT: {gt_path}')
    gt_raw = load_tum(gt_path)
    say(f'  -> {len(gt_raw)} poses, range {gt_raw[0,0]:.3f} .. {gt_raw[-1,0]:.3f}'
        f' ({gt_raw[-1,0]-gt_raw[0,0]:.3f}s)')

    cs, ce = args.crop
    t_lo, t_hi = gt_raw[0, 0] + cs, gt_raw[-1, 0] - ce
    say(f'Crop window: [+{cs}s, -{ce}s]  ->  [{t_lo:.3f}, {t_hi:.3f}]'
        f' ({t_hi - t_lo:.3f}s)')

    gt = crop_tum(gt_raw, t_lo, t_hi)
    gt_c_path = os.path.join(dirs['cropped'], 'gt.tum')
    save_tum(gt, gt_c_path)
    fmt_initial_R(gt, 'Ground Truth', log)

    estimators = {}
    for name in args.estimators:
        raw_path = os.path.join(bag, f'{name}.tum')
        if not os.path.exists(raw_path):
            say(f'[skip] {name}: {raw_path} not found')
            continue

        cfg = ESTIMATOR_CFG.get(name, {'display': name, 'color': None})
        say(f'\nLoading Estimator: {raw_path}')
        d_raw = load_tum(raw_path)
        say(f'  -> {len(d_raw)} poses, range {d_raw[0,0]:.3f} .. {d_raw[-1,0]:.3f}'
            f' ({d_raw[-1,0]-d_raw[0,0]:.3f}s)')
        say(f'  -> Offset from GT start: {d_raw[0,0] - gt_raw[0,0]:.3f}s')

        d = crop_tum(d_raw, t_lo, t_hi)
        if len(d) < 10:
            say(f'  [skip] {cfg["display"]}: only {len(d)} poses after crop')
            continue
        say(f'  -> after crop: {len(d)} poses')

        cropped_path = os.path.join(dirs['cropped'], f'{name}.tum')
        save_tum(d, cropped_path)
        fmt_initial_R(d, cfg['display'], log)

        estimators[name] = {
            'display': cfg['display'],
            'color':   cfg['color'],
            'cropped_path': cropped_path,
            'cropped':      d,
        }

    if not estimators:
        sys.exit('[error] no estimators loaded.')

    # === 2. Run evo (with -a) on cropped to get alignment + APE ==========
    say(f'\n=== Running evo (with -a) on cropped trajectories ===')
    for name, info in estimators.items():
        zdir = os.path.join(dirs['evo'], name + '_cropped')
        os.makedirs(zdir, exist_ok=True)
        info['evo_cropped_zips'] = {
            'ape_default': os.path.join(zdir, 'ape_default.zip'),
            'ape_trans':   os.path.join(zdir, 'ape_trans.zip'),
            'ape_angle':   os.path.join(zdir, 'ape_angle.zip'),
            'rpe_trans':   os.path.join(zdir, 'rpe_trans.zip'),
        }
        evo_ape(env, venv_bin, gt_c_path, info['cropped_path'],
                info['evo_cropped_zips']['ape_default'])
        evo_ape(env, venv_bin, gt_c_path, info['cropped_path'],
                info['evo_cropped_zips']['ape_trans'], pose_relation='trans_part')
        evo_ape(env, venv_bin, gt_c_path, info['cropped_path'],
                info['evo_cropped_zips']['ape_angle'], pose_relation='angle_deg')
        try:
            evo_rpe(env, venv_bin, gt_c_path, info['cropped_path'],
                    info['evo_cropped_zips']['rpe_trans'],
                    delta=args.rpe_delta, unit=args.rpe_unit)
        except RuntimeError as e:
            say(f'  [warn] RPE on cropped failed: {e}')
            info['evo_cropped_zips'].pop('rpe_trans', None)

    # === 3. Apply alignment + body offset, save aligned TUMs =============
    say(f'\n=== Applying alignment ===')
    for name, info in estimators.items():
        rz = read_evo_zip(info['evo_cropped_zips']['ape_default'])
        T = rz['alignment']
        rot, t_align, s = decompose_sim3(T)
        say(f'  [{info["display"]}] evo Sim(3): scale={s:.6f}  |t|={np.linalg.norm(t_align):.4f} m')

        aligned = apply_sim3_to_tum(info['cropped'], T)
        try:
            r_body = estimate_body_rotation_offset(gt, aligned)
            bo = r_body.as_euler('xyz', degrees=True)
            say(f'    body-frame offset (xyz deg): '
                f'roll={bo[0]:+.2f}  pitch={bo[1]:+.2f}  yaw={bo[2]:+.2f}')
            aligned = apply_right_rotation_to_tum(aligned, r_body)
            info['body_offset_deg'] = bo.tolist()
        except RuntimeError as e:
            say(f'    [warn] body-frame correction skipped: {e}')
            info['body_offset_deg'] = None

        ap = os.path.join(dirs['aligned'], f'{name}_aligned.tum')
        save_tum(aligned, ap)
        info['aligned_path'] = ap
        info['aligned'] = aligned
        info['evo_alignment_T'] = T.tolist()
        info['evo_alignment_scale'] = s

    # === 4. Re-run evo (no -a) on aligned for post-alignment metrics =====
    say(f'\n=== Running evo (no -a) on aligned trajectories ===')
    for name, info in estimators.items():
        zdir = os.path.join(dirs['evo'], name + '_aligned')
        os.makedirs(zdir, exist_ok=True)
        info['evo_aligned_zips'] = {
            'ape_trans':   os.path.join(zdir, 'ape_trans.zip'),
            'ape_angle':   os.path.join(zdir, 'ape_angle.zip'),
            'rpe_trans':   os.path.join(zdir, 'rpe_trans.zip'),
        }
        evo_ape(env, venv_bin, gt_c_path, info['aligned_path'],
                info['evo_aligned_zips']['ape_trans'],
                pose_relation='trans_part', align=False)
        evo_ape(env, venv_bin, gt_c_path, info['aligned_path'],
                info['evo_aligned_zips']['ape_angle'],
                pose_relation='angle_deg', align=False)
        try:
            evo_rpe(env, venv_bin, gt_c_path, info['aligned_path'],
                    info['evo_aligned_zips']['rpe_trans'],
                    delta=args.rpe_delta, unit=args.rpe_unit, align=False)
        except RuntimeError as e:
            say(f'  [warn] RPE on aligned failed: {e}')
            info['evo_aligned_zips'].pop('rpe_trans', None)

    # === 5. Compute metrics ==============================================
    for name, info in estimators.items():
        info['evo_metrics'] = {
            'ape_trans_m':   read_evo_zip(info['evo_aligned_zips']['ape_trans'])['stats'],
            'ape_angle_deg': read_evo_zip(info['evo_aligned_zips']['ape_angle'])['stats'],
        }
        if 'rpe_trans' in info['evo_aligned_zips']:
            info['evo_metrics']['rpe_trans_m'] = read_evo_zip(
                info['evo_aligned_zips']['rpe_trans'])['stats']
        info['per_axis_rotation'] = per_axis_drift(gt, info['aligned'])
        info['per_dof_translation'] = per_dof_translation_error(gt, info['aligned'])

        arrays = compute_ape_arrays(gt, info['aligned'])
        npz_path = os.path.join(out, f'{name}_ape.npz')
        np.savez_compressed(npz_path, **arrays)
        info['ape_arrays_path'] = npz_path
        info['n_ape_samples'] = int(arrays['t'].size)

    # === 6. Plots ========================================================
    say(f'\n=== Generating plots ===')
    gt_plot, est_plots_data = shift_to_gt_origin(
        gt, [info['aligned'] for info in estimators.values()]
    )
    est_for_plot = []
    for (name, info), d in zip(estimators.items(), est_plots_data):
        est_for_plot.append({
            'name':  info['display'],
            'data':  d,
            'color': info['color'],
        })
    plot_6dof_relative(gt_plot, est_for_plot, dirs['plots'])

    for name, info in estimators.items():
        plot_ape_over_time(gt, info['aligned'], info['display'],
                           info['color'], dirs['plots'])
    say(f'  plots written to {dirs["plots"]}')

    # === 7. Summary + metrics ============================================
    say(f'\n========================================')
    say('EVALUATION METRICS SUMMARY')
    say('========================================')
    for name, info in estimators.items():
        d = info['display']
        say(f'\n--- {d} ---')
        ape_t = info['evo_metrics'].get('ape_trans_m')
        ape_a = info['evo_metrics'].get('ape_angle_deg')
        rpe_t = info['evo_metrics'].get('rpe_trans_m')
        if info['body_offset_deg']:
            bo = info['body_offset_deg']
            say(f'Body-frame offset (deg): R={bo[0]:+.2f}  P={bo[1]:+.2f}  Y={bo[2]:+.2f}')
        if ape_t:
            say('APE — translation (m) [after Sim(3) + body align]:')
            say(f"  RMSE:   {ape_t['rmse']:.4f}")
            say(f"  Mean:   {ape_t['mean']:.4f}")
            say(f"  Median: {ape_t['median']:.4f}")
            say(f"  Std:    {ape_t['std']:.4f}")
            say(f"  Max:    {ape_t['max']:.4f}")
        if ape_a:
            say('APE — rotation (deg):')
            say(f"  RMSE:   {ape_a['rmse']:.4f}")
            say(f"  Mean:   {ape_a['mean']:.4f}")
            say(f"  Median: {ape_a['median']:.4f}")
            say(f"  Std:    {ape_a['std']:.4f}")
            say(f"  Max:    {ape_a['max']:.4f}")
        if rpe_t:
            say(f"RPE — translation (m, delta={args.rpe_delta}{args.rpe_unit}):")
            say(f"  RMSE:   {rpe_t['rmse']:.4f}")
            say(f"  Mean:   {rpe_t['mean']:.4f}")
            say(f"  Std:    {rpe_t['std']:.4f}")

        say('\nPer-DOF translation error (m):')
        tdof = info['per_dof_translation']
        for ax in ['X', 'Y', 'Z']:
            v = tdof[ax]
            say(f"  {ax}:  RMSE {v['rmse']:.4f}  Mean {v['mean']:+.4f}  Std {v['std']:.4f}")

        say('\nPer-axis rotation error + linear drift (deg):')
        say(f"  {'Axis':6s}{'GTrng':>8}{'ERng':>8}{'RMSE':>8}{'Mean':>8}{'Std':>8}{'Drift/1000s':>14}{'AccumDrift':>13}")
        for ax in ['Roll', 'Pitch', 'Yaw']:
            v = info['per_axis_rotation'][ax]
            say(f"  {ax:6s}{v['gt_range_deg']:8.2f}{v['est_range_deg']:8.2f}"
                f"{v['err_rmse_deg']:8.2f}{v['err_mean_deg']:+8.2f}{v['err_std_deg']:8.2f}"
                f"{v['drift_deg_per_1000s']:+14.2f}{v['accum_drift_deg']:+13.2f}")

    summary_path = os.path.join(out, 'summary.txt')
    with open(summary_path, 'w') as f:
        f.write('\n'.join(log) + '\n')
    say(f'\nSaved summary -> {summary_path}')

    metrics = {
        'bag': bag,
        'crop_s': {'start': cs, 'end': ce},
        'gt_duration_s': float(gt[-1, 0] - gt[0, 0]),
        'gt_poses': int(len(gt)),
        'estimators': {
            info['display']: {
                'source_file':  f'{name}.tum',
                'color':        info['color'],
                'aligned_path': info['aligned_path'],
                'ape_arrays_path':     info['ape_arrays_path'],
                'n_ape_samples':       info['n_ape_samples'],
                'body_offset_deg': info['body_offset_deg'],
                'evo_alignment_T': info['evo_alignment_T'],
                'evo_alignment_scale': info['evo_alignment_scale'],
                'evo_metrics': info['evo_metrics'],
                'per_dof_translation': info['per_dof_translation'],
                'per_axis_rotation':   info['per_axis_rotation'],
            }
            for name, info in estimators.items()
        }
    }
    metrics_path = os.path.join(out, 'metrics.json')
    with open(metrics_path, 'w') as f:
        json.dump(metrics, f, indent=2)
    print(f'Saved metrics -> {metrics_path}')
    print(f'\n[done] all output in {out}')


if __name__ == '__main__':
    main()
