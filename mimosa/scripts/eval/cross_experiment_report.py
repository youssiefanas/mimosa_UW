#!/usr/bin/env python3
"""Cross-experiment thesis figures (Tier 1).

Reads <bag>/evaluation/metrics.json + <bag>/evaluation/<name>_ape.npz +
<bag>/evaluation/cropped/gt.tum for each experiment and emits:

  1. master_table.{csv,md}       — one row per experiment (MUSE-K vs EKF
                                    estimation accuracy + duration + max).
  2. drift_table_6dof.{csv,md}   — per-axis (X/Y/Z, Roll/Pitch/Yaw) linear-
                                    fit drift rate per 1000 s, both
                                    estimators, every experiment.
  3. bar_chart_ape_rmse.{png,pdf}— grouped bars: translation + rotation
                                    APE RMSE across all experiments.
  4. rmse_std_chart.{png,pdf}    — same bars as (3) but with ±std error
                                    bars on top of each RMSE bar.
  5. box_errors.{png,pdf}        — box-and-whisker of per-sample translation
                                    and rotation APE per experiment, both
                                    estimators side-by-side. Reveals
                                    distribution shape + outliers that
                                    RMSE/std summary stats hide.
  6. degradation_panel.{png,pdf} — APE-translation over time, one panel per
                                    degraded-sensor experiment, both
                                    estimators overlaid. The "H2" figure.
  7. long_duration_drift.{png,pdf}— per-axis Euler error over time for the
                                    long nominal run. Linear trend lines.
  8. station_keeping_cdf.{png,pdf}— horizontal deviation of GT from setpoint
                                    (GT[0]) for the two nominal runs.
                                    Controller comparison.

Experiment list is hardcoded in EXPERIMENTS below — edit there to add bags
or change the labelling.

Usage:
    cross_experiment_report.py [--base-dir DIR] [--output-dir DIR]
"""

import argparse
import csv
import json
import os
from collections import OrderedDict

import numpy as np
import matplotlib.pyplot as plt

# ----------------------------------------------------------- configuration

EKF_NAME, EKF_COLOR = 'EKF',    '#1f77b4'
OURS_NAME, OURS_COLOR = 'MUSE-K', '#ff7f0e'
GT_COLOR = 'black'

EXPERIMENTS = [
    {
        'bag': 'run_20260519_142201',
        'name': 'Nominal — ArduSub ctrl',
        'short': 'nominal_ekf_ctrl',
        'condition': 'nominal',
        'controller': 'ArduSub (EKF)',
    },
    {
        'bag': 'run_20260519_144629',
        'name': 'Nominal — MUSE-K ctrl',
        'short': 'nominal_musek_ctrl',
        'condition': 'nominal',
        'controller': 'MUSE-K',
    },
    {
        'bag': 'run_20260519_155540',
        'name': 'Camera occlusion',
        'short': 'occlusion',
        'condition': 'visual_occlusion',
        'controller': 'MUSE-K',
    },
    {
        'bag': 'run_20260519_164139',
        'name': 'Reduced visual features',
        'short': 'featureless',
        'condition': 'featureless',
        'controller': 'MUSE-K',
    },
    {
        'bag': 'run_20260519_161449',
        'name': 'DVL disturbance',
        'short': 'dvl_loss',
        'condition': 'dvl_loss',
        'controller': 'MUSE-K',
    },
    {
        'bag': 'run_20260519_160052',
        'name': 'External disturbance (stick)',
        'short': 'push_disturbance',
        'condition': 'disturbance',
        'controller': 'MUSE-K',
    },
]
DEGRADATION_CONDITIONS = {'visual_occlusion', 'featureless',
                          'dvl_loss', 'disturbance'}


# ----------------------------------------------------------- loading

def load_experiment(exp, base_dir):
    """Attach paths + parsed metrics/npz to each experiment dict."""
    exp['bag_path']     = os.path.join(base_dir, exp['bag'])
    exp['eval_dir']     = os.path.join(exp['bag_path'], 'evaluation')
    exp['metrics']      = json.load(open(os.path.join(exp['eval_dir'], 'metrics.json')))
    exp['gt_duration_s'] = exp['metrics']['gt_duration_s']
    exp['gt_path']      = os.path.join(exp['eval_dir'], 'cropped', 'gt.tum')
    est = exp['metrics'].get('estimators', {})
    exp['npz'] = {}
    for display in (OURS_NAME, EKF_NAME):
        info = est.get(display, {})
        path = info.get('ape_arrays_path')
        if path and os.path.exists(path):
            exp['npz'][display] = np.load(path)
    return exp


def get_rmse(exp, display, kind):
    """kind in {'ape_trans_m','ape_angle_deg','rpe_trans_m'}; return rmse or NaN."""
    s = (exp['metrics']['estimators']
              .get(display, {})
              .get('evo_metrics', {})
              .get(kind) or {})
    return float(s.get('rmse', float('nan')))


def get_max(exp, display, kind):
    s = (exp['metrics']['estimators']
              .get(display, {})
              .get('evo_metrics', {})
              .get(kind) or {})
    return float(s.get('max', float('nan')))


def get_std(exp, display, kind):
    s = (exp['metrics']['estimators']
              .get(display, {})
              .get('evo_metrics', {})
              .get(kind) or {})
    return float(s.get('std', float('nan')))


def compute_6dof_drift(npz_data):
    """Linear-fit slope of per-axis error vs time, scaled to per-1000s.
    Returns dict with X/Y/Z (m/1000s) and Roll/Pitch/Yaw (deg/1000s)."""
    t = npz_data['t']
    err_xyz = npz_data['err_xyz_m']
    err_rpy = npz_data['err_rpy_deg']
    out = {}
    for k, ax in enumerate(['X', 'Y', 'Z']):
        out[ax] = float(np.polyfit(t, err_xyz[:, k], 1)[0]) * 1000.0
    for k, ax in enumerate(['Roll', 'Pitch', 'Yaw']):
        out[ax] = float(np.polyfit(t, err_rpy[:, k], 1)[0]) * 1000.0
    return out


# ------------------------------------------------------ master table

def build_master_table(experiments, out_dir):
    cols = ['Experiment', 'Condition', 'Controller', 'Duration [s]',
            'MUSE-K ATE RMSE [m]', 'EKF ATE RMSE [m]',
            'MUSE-K APE-rot RMSE [deg]', 'EKF APE-rot RMSE [deg]',
            'MUSE-K ATE max [m]', 'EKF ATE max [m]']
    rows = []
    for exp in experiments:
        rows.append([
            exp['name'], exp['condition'], exp['controller'],
            f"{exp['gt_duration_s']:.1f}",
            f"{get_rmse(exp, OURS_NAME, 'ape_trans_m'):.4f}",
            f"{get_rmse(exp, EKF_NAME,  'ape_trans_m'):.4f}",
            f"{get_rmse(exp, OURS_NAME, 'ape_angle_deg'):.3f}",
            f"{get_rmse(exp, EKF_NAME,  'ape_angle_deg'):.3f}",
            f"{get_max (exp, OURS_NAME, 'ape_trans_m'):.4f}",
            f"{get_max (exp, EKF_NAME,  'ape_trans_m'):.4f}",
        ])

    csv_path = os.path.join(out_dir, 'master_table.csv')
    with open(csv_path, 'w', newline='') as f:
        w = csv.writer(f); w.writerow(cols); w.writerows(rows)

    md_path = os.path.join(out_dir, 'master_table.md')
    with open(md_path, 'w') as f:
        f.write('| ' + ' | '.join(cols) + ' |\n')
        f.write('|' + '|'.join(['---'] * len(cols)) + '|\n')
        for r in rows:
            f.write('| ' + ' | '.join(r) + ' |\n')

    print(f'  wrote {csv_path}')
    print(f'  wrote {md_path}')

    print('\n=== Master table ===')
    widths = [max(len(c), max(len(r[i]) for r in rows)) for i, c in enumerate(cols)]
    print('  '.join(c.ljust(w) for c, w in zip(cols, widths)))
    print('  '.join('-' * w for w in widths))
    for r in rows:
        print('  '.join(c.ljust(w) for c, w in zip(r, widths)))


# ------------------------------------------------------ drift table

def build_drift_table(experiments, out_dir):
    """Linear-trend drift rate of per-axis error over each cropped run,
    scaled to per-1000s. Two rows per experiment (MUSE-K, then EKF).
    Translation drift in m/1000s, rotation drift in deg/1000s.
    Slopes come from the per-sample arrays in <name>_ape.npz."""
    for exp in experiments:
        exp['drifts'] = {}
        for display, d in exp['npz'].items():
            exp['drifts'][display] = compute_6dof_drift(d)

    cols = ['Experiment', 'Dur [s]', 'Estimator',
            'X [m/1000s]', 'Y [m/1000s]', 'Z [m/1000s]',
            'Roll [deg/1000s]', 'Pitch [deg/1000s]', 'Yaw [deg/1000s]']
    rows = []
    for exp in experiments:
        for est in (OURS_NAME, EKF_NAME):
            drifts = exp['drifts'].get(est, {})
            row = [exp['name'], f'{exp["gt_duration_s"]:.0f}', est]
            for ax, fmt in [('X', '{:+.4f}'), ('Y', '{:+.4f}'), ('Z', '{:+.4f}'),
                            ('Roll', '{:+.3f}'), ('Pitch', '{:+.3f}'),
                            ('Yaw', '{:+.3f}')]:
                v = drifts.get(ax, float('nan'))
                row.append(fmt.format(v) if np.isfinite(v) else 'nan')
            rows.append(row)

    csv_path = os.path.join(out_dir, 'drift_table_6dof.csv')
    with open(csv_path, 'w', newline='') as f:
        w = csv.writer(f); w.writerow(cols); w.writerows(rows)

    md_path = os.path.join(out_dir, 'drift_table_6dof.md')
    with open(md_path, 'w') as f:
        f.write('| ' + ' | '.join(cols) + ' |\n')
        f.write('|' + '|'.join(['---'] * len(cols)) + '|\n')
        for r in rows:
            f.write('| ' + ' | '.join(r) + ' |\n')

    print(f'  wrote {csv_path}')
    print(f'  wrote {md_path}')

    print('\n=== 6-DoF drift rate (linear fit, /1000s) ===')
    widths = [max(len(c), max(len(r[i]) for r in rows)) for i, c in enumerate(cols)]
    print('  '.join(c.ljust(w) for c, w in zip(cols, widths)))
    print('  '.join('-' * w for w in widths))
    for r in rows:
        print('  '.join(c.ljust(w) for c, w in zip(r, widths)))


# ------------------------------------------------------------ bar chart

def plot_bar_chart(experiments, out_dir):
    labels  = [e['name'] for e in experiments]
    ekf_t   = [get_rmse(e, EKF_NAME,  'ape_trans_m')   for e in experiments]
    ours_t  = [get_rmse(e, OURS_NAME, 'ape_trans_m')   for e in experiments]
    ekf_a   = [get_rmse(e, EKF_NAME,  'ape_angle_deg') for e in experiments]
    ours_a  = [get_rmse(e, OURS_NAME, 'ape_angle_deg') for e in experiments]
    durs    = [e['gt_duration_s'] for e in experiments]
    xlabels = [f'{l}\n({d:.0f} s)' for l, d in zip(labels, durs)]

    x = np.arange(len(labels))
    w = 0.36
    fig, (ax_t, ax_a) = plt.subplots(
        2, 1, figsize=(max(7.5, 1.5 * len(labels)), 8), sharex=True,
        gridspec_kw={'height_ratios': [1, 1], 'hspace': 0.18},
    )

    def _draw(ax, ekf_vals, ours_vals, ylabel, fmt):
        b1 = ax.bar(x - w / 2, ekf_vals,  w, label=EKF_NAME,
                    color=EKF_COLOR,  edgecolor='black', linewidth=0.5)
        b2 = ax.bar(x + w / 2, ours_vals, w, label=OURS_NAME,
                    color=OURS_COLOR, edgecolor='black', linewidth=0.5)
        ax.set_ylabel(ylabel, fontsize=11)
        ax.yaxis.grid(True, linestyle='--', alpha=0.45)
        ax.set_axisbelow(True)
        for sp in ('top', 'right'):
            ax.spines[sp].set_visible(False)
        ymax = np.nanmax(ekf_vals + ours_vals)
        ax.set_ylim(0, ymax * 1.18)
        for bars in (b1, b2):
            for bar in bars:
                h = bar.get_height()
                if np.isfinite(h):
                    ax.annotate(fmt.format(h),
                                xy=(bar.get_x() + bar.get_width() / 2, h),
                                xytext=(0, 3), textcoords='offset points',
                                ha='center', va='bottom', fontsize=8)
        return b1, b2

    b1, b2 = _draw(ax_t, ekf_t, ours_t, 'Translation APE RMSE [m]', '{:.3f}')
    _draw(ax_a, ekf_a, ours_a, 'Rotation APE RMSE [deg]', '{:.2f}')

    fig.suptitle(f'Estimation accuracy: {OURS_NAME} vs. {EKF_NAME} (APE RMSE)',
                 fontsize=13, fontweight='bold', y=0.995)
    fig.legend(handles=[b1, b2], loc='upper center',
               bbox_to_anchor=(0.5, 0.955),
               frameon=False, fontsize=10, ncol=2)
    ax_a.set_xticks(x)
    ax_a.set_xticklabels(xlabels, fontsize=9, rotation=15, ha='right')
    fig.tight_layout(rect=(0, 0, 1, 0.92))

    for ext in ('png', 'pdf'):
        p = os.path.join(out_dir, f'bar_chart_ape_rmse.{ext}')
        fig.savefig(p, bbox_inches='tight', dpi=200 if ext == 'png' else None)
        print(f'  wrote {p}')
    plt.close(fig)


# --------------------------------------------------- RMSE ± std chart

def plot_rmse_std_chart(experiments, out_dir):
    """Grouped bars across experiments. Bar height = RMSE from evo,
    error bar = ±std of per-sample error. Two panels: translation + rotation.
    Same data source as plot_bar_chart, with std overlaid as an error bar."""
    labels  = [e['name'] for e in experiments]
    ekf_t_r = [get_rmse(e, EKF_NAME,  'ape_trans_m')   for e in experiments]
    ekf_t_s = [get_std (e, EKF_NAME,  'ape_trans_m')   for e in experiments]
    our_t_r = [get_rmse(e, OURS_NAME, 'ape_trans_m')   for e in experiments]
    our_t_s = [get_std (e, OURS_NAME, 'ape_trans_m')   for e in experiments]
    ekf_a_r = [get_rmse(e, EKF_NAME,  'ape_angle_deg') for e in experiments]
    ekf_a_s = [get_std (e, EKF_NAME,  'ape_angle_deg') for e in experiments]
    our_a_r = [get_rmse(e, OURS_NAME, 'ape_angle_deg') for e in experiments]
    our_a_s = [get_std (e, OURS_NAME, 'ape_angle_deg') for e in experiments]
    durs    = [e['gt_duration_s'] for e in experiments]
    xlabels = [f'{l}\n({d:.0f} s)' for l, d in zip(labels, durs)]

    x = np.arange(len(labels))
    w = 0.36
    fig, (ax_t, ax_a) = plt.subplots(
        2, 1, figsize=(max(7.5, 1.5 * len(labels)), 8), sharex=True,
        gridspec_kw={'height_ratios': [1, 1], 'hspace': 0.18},
    )

    def _draw(ax, r_ekf, s_ekf, r_our, s_our, ylabel, fmt):
        b1 = ax.bar(x - w / 2, r_ekf, w, yerr=s_ekf,
                    label=EKF_NAME, color=EKF_COLOR, edgecolor='black',
                    linewidth=0.5, capsize=4,
                    error_kw={'linewidth': 1.0, 'ecolor': 'black'})
        b2 = ax.bar(x + w / 2, r_our, w, yerr=s_our,
                    label=OURS_NAME, color=OURS_COLOR, edgecolor='black',
                    linewidth=0.5, capsize=4,
                    error_kw={'linewidth': 1.0, 'ecolor': 'black'})
        ax.set_ylabel(ylabel, fontsize=11)
        ax.yaxis.grid(True, linestyle='--', alpha=0.45)
        ax.set_axisbelow(True)
        for sp in ('top', 'right'):
            ax.spines[sp].set_visible(False)
        ymax = np.nanmax(
            [r + s for r, s in zip(r_ekf, s_ekf)] +
            [r + s for r, s in zip(r_our, s_our)]
        )
        ax.set_ylim(0, ymax * 1.25)
        for bars, rs, ss in [(b1, r_ekf, s_ekf), (b2, r_our, s_our)]:
            for bar, r, s in zip(bars, rs, ss):
                if np.isfinite(r):
                    top = r + (s if np.isfinite(s) else 0.0)
                    ax.annotate(f'{fmt.format(r)}\n±{fmt.format(s)}',
                                xy=(bar.get_x() + bar.get_width() / 2, top),
                                xytext=(0, 4), textcoords='offset points',
                                ha='center', va='bottom', fontsize=7,
                                linespacing=0.95)
        return b1, b2

    b1, b2 = _draw(ax_t, ekf_t_r, ekf_t_s, our_t_r, our_t_s,
                   'Translation APE [m]', '{:.3f}')
    _draw(ax_a, ekf_a_r, ekf_a_s, our_a_r, our_a_s,
          'Rotation APE [deg]', '{:.2f}')

    fig.suptitle(f'APE: RMSE ± std — {OURS_NAME} vs. {EKF_NAME}',
                 fontsize=13, fontweight='bold', y=0.995)
    fig.legend(handles=[b1, b2], loc='upper center',
               bbox_to_anchor=(0.5, 0.955),
               frameon=False, fontsize=10, ncol=2)
    ax_a.set_xticks(x)
    ax_a.set_xticklabels(xlabels, fontsize=9, rotation=15, ha='right')
    fig.tight_layout(rect=(0, 0, 1, 0.92))

    for ext in ('png', 'pdf'):
        p = os.path.join(out_dir, f'rmse_std_chart.{ext}')
        fig.savefig(p, bbox_inches='tight', dpi=200 if ext == 'png' else None)
        print(f'  wrote {p}')
    plt.close(fig)


# ----------------------------------------------------- box plot

def plot_box_errors(experiments, out_dir):
    """Box-and-whisker of per-sample translation and rotation APE errors,
    grouped by experiment with MUSE-K and EKF side-by-side. Shows the full
    distribution (median, IQR, whiskers, outliers) — the tail information
    that RMSE/std summary stats hide.

    Reads from <name>_ape.npz so the sample is the *cropped* run's data."""
    import matplotlib.patches as mpatches

    n = len(experiments)
    if n == 0:
        print('  [skip] no experiments')
        return

    labels = [e['name'] for e in experiments]
    durs   = [e['gt_duration_s'] for e in experiments]

    def collect(field, scale=1.0):
        ekf, our = [], []
        for e in experiments:
            d_e = e['npz'].get(EKF_NAME)
            d_o = e['npz'].get(OURS_NAME)
            ekf.append(d_e[field] * scale if d_e is not None else np.array([np.nan]))
            our.append(d_o[field] * scale if d_o is not None else np.array([np.nan]))
        return ekf, our

    trans_ekf, trans_our = collect('err_trans_m', 100.0)
    rot_ekf,   rot_our   = collect('err_rot_deg')

    x = np.arange(n)
    width = 0.36

    fig, (ax_t, ax_a) = plt.subplots(
        2, 1, figsize=(max(7.5, 1.6 * n), 9), sharex=True,
        gridspec_kw={'height_ratios': [1, 1], 'hspace': 0.18},
    )

    def _draw(ax, data_ekf, data_our, ylabel):
        def _style(bp, color):
            for patch in bp['boxes']:
                patch.set_facecolor(color)
                patch.set_edgecolor('black')
                patch.set_linewidth(0.5)
                patch.set_alpha(0.85)
            for median in bp['medians']:
                median.set_color('black')
                median.set_linewidth(1.5)
            for whisker in bp['whiskers'] + bp['caps']:
                whisker.set_color('black')
                whisker.set_linewidth(0.8)

        bp1 = ax.boxplot(
            data_ekf, positions=x - width / 2, widths=width * 0.9,
            patch_artist=True, showfliers=True,
            flierprops={'marker': '.', 'markersize': 2,
                        'markerfacecolor': EKF_COLOR,
                        'markeredgecolor': EKF_COLOR, 'alpha': 0.25},
        )
        _style(bp1, EKF_COLOR)
        bp2 = ax.boxplot(
            data_our, positions=x + width / 2, widths=width * 0.9,
            patch_artist=True, showfliers=True,
            flierprops={'marker': '.', 'markersize': 2,
                        'markerfacecolor': OURS_COLOR,
                        'markeredgecolor': OURS_COLOR, 'alpha': 0.25},
        )
        _style(bp2, OURS_COLOR)

        ax.set_ylabel(ylabel, fontsize=11)
        ax.yaxis.grid(True, linestyle='--', alpha=0.45)
        ax.set_axisbelow(True)
        for sp in ('top', 'right'):
            ax.spines[sp].set_visible(False)
        ax.set_ylim(bottom=0)

    _draw(ax_t, trans_ekf, trans_our, 'Translation APE [cm]')
    _draw(ax_a, rot_ekf,   rot_our,   'Rotation APE [deg]')

    xlabels = [f'{l}\n({d:.0f} s)' for l, d in zip(labels, durs)]
    ax_a.set_xticks(x)
    ax_a.set_xticklabels(xlabels, fontsize=9, rotation=15, ha='right')
    ax_t.set_xticks(x)

    h_ekf = mpatches.Patch(color=EKF_COLOR, label=EKF_NAME, alpha=0.85)
    h_our = mpatches.Patch(color=OURS_COLOR, label=OURS_NAME, alpha=0.85)
    fig.suptitle(f'Per-sample APE distribution — {OURS_NAME} vs. {EKF_NAME}',
                 fontsize=13, fontweight='bold', y=0.995)
    fig.legend(handles=[h_ekf, h_our], loc='upper center',
               bbox_to_anchor=(0.5, 0.955),
               frameon=False, fontsize=10, ncol=2)
    fig.tight_layout(rect=(0, 0, 1, 0.92))

    for ext in ('png', 'pdf'):
        p = os.path.join(out_dir, f'box_errors.{ext}')
        fig.savefig(p, bbox_inches='tight', dpi=200 if ext == 'png' else None)
        print(f'  wrote {p}')
    plt.close(fig)


# ----------------------------------------------------- degradation panel

def plot_degradation_panel(experiments, out_dir):
    degs = [e for e in experiments if e['condition'] in DEGRADATION_CONDITIONS]
    if not degs:
        print('  [skip] no degradation experiments')
        return
    n = len(degs)
    cols = 2 if n > 2 else n
    rows = (n + cols - 1) // cols

    fig, axs = plt.subplots(rows, cols, figsize=(6 * cols, 3.5 * rows),
                            sharex=False)
    axs = np.atleast_1d(axs).flatten()

    for ax, exp in zip(axs, degs):
        for display, color in [(EKF_NAME, EKF_COLOR), (OURS_NAME, OURS_COLOR)]:
            d = exp['npz'].get(display)
            if d is None:
                continue
            ax.plot(d['t'], d['err_trans_m'] * 100,
                    color=color, linewidth=1.4, alpha=0.85, label=display)
        ax.set_title(exp['name'], fontsize=11)
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('APE-trans [cm]')
        ax.grid(True, linestyle='--', alpha=0.5)
        ax.set_ylim(bottom=0)
        ax.legend(loc='upper right', fontsize=9)

    for ax in axs[len(degs):]:
        ax.set_visible(False)

    fig.suptitle('Estimator robustness under sensor degradation',
                 fontsize=13, fontweight='bold', y=1.00)
    fig.tight_layout()

    for ext in ('png', 'pdf'):
        p = os.path.join(out_dir, f'degradation_panel.{ext}')
        fig.savefig(p, bbox_inches='tight', dpi=200 if ext == 'png' else None)
        print(f'  wrote {p}')
    plt.close(fig)


# ----------------------------------------------------- long-duration drift

def plot_long_duration_drift(experiments, out_dir):
    candidates = [e for e in experiments if e['condition'] == 'nominal']
    if not candidates:
        print('  [skip] no nominal experiment for long-duration drift')
        return
    exp = max(candidates, key=lambda e: e['gt_duration_s'])

    fig, axs = plt.subplots(4, 1, figsize=(10, 11), sharex=True,
                            gridspec_kw={'hspace': 0.15})

    ax_t = axs[0]
    for display, color in [(EKF_NAME, EKF_COLOR), (OURS_NAME, OURS_COLOR)]:
        d = exp['npz'].get(display)
        if d is None:
            continue
        ax_t.plot(d['t'], d['err_trans_m'] * 100,
                  color=color, linewidth=1.2, alpha=0.85, label=display)
    ax_t.set_ylabel('APE-trans [cm]')
    ax_t.set_title(f'{exp["name"]}  ({exp["gt_duration_s"]:.0f} s, '
                   f'{exp["controller"]} controller)',
                   fontsize=12, fontweight='bold')
    ax_t.grid(True, linestyle='--', alpha=0.5)
    ax_t.set_ylim(bottom=0)
    ax_t.legend(loc='upper right')

    for k, axis_name in enumerate(['Roll', 'Pitch', 'Yaw']):
        ax = axs[k + 1]
        for display, color in [(EKF_NAME, EKF_COLOR), (OURS_NAME, OURS_COLOR)]:
            d = exp['npz'].get(display)
            if d is None:
                continue
            err = d['err_rpy_deg'][:, k]
            ax.plot(d['t'], err, color=color, linewidth=1.0, alpha=0.55,
                    label=f'{display} error')

            # Linear trend line
            if len(d['t']) > 2:
                p = np.polyfit(d['t'], err, 1)
                trend = np.polyval(p, d['t'])
                ax.plot(d['t'], trend, color=color, linewidth=2.0,
                        linestyle='--', alpha=0.95,
                        label=f'{display} linear fit ({p[0] * 1000:+.2f}°/1000s)')
        ax.set_ylabel(f'{axis_name} err [deg]')
        ax.axhline(0, color='gray', linewidth=0.5, alpha=0.5)
        ax.grid(True, linestyle='--', alpha=0.5)
        ax.legend(loc='upper right', fontsize=8)

    axs[-1].set_xlabel('Time [s]')

    for ext in ('png', 'pdf'):
        p = os.path.join(out_dir, f'long_duration_drift.{ext}')
        fig.savefig(p, bbox_inches='tight', dpi=200 if ext == 'png' else None)
        print(f'  wrote {p}')
    plt.close(fig)


# ---------------------------------------------------- station-keeping

def load_gt_horizontal_deviation(gt_tum_path):
    gt = np.loadtxt(gt_tum_path)
    t = gt[:, 0] - gt[0, 0]
    horiz = np.linalg.norm(gt[:, 1:3] - gt[0, 1:3], axis=1)
    vert  = np.abs(gt[:, 3] - gt[0, 3])
    return t, horiz, vert


def plot_station_keeping_cdf(experiments, out_dir):
    """Compare the two nominal runs by GT horizontal deviation from setpoint.
    Colour by controller (matches estimator colour for that controller)."""
    nominals = [e for e in experiments if e['condition'] == 'nominal']
    if len(nominals) < 2:
        print('  [skip] need at least two nominal experiments for CDF')
        return

    fig, axs = plt.subplots(1, 2, figsize=(13, 5),
                            gridspec_kw={'wspace': 0.25})

    colour_for = {'ArduSub (EKF)': EKF_COLOR, 'MUSE-K': OURS_COLOR}

    stats_lines = []
    for exp in nominals:
        t, horiz, vert = load_gt_horizontal_deviation(exp['gt_path'])
        colour = colour_for.get(exp['controller'], 'grey')
        label = f'{exp["controller"]} ctrl ({exp["gt_duration_s"]:.0f} s)'

        axs[0].plot(t, horiz * 100, color=colour, linewidth=1.4,
                    alpha=0.9, label=label)

        s = np.sort(horiz)
        cdf = np.arange(1, len(s) + 1) / len(s)
        axs[1].plot(s * 100, cdf, color=colour, linewidth=2.0, label=label)

        med = float(np.median(horiz))
        p95 = float(np.percentile(horiz, 95))
        mx  = float(np.max(horiz))
        rms = float(np.sqrt(np.mean(horiz ** 2)))
        within_5  = float(np.mean(horiz <= 0.05))
        within_10 = float(np.mean(horiz <= 0.10))
        stats_lines.append(
            f'  {exp["controller"]:18s}'
            f'  median {med * 100:5.2f}cm'
            f'  p95 {p95 * 100:5.2f}cm'
            f'  max {mx * 100:5.2f}cm'
            f'  RMS {rms * 100:5.2f}cm'
            f'  ≤5cm {within_5 * 100:5.1f}%'
            f'  ≤10cm {within_10 * 100:5.1f}%'
        )

    axs[0].set_xlabel('Time [s]')
    axs[0].set_ylabel('Horizontal deviation from setpoint [cm]')
    axs[0].set_title('Station-keeping deviation over time')
    axs[0].grid(True, linestyle='--', alpha=0.5)
    axs[0].set_ylim(bottom=0)
    axs[0].legend(loc='upper right')

    axs[1].set_xlabel('Horizontal deviation from setpoint [cm]')
    axs[1].set_ylabel('Empirical CDF')
    axs[1].set_title('Station-keeping CDF — controller comparison')
    axs[1].set_xlim(left=0)
    axs[1].set_ylim(0, 1.02)
    axs[1].grid(True, linestyle='--', alpha=0.5)
    axs[1].legend(loc='lower right')

    fig.suptitle('Closed-loop station-keeping (GT only, setpoint = GT[0])',
                 fontsize=12, fontweight='bold', y=1.02)
    fig.tight_layout()

    for ext in ('png', 'pdf'):
        p = os.path.join(out_dir, f'station_keeping_cdf.{ext}')
        fig.savefig(p, bbox_inches='tight', dpi=200 if ext == 'png' else None)
        print(f'  wrote {p}')
    plt.close(fig)

    txt_path = os.path.join(out_dir, 'station_keeping_stats.txt')
    with open(txt_path, 'w') as f:
        f.write('Station-keeping (horizontal deviation of GT from GT[0])\n')
        f.write('\n'.join(stats_lines) + '\n')
    print(f'  wrote {txt_path}')
    for line in stats_lines:
        print(line)


# --------------------------------------------------------------- main

def main():
    p = argparse.ArgumentParser(description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--base-dir',
                   default='/mnt/56F01D0DF01CF4C9/Thesis_MSc_data/19_05/good_data',
                   help='directory containing the run_* bag folders')
    p.add_argument('--output-dir', default=None,
                   help='default: <base-dir>/thesis_figures')
    args = p.parse_args()

    out_dir = args.output_dir or os.path.join(args.base_dir, 'thesis_figures')
    os.makedirs(out_dir, exist_ok=True)

    print(f'Loading {len(EXPERIMENTS)} experiments from {args.base_dir} ...')
    for e in EXPERIMENTS:
        load_experiment(e, args.base_dir)
        loaded = ', '.join(e['npz'].keys())
        print(f'  {e["name"]:32s}  {e["gt_duration_s"]:6.1f}s  npz=[{loaded}]')

    print('\n[1/8] Master table')
    build_master_table(EXPERIMENTS, out_dir)

    print('\n[2/8] 6-DoF drift table')
    build_drift_table(EXPERIMENTS, out_dir)

    print('\n[3/8] Bar chart')
    plot_bar_chart(EXPERIMENTS, out_dir)

    print('\n[4/8] RMSE ± std chart')
    plot_rmse_std_chart(EXPERIMENTS, out_dir)

    print('\n[5/8] Box-plot per-sample errors')
    plot_box_errors(EXPERIMENTS, out_dir)

    print('\n[6/8] Degradation panel')
    plot_degradation_panel(EXPERIMENTS, out_dir)

    print('\n[7/8] Long-duration drift')
    plot_long_duration_drift(EXPERIMENTS, out_dir)

    print('\n[8/8] Station-keeping CDF')
    plot_station_keeping_cdf(EXPERIMENTS, out_dir)

    print(f'\n[done] all figures in {out_dir}')


if __name__ == '__main__':
    main()
