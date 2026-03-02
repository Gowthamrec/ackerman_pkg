#!/usr/bin/env python3
"""
Aggregate experiment result JSON files and generate simple summaries/plots.

Usage:
  ros2 run ackerman_pkg generate_results.py [results_dir]
Default results_dir: /home/ros2/car_project_ws/results

Outputs:
- prints summary per mode
- saves bar plots (success rate, avg time, avg path) to research_paper/*.png if matplotlib is available
"""

import json
import sys
from pathlib import Path

import numpy as np

try:
    import matplotlib.pyplot as plt
    HAVE_PLOT = True
except Exception:
    HAVE_PLOT = False


def load_results(dir_path: Path):
    """Load all summary JSON files from mode subdirectories."""
    data = {}
    # Check subdirectories (baseline, lane_only, proposed, weight_variant1, weight_variant2, weight_variant3)
    for mode_dir in dir_path.iterdir():
        if not mode_dir.is_dir():
            continue
        mode_name = mode_dir.name
        # Load summary files from this mode
        for f in sorted(mode_dir.glob('*summary*.json')):
            try:
                with f.open() as fh:
                    obj = json.load(fh)
                data.setdefault(mode_name, []).append(obj)
            except Exception as e:
                print(f"Skipping {f.name}: {e}")
    return data


def summarize(mode, runs):
    all_runs = []
    for r in runs:
        all_runs.extend(r.get('runs', []))
    if not all_runs:
        return None
    successes = [x for x in all_runs if x.get('success')]
    times = [x['completion_time_sec'] for x in all_runs]
    paths = [x['path_length_m'] for x in all_runs]
    return {
        'n': len(all_runs),
        'success_rate': len(successes) / len(all_runs),
        'avg_time': float(np.mean(times)),
        'std_time': float(np.std(times)),
        'avg_path': float(np.mean(paths)),
        'std_path': float(np.std(paths)),
    }


def maybe_plot(summary_by_mode, out_dir: Path):
    if not HAVE_PLOT:
        print("matplotlib not available; skipping plots")
        return
    
    # Order modes consistently (all 6 modes)
    mode_order = ['baseline', 'lane_only', 'proposed', 
                  'weight_variant1', 'weight_variant2', 'weight_variant3']
    modes = [m for m in mode_order if m in summary_by_mode]
    if not modes:
        return

    # Professional labels
    labels = {
        'baseline': 'Baseline\n(Nav2 only)', 
        'lane_only': 'Lane Following\n(Nav2 + Lanes)',
        'proposed': 'Proposed\n(Full System)',
        'weight_variant1': 'Variant 1\n(α=0.6, β=0.3, γ=0.1)',
        'weight_variant2': 'Variant 2\n(α=0.5, β=0.3, γ=0.2)',
        'weight_variant3': 'Variant 3\n(α=0.4, β=0.4, γ=0.2)'
    }
    mode_labels = [labels.get(m, m) for m in modes]
    
    sr = [summary_by_mode[m]['success_rate'] * 100 for m in modes]
    tavg = [summary_by_mode[m]['avg_time'] for m in modes]
    tstd = [summary_by_mode[m]['std_time'] for m in modes]
    pavg = [summary_by_mode[m]['avg_path'] for m in modes]
    pstd = [summary_by_mode[m]['std_path'] for m in modes]

    out_dir.mkdir(parents=True, exist_ok=True)

    # Publication-quality settings (6 colors for 6 modes)
    colors = ['#3498db', '#e74c3c', '#2ecc71', '#f39c12', '#9b59b6', '#1abc9c']
    plt.rcParams.update({'font.size': 10, 'font.family': 'serif', 'axes.labelsize': 11,
                         'axes.titlesize': 12, 'figure.titlesize': 13})

    # Success Rate
    fig, ax = plt.subplots(figsize=(10, 5))
    bars = ax.bar(mode_labels, sr, color=colors[:len(modes)], edgecolor='black', linewidth=1.2, alpha=0.85)
    ax.set_ylabel('Success Rate (%)', fontweight='bold')
    ax.set_title('Navigation Success Rate Comparison', fontweight='bold', pad=15)
    ax.set_ylim([0, 105])
    ax.grid(axis='y', alpha=0.3, linestyle='--')
    plt.xticks(rotation=15, ha='right')
    for bar, val in zip(bars, sr):
        ax.text(bar.get_x() + bar.get_width()/2., bar.get_height() + 1,
                f'{val:.1f}%', ha='center', va='bottom', fontweight='bold', fontsize=9)
    plt.tight_layout()
    plt.savefig(out_dir / 'figure_success_rate.png', dpi=300, bbox_inches='tight')
    plt.close()

    # Completion Time
    fig, ax = plt.subplots(figsize=(10, 5))
    bars = ax.bar(mode_labels, tavg, yerr=tstd, capsize=6, color=colors[:len(modes)], 
                   edgecolor='black', linewidth=1.2, alpha=0.85,
                   error_kw={'linewidth': 1.5, 'ecolor': 'gray'})
    ax.set_ylabel('Completion Time (seconds)', fontweight='bold')
    ax.set_title('Average Navigation Time (±SD)', fontweight='bold', pad=15)
    ax.grid(axis='y', alpha=0.3, linestyle='--')
    plt.xticks(rotation=15, ha='right')
    for bar, avg, std in zip(bars, tavg, tstd):
        ax.text(bar.get_x() + bar.get_width()/2., bar.get_height() + std + 3,
                f'{avg:.1f}s\n±{std:.1f}', ha='center', va='bottom', fontsize=8)
    plt.tight_layout()
    plt.savefig(out_dir / 'figure_time.png', dpi=300, bbox_inches='tight')
    plt.close()

    # Path Length
    fig, ax = plt.subplots(figsize=(10, 5))
    bars = ax.bar(mode_labels, pavg, yerr=pstd, capsize=6, color=colors[:len(modes)],
                   edgecolor='black', linewidth=1.2, alpha=0.85,
                   error_kw={'linewidth': 1.5, 'ecolor': 'gray'})
    ax.set_ylabel('Path Length (meters)', fontweight='bold')
    ax.set_title('Average Path Length (±SD)', fontweight='bold', pad=15)
    ax.grid(axis='y', alpha=0.3, linestyle='--')
    plt.xticks(rotation=15, ha='right')
    for bar, avg, std in zip(bars, pavg, pstd):
        ax.text(bar.get_x() + bar.get_width()/2., bar.get_height() + std + 0.15,
                f'{avg:.2f}m\n±{std:.2f}', ha='center', va='bottom', fontsize=8)
    plt.tight_layout()
    plt.savefig(out_dir / 'figure_path.png', dpi=300, bbox_inches='tight')
    plt.close()
    print(f"Plots saved to {out_dir}")


def main():
    results_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path('/home/ros2/car_project_ws/results')
    if not results_dir.exists():
        print(f"Results dir not found: {results_dir}")
        sys.exit(1)

    data = load_results(results_dir)
    if not data:
        print("No results found")
        sys.exit(0)

    summary_by_mode = {}
    for mode, runs in data.items():
        s = summarize(mode, runs)
        if s:
            summary_by_mode[mode] = s
            print(f"\nMode: {mode}")
            print(f"  Runs: {s['n']}")
            print(f"  Success rate: {s['success_rate']*100:.1f}%")
            print(f"  Avg time: {s['avg_time']:.2f} ± {s['std_time']:.2f} s")
            print(f"  Avg path: {s['avg_path']:.2f} ± {s['std_path']:.2f} m")

    maybe_plot(summary_by_mode, Path('/home/ros2/car_project_ws/research_paper'))


if __name__ == '__main__':
    main()
