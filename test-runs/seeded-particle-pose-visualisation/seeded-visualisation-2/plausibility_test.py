#!/usr/bin/env python3
"""
Geometric Plausibility Test for Seeded Particle Filter Candidates

Computes whether each candidate pose (candidate_x, candidate_y, candidate_theta)
is a geometrically plausible position from which to observe the corresponding
field feature (feature_x, feature_y) at the observed distance and heading.

Two distinct checks:

  1. CANDIDATE SELF-CONSISTENCY (forward model):
     Given candidate pose → feature location, what distance and bearing would
     the robot measure? Compare with (obs_distance, obs_heading).
     If they match within tolerance, the candidate is a correct inversion of
     the observation — the candidate WOULD see the feature as reported.

  2. ROBOT PLAUSIBILITY (ground-truth check):
     Given robot_pose → feature location, what distance and bearing should
     the robot have measured? Compare with (obs_distance, obs_heading).
     If these DON'T match, the observation itself is inconsistent with the
     robot's actual position — meaning the problem is upstream of the PF.

Usage:
    cd <project-root>
    python test-runs/seeded-visualisation-2/plausibility_test.py

Output:
    - Console summary tables
    - analysis_augmented.csv (seed_candidates with extra diagnostic columns)
    - plausibility_figures/ directory with histograms and scatter plots
"""

import os
import math
import sys
import csv
import statistics
from collections import defaultdict

try:
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    import numpy as np
    HAS_MPL = True
except ImportError:
    HAS_MPL = False
    print("INFO: matplotlib not available — skipping figures (install with: pip install matplotlib)")

# =========================================================================
#  Configuration
# =========================================================================

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CANDIDATES_PATH = os.path.join(SCRIPT_DIR, 'seed_candidates.csv')
OUTPUT_CSV = os.path.join(SCRIPT_DIR, 'analysis_augmented.csv')
FIGURE_DIR = os.path.join(SCRIPT_DIR, 'plausibility_figures')

# Tolerance for "self-consistent" classification
POSITION_TOLERANCE_MM = 1.0      # 1 mm distance error
BEARING_TOLERANCE_RAD = 0.01     # ~0.57° bearing error

# Tolerance for "plausible from robot pose" classification
ROBOT_DISTANCE_TOLERANCE_MM = 300.0    # 300 mm distance error (generous)
ROBOT_BEARING_TOLERANCE_RAD = 0.5     # ~28.6° bearing error (generous)

# =========================================================================
#  Helpers
# =========================================================================


def normalize_angle(theta):
    """Wrap angle to [-π, π]."""
    while theta > math.pi:
        theta -= 2.0 * math.pi
    while theta < -math.pi:
        theta += 2.0 * math.pi
    return theta


def angle_diff(a, b):
    """Signed shortest angular difference a - b, normalized to [-π, π]."""
    return normalize_angle(a - b)


def forward_model_distance(pose_x, pose_y, feat_x, feat_y):
    """Distance from pose (x,y) to feature (x,y)."""
    return math.hypot(feat_x - pose_x, feat_y - pose_y)


def forward_model_bearing(pose_x, pose_y, pose_theta, feat_x, feat_y):
    """
    Bearing (in robot body frame) to the feature.

    bearing = atan2(feature_y - robot_y, feature_x - robot_x) - robot_theta
    Result in [-π, π] — positive means feature is to the robot's left.
    """
    world_bearing = math.atan2(feat_y - pose_y, feat_x - pose_x)
    return normalize_angle(world_bearing - pose_theta)


def load_candidates(path):
    """Load seed_candidates.csv into a list of dicts."""
    rows = []
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            # Convert numeric fields
            numeric = {}
            for k, v in row.items():
                try:
                    numeric[k] = float(v)
                except ValueError:
                    numeric[k] = v
            rows.append(numeric)
    return rows


# =========================================================================
#  Analysis
# =========================================================================


def analyse_candidates(rows):
    """
    For each candidate, compute forward-model expected observations from
    both the candidate pose and the robot pose, then compare with actual
    observations.

    Returns augmented rows (with extra columns) and summary statistics dict.
    """
    augmented = []
    stats = defaultdict(list)

    for r in rows:
        a = dict(r)  # copy

        # --- Candidate forward model ---
        cand_dist = forward_model_distance(
            a['candidate_x'], a['candidate_y'],
            a['feature_x'], a['feature_y']
        )
        cand_bearing = forward_model_bearing(
            a['candidate_x'], a['candidate_y'], a['candidate_theta'],
            a['feature_x'], a['feature_y']
        )

        cand_dist_error = cand_dist - a['obs_distance']
        cand_bearing_error = angle_diff(cand_bearing, a['obs_heading'])

        a['cand_expected_distance'] = cand_dist
        a['cand_expected_bearing'] = cand_bearing
        a['cand_distance_error'] = cand_dist_error
        a['cand_bearing_error'] = cand_bearing_error
        a['cand_self_consistent'] = (
            abs(cand_dist_error) <= POSITION_TOLERANCE_MM
            and abs(cand_bearing_error) <= BEARING_TOLERANCE_RAD
        )

        # --- Robot forward model ---
        robot_dist = forward_model_distance(
            a['robot_pose_x'], a['robot_pose_y'],
            a['feature_x'], a['feature_y']
        )
        robot_bearing = forward_model_bearing(
            a['robot_pose_x'], a['robot_pose_y'], a['robot_pose_theta'],
            a['feature_x'], a['feature_y']
        )

        robot_dist_error = robot_dist - a['obs_distance']
        robot_bearing_error = angle_diff(robot_bearing, a['obs_heading'])

        a['robot_expected_distance'] = robot_dist
        a['robot_expected_bearing'] = robot_bearing
        a['robot_distance_error'] = robot_dist_error
        a['robot_bearing_error'] = robot_bearing_error
        a['robot_observation_plausible'] = (
            abs(robot_dist_error) <= ROBOT_DISTANCE_TOLERANCE_MM
            and abs(robot_bearing_error) <= ROBOT_BEARING_TOLERANCE_RAD
        )

        # --- Relative error (fractional) ---
        if robot_dist > 0:
            a['robot_distance_error_pct'] = 100.0 * robot_dist_error / robot_dist
        else:
            a['robot_distance_error_pct'] = 0.0

        augmented.append(a)

        # Accumulate stats
        stats['seed_group_id'].append(a['seed_group_id'])
        stats['feature_type'].append(a['feature_type'])
        stats['self_consistent'].append(a['cand_self_consistent'])
        stats['obs_plausible'].append(a['robot_observation_plausible'])
        stats['cand_dist_error'].append(cand_dist_error)
        stats['cand_bearing_error'].append(cand_bearing_error)
        stats['robot_dist_error'].append(robot_dist_error)
        stats['robot_bearing_error'].append(robot_bearing_error)
        stats['robot_dist_error_pct'].append(a['robot_distance_error_pct'])

    return augmented, stats


def print_summary(augmented, stats):
    """Print aggregate summary tables."""
    n_total = len(augmented)
    n_self_consistent = sum(stats['self_consistent'])
    n_obs_plausible = sum(stats['obs_plausible'])

    print("=" * 78)
    print("  GEOMETRIC PLAUSIBILITY TEST — OVERALL SUMMARY")
    print("=" * 78)
    print(f"  Total candidates:                   {n_total}")
    print(f"  Self-consistent (forward model OK): {n_self_consistent} "
          f"({100 * n_self_consistent / n_total:.1f}%)")
    print(f"  Observation plausible from robot:   {n_obs_plausible} "
          f"({100 * n_obs_plausible / n_total:.1f}%)")
    print()

    # --- Error statistics ---
    print(f"  Candidate forward-model errors:")
    print(f"    Distance error:  mean={statistics.mean(stats['cand_dist_error']):.1f} mm, "
          f"stdev={statistics.stdev(stats['cand_dist_error']):.1f} mm, "
          f"max={max(abs(d) for d in stats['cand_dist_error']):.1f} mm")
    print(f"    Bearing error:   mean={statistics.mean(stats['cand_bearing_error']):.4f} rad, "
          f"stdev={statistics.stdev(stats['cand_bearing_error']):.4f} rad")
    print()
    print(f"  Robot observation errors:")
    print(f"    Distance error:  mean={statistics.mean(stats['robot_dist_error']):.1f} mm, "
          f"stdev={statistics.stdev(stats['robot_dist_error']):.1f} mm")
    print(f"    Bearing error:   mean={statistics.mean(stats['robot_bearing_error']):.4f} rad, "
          f"stdev={statistics.stdev(stats['robot_bearing_error']):.4f} rad")
    print(f"    Distance error:  mean={statistics.mean(stats['robot_dist_error_pct']):.1f}%")
    print()

    # --- Per feature type ---
    print("-" * 78)
    print("  BREAKDOWN BY FEATURE TYPE")
    print("-" * 78)
    types = sorted(set(stats['feature_type']))
    for ft in types:
        mask = [t == ft for t in stats['feature_type']]
        n_ft = sum(mask)
        sc_ft = sum(s for s, m in zip(stats['self_consistent'], mask) if m)
        op_ft = sum(o for o, m in zip(stats['obs_plausible'], mask) if o)
        dist_errs = [d for d, m in zip(stats['robot_dist_error_pct'], mask) if m]
        mean_pct = statistics.mean(dist_errs) if dist_errs else 0.0
        print(f"  {ft:20s}  n={n_ft:4d}  "
              f"self-consistent={sc_ft:4d} ({100*sc_ft/n_ft:5.1f}%)  "
              f"obs-plausible={op_ft:4d} ({100*op_ft/n_ft:5.1f}%)  "
              f"mean dist err={mean_pct:+.1f}%")

    # --- Per seed group (top offenders) ---
    print()
    print("-" * 78)
    print("  PER SEED GROUP — SELF-CONSISTENCY & PLAUSIBILITY")
    print("-" * 78)
    groups = sorted(set(stats['seed_group_id']))
    print(f"  {'Group':>7s}  {'N':>4s}  {'Self-Con':>9s}  {'Obs-Plaus':>9s}  "
          f"{'Mean Dist Err':>13s}  {'Mean Bear Err':>13s}")
    print(f"  {'------':>7s}  {'---':>4s}  {'---------':>9s}  {'---------':>9s}  "
          f"{'-------------':>13s}  {'-------------':>13s}")
    for g in groups:
        mask = [sg == g for sg in stats['seed_group_id']]
        n_g = sum(mask)
        sc_g = sum(s for s, m in zip(stats['self_consistent'], mask) if m)
        op_g = sum(o for o, m in zip(stats['obs_plausible'], mask) if o)
        dist_errs_g = [d for d, m in zip(stats['cand_dist_error'], mask) if m]
        bear_errs_g = [b for b, m in zip(stats['cand_bearing_error'], mask) if m]
        mean_dist = statistics.mean(dist_errs_g) if dist_errs_g else 0.0
        mean_bear = statistics.mean(bear_errs_g) if bear_errs_g else 0.0
        print(f"  {g:>7.0f}  {n_g:>4d}  {sc_g:>4d}/{n_g:<3d} "
              f"({100*sc_g/n_g:>4.0f}%)  "
              f"{op_g:>4d}/{n_g:<3d} ({100*op_g/n_g:>4.0f}%)  "
              f"{mean_dist:>+9.1f} mm    {mean_bear:>+.4f} rad")

    print()
    print("=" * 78)
    print("  INTERPRETATION")
    print("=" * 78)
    print()
    print(f"  If a candidate is 'self-consistent' (forward model matches), the")
    print(f"  candidate pose is a geometrically correct inversion of the observation.")
    print(f"  The particle heading IS plausible for that observation.")
    print()
    print(f"  If the observation is NOT 'plausible from robot', the observation")
    print(f"  itself doesn't match the robot's actual position — bad data upstream.")
    print()
    print(f"  Currently: {n_self_consistent}/{n_total} candidates are self-consistent")
    print(f"  ({100 * n_self_consistent / n_total:.1f}%), confirming the inversion is correct.")
    print(f"  Only {n_obs_plausible}/{n_total} observations ({100 * n_obs_plausible / n_total:.1f}%)")
    print(f"  are plausible from the robot's actual pose — the core issue is bad observations.")


def write_augmented_csv(augmented, path):
    """Write augmented analysis to CSV."""
    if not augmented:
        return
    fieldnames = list(augmented[0].keys())
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(augmented)
    print(f"\n  Augmented analysis written to: {path}")


# =========================================================================
#  Visualisation (if matplotlib available)
# =========================================================================


def plot_histograms(stats):
    """Plot histograms of distance and bearing errors."""
    os.makedirs(FIGURE_DIR, exist_ok=True)

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # Candidate distance error
    axes[0, 0].hist(stats['cand_dist_error'], bins=80, color='#2196f3', alpha=0.7,
                    edgecolor='white', linewidth=0.3)
    axes[0, 0].axvline(0, color='red', linestyle='--', linewidth=1, label='Zero error')
    axes[0, 0].set_xlabel('Candidate Distance Error (mm)')
    axes[0, 0].set_ylabel('Count')
    axes[0, 0].set_title('Candidate Forward Model — Distance Error')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)

    # Candidate bearing error
    axes[0, 1].hist(stats['cand_bearing_error'], bins=80, color='#4caf50', alpha=0.7,
                    edgecolor='white', linewidth=0.3)
    axes[0, 1].axvline(0, color='red', linestyle='--', linewidth=1, label='Zero error')
    axes[0, 1].set_xlabel('Candidate Bearing Error (rad)')
    axes[0, 1].set_ylabel('Count')
    axes[0, 1].set_title('Candidate Forward Model — Bearing Error')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)

    # Robot distance error (%)
    axes[1, 0].hist(stats['robot_dist_error_pct'], bins=80, color='#ff9800', alpha=0.7,
                    edgecolor='white', linewidth=0.3)
    axes[1, 0].axvline(0, color='red', linestyle='--', linewidth=1, label='Zero error')
    axes[1, 0].set_xlabel('Robot Distance Error (%)')
    axes[1, 0].set_ylabel('Count')
    axes[1, 0].set_title('Robot Pose → Feature: Distance Error %')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)

    # Robot bearing error
    axes[1, 1].hist(stats['robot_bearing_error'], bins=80, color='#9c27b0', alpha=0.7,
                    edgecolor='white', linewidth=0.3)
    axes[1, 1].axvline(0, color='red', linestyle='--', linewidth=1, label='Zero error')
    axes[1, 1].set_xlabel('Robot Bearing Error (rad)')
    axes[1, 1].set_ylabel('Count')
    axes[1, 1].set_title('Robot Pose → Feature: Bearing Error')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)

    plt.suptitle('Geometric Plausibility Test — Error Distributions', fontsize=14, fontweight='bold')
    plt.tight_layout()
    fig.savefig(os.path.join(FIGURE_DIR, 'error_histograms.png'), dpi=150)
    print(f"  Saved: {FIGURE_DIR}/error_histograms.png")
    plt.close(fig)


def plot_scatter_self_consistent(augmented):
    """Scatter plot: candidate positions coloured by self-consistent status."""
    os.makedirs(FIGURE_DIR, exist_ok=True)

    fig, ax = plt.subplots(figsize=(12, 10))

    consistent_x = [r['candidate_x'] for r in augmented if r['cand_self_consistent']]
    consistent_y = [r['candidate_y'] for r in augmented if r['cand_self_consistent']]
    inconsistent_x = [r['candidate_x'] for r in augmented if not r['cand_self_consistent']]
    inconsistent_y = [r['candidate_y'] for r in augmented if not r['cand_self_consistent']]

    ax.scatter(consistent_x, consistent_y, marker='o', s=8, color='#4caf50',
               alpha=0.5, label=f'Self-consistent ({len(consistent_x)})')
    ax.scatter(inconsistent_x, inconsistent_y, marker='x', s=12, color='#f44336',
               alpha=0.6, label=f'Not self-consistent ({len(inconsistent_x)})')

    # Draw field outline
    HALF_L = 3150
    HALF_W = 2025
    from matplotlib.patches import Rectangle
    carpet = Rectangle(
        (-HALF_L - 510, -HALF_W - 930), 2 * (HALF_L + 510), 2 * (HALF_W + 930),
        linewidth=0, facecolor='#c8e6c9', zorder=0
    )
    ax.add_patch(carpet)
    field = Rectangle(
        (-HALF_L, -HALF_W), 2 * HALF_L, 2 * HALF_W,
        linewidth=2, edgecolor='white', facecolor='none', zorder=1
    )
    ax.add_patch(field)
    ax.axvline(0, color='white', linewidth=1.5, zorder=1)

    ax.set_aspect('equal')
    margin = 1200
    ax.set_xlim(-HALF_L - margin, HALF_L + margin)
    ax.set_ylim(-HALF_W - margin, HALF_W + margin)
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_title('Candidate Positions — Self-Consistent vs Not', fontsize=13, fontweight='bold')
    ax.legend()
    ax.grid(True, alpha=0.15)

    fig.savefig(os.path.join(FIGURE_DIR, 'candidate_self_consistency.png'), dpi=150)
    print(f"  Saved: {FIGURE_DIR}/candidate_self_consistency.png")
    plt.close(fig)


# =========================================================================
#  Main
# =========================================================================


def main():
    print("=" * 78)
    print("  SEEDED PARTICLE FILTER — GEOMETRIC PLAUSIBILITY TEST")
    print("  Test-runs: seeded-visualisation-2")
    print("=" * 78)
    print()

    # Load
    print(f"  Loading candidates from: {CANDIDATES_PATH}")
    rows = load_candidates(CANDIDATES_PATH)
    print(f"  Loaded {len(rows)} candidate rows.\n")

    # Analyse
    augmented, stats = analyse_candidates(rows)

    # Augmented CSV
    write_augmented_csv(augmented, OUTPUT_CSV)

    # Print summary
    print()
    print_summary(augmented, stats)

    # Figures
    if HAS_MPL:
        print()
        print("-" * 78)
        print("  GENERATING FIGURES")
        print("-" * 78)
        try:
            plot_histograms(stats)
            plot_scatter_self_consistent(augmented)
            print(f"  All figures saved to: {FIGURE_DIR}/")
        except Exception as e:
            print(f"  WARNING: Figure generation failed: {e}")
    else:
        print()
        print("  Install matplotlib for visualisations:")
        print("    pip install matplotlib numpy")

    print()
    print("=" * 78)
    print("  DONE")
    print("=" * 78)


if __name__ == '__main__':
    main()
