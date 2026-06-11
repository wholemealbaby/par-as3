#!/usr/bin/env python3
"""
Seeded Particle Filter — Per-Candidate RMSE Against Robot Ground-Truth Pose

For a single candidate (specified by --group and --candidate, or the first
candidate by default), compares every particle's pose (x, y, theta) against
the robot's actual ground-truth pose.

Reports separate RMSE for:
  - RMSE_x     — root-mean-square error in x (mm)
  - RMSE_y     — root-mean-square error in y (mm)
  - RMSE_pos   — combined position RMSE = sqrt(RMSE_x² + RMSE_y²) (mm)
  - RMSE_theta — heading RMSE using circular difference (rad)

Also prints per-particle error breakdown and a summary table.

Usage:
    # Default: first candidate in the dataset (seed_group 199, candidate 0)
    python test-runs/seeded-visualisation-2/particle_rmse_test.py

    # Specific candidate
    python test-runs/seeded-visualisation-2/particle_rmse_test.py --group 207 --candidate 0

    # With feature type filter
    python test-runs/seeded-visualisation-2/particle_rmse_test.py --feature fCorner
"""

import os
import math
import sys
import csv
import argparse
import statistics

try:
    import matplotlib.pyplot as plt
    import numpy as np
    HAS_MPL = True
except ImportError:
    HAS_MPL = False

# =========================================================================
#  Config
# =========================================================================

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CANDIDATES_PATH = os.path.join(SCRIPT_DIR, 'seed_candidates.csv')
PARTICLES_PATH = os.path.join(SCRIPT_DIR, 'seed_particles.csv')
FIGURE_DIR = os.path.join(SCRIPT_DIR, 'rmse_figures')

# =========================================================================
#  Helpers
# =========================================================================


def normalize_angle(theta):
    while theta > math.pi:
        theta -= 2.0 * math.pi
    while theta < -math.pi:
        theta += 2.0 * math.pi
    return theta


def angle_diff(a, b):
    return normalize_angle(a - b)


def rmse(values, reference):
    if not values:
        return float('nan')
    return math.sqrt(sum((v - reference) ** 2 for v in values) / len(values))


def circular_rmse(angles_rad, reference_rad):
    """RMSE of circular angles around a reference, using circular difference."""
    if not angles_rad:
        return float('nan')
    return math.sqrt(sum(angle_diff(a, reference_rad) ** 2 for a in angles_rad) / len(angles_rad))


def circular_mean(thetas):
    """Circular mean of a list of angles in radians."""
    if not thetas:
        return float('nan')
    sin_sum = sum(math.sin(t) for t in thetas)
    cos_sum = sum(math.cos(t) for t in thetas)
    return math.atan2(sin_sum, cos_sum)


def load_csv(path):
    """Load CSV into list of dicts, converting numeric fields."""
    rows = []
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            numeric = {}
            for k, v in row.items():
                try:
                    numeric[k] = float(v)
                except ValueError:
                    numeric[k] = v
            rows.append(numeric)
    return rows


# =========================================================================
#  Candidate Selection
# =========================================================================


def list_available(candidates):
    """Print available seed groups and candidate ranges."""
    from collections import Counter
    groups = sorted(set(c['seed_group_id'] for c in candidates))
    print(f"\n  Available seed groups: {len(groups)}")
    for g in groups[:10]:
        c_in_g = [c for c in candidates if c['seed_group_id'] == g]
        idxs = sorted(set(c['candidate_idx'] for c in c_in_g))
        ft = c_in_g[0]['feature_type']
        print(f"    Group {g:>7.0f}  ({ft:>12s})  candidates: {min(idxs):.0f}–{max(idxs):.0f}"
              f"  ({len(idxs)} total)")
    if len(groups) > 10:
        print(f"    ... and {len(groups) - 10} more groups")


def find_candidate(candidates, particles, group_id, candidate_idx):
    """
    Find a specific candidate and its particles.
    If group_id is None, use the first seed_group in the dataset.
    If candidate_idx is None, use 0.
    """
    # Determine the group
    if group_id is None:
        first_row = candidates[0]
        group_id = first_row['seed_group_id']
        print(f"  No group specified — defaulting to first seed_group {group_id:.0f}")
        # Use robot pose from first row of this group
        robot_pose = (first_row['robot_pose_x'], first_row['robot_pose_y'], first_row['robot_pose_theta'])
    else:
        # Find a candidate row for this group
        group_rows = [c for c in candidates if c['seed_group_id'] == group_id]
        if not group_rows:
            print(f"  ERROR: Group {group_id:.0f} not found!")
            list_available(candidates)
            sys.exit(1)
        robot_pose = (group_rows[0]['robot_pose_x'], group_rows[0]['robot_pose_y'], group_rows[0]['robot_pose_theta'])

    if candidate_idx is None:
        candidate_idx = 0.0
        print(f"  No candidate specified — defaulting to candidate_idx {candidate_idx:.0f}")

    # Find the candidate row
    cand_row = None
    for c in candidates:
        if c['seed_group_id'] == group_id and c['candidate_idx'] == candidate_idx:
            cand_row = c
            break

    if cand_row is None:
        print(f"  ERROR: Candidate (group={group_id:.0f}, idx={candidate_idx:.0f}) not found!")
        # Show available candidates for this group
        group_rows = [c for c in candidates if c['seed_group_id'] == group_id]
        if group_rows:
            idxs = sorted(set(c['candidate_idx'] for c in group_rows))
            print(f"  Available candidate indices for group {group_id:.0f}: "
                  f"{[f'{i:.0f}' for i in idxs]}")
        sys.exit(1)

    # Find matching particles
    cand_particles = [
        p for p in particles
        if p['seed_group_id'] == group_id and p['candidate_idx'] == candidate_idx
    ]

    if not cand_particles:
        print(f"  ERROR: No particles found for candidate (group={group_id:.0f}, idx={candidate_idx:.0f})!")
        sys.exit(1    )

    return cand_row, cand_particles, robot_pose


# =========================================================================
#  Analysis
# =========================================================================


def analyse_candidate(cand_row, particles, robot_pose):
    """Compute detailed per-particle and aggregate RMSE statistics."""
    rx, ry, rt = robot_pose

    px = [p['particle_x'] for p in particles]
    py = [p['particle_y'] for p in particles]
    pt = [p['particle_theta'] for p in particles]

    # Per-particle errors
    per_particle = []
    for p in particles:
        dx = p['particle_x'] - rx
        dy = p['particle_y'] - ry
        dpos = math.hypot(dx, dy)
        dtheta = angle_diff(p['particle_theta'], rt)
        per_particle.append({
            'x': p['particle_x'],
            'y': p['particle_y'],
            'theta': p['particle_theta'],
            'err_x': dx,
            'err_y': dy,
            'err_pos': dpos,
            'err_theta': dtheta,
        })

    # Aggregate RMSE
    rmse_x = rmse(px, rx)
    rmse_y = rmse(py, ry)
    rmse_pos = math.hypot(rmse_x, rmse_y)
    rmse_theta = circular_rmse(pt, rt)

    # Mean particle (bias)
    mean_x = statistics.mean(px)
    mean_y = statistics.mean(py)
    mean_theta = circular_mean(pt)

    bias_x = mean_x - rx
    bias_y = mean_y - ry
    bias_theta = angle_diff(mean_theta, rt)

    # Candidate's own distance to robot
    cx = cand_row['candidate_x']
    cy = cand_row['candidate_y']
    ct = cand_row['candidate_theta']
    cand_dist_to_robot = math.hypot(cx - rx, cy - ry)
    cand_theta_diff = angle_diff(ct, rt)

    # Feature geometry
    fx = cand_row['feature_x']
    fy = cand_row['feature_y']
    fo = cand_row['feature_orientation']
    robot_to_feature_dist = math.hypot(fx - rx, fy - ry)
    robot_to_feature_heading = normalize_angle(
        math.atan2(fy - ry, fx - rx) - rt
    )
    cand_to_feature_dist = math.hypot(fx - cx, fy - cy)
    cand_to_feature_heading = normalize_angle(
        math.atan2(fy - cy, fx - cx) - ct
    )

    return {
        'per_particle': per_particle,
        'n_particles': len(particles),
        'rmse_x': rmse_x,
        'rmse_y': rmse_y,
        'rmse_pos': rmse_pos,
        'rmse_theta': rmse_theta,
        'mean_x': mean_x,
        'mean_y': mean_y,
        'mean_theta': mean_theta,
        'bias_x': bias_x,
        'bias_y': bias_y,
        'bias_theta': bias_theta,
        'candidate_x': cx,
        'candidate_y': cy,
        'candidate_theta': ct,
        'candidate_dist_to_robot': cand_dist_to_robot,
        'candidate_theta_diff': cand_theta_diff,
        'robot_x': rx,
        'robot_y': ry,
        'robot_theta': rt,
        'feature_x': fx,
        'feature_y': fy,
        'feature_orientation': fo,
        'robot_to_feature_dist': robot_to_feature_dist,
        'robot_to_feature_heading': robot_to_feature_heading,
        'cand_to_feature_dist': cand_to_feature_dist,
        'cand_to_feature_heading': cand_to_feature_heading,
    }


# =========================================================================
#  Output
# =========================================================================


def print_header():
    print()
    print("=" * 78)
    print("  SEEDED PARTICLE FILTER — PER-CANDIDATE RMSE TEST")
    print("  Data: test-runs/seeded-visualisation-2")
    print("=" * 78)


def print_candidate_info(cand_row):
    """Print the candidate's metadata."""
    ft = cand_row['feature_type']
    fx = cand_row['feature_x']
    fy = cand_row['feature_y']
    cx = cand_row['candidate_x']
    cy = cand_row['candidate_y']
    ct = cand_row['candidate_theta']
    od = cand_row['obs_distance']
    oh = cand_row['obs_heading']
    print(f"\n  Candidate (Group {cand_row['seed_group_id']:.0f}, "
          f"Idx {cand_row['candidate_idx']:.0f})")
    print(f"    Feature:           {ft} at ({fx:.0f}, {fy:.0f})")
    print(f"    Observation:       dist={od:.1f} mm, heading={oh:.4f} rad ({math.degrees(oh):.1f}°)")
    print(f"    Candidate pose:    ({cx:.1f}, {cy:.1f}, {math.degrees(ct):.1f}°)")


def print_robot_pose(rx, ry, rt):
    """Print the robot's ground-truth pose."""
    print(f"\n  Robot ground-truth pose:")
    print(f"    ({rx:.1f}, {ry:.1f}, {math.degrees(rt):.1f}°)")


def print_rmse_summary(stats):
    """Print the RMSE summary table."""
    print()
    print("  " + "-" * 56)
    print("  RMSE vs Robot Ground-Truth Pose")
    print("  " + "-" * 56)
    print(f"  {'Quantity':>20s}  {'Value':>14s}  {'Unit':>8s}")
    print(f"  {'--------':>20s}  {'-----':>14s}  {'----':>8s}")
    print(f"  {'RMSE_x':>20s}  {stats['rmse_x']:>10.1f}  {'mm':>8s}")
    print(f"  {'RMSE_y':>20s}  {stats['rmse_y']:>10.1f}  {'mm':>8s}")
    print(f"  {'RMSE_pos (combined)':>20s}  {stats['rmse_pos']:>10.1f}  {'mm':>8s}")
    print(f"  {'RMSE_theta':>20s}  {stats['rmse_theta']:>10.4f}  {'rad':>8s}")
    print(f"  {'RMSE_theta':>20s}  {math.degrees(stats['rmse_theta']):>10.1f}  {'deg':>8s}")
    print(f"  {'N particles':>20s}  {stats['n_particles']:>10d}  {'':>8s}")
    print()
    print(f"  {'Bias_x (mean - robot)':>20s}  {stats['bias_x']:>+10.1f}  {'mm':>8s}")
    print(f"  {'Bias_y (mean - robot)':>20s}  {stats['bias_y']:>+10.1f}  {'mm':>8s}")
    print(f"  {'Bias_theta (mean - robot)':>20s}  {math.degrees(stats['bias_theta']):>+10.1f}  {'deg':>8s}")
    print()
    print(f"  {'Mean particle pose':>20s}  ({stats['mean_x']:>7.0f}, {stats['mean_y']:>7.0f}, "
          f"{math.degrees(stats['mean_theta']):>5.1f}°)")
    print(f"  {'Candidate pose':>20s}  ({stats['candidate_x']:>7.0f}, {stats['candidate_y']:>7.0f}, "
          f"{math.degrees(stats['candidate_theta']):>5.1f}°)")
    print(f"  {'Feature pose':>20s}  ({stats['feature_x']:>7.0f}, {stats['feature_y']:>7.0f}, "
          f"{math.degrees(stats['feature_orientation']):>5.1f}°)")
    print(f"  {'Robot pose':>20s}  ({stats['robot_x']:>7.0f}, {stats['robot_y']:>7.0f}, "
          f"{math.degrees(stats['robot_theta']):>5.1f}°)")
    print()
    print(f"  {'Candidate→robot dist':>20s}  {stats['candidate_dist_to_robot']:>10.1f}  {'mm':>8s}")
    print(f"  {'Candidate θ diff→robot':>20s}  {math.degrees(stats['candidate_theta_diff']):>+10.1f}  {'deg':>8s}")
    print(f"  {'Candidate→feature dist':>20s}  {stats['cand_to_feature_dist']:>10.1f}  {'mm':>8s}")
    print(f"  {'Candidate→feature heading':>20s}  {math.degrees(stats['cand_to_feature_heading']):>+10.1f}  {'deg':>8s}")
    print(f"  {'Robot→feature dist':>20s}  {stats['robot_to_feature_dist']:>10.1f}  {'mm':>8s}")
    print(f"  {'Robot→feature heading':>20s}  {math.degrees(stats['robot_to_feature_heading']):>+10.1f}  {'deg':>8s}")
    print("  " + "-" * 56)


def print_per_particle(stats, max_rows=20):
    """Print per-particle error table (first max_rows)."""
    print()
    print(f"  Per-particle errors (showing first {min(max_rows, len(stats['per_particle']))} of "
          f"{len(stats['per_particle'])} particles):")
    print(f"  {'#':>4s}  {'Particle_x':>10s}  {'Particle_y':>10s}  "
          f"{'Particle_θ':>10s}  {'Err_x':>9s}  {'Err_y':>9s}  "
          f"{'Err_pos':>9s}  {'Err_θ(deg)':>10s}")
    print(f"  {'----':>4s}  {'----------':>10s}  {'----------':>10s}  "
          f"{'----------':>10s}  {'------':>9s}  {'------':>9s}  "
          f"{'-------':>9s}  {'----------':>10s}")
    for i, pp in enumerate(stats['per_particle'][:max_rows]):
        print(f"  {i:>4d}  {pp['x']:>10.1f}  {pp['y']:>10.1f}  "
              f"{math.degrees(pp['theta']):>10.1f}  {pp['err_x']:>+9.1f}  "
              f"{pp['err_y']:>+9.1f}  {pp['err_pos']:>9.1f}  "
              f"{math.degrees(pp['err_theta']):>+10.1f}")

    if len(stats['per_particle']) > max_rows:
        print(f"  ... ({len(stats['per_particle']) - max_rows} more particles)")


def write_results_csv(stats, path):
    """Write per-particle errors to CSV."""
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['=== CANDIDATE SUMMARY ==='])
        writer.writerow(['seed_group_id', stats.get('seed_group_id', '')])
        writer.writerow(['candidate_idx', stats.get('candidate_idx', '')])
        writer.writerow(['feature_type', stats.get('feature_type', '')])
        writer.writerow(['n_particles', stats['n_particles']])
        writer.writerow(['feature_x', f"{stats['feature_x']:.1f}"])
        writer.writerow(['feature_y', f"{stats['feature_y']:.1f}"])
        writer.writerow(['feature_orientation_rad', f"{stats['feature_orientation']:.4f}"])
        writer.writerow(['feature_orientation_deg', f"{math.degrees(stats['feature_orientation']):.1f}"])
        writer.writerow(['robot_x', f"{stats['robot_x']:.1f}"])
        writer.writerow(['robot_y', f"{stats['robot_y']:.1f}"])
        writer.writerow(['robot_theta_rad', f"{stats['robot_theta']:.4f}"])
        writer.writerow(['robot_theta_deg', f"{math.degrees(stats['robot_theta']):.1f}"])
        writer.writerow(['rmse_x_mm', f"{stats['rmse_x']:.1f}"])
        writer.writerow(['rmse_y_mm', f"{stats['rmse_y']:.1f}"])
        writer.writerow(['rmse_pos_mm', f"{stats['rmse_pos']:.1f}"])
        writer.writerow(['rmse_theta_rad', f"{stats['rmse_theta']:.4f}"])
        writer.writerow(['rmse_theta_deg', f"{math.degrees(stats['rmse_theta']):.1f}"])
        writer.writerow(['bias_x_mm', f"{stats['bias_x']:.1f}"])
        writer.writerow(['bias_y_mm', f"{stats['bias_y']:.1f}"])
        writer.writerow(['bias_theta_deg', f"{math.degrees(stats['bias_theta']):.1f}"])
        writer.writerow([])
        writer.writerow(['=== PER-PARTICLE ERRORS ==='])
        writer.writerow([
            'particle_x', 'particle_y', 'particle_theta_rad', 'particle_theta_deg',
            'err_x_mm', 'err_y_mm', 'err_pos_mm',
            'err_theta_rad', 'err_theta_deg',
        ])
        for pp in stats['per_particle']:
            writer.writerow([
                f"{pp['x']:.1f}", f"{pp['y']:.1f}",
                f"{pp['theta']:.4f}", f"{math.degrees(pp['theta']):.1f}",
                f"{pp['err_x']:.1f}", f"{pp['err_y']:.1f}", f"{pp['err_pos']:.1f}",
                f"{pp['err_theta']:.4f}", f"{math.degrees(pp['err_theta']):.1f}",
            ])

    print(f"\n  Per-particle CSV written to: {path}")


# =========================================================================
#  Plotting
# =========================================================================


def plot_particle_errors(stats):
    """Plot diagnostic figures for the selected candidate."""
    os.makedirs(FIGURE_DIR, exist_ok=True)
    pp = stats['per_particle']

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # 1. Scatter: particle positions with robot pose
    ax = axes[0, 0]
    xs = [p['x'] for p in pp]
    ys = [p['y'] for p in pp]
    errs_pos = [p['err_pos'] for p in pp]
    sc = ax.scatter(xs, ys, c=errs_pos, cmap='viridis', s=30, alpha=0.7,
                    vmin=0, vmax=max(errs_pos))
    ax.scatter(stats['robot_x'], stats['robot_y'], marker='*', s=300,
               color='red', edgecolors='white', linewidths=2, zorder=10,
               label='Robot pose')
    ax.scatter(stats['candidate_x'], stats['candidate_y'], marker='s', s=150,
               color='blue', edgecolors='white', linewidths=2, zorder=9,
               label='Candidate pose')
    ax.scatter(stats['feature_x'], stats['feature_y'], marker='D', s=180,
               color='green', edgecolors='white', linewidths=2, zorder=9,
               label=f"Feature ({stats.get('feature_type','?')})")
    # Draw heading arrows
    arrow_len = 200
    ax.arrow(stats['robot_x'], stats['robot_y'],
             arrow_len * math.cos(stats['robot_theta']),
             arrow_len * math.sin(stats['robot_theta']),
             head_width=60, head_length=80, fc='red', ec='red', alpha=0.7)
    ax.arrow(stats['candidate_x'], stats['candidate_y'],
             arrow_len * math.cos(stats['candidate_theta']),
             arrow_len * math.sin(stats['candidate_theta']),
             head_width=60, head_length=80, fc='blue', ec='blue', alpha=0.7)
    ax.arrow(stats['feature_x'], stats['feature_y'],
             arrow_len * math.cos(stats['feature_orientation']),
             arrow_len * math.sin(stats['feature_orientation']),
             head_width=60, head_length=80, fc='green', ec='green', alpha=0.7)
    # Pose annotations
    offset = 180
    ax.annotate(
        f"Robot: ({stats['robot_x']:.0f}, {stats['robot_y']:.0f}, {math.degrees(stats['robot_theta']):.1f}°)",
        xy=(stats['robot_x'], stats['robot_y']),
        xytext=(stats['robot_x'] + offset, stats['robot_y'] + offset),
        fontsize=8, color='red', fontweight='bold',
        arrowprops=dict(arrowstyle='->', color='red', alpha=0.6))
    ax.annotate(
        f"Cand: ({stats['candidate_x']:.0f}, {stats['candidate_y']:.0f}, {math.degrees(stats['candidate_theta']):.1f}°)",
        xy=(stats['candidate_x'], stats['candidate_y']),
        xytext=(stats['candidate_x'] + offset, stats['candidate_y'] - offset),
        fontsize=8, color='blue', fontweight='bold',
        arrowprops=dict(arrowstyle='->', color='blue', alpha=0.6))
    ax.annotate(
        f"Feat: ({stats['feature_x']:.0f}, {stats['feature_y']:.0f}, {math.degrees(stats['feature_orientation']):.1f}°)",
        xy=(stats['feature_x'], stats['feature_y']),
        xytext=(stats['feature_x'] - offset, stats['feature_y'] + offset),
        fontsize=8, color='green', fontweight='bold',
        arrowprops=dict(arrowstyle='->', color='green', alpha=0.6))
    plt.colorbar(sc, ax=ax, label='Position error (mm)')
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_title(f'Particle Positions (n={len(pp)})')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

    # 2. Histogram: position error
    ax = axes[0, 1]
    ax.hist(errs_pos, bins=30, color='#2196f3', alpha=0.7, edgecolor='white')
    ax.axvline(stats['rmse_pos'], color='red', linestyle='--',
               label=f"RMSE_pos={stats['rmse_pos']:.0f} mm")
    ax.set_xlabel('Position error (mm)')
    ax.set_ylabel('Number of particles')
    ax.set_title('Position Error Distribution')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 3. Histogram: heading error
    ax = axes[1, 0]
    err_thetas = [math.degrees(p['err_theta']) for p in pp]
    ax.hist(err_thetas, bins=30, color='#4caf50', alpha=0.7, edgecolor='white')
    ax.axvline(math.degrees(stats['rmse_theta']), color='red', linestyle='--',
               label=f"RMSE_θ={math.degrees(stats['rmse_theta']):.1f}°")
    ax.set_xlabel('Heading error (deg)')
    ax.set_ylabel('Number of particles')
    ax.set_title('Heading Error Distribution')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 4. Error vs particle index (to check for systematic patterns)
    ax = axes[1, 1]
    ax.plot(range(len(pp)), errs_pos, 'o-', markersize=3, color='#ff9800', alpha=0.6,
            label='Position error')
    ax.axhline(stats['rmse_pos'], color='red', linestyle='--',
               label=f"RMSE_pos={stats['rmse_pos']:.0f} mm")
    ax.set_xlabel('Particle index')
    ax.set_ylabel('Position error (mm)')
    ax.set_title('Position Error by Particle Index')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.suptitle(
        f"Candidate RMSE — Group {stats.get('seed_group_id','?'):.0f}, "
        f"Idx {stats.get('candidate_idx','?'):.0f}  "
        f"({stats.get('feature_type','?')})",
        fontsize=14, fontweight='bold'
    )
    plt.tight_layout()
    fig_path = os.path.join(FIGURE_DIR,
                            f"candidate_g{stats['seed_group_id']:.0f}_"
                            f"c{stats['candidate_idx']:.0f}.png")
    fig.savefig(fig_path, dpi=150)
    print(f"  Figure saved to: {fig_path}")
    plt.close(fig)


# =========================================================================
#  Main
# =========================================================================


def parse_args():
    parser = argparse.ArgumentParser(
        description='Per-Candidate RMSE Test: compare seeded PF particles '
                    'against robot ground-truth pose.'
    )
    parser.add_argument('-g', '--group', type=float, default=None,
                        help='Seed group ID (default: first group in dataset)')
    parser.add_argument('-c', '--candidate', type=float, default=None,
                        help='Candidate index within group (default: 0)')
    parser.add_argument('-f', '--feature', type=str, default=None,
                        help='Filter by feature type (e.g., fCorner, fXJunction)')
    parser.add_argument('--list', action='store_true',
                        help='List available seed groups and exit')
    parser.add_argument('--csv', type=str, default=None,
                        help='Output CSV path (default: rmse_figures/candidate_*.csv)')
    parser.add_argument('--no-fig', action='store_true',
                        help='Skip figure generation')
    return parser.parse_args()


def main():
    args = parse_args()

    print_header()

    # Load data
    print(f"\n  Loading candidates: {CANDIDATES_PATH}")
    candidates = load_csv(CANDIDATES_PATH)
    print(f"  Loading particles:  {PARTICLES_PATH}")
    particles = load_csv(PARTICLES_PATH)
    print(f"  {len(candidates)} candidates, {len(particles)} particles")

    # --list mode
    if args.list:
        list_available(candidates)
        print()
        return

    # Filter by feature type if requested
    if args.feature:
        candidates = [c for c in candidates if c['feature_type'] == args.feature]
        if not candidates:
            print(f"  ERROR: No candidates found with feature_type='{args.feature}'")
            sys.exit(1)
        print(f"  Filtered to {len(candidates)} candidates with feature_type='{args.feature}'")
        # Also filter particles to only matching (group, candidate) pairs
        valid_keys = set((c['seed_group_id'], c['candidate_idx']) for c in candidates)
        particles = [p for p in particles
                     if (p['seed_group_id'], p['candidate_idx']) in valid_keys]
        print(f"  Filtered to {len(particles)} particles for matching candidates")

    # Select candidate
    cand_row, cand_particles, robot_pose = find_candidate(
        candidates, particles, args.group, args.candidate
    )

    # Analyse
    stats = analyse_candidate(cand_row, cand_particles, robot_pose)
    stats['seed_group_id'] = cand_row['seed_group_id']
    stats['candidate_idx'] = cand_row['candidate_idx']
    stats['feature_type'] = cand_row['feature_type']

    # Print
    print_candidate_info(cand_row)
    print_robot_pose(*robot_pose)
    print_rmse_summary(stats)
    print_per_particle(stats, max_rows=20)

    # CSV
    csv_path = args.csv or os.path.join(
        FIGURE_DIR,
        f"candidate_g{cand_row['seed_group_id']:.0f}_c{cand_row['candidate_idx']:.0f}.csv"
    )
    write_results_csv(stats, csv_path)

    # Figures
    if HAS_MPL and not args.no_fig:
        print()
        print("-" * 78)
        print("  GENERATING FIGURES")
        print("-" * 78)
        try:
            plot_particle_errors(stats)
        except Exception as e:
            print(f"  WARNING: Figure generation failed: {e}")
            import traceback
            traceback.print_exc()

    print()
    print("=" * 78)
    print("  DONE")
    print("=" * 78)


if __name__ == '__main__':
    main()
