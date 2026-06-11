#!/usr/bin/env python3
"""
Visualise Seeded Particle Filter Data (v2 — with robot pose)

Reads seed_candidates.csv and seed_particles.csv from the same directory,
and renders a matplotlib plot for the first seed_group_id showing:
  - The soccer field outline (LabLarge: 6300 x 4050 mm)
  - Known field features (corners, T-junctions, X-junctions)
  - Feature observations (coloured markers)
  - Candidate robot poses (black arrows with orientation)
  - Associated particles (scatter points, colour-coded by candidate_idx)
  - **Robot's actual ground-truth pose** (large blue arrow + label)

Also prints detailed per-candidate information:
  - Mean particle cluster pose (circular mean of theta)
  - Candidate pose coords
  - Feature observation coords

Usage:
    python visualise_seeded_pf.py
"""

import os
import math
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.colors import Normalize
import numpy as np


def circular_mean(thetas):
    """Compute the circular mean of a list of angles (in radians).

    Uses the standard circular mean via sin/cos averaging.
    Returns the mean angle in radians in the range [-pi, pi].
    """
    sin_sum = sum(math.sin(t) for t in thetas)
    cos_sum = sum(math.cos(t) for t in thetas)
    return math.atan2(sin_sum, cos_sum)

# Field dimensions: LabLarge configuration
FIELD_LENGTH = 6300   # mm
FIELD_WIDTH = 4050    # mm
FIELD_LENGTH_OFFSET = 510   # mm
FIELD_WIDTH_OFFSET = 930    # mm

# Derived field feature positions (from FieldFeatureLocations.cpp using LabLarge)
HALF_L = FIELD_LENGTH / 2.0   # 3150
HALF_W = FIELD_WIDTH / 2.0    # 2025

# Goal / Penalty / Goal-box dimensions for LabLarge
# (from the definition headers; approximate for visualisation)
GOAL_BOX_LENGTH = 600
GOAL_BOX_WIDTH = 2200
PENALTY_BOX_LENGTH = 1350
PENALTY_BOX_WIDTH = 4050
CENTER_CIRCLE_DIAMETER = 1200


def build_field_features():
    """Return lists of known field feature positions for LabLarge."""
    corners = [
        (-HALF_L, HALF_W, -math.pi/4),                          # Left Top
        (-HALF_L + GOAL_BOX_LENGTH, GOAL_BOX_WIDTH/2, -3*math.pi/4),   # Left Goal Top
        (-HALF_L + PENALTY_BOX_LENGTH, PENALTY_BOX_WIDTH/2, -3*math.pi/4), # Left Penalty Top
        (-HALF_L + GOAL_BOX_LENGTH, -GOAL_BOX_WIDTH/2, 3*math.pi/4),    # Left Goal Bottom
        (-HALF_L + PENALTY_BOX_LENGTH, -PENALTY_BOX_WIDTH/2, 3*math.pi/4), # Left Penalty Bottom
        (-HALF_L, -HALF_W, math.pi/4),                          # Left Bottom
        (HALF_L, HALF_W, -3*math.pi/4),                         # Right Top
        (HALF_L - GOAL_BOX_LENGTH, GOAL_BOX_WIDTH/2, -math.pi/4),      # Right Goal Top
        (HALF_L - PENALTY_BOX_LENGTH, PENALTY_BOX_WIDTH/2, -math.pi/4), # Right Penalty Top
        (HALF_L - GOAL_BOX_LENGTH, -GOAL_BOX_WIDTH/2, math.pi/4),      # Right Goal Bottom
        (HALF_L - PENALTY_BOX_LENGTH, -PENALTY_BOX_WIDTH/2, math.pi/4), # Right Penalty Bottom
        (HALF_L, -HALF_W, 3*math.pi/4),                         # Right Bottom
    ]
    t_junctions = [
        (-HALF_L, GOAL_BOX_WIDTH/2, 0),
        (-HALF_L, PENALTY_BOX_WIDTH/2, 0),
        (-HALF_L, -GOAL_BOX_WIDTH/2, 0),
        (-HALF_L, -PENALTY_BOX_WIDTH/2, 0),
        (0, HALF_W, -math.pi/2),
        (0, -HALF_W, math.pi/2),
        (HALF_L, GOAL_BOX_WIDTH/2, math.pi),
        (HALF_L, PENALTY_BOX_WIDTH/2, math.pi),
        (HALF_L, -GOAL_BOX_WIDTH/2, math.pi),
        (HALF_L, -PENALTY_BOX_WIDTH/2, math.pi),
    ]
    x_junctions = [
        (0, CENTER_CIRCLE_DIAMETER/2, 0),
        (0, -CENTER_CIRCLE_DIAMETER/2, 0),
    ]
    return corners, t_junctions, x_junctions


def draw_field(ax):
    """Draw the soccer field outline and known features on the given axes."""
    # --- Green carpet (full court including offset) ---
    carpet_half_x = HALF_L + FIELD_LENGTH_OFFSET
    carpet_half_y = HALF_W + FIELD_WIDTH_OFFSET
    carpet = mpatches.Rectangle(
        (-carpet_half_x, -carpet_half_y),
        2 * carpet_half_x, 2 * carpet_half_y,
        linewidth=0, facecolor='#c8e6c9', zorder=0
    )
    ax.add_patch(carpet)

    # --- Field playing area (white boundary) ---
    field = mpatches.Rectangle(
        (-HALF_L, -HALF_W),
        FIELD_LENGTH, FIELD_WIDTH,
        linewidth=2, edgecolor='white', facecolor='none', zorder=1
    )
    ax.add_patch(field)

    # --- Centre line ---
    ax.axvline(0, ymin=0.05, ymax=0.95, color='white', linewidth=1.5, zorder=1)

    # --- Centre circle ---
    centre_circle = plt.Circle(
        (0, 0), CENTER_CIRCLE_DIAMETER / 2,
        linewidth=1.5, edgecolor='white', facecolor='none', zorder=1
    )
    ax.add_patch(centre_circle)

    # --- Penalty boxes ---
    left_penalty = mpatches.Rectangle(
        (-HALF_L, -PENALTY_BOX_WIDTH/2),
        PENALTY_BOX_LENGTH, PENALTY_BOX_WIDTH,
        linewidth=1.5, edgecolor='white', facecolor='none', zorder=1
    )
    ax.add_patch(left_penalty)
    right_penalty = mpatches.Rectangle(
        (HALF_L - PENALTY_BOX_LENGTH, -PENALTY_BOX_WIDTH/2),
        PENALTY_BOX_LENGTH, PENALTY_BOX_WIDTH,
        linewidth=1.5, edgecolor='white', facecolor='none', zorder=1
    )
    ax.add_patch(right_penalty)

    # --- Goal boxes ---
    left_goal = mpatches.Rectangle(
        (-HALF_L, -GOAL_BOX_WIDTH/2),
        GOAL_BOX_LENGTH, GOAL_BOX_WIDTH,
        linewidth=1.5, edgecolor='white', facecolor='none', zorder=1
    )
    ax.add_patch(left_goal)
    right_goal = mpatches.Rectangle(
        (HALF_L - GOAL_BOX_LENGTH, -GOAL_BOX_WIDTH/2),
        GOAL_BOX_LENGTH, GOAL_BOX_WIDTH,
        linewidth=1.5, edgecolor='white', facecolor='none', zorder=1
    )
    ax.add_patch(right_goal)

    # --- Known feature reference points (faint) ---
    corners, t_juncs, x_juncs = build_field_features()

    for cx, cy, _ in corners:
        ax.plot(cx, cy, marker='s', color='gray', markersize=3,
                alpha=0.3, zorder=1)
    for tx, ty, _ in t_juncs:
        ax.plot(tx, ty, marker='^', color='gray', markersize=3,
                alpha=0.3, zorder=1)
    for xx, xy, _ in x_juncs:
        ax.plot(xx, xy, marker='x', color='gray', markersize=4,
                alpha=0.3, zorder=1)


def draw_robot_pose(ax, rx, ry, rtheta):
    """Draw the robot's actual ground-truth pose as a prominent blue arrow.

    Returns a legend handle and label string.
    """
    arrow_len = 500  # mm — slightly longer than candidate arrows for prominence
    dx = arrow_len * math.cos(rtheta)
    dy = arrow_len * math.sin(rtheta)

    ax.arrow(rx, ry, dx, dy,
             head_width=180, head_length=220, fc='#1565c0', ec='#0d47a1',
             linewidth=4, zorder=10, alpha=0.9)

    # Bold dot at robot position
    ax.scatter(rx, ry, marker='o', s=120, color='#1565c0',
               edgecolors='white', linewidths=2, zorder=10)

    # Text label showing coordinates + heading
    label = f"Robot pose\n({rx:.0f}, {ry:.0f}, {math.degrees(rtheta):.1f}°)"
    ax.annotate(
        label,
        xy=(rx, ry),
        xytext=(rx + arrow_len + 250, ry + arrow_len + 250),
        fontsize=9, color='#0d47a1', fontweight='bold',
        bbox=dict(boxstyle='round,pad=0.3', facecolor='white',
                  edgecolor='#1565c0', alpha=0.9),
        arrowprops=dict(arrowstyle='->', color='#1565c0', lw=1.5),
        zorder=11
    )

    return mpatches.Patch(color='#1565c0', label=f'Robot pose ({rx:.0f}, {ry:.0f})')


def main():
    # Determine data directory (same as script location)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidates_path = os.path.join(script_dir, 'seed_candidates.csv')
    particles_path = os.path.join(script_dir, 'seed_particles.csv')

    # Load data
    candidates = pd.read_csv(candidates_path)
    particles = pd.read_csv(particles_path)

    # Verify robot_pose columns exist
    required_pose_cols = ['robot_pose_x', 'robot_pose_y', 'robot_pose_theta']
    has_robot_pose = all(col in candidates.columns for col in required_pose_cols)
    if not has_robot_pose:
        print("WARNING: robot_pose columns not found in candidates CSV. "
              "Falling back to original v1 behaviour (no robot pose).")
        robot_pose_x = robot_pose_y = robot_pose_theta = None
    else:
        robot_pose_x = candidates['robot_pose_x'].iloc[0]
        robot_pose_y = candidates['robot_pose_y'].iloc[0]
        robot_pose_theta = candidates['robot_pose_theta'].iloc[0]

    for i in range(0, 10):
        # Get the seed_group_id
        first_group = candidates['seed_group_id'].iloc[i]
        print(f"\n{'='*72}")
        print(f"Visualising seed_group_id = {first_group}")

        # Filter for this group
        group_candidates = candidates[candidates['seed_group_id'] == first_group].copy()
        group_particles = particles[particles['seed_group_id'] == first_group].copy()

        n_candidates = group_candidates['candidate_idx'].nunique()
        n_particles = len(group_particles)
        print(f"  Candidates: {n_candidates}")
        print(f"  Particles:  {n_particles}")
        print(f"  Feature types: {group_candidates['feature_type'].unique()}")

        # --- Print robot pose ---
        if has_robot_pose:
            print(f"\n  Robot pose:")
            print(f"    (x={robot_pose_x:.1f}, y={robot_pose_y:.1f}, "
                  f"theta={math.degrees(robot_pose_theta):.1f}°)")

        # --- Print per-candidate details ---
        print(f"\n  Per-candidate details:")
        for _, cand in group_candidates.iterrows():
            idx = cand['candidate_idx']
            ft = cand['feature_type']

            # Candidate pose
            cx, cy, ctheta = cand['candidate_x'], cand['candidate_y'], cand['candidate_theta']
            print(f"\n    Candidate {idx} ({ft}):")
            print(f"      Candidate pose:    (x={cx:.1f}, y={cy:.1f}, "
                  f"theta={math.degrees(ctheta):.1f}°)")

            # Feature observation
            fx, fy, fori = cand['feature_x'], cand['feature_y'], cand['feature_orientation']
            print(f"      Feature obs:       (x={fx:.1f}, y={fy:.1f}, "
                  f"orientation={math.degrees(fori):.1f}°)")

            # Mean particle pose for this candidate
            cand_parts = group_particles[group_particles['candidate_idx'] == idx]
            if len(cand_parts) > 0:
                mean_x = cand_parts['particle_x'].mean()
                mean_y = cand_parts['particle_y'].mean()
                mean_theta = circular_mean(cand_parts['particle_theta'].tolist())
                print(f"      Mean particle pose: (x={mean_x:.1f}, y={mean_y:.1f}, "
                      f"theta={math.degrees(mean_theta):.1f}°)  "
                      f"(n={len(cand_parts)} particles)")
            else:
                print(f"      Mean particle pose: (no particles for this candidate)")

        print()

        # --- Create the plot ---
        fig, ax = plt.subplots(figsize=(12, 9))

        # Draw field background
        draw_field(ax)

        # Colour mapping by candidate_idx
        cmap = plt.get_cmap('tab10')
        norm = Normalize(vmin=0, vmax=max(group_candidates['candidate_idx'].max(), 1))

        # Feature-type markers and colours
        feature_type_markers = {
            'fCorner': 's',
            'fXJunction': 'X',
            'fTJunction': '^',
        }
        feature_type_colors = {
            'fCorner': '#e53935',
            'fXJunction': '#ffb300',
            'fTJunction': '#8e24aa',
        }

        # For each candidate, draw feature, pose arrow, and particles
        for _, cand in group_candidates.iterrows():
            idx = cand['candidate_idx']
            ft = cand['feature_type']
            color = cmap(norm(idx))

            # --- Feature observation ---
            fx, fy, fori = cand['feature_x'], cand['feature_y'], cand['feature_orientation']
            marker = feature_type_markers.get(ft, 'o')
            fc = feature_type_colors.get(ft, '#333333')
            ax.scatter(fx, fy, marker=marker, s=120, color=fc,
                    edgecolors='white', linewidths=1.5, zorder=5,
                    label=f'Feature {ft}' if idx == 0 else '')

            # --- Feature observation heading arrow ---
            heading_arrow_len = 200  # mm
            hdx = heading_arrow_len * math.cos(fori)
            hdy = heading_arrow_len * math.sin(fori)
            ax.arrow(fx, fy, hdx, hdy,
                    head_width=80, head_length=100, fc=fc, ec=fc,
                    linewidth=2.0, zorder=6, alpha=0.9)

            # --- Candidate pose arrow ---
            cx, cy, ctheta = cand['candidate_x'], cand['candidate_y'], cand['candidate_theta']
            arrow_len = 300  # mm
            dx = arrow_len * math.cos(ctheta)
            dy = arrow_len * math.sin(ctheta)
            ax.arrow(cx, cy, dx, dy,
                    head_width=120, head_length=150, fc='black', ec='black',
                    linewidth=2.5, zorder=6, alpha=0.8)

            # Small dot at candidate position
            ax.scatter(cx, cy, marker='o', s=40, color='black', zorder=6)

            # --- Particles for this candidate ---
            cand_particles = group_particles[
                group_particles['candidate_idx'] == idx
            ]
            if len(cand_particles) > 0:
                ax.scatter(
                    cand_particles['particle_x'],
                    cand_particles['particle_y'],
                    marker='.', s=15, color=color, alpha=0.6, zorder=4,
                    label=f'Candidate {idx} particles ({len(cand_particles)})' if idx == 0 else ''
                )

        # --- Robot's actual ground-truth pose ---
        legend_handles = []
        if has_robot_pose:
            handle = draw_robot_pose(ax, robot_pose_x, robot_pose_y, robot_pose_theta)
            legend_handles.append(handle)

        # Build legend entries for feature types
        for ft, marker in feature_type_markers.items():
            fc = feature_type_colors[ft]
            legend_handles.append(
                mpatches.Patch(color=fc, label=f'Observed {ft}', alpha=0.8)
            )

        # Legend
        ax.legend(loc='upper right', fontsize=8, framealpha=0.9)

        # Build title
        if has_robot_pose:
            title = (
                f'Seeded PF — Group {first_group} — Robot at '
                f'({robot_pose_x:.0f}, {robot_pose_y:.0f})\n'
                f'{n_candidates} candidates × ~{n_particles // n_candidates} particles'
            )
        else:
            title = (
                f'Seeded PF — Group {first_group}\n'
                f'{n_candidates} candidates × ~{n_particles // n_candidates} particles'
            )

        # Labels and title
        ax.set_xlabel('X (mm)', fontsize=11)
        ax.set_ylabel('Y (mm)', fontsize=11)
        ax.set_title(title, fontsize=13, fontweight='bold')

        # Aspect ratio and limits
        ax.set_aspect('equal')
        margin = 800
        ax.set_xlim(-HALF_L - margin, HALF_L + margin)
        ax.set_ylim(-HALF_W - margin, HALF_W + margin)

        # Grid
        ax.grid(True, alpha=0.15, linestyle='--')

        plt.tight_layout()
        plt.show()


if __name__ == '__main__':
    main()
