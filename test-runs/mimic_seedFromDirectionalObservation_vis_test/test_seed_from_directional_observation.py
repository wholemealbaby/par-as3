#!/usr/bin/env python3
"""
Test seedFromDirectionalObservation — Visualisable Geometry Test
=================================================================

Replicates the exact geometry computation from the C++ implementation:
    ParticleFilter::seedFromDirectionalObservation() in
    soccer/Src/robot/src/perception/stateestimation/localiser/particlefilter/ParticleFilter.cpp

The C++ code (lines 609–612):
    obsX     = ff.x + distance * cos(ff.orientation + obsOrientation)
    obsY     = ff.y + distance * sin(ff.orientation + obsOrientation)
    obsTheta = normaliseTheta(atan2(ff.y - obsY, ff.x - obsX) - obsHeading)

This test:
  1️⃣ Defines the LabLarge field geometry and known field features (same as
     FieldFeatureLocations.cpp).
  2️⃣ Creates synthetic test scenarios: robot at known poses looking at
     specific field features.
  3️⃣ For each scenario, generates a synthetic observation (distance, heading,
     orientation) that the robot would see — computed *forward* from the
     known robot pose and feature location.
  4️⃣ Runs the seed geometry *backward* to compute the implied robot pose.
  5️⃣ Verifies geometric correctness (computed pose should match the known
     robot pose to numerical precision).
  6️⃣ Prints detailed per-candidate analysis.
  7️⃣ Visualises multiple test cases on a RoboCup field diagram.

Usage:
    python test_seed_from_directional_observation.py

Dependencies:
    numpy, matplotlib
"""

import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from dataclasses import dataclass
from typing import List, Tuple


# ═══════════════════════════════════════════════════════════════════════
#  1. Field Definitions  —  LabLarge  (matches C++ LabLarge.hpp)
# ═══════════════════════════════════════════════════════════════════════

FIELD_LENGTH = 6300.0       # mm
FIELD_WIDTH = 4050.0        # mm
FIELD_LENGTH_OFFSET = 510.0 # mm
FIELD_WIDTH_OFFSET = 930.0  # mm

HALF_L = FIELD_LENGTH / 2.0    # 3150
HALF_W = FIELD_WIDTH / 2.0     # 2025

GOAL_BOX_LENGTH = 450.0
GOAL_BOX_WIDTH = 1800.0
PENALTY_BOX_LENGTH = 1350.0
PENALTY_BOX_WIDTH = 2700.0
CENTER_CIRCLE_DIAMETER = 1350.0
CORNER_ARC_RADIUS = 225.0

M_PI_2 = math.pi / 2.0
M_PI_4 = math.pi / 4.0


@dataclass
class FieldFeature:
    """Matches C++ types/field/FieldFeature.hpp"""
    x: float
    y: float
    orientation: float


@dataclass
class DirectionlessFieldFeature:
    x: float
    y: float


def normalise_theta(theta: float) -> float:
    """Normalise angle to [-π, π] — matches C++ normaliseTheta()."""
    while theta > math.pi:
        theta -= 2.0 * math.pi
    while theta < -math.pi:
        theta += 2.0 * math.pi
    return theta


# ── Build field feature database (mirrors FieldFeatureLocations.cpp) ──

def build_field_features():
    """Return field feature lists exactly as in C++ FieldFeatureLocations.cpp
    for the LabLarge field configuration."""

    # ── Corners ────────────────────────────────────────────────────
    corners = [
        FieldFeature(-HALF_L,              HALF_W,            -M_PI_4),
        FieldFeature(-HALF_L + GOAL_BOX_LENGTH,     GOAL_BOX_WIDTH / 2.0,  -3.0 * M_PI_4),
        FieldFeature(-HALF_L + PENALTY_BOX_LENGTH,   PENALTY_BOX_WIDTH / 2.0,  -3.0 * M_PI_4),
        FieldFeature(-HALF_L + GOAL_BOX_LENGTH,    -GOAL_BOX_WIDTH / 2.0,   3.0 * M_PI_4),
        FieldFeature(-HALF_L + PENALTY_BOX_LENGTH,  -PENALTY_BOX_WIDTH / 2.0,  3.0 * M_PI_4),
        FieldFeature(-HALF_L,             -HALF_W,            M_PI_4),
        FieldFeature( HALF_L,              HALF_W,            -3.0 * M_PI_4),
        FieldFeature( HALF_L - GOAL_BOX_LENGTH,     GOAL_BOX_WIDTH / 2.0,   -M_PI_4),
        FieldFeature( HALF_L - PENALTY_BOX_LENGTH,   PENALTY_BOX_WIDTH / 2.0,  -M_PI_4),
        FieldFeature( HALF_L - GOAL_BOX_LENGTH,    -GOAL_BOX_WIDTH / 2.0,   M_PI_4),
        FieldFeature( HALF_L - PENALTY_BOX_LENGTH,  -PENALTY_BOX_WIDTH / 2.0,  M_PI_4),
        FieldFeature( HALF_L,             -HALF_W,            3.0 * M_PI_4),
    ]

    # ── T-Junctions ────────────────────────────────────────────────
    t_junctions = [
        FieldFeature(-HALF_L,  GOAL_BOX_WIDTH / 2.0,   0.0),
        FieldFeature(-HALF_L,  PENALTY_BOX_WIDTH / 2.0, 0.0),
        FieldFeature(-HALF_L, -GOAL_BOX_WIDTH / 2.0,   0.0),
        FieldFeature(-HALF_L, -PENALTY_BOX_WIDTH / 2.0, 0.0),
        FieldFeature(0.0,      HALF_W,                 -M_PI_2),
        FieldFeature(0.0,     -HALF_W,                  M_PI_2),
        FieldFeature( HALF_L,  GOAL_BOX_WIDTH / 2.0,   math.pi),
        FieldFeature( HALF_L,  PENALTY_BOX_WIDTH / 2.0, math.pi),
        FieldFeature( HALF_L, -GOAL_BOX_WIDTH / 2.0,   math.pi),
        FieldFeature( HALF_L, -PENALTY_BOX_WIDTH / 2.0, math.pi),
    ]

    # ── Corner arcs (T-junctions at arc endpoints) ─────────────────
    if CORNER_ARC_RADIUS > 0:
        # Left Top Arc
        t_junctions.append(FieldFeature(-HALF_L,  HALF_W - CORNER_ARC_RADIUS, 0.0))
        t_junctions.append(FieldFeature(-HALF_L + CORNER_ARC_RADIUS, HALF_W, -M_PI_2))
        # Left Bottom Arc
        t_junctions.append(FieldFeature(-HALF_L, -HALF_W + CORNER_ARC_RADIUS, 0.0))
        t_junctions.append(FieldFeature(-HALF_L + CORNER_ARC_RADIUS, -HALF_W, M_PI_2))
        # Right Top Arc
        t_junctions.append(FieldFeature(HALF_L,  HALF_W - CORNER_ARC_RADIUS, math.pi))
        t_junctions.append(FieldFeature(HALF_L - CORNER_ARC_RADIUS, HALF_W, -M_PI_2))
        # Right Bottom Arc
        t_junctions.append(FieldFeature(HALF_L, -HALF_W + CORNER_ARC_RADIUS, math.pi))
        t_junctions.append(FieldFeature(HALF_L - CORNER_ARC_RADIUS, -HALF_W, M_PI_2))

    # ── X-Junctions ────────────────────────────────────────────────
    x_junctions = [
        FieldFeature(0.0,  CENTER_CIRCLE_DIAMETER / 2.0, 0.0),
        FieldFeature(0.0, -CENTER_CIRCLE_DIAMETER / 2.0, 0.0),
    ]

    # ── Centre Circles (directional — two orientations) ────────────
    centre_circles = [
        FieldFeature(0.0, 0.0,  M_PI_2),   # orientation 1
        FieldFeature(0.0, 0.0, -M_PI_2),   # orientation 2
    ]

    return corners, t_junctions, x_junctions, centre_circles


# ═══════════════════════════════════════════════════════════════════════
#  2. Geometry Functions  —  matches C++ seedFromDirectionalObservation
# ═══════════════════════════════════════════════════════════════════════

def compute_seed_pose(
    ff_x: float, ff_y: float, ff_orientation: float,
    obs_distance: float, obs_heading: float, obs_orientation: float
) -> Tuple[float, float, float]:
    """
    Replicates ParticleFilter::seedFromDirectionalObservation() exactly.

    Args:
        ff_x, ff_y, ff_orientation: Known field feature location.
        obs_distance:  Measured distance from robot to feature (mm).
        obs_heading:   Bearing of feature relative to robot heading (rad).
        obs_orientation: Observed orientation offset of feature (rad).

    Returns:
        (obsX, obsY, obsTheta) — the computed robot pose.
    """
    obs_x = ff_x + obs_distance * math.cos(ff_orientation + obs_orientation)
    obs_y = ff_y + obs_distance * math.sin(ff_orientation + obs_orientation)
    obs_theta = normalise_theta(
        math.atan2(ff_y - obs_y, ff_x - obs_x) - obs_heading
    )
    return obs_x, obs_y, obs_theta


def generate_observation(
    robot_x: float, robot_y: float, robot_theta: float,
    ff_x: float, ff_y: float, ff_orientation: float
) -> Tuple[float, float, float]:
    """
    Generate a synthetic observation that a robot at (robot_x, robot_y,
    robot_theta) would see of a field feature at (ff_x, ff_y, ff_orientation).

    This is the *forward* computation: given known robot pose and feature,
    compute what the sensor would report.

    Returns:
        (distance, heading, orientation) — the observation tuple.
    """
    # Vector from robot to feature
    dx = ff_x - robot_x
    dy = ff_y - robot_y
    distance = math.sqrt(dx * dx + dy * dy)

    # Heading: bearing of feature relative to robot's forward direction
    # direction_to_feature = atan2(dy, dx)
    # heading = direction_to_feature - robot_theta
    heading = normalise_theta(math.atan2(dy, dx) - robot_theta)

    # Orientation: angle from feature's known orientation to the direction
    # from feature to robot
    # direction_from_feature_to_robot = atan2(robot_y - ff_y, robot_x - ff_x)
    # orientation = direction_from_feature_to_robot - ff_orientation
    orientation = normalise_theta(
        math.atan2(robot_y - ff_y, robot_x - ff_x) - ff_orientation
    )

    return distance, heading, orientation


# ═══════════════════════════════════════════════════════════════════════
#  3. Test Scenarios
# ═══════════════════════════════════════════════════════════════════════

@dataclass
class TestCase:
    """A single test case for seedFromDirectionalObservation."""
    name: str
    robot_x: float
    robot_y: float
    robot_theta: float
    feature: FieldFeature
    feature_type: str           # e.g. "Corner", "T-Junction", "X-Junction", "CentreCircle"


def build_test_cases() -> List[TestCase]:
    """Build a battery of synthetic test scenarios."""
    corners, t_junctions, x_junctions, centre_circles = build_field_features()

    cases = []

    # ── Test 1: Robot at origin looking at Left Top corner ─────────
    cases.append(TestCase(
        name="Robot at origin → Left Top corner",
        robot_x=0.0, robot_y=0.0, robot_theta=0.0,
        feature=corners[0],  # Left Top corner
        feature_type="fCorner",
    ))

    # ── Test 2: Robot in bottom-left quadrant, facing right ────────
    cases.append(TestCase(
        name="Robot in bottom-left → Right Goal Top T-Junction",
        robot_x=-2000.0, robot_y=-1000.0, robot_theta=0.5,
        feature=t_junctions[6],  # Right Goal Top T-Junction
        feature_type="fTJunction",
    ))

    # ── Test 3: Robot near right goal, looking back at centre circle ─
    cases.append(TestCase(
        name="Robot near right goal → Centre Circle (ori 1)",
        robot_x=2500.0, robot_y=500.0, robot_theta=math.pi,
        feature=centre_circles[0],  # Centre Circle orientation 1
        feature_type="fCentreCircle",
    ))

    # ── Test 4: Robot at top-centre looking down at bottom X-junction ─
    cases.append(TestCase(
        name="Robot at top-centre → Bottom X-Junction",
        robot_x=0.0, robot_y=1500.0, robot_theta=-math.pi/2,
        feature=x_junctions[1],  # Bottom X-Junction
        feature_type="fXJunction",
    ))

    # ── Test 5: Robot at various positions near centre ──────────────
    cases.append(TestCase(
        name="Robot slightly right of centre → Left Penalty Bottom corner",
        robot_x=500.0, robot_y=300.0, robot_theta=2.0,
        feature=corners[4],  # Left Penalty Bottom corner
        feature_type="fCorner",
    ))

    # ── Test 6: Robot near the centre line, facing goal ─────────────
    cases.append(TestCase(
        name="Robot at right side → Centre Circle (ori 2)",
        robot_x=1000.0, robot_y=-800.0, robot_theta=2.5,
        feature=centre_circles[1],  # Centre Circle orientation 2
        feature_type="fCentreCircle",
    ))

    # ── Test 7: Robot near left penalty box ─────────────────────────
    cases.append(TestCase(
        name="Robot near left penalty → Left Goal Top corner",
        robot_x=-2500.0, robot_y=500.0, robot_theta=1.2,
        feature=corners[1],  # Left Goal Top corner
        feature_type="fCorner",
    ))

    # ── Test 8: Robot at top sideline, looking at centre T ──────────
    cases.append(TestCase(
        name="Robot at top sideline → Centre Top T-Junction",
        robot_x=800.0, robot_y=1800.0, robot_theta=-0.8,
        feature=t_junctions[4],  # Centre Top T-Junction
        feature_type="fTJunction",
    ))

    return cases


# ═══════════════════════════════════════════════════════════════════════
#  4. Verification and Analysis
# ═══════════════════════════════════════════════════════════════════════

@dataclass
class TestResult:
    """Result of running a single test case."""
    tc: TestCase
    computed_x: float
    computed_y: float
    computed_theta: float
    error_x: float
    error_y: float
    error_theta: float
    error_position: float
    obs_distance: float
    obs_heading: float
    obs_orientation: float
    passed: bool
    feature_world_angle: float       # direction from robot to feature in world frame


def run_test(tc: TestCase) -> TestResult:
    """Run a single test case through the seed geometry and verify."""
    # Generate synthetic observation
    dist, heading, ori = generate_observation(
        tc.robot_x, tc.robot_y, tc.robot_theta,
        tc.feature.x, tc.feature.y, tc.feature.orientation
    )

    # Compute seed pose
    cx, cy, ctheta = compute_seed_pose(
        tc.feature.x, tc.feature.y, tc.feature.orientation,
        dist, heading, ori
    )

    # Errors
    error_x = cx - tc.robot_x
    error_y = cy - tc.robot_y
    error_theta = normalise_theta(ctheta - tc.robot_theta)
    error_position = math.sqrt(error_x**2 + error_y**2)

    # PASS if position error < 1 mm and heading error < 1e-6 rad
    passed = error_position < 1.0 and abs(error_theta) < 1e-6

    # Direction from robot to feature in world frame
    feature_world_angle = math.atan2(
        tc.feature.y - tc.robot_y,
        tc.feature.x - tc.robot_x
    )

    return TestResult(
        tc=tc,
        computed_x=cx, computed_y=cy, computed_theta=ctheta,
        error_x=error_x, error_y=error_y, error_theta=error_theta,
        error_position=error_position,
        obs_distance=dist, obs_heading=heading, obs_orientation=ori,
        passed=passed,
        feature_world_angle=feature_world_angle,
    )


def print_analysis(results: List[TestResult]):
    """Print detailed per-candidate analysis to stdout."""
    passed_count = sum(1 for r in results if r.passed)
    total = len(results)

    print("=" * 88)
    print(f"  seedFromDirectionalObservation  —  Geometry Test Results")
    print(f"  (Replicating ParticleFilter.cpp lines 609-612)")
    print("=" * 88)
    print(f"  LabLarge field: {FIELD_LENGTH} × {FIELD_WIDTH} mm")
    print(f"  Test cases:     {total}  ({passed_count} passed, "
          f"{total - passed_count} failed)")
    print()

    for i, r in enumerate(results):
        tc = r.tc
        status = "✅ PASS" if r.passed else "❌ FAIL"
        print(f"  ── Test {i+1}: {tc.name}  {status}")
        print(f"     Feature type:  {tc.feature_type}")
        print(f"     Feature loc:   ({tc.feature.x:8.1f}, {tc.feature.y:8.1f}, "
              f"{math.degrees(tc.feature.orientation):7.2f}°)")

        print(f"     ┌─ Known robot pose")
        print(f"     │  x     = {tc.robot_x:12.2f} mm")
        print(f"     │  y     = {tc.robot_y:12.2f} mm")
        print(f"     │  theta = {math.degrees(tc.robot_theta):12.4f}°")

        print(f"     ├─ Synthetic observation (what the robot would see)")
        print(f"     │  distance    = {r.obs_distance:12.2f} mm")
        print(f"     │  heading     = {math.degrees(r.obs_heading):12.4f}°")
        print(f"     │  orientation = {math.degrees(r.obs_orientation):12.4f}°")
        print(f"     │  (bearing to feature in robot frame: "
              f"{math.degrees(r.feature_world_angle - tc.robot_theta):7.2f}°)")

        print(f"     ├─ Computed seed pose (should match robot pose)")
        print(f"     │  x     = {r.computed_x:12.2f} mm")
        print(f"     │  y     = {r.computed_y:12.2f} mm")
        print(f"     │  theta = {math.degrees(r.computed_theta):12.4f}°")

        print(f"     └─ Error")
        print(f"        Δx       = {r.error_x:12.4f} mm")
        print(f"        Δy       = {r.error_y:12.4f} mm")
        print(f"        Δtheta   = {math.degrees(r.error_theta):12.6f}°")
        print(f"        position = {r.error_position:12.4f} mm")
        print()

    print("=" * 88)
    print(f"  Summary: {passed_count}/{total} tests passed")
    if passed_count == total:
        print("  ✓ All tests passed — seed geometry is geometrically correct.")
    else:
        print(f"  ✗ {total - passed_count} tests failed.")
    print("=" * 88)


# ═══════════════════════════════════════════════════════════════════════
#  5. Visualisation
# ═══════════════════════════════════════════════════════════════════════

def draw_field(ax):
    """Draw the LabLarge field outline and all known features."""
    # ── Green carpet ──
    carpet_half_x = HALF_L + FIELD_LENGTH_OFFSET
    carpet_half_y = HALF_W + FIELD_WIDTH_OFFSET
    carpet = mpatches.Rectangle(
        (-carpet_half_x, -carpet_half_y),
        2 * carpet_half_x, 2 * carpet_half_y,
        linewidth=0, facecolor='#c8e6c9', zorder=0
    )
    ax.add_patch(carpet)

    # ── Field boundary ──
    field = mpatches.Rectangle(
        (-HALF_L, -HALF_W), FIELD_LENGTH, FIELD_WIDTH,
        linewidth=2, edgecolor='white', facecolor='none', zorder=1
    )
    ax.add_patch(field)

    # ── Centre line ──
    ax.axvline(0, color='white', linewidth=1.5, zorder=1, alpha=0.7)

    # ── Centre circle ──
    centre_circle = plt.Circle(
        (0, 0), CENTER_CIRCLE_DIAMETER / 2,
        linewidth=1.5, edgecolor='white', facecolor='none', zorder=1
    )
    ax.add_patch(centre_circle)

    # ── Penalty boxes ──
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

    # ── Goal boxes ──
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

    # ── Known features (faint reference markers) ──
    corners, t_juncs, x_juncs, centre_circles = build_field_features()
    for f in corners:
        ax.plot(f.x, f.y, marker='s', color='gray', markersize=3, alpha=0.3, zorder=1)
    for f in t_juncs:
        ax.plot(f.x, f.y, marker='^', color='gray', markersize=3, alpha=0.3, zorder=1)
    for f in x_juncs:
        ax.plot(f.x, f.y, marker='x', color='gray', markersize=4, alpha=0.3, zorder=1)
    for f in centre_circles:
        ax.plot(f.x, f.y, marker='o', color='gray', markersize=4, alpha=0.3, zorder=1)


def visualise_test_case(
    r: TestResult,
    ax,
    show_candidate_label: bool = True,
    show_robot_label: bool = True,
):
    """
    Visualise a single test case on the given axes.

    Draws:
      - The field feature (coloured marker with orientation arrow)
      - An observation line from robot to feature
      - The known robot pose (blue arrow)
      - The computed seed pose (green arrow — should overlap blue)
    """
    tc = r.tc
    feat = tc.feature

    # ── Feature colour mapping ──
    feature_colours = {
        'fCorner': '#e53935',
        'fTJunction': '#8e24aa',
        'fXJunction': '#ffb300',
        'fCentreCircle': '#00acc1',
    }
    feat_colour = feature_colours.get(tc.feature_type, '#333333')

    # ── Draw feature marker with orientation arrow ──
    ax.scatter(feat.x, feat.y, marker='s' if tc.feature_type == 'fCorner'
               else ('^' if tc.feature_type == 'fTJunction'
                     else ('X' if tc.feature_type == 'fXJunction' else 'o')),
               s=100, color=feat_colour, edgecolors='white', linewidths=1.5,
               zorder=6, label=f'{tc.feature_type}' if show_candidate_label else '')

    # Feature orientation arrow
    ori_len = 200
    ori_dx = ori_len * math.cos(feat.orientation)
    ori_dy = ori_len * math.sin(feat.orientation)
    ax.arrow(feat.x, feat.y, ori_dx, ori_dy,
             head_width=60, head_length=80, fc=feat_colour, ec=feat_colour,
             linewidth=2.0, zorder=6, alpha=0.7)

    # ── Observation line (robot → feature) ──
    ax.plot([tc.robot_x, feat.x], [tc.robot_y, feat.y],
            color=feat_colour, linewidth=1.5, linestyle='--', alpha=0.6, zorder=3)

    # ── Known robot pose (blue) ──
    arrow_len = 400
    rdx = arrow_len * math.cos(tc.robot_theta)
    rdy = arrow_len * math.sin(tc.robot_theta)
    ax.arrow(tc.robot_x, tc.robot_y, rdx, rdy,
             head_width=120, head_length=150, fc='#1565c0', ec='#0d47a1',
             linewidth=3, zorder=8, alpha=0.9)
    ax.scatter(tc.robot_x, tc.robot_y, marker='o', s=80,
               color='#1565c0', edgecolors='white', linewidths=2, zorder=8)

    if show_robot_label:
        ax.annotate(
            f"Robot\n({tc.robot_x:.0f}, {tc.robot_y:.0f})",
            xy=(tc.robot_x, tc.robot_y),
            xytext=(tc.robot_x + arrow_len + 150, tc.robot_y + arrow_len + 150),
            fontsize=8, color='#0d47a1', fontweight='bold',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='white',
                      edgecolor='#1565c0', alpha=0.85),
            arrowprops=dict(arrowstyle='->', color='#1565c0', lw=1.2),
            zorder=9
        )

    # ── Computed seed pose (green — should overlap blue exactly) ──
    cdx = arrow_len * math.cos(r.computed_theta)
    cdy = arrow_len * math.sin(r.computed_theta)
    ax.arrow(r.computed_x, r.computed_y, cdx, cdy,
             head_width=120, head_length=150, fc='#2e7d32', ec='#1b5e20',
             linewidth=2, zorder=7, alpha=0.7, linestyle='dotted')
    ax.scatter(r.computed_x, r.computed_y, marker='o', s=60,
               color='#2e7d32', edgecolors='white', linewidths=1.5, zorder=7)

    if show_candidate_label:
        ax.annotate(
            f"Seed\n({r.computed_x:.0f}, {r.computed_y:.0f})",
            xy=(r.computed_x, r.computed_y),
            xytext=(r.computed_x + arrow_len + 100, r.computed_y - arrow_len - 100),
            fontsize=8, color='#1b5e20', fontweight='bold',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='white',
                      edgecolor='#2e7d32', alpha=0.85),
            arrowprops=dict(arrowstyle='->', color='#2e7d32', lw=1.2),
            zorder=9
        )

    # ── Observation annotation ──
    mid_x = (tc.robot_x + feat.x) / 2
    mid_y = (tc.robot_y + feat.y) / 2
    ax.annotate(
        f"d={r.obs_distance:.0f} mm\n"
        f"h={math.degrees(r.obs_heading):.1f}°",
        xy=(mid_x, mid_y),
        fontsize=7, color=feat_colour, alpha=0.7,
        bbox=dict(boxstyle='round,pad=0.2', facecolor='white',
                  edgecolor=feat_colour, alpha=0.6),
        zorder=5
    )


def visualise_all_tests(results: List[TestResult]):
    """
    Create a 2×4 grid of subplots, one per test case, plus a combined
    overview plot with all test cases overlaid.
    """
    n_tests = len(results)
    n_cols = 4
    n_rows = (n_tests + n_cols - 1) // n_cols

    # ── Individual subplots ──
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(n_cols * 5, n_rows * 4.5))
    axes = axes.flatten()

    for idx, r in enumerate(results):
        ax = axes[idx]
        tc = r.tc
        status = "✅" if r.passed else "❌"

        draw_field(ax)
        visualise_test_case(r, ax)

        ax.set_title(f"Test {idx+1}: {status}\n{tc.name}", fontsize=9,
                     fontweight='bold')
        ax.set_xlabel('X (mm)', fontsize=8)
        ax.set_ylabel('Y (mm)', fontsize=8)
        ax.set_aspect('equal')
        margin = 600
        ax.set_xlim(-HALF_L - margin, HALF_L + margin)
        ax.set_ylim(-HALF_W - margin, HALF_W + margin)
        ax.grid(True, alpha=0.1, linestyle='--')
        ax.tick_params(labelsize=7)

    # Hide unused subplots
    for idx in range(n_tests, len(axes)):
        axes[idx].set_visible(False)

    plt.suptitle("seedFromDirectionalObservation — Per-Test Visualisation",
                 fontsize=14, fontweight='bold', y=1.01)
    plt.tight_layout()

    # ── Combined overview ──
    fig2, ax2 = plt.subplots(figsize=(14, 10))
    draw_field(ax2)

    # Draw all features faintly
    corners, t_juncs, x_juncs, centre_circles = build_field_features()
    legend_elements = []

    for idx, r in enumerate(results):
        tc = r.tc

        # Feature marker
        fc = {
            'fCorner': '#e53935', 'fTJunction': '#8e24aa',
            'fXJunction': '#ffb300', 'fCentreCircle': '#00acc1',
        }.get(tc.feature_type, '#333333')

        ax2.scatter(tc.feature.x, tc.feature.y,
                    marker='o', s=80, color=fc, edgecolors='white',
                    linewidths=1.5, zorder=6, alpha=0.7)

        # Observation line
        ax2.plot([tc.robot_x, tc.feature.x], [tc.robot_y, tc.feature.y],
                 color=fc, linewidth=1.0, linestyle=':', alpha=0.5, zorder=3)

        # Robot pose (coloured by test index)
        color = plt.cm.tab10(idx / max(n_tests - 1, 1))
        arrow_len = 300
        rdx = arrow_len * math.cos(tc.robot_theta)
        rdy = arrow_len * math.sin(tc.robot_theta)
        ax2.arrow(tc.robot_x, tc.robot_y, rdx, rdy,
                  head_width=100, head_length=120, fc=color, ec=color,
                  linewidth=2, zorder=7, alpha=0.8)
        ax2.scatter(tc.robot_x, tc.robot_y, marker='o', s=50,
                    color=color, edgecolors='white', linewidths=1.5, zorder=7)

        # Test number label
        ax2.annotate(
            str(idx + 1),
            xy=(tc.robot_x, tc.robot_y),
            xytext=(tc.robot_x + rdx + 50, tc.robot_y + rdy + 50),
            fontsize=8, color=color, fontweight='bold',
            bbox=dict(boxstyle='round,pad=0.2', facecolor='white',
                      edgecolor=color, alpha=0.8),
            arrowprops=dict(arrowstyle='->', color=color, lw=1.0),
            zorder=8
        )

        legend_elements.append(
            mpatches.Patch(color=color, label=f"Test {idx+1}: {tc.name[:30]}")
        )

    # Feature type legend
    for ft, fc in [('fCorner', '#e53935'), ('fTJunction', '#8e24aa'),
                   ('fXJunction', '#ffb300'), ('fCentreCircle', '#00acc1')]:
        legend_elements.append(
            mpatches.Patch(color=fc, label=f'Observed {ft}', alpha=0.7)
        )

    ax2.legend(handles=legend_elements, loc='upper right', fontsize=8,
               framealpha=0.9, ncol=2)

    ax2.set_title(f"seedFromDirectionalObservation — Combined Overview "
                  f"({n_tests} test cases)", fontsize=14, fontweight='bold')
    ax2.set_xlabel('X (mm)', fontsize=11)
    ax2.set_ylabel('Y (mm)', fontsize=11)
    ax2.set_aspect('equal')
    margin = 800
    ax2.set_xlim(-HALF_L - margin, HALF_L + margin)
    ax2.set_ylim(-HALF_W - margin, HALF_W + margin)
    ax2.grid(True, alpha=0.15, linestyle='--')

    plt.tight_layout()
    plt.show()


# ═══════════════════════════════════════════════════════════════════════
#  Main
# ═══════════════════════════════════════════════════════════════════════

def main():
    print("Building test cases...")
    test_cases = build_test_cases()
    print(f"  {len(test_cases)} test cases created")
    print()

    print("Running tests...")
    results = [run_test(tc) for tc in test_cases]
    print(f"  All tests complete")
    print()

    # ── Print analysis ──
    print_analysis(results)

    # ── Visualise ──
    print("Rendering visualisations...")
    try:
        visualise_all_tests(results)
    except Exception as e:
        print(f"Warning: visualisation failed: {e}")
        print("(Check that matplotlib is installed and DISPLAY is set.)")


if __name__ == '__main__':
    main()
