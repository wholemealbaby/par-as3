# PF-17 — Selective Initialisation Phase 2: Directionless Feature Seeding (Annulus Sampling)

**Labels:** `enhancement` `pf` `par`
**Priority:** P2 — improves convergence but adds complexity over PF-16
**Depends on:** PF-16 (uses the same `mergeSeededParticles()` helper and `seedParticlesFromObservation()` dispatcher)
**Estimates:** ~60 min

### Objective

Add observation-driven particle seeding for **directionless field features** (penalty spots, centre circle, YOLO-detected features). Directionless features provide only distance — not bearing or orientation — placing the robot anywhere on a **circle (annulus)** around the landmark. Rather than a single pose, we sample particles uniformly along that circle clipped to the field boundary.

### Background

Directionless features provide a robot-relative **distance only**. The bearing and orientation of the feature are unknown, so the robot could be anywhere on a circle of radius `observedDistance` around the landmark.

**Field landmark annotations in `FieldFeatureLocations`:**

| Feature Type | Known Landmarks | Count |
|---|---|---|
| `penalty_spots` | `(-PENALTY_MARK_ABS_X, 0)` and `(+PENALTY_MARK_ABS_X, 0)` | 2 spots |
| `centre_circles_directionless` | `(0, 0)` | 1 centre |

### The Annulus Problem

For a directionless observation of a single landmark at `(lx, ly)` with observed distance `d`:

- The robot's position satisfies: `(x - lx)² + (y - ly)² = d²`
- This is a **continuous circle** — not a discrete set of points
- We cannot enumerate all positions; we must **sample** them
- The circle may extend off-field; we clip to valid field coordinates

**Additional constraint:** The robot's heading is unconstrained by a single directionless observation. We seed particles with heading pointing generally toward the landmark (one plausible hypothesis) or uniformly random.

### How Many Hypothesis Modes?

| Landmark | Annuli | Off-Field Clipping | Effective Modes |
|---|---|---|---|
| Penalty spot (left half) | 1 circle around `(-PENALTY_MARK_ABS_X, 0)` | Portion extending beyond field edges clipped | ~1 arc on field |
| Penalty spot (right half) | 1 circle around `(+PENALTY_MARK_ABS_X, 0)` | Portion extending beyond field edges clipped | ~1 arc on field |
| Centre circle | 1 circle around `(0, 0)` | Portion extending beyond field edges clipped | ~1 full circle on field |

All annuli are seeded simultaneously. The particle filter naturally converges to the correct one.
