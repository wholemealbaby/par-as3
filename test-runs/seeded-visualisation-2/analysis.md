## Root Cause Analysis

The candidate poses from [`seedFromDirectionalObservation()`](soccer/Src/robot/src/perception/stateestimation/localiser/particlefilter/ParticleFilter.cpp:603) are **geometrically correct** given the observation values — the function correctly inverts the observation model. The problem is that the **observation itself doesn't match the actual robot–feature geometry**.

For seed group 199 (LabLarge field: 6300×4050mm):

| Quantity | Value |
|----------|-------|
| Robot pose | (−1518, 2007.5, −113.5°) |
| X‑junction location | (0, 675, orientation=0°) |
| **Actual** distance robot→feature | **~2020 mm** |
| **Observed** distance | **689 mm** (−66% error) |
| **Actual** bearing robot→feature | **−41°** |
| **Observed** heading | **+75°** (+116° error) |

The candidate poses at (689, 675, 105°) are the correct inversion of the observation — "a robot 689 mm to the *right* of the center-circle X-junction, facing ~105°" — but the *actual* robot is 2020 mm to the *left* of it. The observation is inconsistent with reality. Seed groups 207+ (fCorner) show the same ~70% distance error.

---

## Recommended Improvements (Priority Order)

### 1. Scale seed noise by observation uncertainty (most principled)

Instead of a fixed [`seedPositionNoise`](soccer/Src/robot/include/perception/stateestimation/localiser/particlefilter/ParticleFilterParams.hpp:105), use the same uncertainty model as the observation update to scale the cluster spread per-candidate:

```cpp
// In seedFromDirectionalObservation() or mergeSeededParticles()
float radialUncertainty = params->distanceUncertaintyBase 
                        + observation.rr.distance() * params->distanceUncertaintyScale;
float effectivePosNoise = std::max(params->seedPositionNoise, radialUncertainty);
```

A noisy distant observation → wider seed cloud → more overlap with the true pose.

### 2. Increase `seedKeepFraction` to preserve diversity

Currently [`seedKeepFraction = 0.5`](soccer/Src/robot/include/perception/stateestimation/localiser/particlefilter/ParticleFilterParams.hpp:98) — only 50% of existing particles survive. Bump to **0.7–0.8** so the original uniform particles maintain global coverage and act as a safety net when all candidates are wrong.

### 3. Reject off-field candidates

Add a simple sanity check in [`seedFromDirectionalObservation()`](soccer/Src/robot/src/perception/stateestimation/localiser/particlefilter/ParticleFilter.cpp:603) — skip candidates outside the field carpet:

```cpp
if (std::abs(obsX) > FIELD_LENGTH/2 + FIELD_LENGTH_OFFSET ||
    std::abs(obsY) > FIELD_WIDTH/2 + FIELD_WIDTH_OFFSET)
    return;  // Don't seed — implausible pose
```

This would reject many fCorner candidates (e.g., x=−4119, x=−4739) that land far outside the field boundary.

### 4. Reserve 10–20% uniform particles

In [`mergeSeededParticles()`](soccer/Src/robot/src/perception/stateestimation/localiser/particlefilter/ParticleFilter.cpp:684), ensure a minimum fraction of particles are always uniformly distributed as a safety net.

### 5. Investigate observation quality upstream

The 66–70% distance error suggests a possible coordinate-frame mismatch or synthetic-data issue. If this is from a simulation/test harness, verify the observation generator uses the same reference frame (robot body frame vs camera frame vs field frame) as the landmark database.

### 6. Temporal consensus (most robust, most code)

Accumulate candidate poses over N frames and only seed when the same candidate appears consistently — filters out transient bad observations entirely.