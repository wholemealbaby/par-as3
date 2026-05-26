# PF-16 — Selective Initialisation Phase 1: Directional Feature Seeding

**Labels:** `enhancement` `pf` `par`
**Priority:** P1 — improves convergence speed with low complexity
**Estimates:** ~45 min

### Objective

Add observation-driven particle seeding for **directional field features** (corners, T-junctions, X-junctions, directional centre circle). When the robot spots a directional feature with known orientation, we can compute a **unique implied robot pose** and seed a Gaussian cluster there — giving the filter a head start instead of waiting for particles to wander into the right region by random resampling.

### Background

Currently, `ParticleFilter::createParticles()` scatters particles uniformly across the entire field. The filter then relies on weight updates + resampling to gradually converge. This is slow when the robot has no initial pose estimate.

Directional features contain both distance AND bearing information, plus the feature's own orientation on the field. The math to compute the implied robot pose already exists in `ParticleFilter::update()` (lines 389–392).

### Feature Types in Scope (Phase 1)

| Feature Type | Field Locations | Symmetry-Breaking Value | Seed Positions |
|---|---|---|---|
| `fCorner` (non-YOLO) | 12 corners | Unique per corner | 1 pose per corner matched |
| `fTJunction` (non-YOLO) | 10 T-junctions | Unique per junction | 1 pose per junction matched |
| `fXJunction` (non-YOLO) | 2 X-junctions | Unique per junction | 1 pose per junction matched |
| `fCentreCircle` (directional) | 1 centre circle (2 orientations) | Unique per orientation | 1 pose per orientation matched |

Total candidate feature locations: 12 + 10 + 2 + 2 = **26 possible seeded poses** across the field.

### Design: Decomposed Functions

#### 1. `seedParticlesFromObservation()` — Top-Level Dispatcher

Called from `observationUpdate()` **after** the existing weight-update logic, only when the filter has high uncertainty. Iterates over observed field features and dispatches to per-type seed functions.

```
seedParticlesFromObservation(estimatorInfoIn)
├── guard: uncertainty < seedUncertaintyThreshold → return early
├── for each observation in estimatorInfoIn.fieldFeatures:
│   ├── fCorner / fTJunction / fXJunction / fCentreCircle
│   │   └── seedFromDirectionalObservation(observation, candidatePoses)
│   └── (other types: deferred to PF-17)
└── if candidatePoses not empty → mergeSeededParticles(candidatePoses)
```

#### 2. `seedFromDirectionalObservation()` — Per-Feature Matcher

Iterates the known feature locations of the matching type. For each, computes the implied robot pose using the same math as `ParticleFilter::update()` (lines 389–392), but only collects poses — does not modify particle weights.

```
seedFromDirectionalObservation(observation, candidatePoses)
├── select feature vector by type (corners / t_junctions / x_junctions / centre_circles)
├── for each FieldFeature ff in vector:
│   ├── obsX = ff.x + obs.distance * cos(ff.orientation + obs.orientation)
│   ├── obsY = ff.y + obs.distance * sin(ff.orientation + obs.orientation)
│   ├── obsTheta = normaliseTheta(atan2(ff.y - obsY, ff.x - obsX) - obs.heading)
│   └── candidatePoses.push_back({obsX, obsY, obsTheta})
└── return
```

#### 3. `mergeSeededParticles()` — Particle Set Merging

Keeps the highest-weight fraction of existing particles clears the rest and seeds new Gaussian clusters around each candidate pose. This preserves filter state from observations already processed in this frame while adding informed hypotheses.

```
mergeSeededParticles(candidatePoses)
├── sort existing particles by weight (descending)
├── keep top keepFraction → survivors
├── seedCount = numParticles - survivors.size()
├── perPose = seedCount / candidatePoses.size()
├── for each (x, y, theta) in candidatePoses:
│   ├── for i in 0..perPose:
│   │   ├── px = x + Gaussian(0, seedPositionNoise)
│   │   ├── py = y + Gaussian(0, seedPositionNoise)
│   │   ├── pt = normaliseTheta(theta + Gaussian(0, seedHeadingNoise))
│   │   └── survivors.push_back(Particle(px, py, pt, uniformWeight))
│   └──
├── fill any remaining slots with uniform random particles
├── *particles = move(survivors)
└── normaliseWeights()
```

### New Parameters (in `ParticleFilterParams`)

| Parameter | Default | Purpose |
|---|---|---|
| `seedUncertaintyThreshold` | 800.0 mm | Only seed when position uncertainty exceeds this |
| `seedKeepFraction` | 0.5 | Fraction of existing particles to keep (rest are seeded) |
| `seedPositionNoise` | 150.0 mm | Gaussian noise std dev for seeded particle positions |
| `seedHeadingNoise` | 0.3 rad | Gaussian noise std dev for seeded particle headings (~17°) |

### Files to Modify

| File | Change |
|------|--------|
| `ParticleFilter.hpp` | Add 3 new private method declarations |
| `ParticleFilter.cpp` | Add 3 new method implementations; update `observationUpdate()` to call `seedParticlesFromObservation()` |
| `ParticleFilterParams.hpp` | Add 4 new parameter fields |
| `ParticleFilterParams.cpp` | Add default values and config-file options for the 4 params |
| `ParticleFilterConstants.hpp` | Add 4 new `#define` constants for default values |

### Where to Call in `observationUpdate()`

At the **end** of `observationUpdate()`, after the existing weight-update loop:

```
observationUpdate(estimatorInfoIn, estimatorInfoMiddle)
├── guard: !canDoObservations → return
├── ── existing weight-update loop ──
│   for each observation in estimatorInfoIn.fieldFeatures:
│       switch (observation.type): ...  // unchanged
│   └──
├── ── NEW: seed from observations if high uncertainty ──
└── seedParticlesFromObservation(estimatorInfoIn)
```

### Dependencies

- Existing `ParticleFilter::update()` math (reuses the same implied-pose computation)
- Existing `FieldFeatureLocations` database
- Particle normalisation (`normaliseWeights()`)

### Verification

- [ ] `./Make/Linux/compile rbb -p Booster` compiles successfully
- [ ] With high initial uncertainty seeing a corner instantly spawns a particle cluster near the correct region
- [ ] With low uncertainty (converged filter) seeding is skipped (no disruption)
- [ ] Particle count remains constant (300) after seeding
- [ ] Weights sum to 1 after `normaliseWeights()`

### Known Limitations

- Directional features that are YOLO-detected are **not** used (they are directionless). This is correct behaviour per the existing codebase convention.
- If the feature detection is wrong (false positive) the seeded cluster will be in the wrong place — but the particle filter naturally corrects this via weight updates in subsequent frames.
- `mergeSeededParticles()` uses simple weight-based sorting. A more sophisticated approach could use k-means clustering to identify existing particle modes and preserve them. That is out of scope for PF-16.
