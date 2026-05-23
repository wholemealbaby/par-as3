# PF-03-S03 · Compilation & Runtime Blockers for Particle Filter MVP

**Labels:** `bug` `pf` `par`
**Priority:** P1 — blocks compilation and/or runtime of the particle filter
**Estimates:** ~20 min

### Objective

Fix five defects that prevent the particle filter from compiling or running.

---

## Bug 1 · Missing Parameter Defaults (runtime misbehaviour)

**File:** `ParticleFilterParams.cpp` (constructor, lines 20–37)

The constructor sets defaults for most fields but **misses three**:

```cpp
ParticleFilterParams::ParticleFilterParams()
{
    // ... all existing fields are set ...
    // ❌ minWeightAdjustment       NEVER INITIALISED
    // ❌ updateHeadingUncertainty   NEVER INITIALISED
    // ❌ distanceUncertaintyScale   NEVER INITIALISED
}
```

These are **used at runtime** and will contain garbage:

| Field | Used In | Purpose |
|-------|---------|---------|
| `minWeightAdjustment` | `update()`, `updateUsingDirectionlessObservation()` | Weight multiplier floor |
| `updateHeadingUncertainty` | `update()` | Heading uncertainty for obs covariance |
| `distanceUncertaintyScale` | `update()`, `updateUsingDirectionlessObservation()` | Scaled distance uncertainty |

### Fix A1 — Add constants to `ParticleFilterConstants.hpp`

After the `ParticleState` alias:

```cpp
#define MIN_WEIGHT_ADJUSTMENT 0.01f
#define UPDATE_HEADING_UNCERTAINTY 30.0f
#define DISTANCE_UNCERTAINTY_SCALE 0.25f
```

### Fix A2 — Initialize in constructor

Add after `minWeight = 0.05f;`:

```cpp
minWeightAdjustment = MIN_WEIGHT_ADJUSTMENT;
updateHeadingUncertainty = UPDATE_HEADING_UNCERTAINTY;
distanceUncertaintyScale = DISTANCE_UNCERTAINTY_SCALE;
```

### Fix A3 — Config file parser

In `readParams()`, add three new options:

```cpp
("MinWeightAdjustment", po::value<float>(&minWeightAdjustment)->default_value(MIN_WEIGHT_ADJUSTMENT))
("UpdateHeadingUncertainty", po::value<float>(&updateHeadingUncertainty)->default_value(UPDATE_HEADING_UNCERTAINTY))
("DistanceUncertaintyScale", po::value<float>(&distanceUncertaintyScale)->default_value(DISTANCE_UNCERTAINTY_SCALE))
```

---

## Bug 2 · `params` Pointer Never Allocated (Segfault)

**File:** `ParticleFilter.cpp` (constructor, lines 27–36)

```cpp
ParticleFilter::ParticleFilter(const EstimatorInfoInit& estimatorInfoInit)
    : rng(std::random_device{}()), numParticles(500)
{
    // ... allocates particles, transitioner, fieldFeatureLocations ...
    // ❌ params is NEVER allocated!
}
```

Every filtering method accesses `this->params` -> segfault.

Also: `numParticles(500)` hardcoded, mismatches `NUM_PARTICLES` (300).

### Fix B1 — Allocate in initializer list

```cpp
ParticleFilter::ParticleFilter(const EstimatorInfoInit& estimatorInfoInit)
    : rng(std::random_device{}()), numParticles(NUM_PARTICLES),
      params(new ParticleFilterParams())
```

### Fix B2 — Clean up in destructor

```cpp
delete params;  // ADD
```

---

## Bug 3 · Arrow Operator on Reference (Compile Error)

**File:** `ParticleFilter.cpp` (line 210)

```cpp
if (estimatorInfoMiddle->canDoObservations) return;
//                  ^^  COMPILE ERROR
```

`EstimatorInfoMiddle &` has no `operator->()`.

Two bugs in one line: (1) `->` should be `.`, (2) logic inverted (should return when `canDoObservations` is **false**).

### Fix C1

```cpp
if (!estimatorInfoMiddle.canDoObservations) return;
```

---

## Bug 4 · Null `mmcmkf` Dereference (Segfault)

**File:** `Localiser.cpp` (lines 49–56)

`mmcmkf` is null, but two methods dereference it:

```cpp
void Localiser::setDebugger(...) { mmcmkf->setDebugger(...); }    // segfault
MultiModalCMKFParams *Localiser::getCMKFParams() { return mmcmkf->getParams(); } // segfault
```

`getCMKFParams()` is called from `StateEstimationAdapter.cpp` line 122.

### Fix D1 — Guard with null checks

```cpp
void Localiser::setDebugger(...) {
#ifndef TMP_NDEBUG
    if (mmcmkf) mmcmkf->setDebugger(debugger);
#endif
}
MultiModalCMKFParams *Localiser::getCMKFParams() {
    return mmcmkf ? mmcmkf->getParams() : nullptr;
}
```

---

## Summary of Fixes

| # | File | Change |
|---|------|--------|
| A1 | `ParticleFilterConstants.hpp` | Add 3 `#define` constants |
| A2 | `ParticleFilterParams.cpp` | Init 3 fields using constants |
| A3 | `ParticleFilterParams.cpp` | Add 3 config-file options |
| B1 | `ParticleFilter.cpp` | Allocate `params`; fix `numParticles(500)` → `NUM_PARTICLES` |
| B2 | `ParticleFilter.cpp` | `delete params` in destructor |
| C1 | `ParticleFilter.cpp` | Fix `->` to `.` and `!` logic on line 210 |
| D1 | `Localiser.cpp` | Guard null `mmcmkf` in 2 methods |

### Verification

- [ ] **Compile:** `./Make/Linux/compile rbb -p Booster`
- [ ] No segfault on startup
- [ ] Observations skipped correctly when robot cannot observe

### References

- `ParticleFilterConstants.hpp` — `../soccer/Src/robot/include/perception/stateestimation/localiser/particlefilter/ParticleFilterConstants.hpp`
- `ParticleFilterParams.hpp` — `../soccer/Src/robot/include/perception/stateestimation/localiser/particlefilter/ParticleFilterParams.hpp` lines 63–67
- `ParticleFilterParams.cpp` — `../soccer/Src/robot/src/perception/stateestimation/localiser/particlefilter/ParticleFilterParams.cpp` lines 20–37
- `ParticleFilter.cpp` — `../soccer/Src/robot/src/perception/stateestimation/localiser/particlefilter/ParticleFilter.cpp` lines 28, 210
- `Localiser.cpp` — `../soccer/Src/robot/src/perception/stateestimation/localiser/Localiser.cpp` lines 49–56
- README — `../soccer/Src/robot/src/perception/stateestimation/localiser/particlefilter/README.md` lines 96–110
