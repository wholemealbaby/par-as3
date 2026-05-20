# PF-02 Sub-Issues — Project Scaffolding & Build Integration

> These sub-issues break PF-02 into implementable chunks. Each produces compilable files that wire into the existing build system.
>
> **Parent:** `PF-02` — Project Scaffolding & Build Integration
> **Parent Source:** `.issues/pf_planning.md`
> **Reference:** `docs/redback_localization_summary.md`
> **Labels:** `infrastructure` `pf` `par`
> **Milestone:** Phase 1 — Foundation

---

## PF-02-S01 · Data Types — Particle Struct, Constants, and Parameters

**Labels:** `infrastructure` `pf` `par`  
**Priority:** P0 — blocks all other PF-02 work  
**Estimates:** ~45 min

### Objective

Define the fundamental data types that the particle filter will use. These are leaf definitions with no internal dependencies — they can be created first and everything else builds on them.

### Files to Create

#### `Src/robot/include/perception/stateestimation/localiser/particlefilter/Particle.hpp`

A simple struct holding a single particle's state and weight — analogous to `CMKF.hpp` but without a covariance matrix (particles are points, not Gaussians).

```cpp
struct Particle {
    float x, y, heading;  // Pose state (mirrors StateVector)
    float weight;          // Importance weight
};
```

No corresponding `.cpp` needed unless helper methods (e.g., `normaliseHeading()`) are added.

#### `Src/robot/include/perception/stateestimation/localiser/particlefilter/ParticleFilterConstants.hpp`

Constants and type aliases — analogous to `CMKFConstants.hpp`:
- `NUM_PARTICLES = 300` (starting default)
- Dimension indices: `PF_X_DIM 0`, `PF_Y_DIM 1`, `PF_H_DIM 2`
- Type alias: `using ParticleState = Eigen::Vector3f;`

#### `Src/robot/include/perception/stateestimation/localiser/particlefilter/ParticleFilterParams.hpp`
`Src/robot/src/perception/stateestimation/localiser/particlefilter/ParticleFilterParams.cpp`

Parameter class — analogous to `MultiModalCMKFParams.hpp`/`.cpp`: 
- Fields: `numParticles`, `minWeight`, `odometryForwardMultiplyFactor`, `odometryLeftMultiplyFactor`, `odometryHeadingMultiplyFactor`, `angleUncertainty`, `distanceUncertaintyBase`, `resamplingThreshold`, `rougheningScale`
- Constructor with sensible default values
- `readParams()` method (same config file pattern as existing params)
- Destructor (empty)

### Deliverable

All three header files and one source file created in the `particlefilter/` directories. Header guards, includes, and type definitions complete.

### Dependencies

- PF-02 scaffolding: `particlefilter/` include and src directories must exist (already created for includes; src directory also created)

---

## PF-02-S02 · Filter Classes — ParticleFilter + Transitioner Stubs

**Labels:** `infrastructure` `pf` `par`  
**Priority:** P0 — blocks build integration  
**Depends on:** PF-02-S01 (needs Particle struct and Params)  
**Estimates:** ~45 min

### Objective

Create the main `ParticleFilter` class with the `tick()` interface matching `MultiModalCMKF`, and its transitioner that handles game-state resets. Both are stubs — they compile and run but don't perform any filtering yet.

### Files to Create

#### `Src/robot/include/perception/stateestimation/localiser/particlefilter/ParticleFilter.hpp`
`Src/robot/src/perception/stateestimation/localiser/particlefilter/ParticleFilter.cpp`

Class declaration mirroring `MultiModalCMKF`:

```cpp
class ParticleFilter {
public:
    ParticleFilter(const EstimatorInfoInit& estimatorInfoInit);
    ~ParticleFilter();

    void tick(
        const EstimatorInfoIn &estimatorInfoIn,
        EstimatorInfoMiddle &estimatorInfoMiddle,
        EstimatorInfoOut &estimatorInfoOut);

    std::vector<Particle> particles;

private:
    FieldFeatureLocations *fieldFeatureLocations;
    ParticleFilterParams *params;
    ParticleFilterTransitioner *transitioner;
};
```

- Constructor: allocate `params`, `fieldFeatureLocations`, `transitioner`
- Destructor: clean up heap-allocated members (same pattern as `MultiModalCMKF`)
- `tick()`: empty body — filled in by PF-04 through PF-07
- Forward-declare all dependent types in the header (same pattern)

#### `Src/robot/include/perception/stateestimation/localiser/particlefilter/ParticleFilterTransitioner.hpp`
`Src/robot/src/perception/stateestimation/localiser/particlefilter/ParticleFilterTransitioner.cpp`

Transition handler — analogous to `MultiModalCMKFTransitioner`:

- Inherit from `LocaliserTransitioner` base class
- Constructor: `(const EstimatorInfoInit&, ParticleFilter*)` storing a pointer to the filter
- Stub overrides for all required reset methods:
  - `resetToLeftTeamInitialPose()`, `resetToRightTeamInitialPose()`
  - `resetToUnpenalisedPose()`, `resetToPenalisedPose()`
  - `resetToManualPlacementPoseOffense()`, `resetToManualPlacementPoseDefense()`
  - `resetToSpecifiedInitialPose()`, etc.
- Each stub clears `particles` and spawns one particle at the appropriate game-state pose (full initialization logic deferred to PF-03)

### Deliverable

Four files (two headers, two sources) that compile and link. The `ParticleFilter` has a callable `tick()` and the transitioner wires into the filter lifecycle.

### Dependencies

- PF-02-S01 (Particle struct, constants, params must exist)
- Existing `LocaliserTransitioner` base class (already present)
- Existing `FieldFeatureLocations`, `EstimatorInfoInit` types

---

## PF-02-S03 · Build Integration & Compile Verification

**Labels:** `infrastructure` `testing` `pf` `par`  
**Priority:** P0 — final gate for PF-02  
**Depends on:** PF-02-S01, PF-02-S02 (all files must exist)  
**Estimates:** ~15 min + compile time

### Objective

Register all new particle filter source files in the build system and verify the project compiles cleanly.

### Files to Modify

#### `soccer/Make/CMake/Nao.cmake`

After the existing `multimodalcmkf/` entries (lines 97–100), add:

```cmake
${RBB_SRC_DIR}/perception/stateestimation/localiser/particlefilter/ParticleFilter.cpp
${RBB_SRC_DIR}/perception/stateestimation/localiser/particlefilter/ParticleFilterParams.cpp
${RBB_SRC_DIR}/perception/stateestimation/localiser/particlefilter/ParticleFilterTransitioner.cpp
```

(Note: `Particle.hpp` is header-only, no `.cpp` needed unless helpers were added.)

#### `soccer/Make/CMake/Booster.cmake`

Identical additions after the existing `multimodalcmkf/` entries (lines 134–137).

### Verification

Run the build and confirm zero errors:

```bash
cd soccer
./Make/Linux/compile rbb -p Booster
```

- Fix any compilation errors (missing includes, typos, signature mismatches)
- No new warnings introduced (or document acceptable ones)

### Deliverable

- Updated `Nao.cmake` and `Booster.cmake` committed
- Clean build log showing successful compilation of `Booster` target

### Dependencies

- PF-02-S01 (data types must exist to compile)
- PF-02-S02 (filter classes must exist to compile)

---

## Dependency Graph

```
PF-02-S01 (Data types — Particle struct, Constants, Params)
    │
    ▼
PF-02-S02 (Filter classes — ParticleFilter + Transitioner stubs)
    │
    ▼
PF-02-S03 (Build integration & compile verification)
```

**Critical path:** S01 → S02 → S03 (strictly sequential — each needs the previous)

---

## Suggested Labels Summary

| Sub-issue | Labels |
|-----------|--------|
| S01 | `infrastructure` `pf` `par` |
| S02 | `infrastructure` `pf` `par` |
| S03 | `infrastructure` `testing` `pf` `par` |
