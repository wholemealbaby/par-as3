# Particle Filter Localisation — Implementation Epics
## RedBackBots Soccer | PAR Assignment 3 | Particle Filter Planning

> Each section below is an **epic** requiring further breakdown into sub-issues before implementation begins.
> Epics are listed in recommended implementation order.
> 
> All epics carry the `par` (course) and `pf` (particle filter) labels.
> Additional labels indicate the domain area.
>
> **Legend for dependency stubs:**
> - 🟢 = This epic can be developed with **stubbed inputs** (e.g., manually created particle sets, synthetic observations)
> - 🟡 = Requires real output from a dependency, but can use a simplified version
> - 🔴 = Requires a working implementation of at least one dependency

---

## PF-01 · [Team Discussion] Initialization & Reset Strategies

**Labels:** `par` `pf` `epic` `team-discussion`  
**Priority:** P0 — design decisions feed into PF-03, PF-07  
**Milestone:** Planning Phase

### Objective

Reach team consensus on how the particle filter should handle all **initialization scenarios** and **kidnapped robot recovery**. The answers to these questions directly determine how particles are spawned in PF-03 and how the output is validated in PF-07.

### Questions to Resolve

#### Initialization (normal start)
- Robot starts at a known position (own half, facing own goal). Should we spawn all particles around that position with a narrow Gaussian, or spread more broadly?
- What variance should the initial Gaussian have? (e.g., σ_x = 500 mm, σ_y = 500 mm, σ_θ = 15°?)

#### Kidnapped Robot Recovery
- When the robot is picked up and placed elsewhere (as detected by `REF_PICKUP` action), should we:
  - **Keep particles where they are** and hope observations converge them? (Slow recovery)
  - **Add injected random particles** (e.g., 10-20% uniform across field) to maintain multi-modality?
  - **Fully re-initialize** with uniform coverage across the entire field?
- What are the computational implications of uniform field coverage at 300+ particles?

#### Game State Resets
- **Penalised → Unpenalised:** Reset particles to a known position, or keep the estimate?
- **Manual placement:** When a referee places the robot, do we trust the placement location? (Current system uses specific reset poses per team/situation.)
- **Halftime / side switch:** Do we mirror particles across the field axis, or re-initialize?

#### Edge Cases
- How should we handle the first frame after boot with no prior estimate?
- Should we support a "global localization" mode (uniform particles across entire field) as an optional fallback when uncertainty is very high?

### Deliverable

A shared document or team channel post recording the agreed decisions — referenced by PF-03 and PF-07 during implementation.

### Sub-Issue Potential

🔀 Can be split into separate sub-issues per question category (initialization, kidnapped, game state resets) if different team members want to research different aspects.

### Dependencies

- None (this is a discussion issue, no code dependencies)

---

## PF-02 · Project Scaffolding & Build Integration

**Labels:** `par` `pf` `epic` `infrastructure`  
**Priority:** P0 — blocks all implementation work  
**Milestone:** Phase 1 — Foundation

### Objective

Create the directory structure and build system integration for the particle filter. This epic is pure infrastructure — it produces compilable but empty files.

### Tasks

- Create new directory: `Src/robot/include/perception/stateestimation/localiser/particlefilter/`
- Create new directory: `Src/robot/src/perception/stateestimation/localiser/particlefilter/`
- Create header stubs:
  - `ParticleFilter.hpp` — class declaration matching the same `tick()` interface as `MultiModalCMKF`
  - `ParticleFilterParams.hpp` — parameter class (start with a minimal set: `numParticles`, `minWeight`)
  - `ParticleFilterConstants.hpp` — constants (`NUM_PARTICLES = 300`, dimension definitions)
  - `ParticleFilterTransitioner.hpp` — transition handler (reuse base class `LocaliserTransitioner`)
- Create corresponding `.cpp` stubs with minimal constructors/destructors
- Add all new source files to `Nao.cmake` and `Booster.cmake` (alongside the existing `multimodalcmkf/` entries)
- Verify the project compiles with the new (empty) files

### Deliverable

- Directory structure created
- Header and source stubs committed
- Build verified (`cmake --build` passes)

### Sub-Issue Potential

🔀 **Code vs. test split:** Scaffolding + CMake changes vs. a smoke-test build script that confirms the files compile.

### Dependency Chain

- **Provides:** File structure for PF-03, PF-04, PF-05, PF-06, PF-07
- **Depends on:** Nothing (design decisions from PF-01 can be incorporated later)

---

## PF-03 · Core Particle Data Structure & Initialization

**Labels:** `par` `pf` `epic` `core-algorithm`  
**Priority:** P0 — foundational data type  
**Milestone:** Phase 1 — Foundation

### Objective

Define the fundamental particle data type and the initialization logic for creating particle clouds. This is the data backbone that all subsequent epics operate on.

### Tasks

#### Data Structure
- Define a `Particle` struct (or class) containing:
  - `float x, y, heading` — pose state (mirroring `StateVector` from CMKF)
  - `float weight` — importance weight (initialized to `1.0/N`)
- Define the `ParticleFilter` class with:
  - `std::vector<Particle> particles` as the main state
  - `ParticleFilterParams *params`
  - `FieldFeatureLocations *fieldFeatureLocations`
  - `ParticleFilterTransitioner *transitioner`

#### Initialization
- Implement `initializeFromKnownPose(x, y, heading, covariance)` — spawns N particles in a Gaussian around the known pose
- Implement `initializeUniform()` — spawns particles uniformly across the field (for kidnapped recovery / global localization)
- Implement `initializeFromDistribution(mean, cov)` — general-purpose initialization from any Gaussian
- Reuse the existing transitioner pattern: `ParticleFilterTransitioner` calls the appropriate initialization method based on game state

#### Stubbing Note 🟢

This epic can be fully developed and tested with **no external dependencies** — particle sets can be created, printed, and verified in isolation. Unit tests can verify:
- Gaussian spread is statistically correct (χ² test on covariance)
- Uniform distribution covers the field bounds
- Weights sum to 1.0

### Sub-Issue Potential

🔀 **Code vs. test split:** Particle struct + initialization methods vs. property-based tests verifying distribution statistics.

### Dependency Chain

- **Depends on:** PF-02 (file structure must exist)
- **Depends on (design):** PF-01 (initialization strategy decisions)
- **Provides:** Particle data for PF-04, PF-05, PF-06, PF-07

---

## PF-04 · Prediction Step (Motion Model)

**Labels:** `par` `pf` `epic` `core-algorithm`  
**Priority:** P1  
**Milestone:** Phase 1 — Foundation

### Objective

Implement the motion model that propagates each particle forward using odometry readings, adding appropriate process noise. This replaces `MultiModalCMKF::predict()`.

### Tasks

- Implement `predict(Odometry odometry)` method on `ParticleFilter`
- For each particle, apply the odometry delta to the pose:
  ```
  delta_x = forward * cos(heading) - left * sin(heading)
  delta_y = forward * sin(heading) + left * cos(heading)
  heading += turn
  ```
  (Reuse the exact geometry from `MultiModalCMKF::predict()` lines 174-179)
- Add Gaussian noise to each particle after the deterministic odometry update:
  - Noise magnitude proportional to motion magnitude (reuse `odometryForwardMultiplyFactor`, etc.)
  - Noise is sampled from a zero-mean Gaussian with covariance matching the CMKF's process noise model
- Handle heading normalization (keep within -π to π)
- Parameterize noise via `ParticleFilterParams` (can start by copying CMKF noise values)

#### Stubbing Note 🟢

This epic can be developed with **manually created test particles** — no need for a working initialization. Create 5-10 particles at known positions, apply odometry, verify they moved correctly.

### Sub-Issue Potential

🔀 **Code vs. test split:** Motion model implementation vs. unit tests verifying: (a) deterministic movement matches CMKF, (b) noise distribution has correct covariance, (c) heading wraps correctly.

### Dependency Chain

- **Depends on:** PF-03 (needs Particle struct)
- **Provides:** Predicted particles for PF-06 (resampling needs weighted+predicted particles)
- **Parallel to:** PF-05 (observation models), PF-07 (output — can read raw particles)

---

## PF-05 · Observation Models for Field Features

**Labels:** `par` `pf` `epic` `observation-model`  
**Priority:** P1  
**Milestone:** Phase 2 — Core Filter

### Objective

Implement the likelihood computation for each observed field feature type. Each particle's weight is multiplied by the probability of observing the feature given the particle's pose.

### Tasks

#### Oriented Features (corners, T-junctions, X-junctions, centre circle)
- Reuse the expected-position geometry from `MultiModalCMKF::update()` (lines 384-386):
  ```
  x_expected = feature.x + obs.distance * cos(feature.orientation + obs.orientation)
  y_expected = feature.y + obs.distance * sin(feature.orientation + obs.orientation)
  heading_expected = atan2(feature.y - y_expected, feature.x - x_expected) - obs.heading
  ```
- Compute innovation: `[x_expected - particle.x, y_expected - particle.y, heading_expected - particle.heading]`
- Compute weight multiplier: `exp(-0.5 * innovationᵀ * R⁻¹ * innovation)` where R = observation covariance (precomputed once per observation, not per particle)
- R is computed using the same polar→cartesian conversion as lines 393-418 of `MultiModalCMKF.cpp`

#### Directionless Features (penalty spots, goal posts)
- Only distance and bearing constrain the pose, no orientation information
- Simplified innovation: `[expected_distance - obs.distance, expected_bearing - obs.bearing]`
- Use `distanceUncertaintyBase` and `angleUncertainty` from params

#### Field Lines (constant X/Y lines)
- Reuse geometry from `updateUsingConstantXLine()` and `updateUsingConstantYLine()` (lines 211-368)
- For constant-X lines: innovation in x-axis and heading only (Y is unconstrained)
- For constant-Y lines: innovation in y-axis and heading only (X is unconstrained)
- Use `bigNumber = 1,000,000` for unconstrained dimensions (same as CMKF)

#### Common Pattern
- All observation types follow the same flow:
  1. Compute expected measurement from particle pose + database feature
  2. Compute innovation (difference between expected and observed)
  3. Compute weight multiplier from Gaussian likelihood
  4. `particle.weight *= weightMultiplier` (with a floor to prevent complete weight collapse)
- NaN/Inf guard: Skip updates where observation contains NaN (same as lines 57-62 of `MultiModalCMKF.cpp`)

#### Stubbing Note 🟢

This epic can be developed with **manually created particles at known poses** and **synthetic observations** generated from those poses with added noise. This means:
- Create a particle at the ground truth pose
- Create a synthetic observation that should perfectly match that pose
- Verify weight multiplier ≈ 1.0
- Add noise to the observation, verify weight decreases plausibly

### Sub-Issue Potential

🔀 This is the **largest epic** and the most natural candidate for sub-issue splitting:
- `PF-05-S01` · Oriented features (corners, junctions) — code + tests
- `PF-05-S02` · Directionless features (penalty spots, goal posts) — code + tests
- `PF-05-S03` · Field line features — code + tests
Each sub-issue can be independently developed and tested using the stubbing approach above.

### Dependency Chain

- **Depends on:** PF-03 (needs Particle struct with weights)
- **Provides:** Weighted particles for PF-06 (resampling needs weights)
- **Parallel to:** PF-04 (prediction), PF-07 (output)

---

## PF-06 · Adaptive Resampling & Diversity Management

**Labels:** `par` `pf` `epic` `core-algorithm`  
**Priority:** P1  
**Milestone:** Phase 2 — Core Filter

### Objective

Implement the resampling step that prevents particle degeneracy (weight concentration on few particles) while maintaining diversity.

### Tasks

#### Effective Sample Size Calculation
- Compute `N_eff = 1.0 / sum(particle.weight²)` for i in 1..N
- If `N_eff < N/2` (configurable threshold), trigger resampling

#### Systematic Resampling (Low Variance)
- Implement the standard low-variance resampling algorithm:
  1. Compute the cumulative weight distribution
  2. Pick a random starting point in [0, 1/N)
  3. Step through particles at 1/N intervals, selecting the particle whose cumulative weight bracket contains the current position
  4. Replace the particle set with the resampled set, keeping the *state* from selected particles
  5. Reset all weights to `1/N`
- This is O(N) time and has lower variance than multinomial resampling

#### Roughening (Optional Diversity Preservation)
- After resampling, add small Gaussian noise to each particle's state
- Noise standard deviation proportional to the spread of the particle set (Kernel-based roughening)
- Prevents particle collapse where all particles converge to the same point
- Make this configurable (enable/disable, scaling factor)

#### NaN/Inf Guard
- Before resampling, check for NaN/Inf in weights and states
- If any particle has invalid state, replace it with a sample from a prior distribution

#### Stubbing Note 🟢

This epic can be developed with **manually created weighted particle sets** — no need for prediction or observation models. Create a particle set where one particle has weight 0.9 and the rest share 0.1, then verify resampling correctly duplicates the high-weight particle.

### Sub-Issue Potential

🔀 **Code vs. test split:** Resampling algorithm vs. statistical tests verifying: (a) particle diversity after resampling, (b) weights sum to 1.0, (c) high-weight particles are correctly propagated.

### Dependency Chain

- **Depends on:** PF-04 (predicted particles), PF-05 (weighted particles)
- **Provides:** Resampled, diverse particle set for next frame
- **Depends on (design):** PF-07 (output — to decide if particles need to be clustered before output to avoid mode collapse)

---

## PF-07 · Output Estimation & Uncertainty Calculation

**Labels:** `par` `pf` `epic` `core-algorithm`  
**Priority:** P1  
**Milestone:** Phase 2 — Core Filter

### Objective

Extract the best pose estimate and uncertainty from the particle cloud, and format it into `EstimatorInfoOut`. This is the bridge between the particle filter's internal representation and the rest of the system.

### Tasks

#### Weighted Mean Pose
- Compute weighted mean of all particles:
  ```
  mean_x = sum(weight_i * x_i)
  mean_y = sum(weight_i * y_i)
  mean_heading = atan2(sum(weight_i * sin(heading_i)), sum(weight_i * cos(heading_i)))
  ```
- This fills `estimatorInfoOut.robotPos`

#### Uncertainty Estimation
- Compute weighted covariance of the particle cloud:
  ```
  cov_x = sum(weight_i * (x_i - mean_x)²)
  cov_y = sum(weight_i * (y_i - mean_y)²)
  cov_heading = sum(weight_i * angle_diff(heading_i, mean_heading)²)
  robotPosUncertainty = sqrt(cov_x + cov_y)  // Radial uncertainty in mm
  robotHeadingUncertainty = sqrt(cov_heading)  // Heading uncertainty in rad
  ```
- This fills `estimatorInfoOut.robotPosUncertainty` and `robotHeadingUncertainty`

#### Best Particle Selection
- Select the particle with the highest weight as the "best" particle
- Optionally report its pose for debugging
- Apply anti-flip gate logic (mirroring the CMKF's approach): if best particle heading differs from previous best by > 150°, suppress the flip unless weight ratio > 3×

#### Multi-Hypothesis Output (Optional Enhancement)
- Cluster particles using a simple distance-based method (e.g., DBSCAN or k-means with k=3)
- Report cluster centers and weights in `estimatorInfoOut.allRobotPos`
- **Note:** This is optional — the CMKF outputs all modes there. For a first implementation, just output the weighted mean. Add clustering later if needed.

#### Stubbing Note 🟢

This epic can be developed with **manually created particle sets** — create 5-10 particles with known positions and weights, verify the weighted mean and covariance are computed correctly. No need for a running filter.

### Sub-Issue Potential

🔀 **Code vs. test split:** Output computation vs. verification tests comparing weighted mean accuracy to Monte Carlo ground truth.

### Dependency Chain

- **Depends on:** PF-03 (needs Particle struct)
- **Provides:** Output for PF-08 (integration needs output to wire up)
- **Parallel to:** PF-04, PF-05, PF-06 (can compute outputs from any particle set)
- **Depends on (design):** PF-01 (anti-flip gate decisions, whether to cluster)

---

## PF-08 · Integration into Localiser with Side-by-Side Toggle

**Labels:** `par` `pf` `epic` `integration`  
**Priority:** P0 — must be done before real-world testing  
**Milestone:** Phase 3 — Integration

### Objective

Wire the completed particle filter into `Localiser` so it can be used alongside or instead of `MultiModalCMKF`. Implement a toggle mechanism.

### Tasks

#### Class Changes (`Localiser.hpp`)
- Add `ParticleFilter *particleFilter;` member
- Keep existing `MultiModalCMKF *mmcmkf;` member

#### Toggle Mechanism
- **Option A: Compile-time** — `#define USE_PARTICLE_FILTER` switch. Simple, no runtime overhead.
- **Option B: Runtime** — Config parameter or constructor argument. More flexible, allows A/B comparison in the same binary.
- **Recommendation:** Start with compile-time (simpler), add runtime toggle later if needed for validation (PF-09).

#### Localiser Changes (`Localiser.cpp`)
- In constructor: initialize both filters (or just the selected one)
- In `tick()`:
  ```
  if (useParticleFilter) {
      particleFilter->tick(estimatorInfoIn, estimatorInfoMiddle, estimatorInfoOut);
  } else {
      mmcmkf->tick(estimatorInfoIn, estimatorInfoMiddle, estimatorInfoOut);
  }
  ```
- The `fillCanLocaliseInState()` and `fillCanDoObservations()` logic is shared (it's game-state logic, not filter-specific)

#### Transitioner Integration
- `ParticleFilterTransitioner` should be called from within `ParticleFilter::tick()`, matching the existing pattern where `MultiModalCMKF::tick()` calls `transitioner->handleTransition()`
- Reuse the same `LocaliserTransitioner` base class (keep the interface contract)

#### Parameter Wiring
- `ParticleFilterParams` should be loadable/readable similar to `MultiModalCMKFParams`
- Add `getParticleFilterParams()` accessor on `Localiser` if needed for debugging

#### Stubbing Note 🟡

Integration needs the full filter pipeline (PF-03 through PF-07) working — there's no meaningful way to "stub" the integration. However, early integration can be done with a particle filter that **only predicts and outputs a fallback pose** while observation models are still being built.

### Sub-Issue Potential

🔀 **Code vs. test split:** Integration wiring vs. smoke test confirming both filters can run and produce outputs from the same input data.

### Dependency Chain 🔴

- **Depends on:** PF-03 (particle data), PF-04 (prediction), PF-05 (observation models), PF-06 (resampling), PF-07 (output)
- **Depends on:** PF-02 (file structure must already exist)
- **Provides:** Runnable particle filter on the robot for PF-09 and PF-10

---

## PF-09 · Synthetic & Recorded Data Validation

**Labels:** `par` `pf` `epic` `testing`  
**Priority:** P1 — must validate before tuning  
**Milestone:** Phase 3 — Integration

### Objective

Build a test harness that validates the particle filter's correctness against known ground truth, using both synthetic trajectories and recorded field data.

### Tasks

#### Synthetic Trajectory Tests
- Create a simulated robot driving a known path (e.g., figure-8, straight line with rotation, stop-start)
- Generate synthetic field feature observations at each timestep (with configurable noise)
- Run the particle filter on these observations
- Compare output pose vs. ground truth:
  - RMSE (Root Mean Squared Error) in position and heading
  - Percentage of frames where error < threshold (e.g., 15 cm, 10°)
  - Convergence time after initialization
- **Key test:** Run the same trajectory through both particle filter and CMKF, compare outputs

#### Multi-Modality Test
- Create scenarios where the robot has ambiguous observations (e.g., seeing only a penalty spot — could be on either side of the field)
- Verify the particle filter maintains multiple modes naturally
- Compare with CMKF's explicit mode management

#### Kidnapped Robot Test
- Simulate a sudden jump in pose (robot picked up and moved)
- Verify the particle filter recovers (weights re-distribute, uncertainty drops)
- Measure recovery time

#### Recorded Data Replay
- Use existing bag files or recorded log data from actual robot runs
- Replay odometry and field feature observations through both filters
- Compare outputs for consistency (cannot know ground truth, but can check for: no NaN, smooth transitions, plausible poses)

#### Test Harness Structure
- Create `test/particle_filter/` directory
- Standalone test executable that links against the filter code (no robot hardware needed)
- Uses Eigen for math, standard C++ for simulation loop

### Stubbing Note 🟢

This epic is entirely about testing — it creates its own simulated environments. No robot hardware needed.

### Sub-Issue Potential

🔀 **Scenario split:** Each test scenario (synthetic trajectory, multi-modality, kidnapped, recorded replay) can be a separate sub-issue. Each produces its own report and pass/fail criteria.

### Dependency Chain 🔴

- **Depends on:** PF-08 (need integrated filter to test)
- **Parallel to:** PF-10 (tuning can start once basic validation passes)

---

## PF-10 · Parameter Tuning & Performance Optimization

**Labels:** `par` `pf` `epic` `tuning`  
**Priority:** P2  
**Milestone:** Phase 3 — Integration

### Objective

Tune the particle filter's parameters for optimal performance on the Booster K1 hardware, and profile the frame timing to ensure real-time operation (~30 Hz).

### Tasks

#### Parameter Tuning

| Parameter | Starting Value | Tuning Strategy |
|-----------|---------------|-----------------|
| `numParticles` | 300 | Increase until timing budget exceeded, decrease until accuracy degrades |
| `odometryForwardMultiplyFactor` | 20.0 (from CMKF) | Match CMKF, adjust if PF drifts more/less |
| `odometryLeftMultiplyFactor` | 20.0 (from CMKF) | Same |
| `odometryHeadingMultiplyFactor` | 0.5 (from CMKF) | Same |
| `angleUncertainty` | 20° (from CMKF) | Same |
| `distanceUncertaintyBase` | 100 mm (from CMKF) | Same |
| `distanceUncertaintyScale` | 0.25 (from CMKF) | Same |
| `resamplingThreshold` | N/2 | Tune: lower = less frequent resampling (faster), higher = better diversity |
| `rougheningScale` | 0.1 * inter-particle distance | Tune: higher = more diversity but more noise |
| `minWeight` | 0.01 (from CMKF) | Weight floor to prevent numerical underflow |

#### Performance Profiling
- Instrument the particle filter's `tick()` method with high-resolution timers:
  - `t_predict` — time for prediction step
  - `t_update` — time for observation weight computation
  - `t_resample` — time for resampling
  - `t_output` — time for output estimation
- Measure on target hardware (Booster K1 ARM processor)
- Target: total tick time < 33 ms (for 30 Hz operation)
- Report timing breakdown — where is the bottleneck?

#### Optimization Candidates
- If update step is bottleneck:
  - Pre-compute observation covariance inverse `R⁻¹` once per observation, not per particle
  - Use Eigen's `.noalias()` to avoid temporary matrices
  - Parallelize particle weight computation with OpenMP or std::thread
- If resampling is bottleneck:
  - Only resample every N frames (if `N_eff` is stable)
  - Use a simpler resampling algorithm
- If prediction is bottleneck:
  - Use fixed-point noise sampling (pre-compute a table of noise values)

#### Comparison with CMKF
- Run both filters on the same recorded data
- Compare:
  - RMSE vs ground truth (for synthetic data)
  - Frame timing
  - Memory usage
  - Robustness to ambiguity

### Deliverable

- Tuned parameter set (documented with rationale)
- Performance benchmark report (timing breakdown, resource usage)
- Comparison table: particle filter vs. CMKF

### Stubbing Note 🟢

Tuning requires a working filter (PF-08) and validation data (PF-09), but the timing instrumentation can be added earlier.

### Sub-Issue Potential

🔀 **Split by activity:** Parameter tuning on synthetic data (safe, repeatable) vs. on-hardware profiling (requires lab access). Also: timing instrumentation is a separate sub-issue from the actual tuning experiments.

### Dependency Chain 🔴

- **Depends on:** PF-08 (working integration)
- **Depends on:** PF-09 (validation framework to measure accuracy impacts of parameter changes)

---

## Dependency Graph

```
PF-01 (Team discussion — design decisions)
  │
  ▼
PF-02 (Scaffolding — file structure, build system)
  │
  ▼
PF-03 (Core particle struct, initialization) ◄── PF-01 design decisions
  │
  ├─────────────────────┬─────────────────────┐
  ▼                     ▼                     ▼
PF-04 (Prediction)   PF-05 (Observation)   PF-07 (Output)
  │                     │                     │
  └─────────┬───────────┘                     │
            ▼                                 │
        PF-06 (Resampling)                    │
            │                                 │
            └─────────┬───────────────────────┘
                      ▼
                  PF-08 (Integration into Localiser)
                      │
                      ▼
                  PF-09 (Validation — synthetic + recorded data)
                      │
                      ▼
                  PF-10 (Tuning & Performance Optimization)
```

### Stubbing & Parallelization Notes

| Epic | Can be stubbed? | How |
|------|----------------|-----|
| PF-03 | 🟢 Yes | Manually create particle sets in unit tests |
| PF-04 | 🟢 Yes | Create test particles, manually apply odometry |
| PF-05 | 🟢 Yes | Create particles at known poses + synthetic observations |
| PF-06 | 🟢 Yes | Create weighted particle sets manually |
| PF-07 | 🟢 Yes | Create particle sets, verify mean/covariance math |
| PF-08 | 🟡 Partial | Can stub with predict-only filter before observations are ready |
| PF-09 | 🟢 Yes | Entirely self-contained simulation |
| PF-10 | 🟢 Yes | Synthetic data + timing instrumentation |

**Critical path:** PF-02 → PF-03 → PF-04 + PF-05 → PF-06 → PF-07 → PF-08 → PF-09 → PF-10

**Parallel work possible:** PF-04, PF-05, and PF-07 can all be developed simultaneously once PF-03 is complete.

---

## Suggested Labels Overview

| Label | Color | Hex | Purpose |
|---|---|---|---|
| `par` | 🟠 Orange Red | `#FF4500` | Course identifier — all PF issues carry this |
| `pf` | 🔵 Dodger Blue | `#1E90FF` | Particle filter project |
| `epic` | 🟣 Dark Orchid | `#9932CC` | Needs breakdown into smaller sub-issues |
| `infrastructure` | 🟢 Lime Green | `#32CD32` | Build system, file structure, scaffolding |
| `core-algorithm` | 🟡 Gold | `#FFD700` | Core PF logic: prediction, resampling, output |
| `observation-model` | 🩷 Hot Pink | `#FF69B4` | Observation likelihood computation |
| `integration` | 🟪 Medium Purple | `#9370DB` | Localiser integration, toggle mechanism |
| `team-discussion` | 🔷 Dark Turquoise | `#00CED1` | Decisions requiring team consensus |
| `testing` | 🟠 Dark Orange | `#FF8C00` | Unit tests, integration tests, validation harness |
| `tuning` | ⚪ Slate Gray | `#708090` | Parameter tuning, performance profiling |
