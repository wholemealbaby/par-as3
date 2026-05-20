# AGENTS — Root Project Instructions

## Project Context

We are a student group (3 UG + 1 PG) working on **RoboCup Soccer** localisation
(Project 4.2) for the **Programming Autonomous Robots** course (COSC2781/COSC2814)
at RMIT University, Semester 1 2026.

**Robot:** Booster K1 humanoid robot

---

## Project Goals

1. **Replace MultiModalCMKF with a Particle Filter** — Implement a particle filter that
   accepts the same input data structures (`EstimatorInfoIn`, `FieldFeatureInfo`,
   `Odometry`) and outputs pose estimates through `EstimatorInfoOut`, replacing the
   existing Multi-Modal Conditional Mixture Kalman Filter.

2. **HTC Vive External Pose Estimation** — Integrate HTC Vive motion trackers with
   the Booster K1 to provide ground-truth external pose estimation (for evaluation
   and/or sensor fusion). This is primarily for experimental evaluation (PG student
   requirement), not for in-game use.

3. **Perimeter Breaking Features** — Develop vision-only localisation features that
   break the symmetry of the RoboCup field using distinctive perimeter and structural
   landmarks detected through the robot's stereo cameras.

---

## RoboCup HSL Constraint — Human-Like Senses Only

Per RoboCup Humanoid Soccer League regulations, the robot may ONLY use sensors
that are analogous to human senses. This means:

- **Allowed:** Stereo cameras (like human eyes — the primary localisation sensor)
- **Allowed:** IMU (like human inner ear — orientation/balance sense)
- **Allowed:** Joint encoders / odometry (like human proprioception)
- **NOT allowed in-game:** LIDAR, ultrasonic, infrared ranging, external cameras,
  beacons, radio triangulation, or any other non-human sensing modality

The HTC Vive trackers/SteamVR tracking are **for external evaluation/ground-truth
measurement only** and cannot be used as an in-game localisation sensor.

---

## Code Modification Restrictions

This repository contains the **RedBackBots RoboCup Soccer codebase** as a shared
subtree/submodule. The architecture is FIXED to prevent interference with the
RoboCup July 2026 competition preparations.

**DO NOT modify any code outside of the localisation scope unless explicitly instructed.**

Files related to our scope include:
- `soccer/Src/robot/src/perception/stateestimation/localiser/` — Implementation files
- `soccer/Src/robot/include/perception/stateestimation/localiser/` — Header files
- `soccer/Src/robot/include/types/field/FieldFeatureLocations.hpp` — Field landmark database
- `soccer/Src/robot/include/types/FieldFeatureInfo.hpp` — Feature type definitions
- Any new files we create for the particle filter or Vive integration

Areas that are **OFF LIMITS** unless explicitly directed:
- Motion system (`soccer/Src/robot/src/motion/`)
- Vision system outside feature input (`soccer/Src/robot/src/perception/vision/`)
- Behaviour system (`soccer/Src/robot/src/perception/behaviour/`)
- Build system (`soccer/Make/`)
- Communication / networking (`soccer/Src/robot/src/communication/`)
- Game controller (`soccer/Src/robot/src/gamecontroller/`)
- RedBackBots SDK / simulation infrastructure

---

## Pertinent Directories (from Localisation System Summary)

### Existing MultiModalCMKF Implementation

| Component | Path |
|-----------|------|
| MultiModalCMKF (main) | `soccer/Src/robot/include/perception/stateestimation/localiser/multimodalcmkf/MultiModalCMKF.hpp` |
| CMKF (single hypothesis) | `soccer/Src/robot/include/perception/stateestimation/localiser/multimodalcmkf/CMKF.hpp` |
| Parameters | `soccer/Src/robot/include/perception/stateestimation/localiser/multimodalcmkf/MultiModalCMKFParams.hpp` |
| Field features DB | `soccer/Src/robot/include/types/field/FieldFeatureLocations.hpp` |
| Transitioner | `soccer/Src/robot/include/perception/stateestimation/localiser/multimodalcmkf/MultiModalCMKFTransitioner.hpp` |
| Localiser wrapper | `soccer/Src/robot/include/perception/stateestimation/localiser/Localiser.hpp` |
| StateEstimationAdapter | `soccer/Src/robot/src/perception/stateestimation/StateEstimationAdapter.cpp` |
| Source files | `soccer/Src/robot/src/perception/stateestimation/localiser/` |

### Input Structures

- `EstimatorInfoInit` — Initialisation config
- `EstimatorInfoIn` — Per-frame observations (field features, odometry, game state)
- `EstimatorInfoMiddle` — Internal control flags
- `EstimatorInfoOut` — Output state estimate

### Field Feature Types

- **Directional:** Corners, T-Junctions, X-Junctions, Centre Circle
- **Directionless:** Penalty Spot, Centre Circle (dir-less), Goal Post
- **Lines:** Field boundary lines (constant X/Y)

---

## Style Guide & Conventions (from RedBackBots Codebase)

### C++
- Files use `.hpp` / `.cpp` extensions
- Includes use relative paths from `Src/robot/include/`
- Naming: PascalCase for classes (`MultiModalCMKF`), camelCase for members (`fieldFeatures`)
- Compiler: Clang (via Colcon/ROS 2 Humble toolchain)
- C++ standard: C++17 (implicit from ROS 2 Humble)
- Parameters accessed via getter methods: `mmcmkf->getParams()`
- Debug code guarded with `#ifndef TMP_NDEBUG`

### Build
- ROS 2 Humble (Ubuntu 22.04)
- Build command: `./Make/Linux/compile rbb -p Booster` (from `soccer/`)
- Development container: `redbackbots/redbackbots-dev-env:latest`
- Booster simulator container: `redbackbots/issac-robo-sim:1.1.0`
- Protobuf for serialisation, Boost (program_options, python3, regex, system, thread)
- Libraries: Eigen3, OpenCV, TinyDNN, CompiledNN, kissfft, FadBad

### Git
- Branch-based workflow with PRs to `master`
- `master` branch is "competition ready code" — do not commit directly to master
- See `soccer/CONTRIBUTING.md` for full details

---

## Environment

- **OS:** Ubuntu 22.04
- **ROS 2:** Humble (installed at `/opt/ros/humble/`)
- **Python:** 3.10
- **Dev environment:** Docker container (`redbackbots/redbackbots-dev-env:latest`)
- **Booster simulator:** Isaac Sim based (`redbackbots/issac-robo-sim:1.1.0`)
- **Buildchain:** Set up via `soccer/Util/Buildchain/booster/`
- **SSH keys** mounted into container from `~/.ssh`
