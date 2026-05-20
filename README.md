# PAR Assignment 3 — RoboCup Soccer Localisation

**Course:** COSC2781 (PG) / COSC2814 (UG) — Semester 1 2026
**Robot:** Booster K1 (humanoid)
**Organisation:** RMIT University — Programming Autonomous Robots

---

## Project Overview

This project is part of the **RoboCup Soccer** stream (Project 4.2). We are working with the
[RedBackBots](https://github.com/rmit-computing-technologies/redbackbots-soccer) RoboCup
codebase to improve the localisation capabilities of the **Booster K1** humanoid robot for the
RoboCup Humanoid Soccer League (HSL) 2026 competition.

### Key Goals

1. **Particle Filter Localisation** — Replace the existing Multi-Modal Conditional Mixture
   Kalman Filter (MultiModalCMKF) with a particle filter that uses the same input data
   structures (`EstimatorInfoIn`, `FieldFeatureInfo`, odometry) and outputs through the
   same `EstimatorInfoOut` interface.

2. **HTC Vive External Pose Estimation** — Use HTC Vive motion trackers (mounted on the
   robot and tracked by base stations) to provide ground-truth external pose estimation
   for evaluation and potential sensor fusion. See:
   - `docs/equipment/htc_vive_tracker_user_manual.txt`
   - `docs/equipment/htc_vive_tracker_v1.txt`
   - `docs/equipment/htc_vive_base_station_user_guide.txt`
   - `docs/ara/` (Activity Risk Assessments)

3. **Perimeter Breaking for Field Localisation** — Develop vision-only localisation
   features that break the symmetry of the RoboCup field by detecting distinctive
   perimeter/structure features (goal posts, corner arcs, unique line intersections)
   using only the robot's stereo cameras.

---

## Repository Structure

| Path | Description |
|------|-------------|
| `soccer/` | RedBackBots RoboCup Soccer codebase (subtree/submodule) |
| `docs/` | Assignment specification, equipment docs, ARA, research notes |
| `docs/spec/` | Assignment specification and rubrics |
| `docs/ara/` | Activity Risk Assessments for HTC Vive equipment |
| `docs/equipment/` | HTC Vive hardware documentation |
| `docs/redback_localization_summary.md` | Comprehensive summary of the existing localisation system |
| `.issues/` | Issue drafts and planning documents |

### Key Directories in `soccer/`

| Path | Relevance |
|------|-----------|
| `soccer/Src/robot/src/perception/stateestimation/localiser/` | **Main target** — existing MultiModalCMKF implementation to replace |
| `soccer/Src/robot/include/perception/stateestimation/localiser/` | **Main target** — header files for localisation |
| `soccer/Src/robot/include/types/field/FieldFeatureLocations.hpp` | Field landmark database used by the localiser |
| `soccer/Src/robot/include/types/FieldFeatureInfo.hpp` | Input feature type definitions |

---

## Booster K1 Robot Specifications

- **Type:** Humanoid robot (RoboCup HSL compliant)
- **Platform:** ROS 2 Humble
- **Build System:** Colcon (ROS 2) via `./Make/Linux/compile rbb -p Booster`
- **Sensors:** Stereo cameras (primary), IMU, joint encoders
- **HSL Constraint:** Only human-like senses permitted — stereo cameras on the front of the
  head are the primary localisation sensor (no LIDAR, no external markers for in-game use)
- **SDK:** Booster Robotics SDK (see `soccer/booster_ws/`)
- **Development Container:** `redbackbots/redbackbots-dev-env:latest` (Ubuntu 22.04)
  or `redbackbots/issac-robo-sim:1.1.0` (Booster simulator)

---

## Important Constraints

1. **The RedBackBots codebase architecture is FIXED.** We cannot modify the overall robot
   software architecture or files unrelated to our localisation scope. This prevents
   interference with the RoboCup July 2026 competition preparations.
2. **Human-like senses only.** Per HSL rules, the robot cannot use LIDAR, external
   cameras, or other non-human sensing for in-game localisation.
3. **HTC Vive is for evaluation only** (or external ground truth), not for in-game use.
4. **AI tools must be logged** — see `docs/spec/assignment_specification.txt` Section 1.3.

---

## Quick Links

- [Assignment Specification (simple)](docs/spec/assignment_specification_simple.md)
- [Project TL;DR & Gotchas](docs/spec/project_tldr_gotchas.md)
- [Localisation System Summary](docs/redback_localization_summary.md)
- [Soccer Codebase README](soccer/README.md)
