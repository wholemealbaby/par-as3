# Localisation System Summary

**Redback Bots Soccer - RoboCup Localisation Architecture**

## Overview

The localisation system uses a **Multi-Modal Conditional Mixture Kalman Filter (MultiModalCMKF)** to estimate robot pose on the soccer field. It maintains multiple competing hypotheses (modes) to handle ambiguity and kidnapped robot scenarios, with each hypothesis having a probability weight.

---

## 1. Core System Architecture

### Main Components

| Component | Purpose | Location |
|-----------|---------|----------|
| **MultiModalCMKF** | Core multi-hypothesis Kalman filter engine | `Src/robot/include/perception/stateestimation/localiser/multimodalcmkf/MultiModalCMKF.hpp` |
| **CMKF** | Individual Kalman filter hypothesis/mode | `Src/robot/include/perception/stateestimation/localiser/multimodalcmkf/CMKF.hpp` |
| **MultiModalCMKFParams** | Tunable parameters and weights | `Src/robot/include/perception/stateestimation/localiser/multimodalcmkf/MultiModalCMKFParams.hpp` |
| **FieldFeatureLocations** | Database of field landmarks | `Src/robot/include/types/field/FieldFeatureLocations.hpp` |
| **MultiModalCMKFTransitioner** | Handles state transitions and initialization | `Src/robot/include/perception/stateestimation/localiser/multimodalcmkf/MultiModalCMKFTransitioner.hpp` |

### State Representation

Each CMKF hypothesis contains:

```
State Vector (3D):
  - X position (robot's field X coordinate in mm)
  - Y position (robot's field Y coordinate in mm)  
  - Heading (theta, robot orientation in radians)

Covariance Matrix (3×3):
  Represents uncertainty in each dimension and correlations
  Diagonal: Var(x), Var(y), Var(heading)

Weight (float):
  Normalized probability [0.0 to 1.0] across all modes
  Sum of all mode weights = 1.0
```

---

## 2. Weights and Parameters

### MultiModalCMKFParams Structure

All parameters are accessible via `MultiModalCMKFParams *params` from `MultiModalCMKF`:

```cpp
// How to access parameters:
MultiModalCMKF *mmcmkf = /* initialized */;
MultiModalCMKFParams *params = mmcmkf->getParams();
float value = params->cornerModeSplitWeightMultiplyFactor;
```

### Mode-Split Weights

These weights are applied when creating new hypotheses from field feature observations. Lower values = less likely to spawn a new mode.

| Parameter | Default Value | Purpose |
|-----------|---------------|---------|
| `penaltySpotModeSplitWeightMultiplyFactor` | 0.5 | Penalty spot observation weight |
| `cornerModeSplitWeightMultiplyFactor` | 0.1 | Field corner observation weight |
| `tJunctionModeSplitWeightMultiplyFactor` | 0.1 | T-junction observation weight |
| `xJunctionModeSplitWeightMultiplyFactor` | 0.1 | X-junction (center) observation weight |
| `centreCircleModeSplitWeightMultiplyFactor` | 0.2 | Center circle observation weight |
| `centreCircleDirectionlessModeSplitWeightMultiplyFactor` | 0.5 | Directionless center circle weight |
| `lineModeSplitWeightMultiplyFactor` | 0.2 | Field line observation weight |
| `yoloCornerModeSplitWeightMultiplyFactor` | 0.05 | YOLO corner (directionless) |
| `yoloTJunctionModeSplitWeightMultiplyFactor` | 0.05 | YOLO T-junction (directionless) |
| `yoloXJunctionModeSplitWeightMultiplyFactor` | 0.05 | YOLO X-junction (directionless) |
| `yoloPenaltySpotModeSplitWeightMultiplyFactor` | 0.1 | YOLO penalty spot (directionless) |
| `yoloGoalPostModeSplitWeightMultiplyFactor` | 0.05 | YOLO goal post (directionless) |

**Key Insight**: YOLO detections have lower weights because they lack orientation information, producing higher uncertainty.

### Odometry Process Noise

Noise applied during prediction phase based on motion magnitude:

| Parameter | Default Value | Purpose |
|-----------|---------------|---------|
| `odometryForwardMultiplyFactor` | 20.0 | Uncertainty scaling for forward motion |
| `odometryLeftMultiplyFactor` | 20.0 | Uncertainty scaling for lateral motion |
| `odometryHeadingMultiplyFactor` | 0.5 | Uncertainty scaling for rotational motion |

**Usage**: `covariance += motionNoise * factor`

### Observation Uncertainty (Measurement Noise)

These parameters define how much we trust vision observations:

| Parameter | Default Value | Purpose |
|-----------|---------------|---------|
| `angleUncertainty` | 20° | Angular resolution of feature detection |
| `updateHeadingUncertainty` | 20° | Heading update uncertainty in degrees |
| `distanceUncertaintyBase` | 100 mm | Baseline distance measurement noise |
| `distanceUncertaintyScale` | 0.25 | Distance-proportional noise (25% of distance) |

**Calculation**: `distanceNoise = base + distance * scale`

### Mode Management Thresholds

These control when modes are merged, deleted, or kept:

| Parameter | Default Value | Purpose |
|-----------|---------------|---------|
| `similarHeadingThresh` | 30° | Heading difference to trigger merge |
| `similarXThresh` | 500 mm | X position difference to trigger merge |
| `similarYThresh` | 500 mm | Y position difference to trigger merge |
| `minCMKFWeight` | 0.03 (3%) | Modes below this probability are deleted |
| `minWeightAdjustment` | 0.01 | Floor for weight multiplier during updates |

**Usage**: Two modes are merged if all three differences are below thresholds.

### Line Feature Parameters

For field boundary line observations:

| Parameter | Default Value | Purpose |
|-----------|---------------|---------|
| `lineSegmentOverlapSigmaFactor` | 0.25 | Gaussian scoring width (relative to line length) |
| `lineMaxLengthRatio` | 1.2 | Maximum ratio of observed/expected line length |

**Usage**: Higher overlap scores increase the weight update multiplier.

### Anti-Flip Gate

Prevents spurious heading reversals:

| Parameter | Default Value | Purpose |
|-----------|---------------|---------|
| `antiFlipHeadingThreshold` | 150° | Heading difference threshold for detection |
| `antiFlipMinWeightRatio` | 3.0 | New mode must have 3× weight to override |

**Logic**: If best mode changes with heading > 150°, suppress unless new mode has 3× weight.

---

## 3. Input Data Structures

### EstimatorInfoInit - Initialization Configuration

Called once at system startup:

```cpp
// Access example in Localiser.cpp constructor:
Localiser::Localiser(const EstimatorInfoInit &estimatorInfoInit)
    : Estimator(estimatorInfoInit) { ... }

// Fields:
int playerNumber;                          // 1-based robot ID (1-6)
int teamNumber;                            // Team identifier
std::string initialPoseType;               // "left", "right", "specified", "manual"
AbsCoord specifiedInitialPose;             // If specified type, position here
std::string skill;                         // Current role ("Game", "Goalkeeper", etc)
uint8_t competitionType;                   // League type
```

### EstimatorInfoIn - Per-Frame Observations

Called every frame with sensor data:

```cpp
// Main data containers:
std::vector<FieldFeatureInfo> fieldFeatures;   // Detected field landmarks (see Section 4)
std::vector<BallInfo> balls;                   // Detected balls
Odometry odometryDiff;                         // Motion since last frame (vx, vy, omega)
float dtInSeconds;                             // Frame delta time
float headYaw;                                 // Head yaw angle

// Game state:
uint8_t state;                    // STATE_READY, STATE_SET, STATE_PLAYING, etc
uint8_t phase;                    // Game phase
uint8_t penalty;                  // Penalty type (if any)
bool isIncapacitated;             // Robot disabled/picked up
bool seenRefGesture;              // Referee gesture detected

// Team data:
std::vector<BroadcastData> incomingBroadcastData;  // Other robots' poses for sensor fusion
```

### EstimatorInfoMiddle - Internal Control Flags

Set by `Localiser` to control processing:

```cpp
bool canLocaliseInState;      // Whether to run update phase (false during inactive states)
bool canDoObservations;       // Whether observations are valid (false during actions that invalidate position)

// Set to false when:
//   canLocaliseInState = false:  If state != READY/SET/PLAYING and no ref gesture
//   canDoObservations = false:   During pickup, diving, dead, or getting-up animations
```

### EstimatorInfoOut - Output State Estimate

Primary output after `mmcmkf->tick()`:

```cpp
AbsCoord robotPos;                    // Best estimate position (x, y in mm)
float robotPosUncertainty;            // Radial uncertainty (mm)
float robotHeadingUncertainty;        // Heading uncertainty (radians)

std::vector<AbsCoord> allRobotPos;    // All hypothesis positions (for debugging)

AbsCoord ballPos;                     // Best estimate ball position
AbsCoord ballVel;                     // Ball velocity

AbsCoord teamBallPos;                 // Fused ball position from all teammates
float teamBallPosUncertainty;         // Team ball uncertainty
```

---

## 4. Field Features

### FieldFeatureInfo - Observed Feature Structure

Every detected field landmark is a `FieldFeatureInfo`:

```cpp
struct FieldFeatureInfo {
    RRCoord rr;                         // Robot-relative: distance, bearing, variance
    Type type;                          // Feature type (see types below)
    
    // For line features:
    Point p1, p2;                       // Line endpoints in image coordinates
    PointF field1, field2;              // Line endpoints in field coordinates
    RRCoord rrEndpoint1, rrEndpoint2;   // Polar coords of line endpoints
    float fieldLinesWidth;              // Observed line width
    
    // Sensor metadata:
    bool topCamera;                     // True if from top camera
    bool isYOLO;                        // True if from deep learning detector
    BBox bbox;                          // YOLO bounding box
};
```

### Field Feature Types

| Type | Enum Value | Description | Orientation | Database Field |
|------|-----------|-------------|-------------|-|
| Line | `fLine` | Field line segment | Directional | `constantXLines`, `constantYLines` |
| Corner | `fCorner` | Field corner (4 total) | Directional | `corners` |
| T-Junction | `fTJunction` | T-shaped junction | Directional | `t_junctions` |
| Penalty Spot | `fPenaltySpot` | Center penalty spot | Directionless | `penalty_spots` |
| Centre Circle | `fCentreCircle` | Center circle perimeter | Directional | `centre_circles` |
| Centre Circle Directionless | `fCentreCircleDirectionless` | Center circle | Directionless | `centre_circles_directionless` |
| X-Junction | `fXJunction` | Center (4-way junction) | Directional | `x_junctions` |
| Goal Post | `fGoalPost` | Goal post | Directionless | (YOLO only) |

### Oriented Field Features

Features with bearing information (corners, junctions, etc):

```cpp
struct FieldFeature {
    double x;                // Field X coordinate (mm)
    double y;                // Field Y coordinate (mm)
    double orientation;      // Expected feature orientation (radians)
};

// Access example:
FieldFeatureLocations *ffl = mmcmkf->getFieldFeatureLocations();
for (auto &corner : ffl->corners) {
    float fx = corner.x;          // Field position
    float fy = corner.y;
    float bearing = corner.orientation;
}
```

### Directionless Field Features

Features with position only (penalty spot, center circle):

```cpp
struct DirectionlessFieldFeature {
    float x;                // Field X coordinate (mm)
    float y;                // Field Y coordinate (mm)
    // No orientation - only distance/bearing constrain position
};

// Access example:
for (auto &spot : ffl->penalty_spots) {
    float dist = rr.distance;           // Observed distance
    float bearing = rr.bearing;         // Observed bearing
    // Update constrains: x = spot.x ± error, y = spot.y ± error
}
```

### Field Line Segments

Boundary lines (field edges):

```cpp
struct FieldLineSegment {
    float value;           // Constant coordinate (e.g., x=0 for left boundary)
    float min;             // Minimum extent along other axis
    float max;             // Maximum extent along other axis
};

// Example: left boundary
// x = 0 (vertical line at x=0), y ranges from 0 to ~9000mm

// Access example:
for (auto &line : ffl->constantXLines) {
    if (line.value == 0.0f) {  // Left boundary
        // Use this line for localization update
    }
}
```

### FieldFeatureLocations Database

Pre-initialized reference database of all field landmarks:

```cpp
class FieldFeatureLocations {
    // Oriented features:
    std::vector<FieldFeature> corners;              // 4 field corners
    std::vector<FieldFeature> t_junctions;          // Goal box corners
    std::vector<FieldFeature> x_junctions;          // Center junction (x, y)
    std::vector<FieldFeature> centre_circles;       // Multiple center circle points
    
    // Directionless features:
    std::vector<DirectionlessFieldFeature> centre_circles_directionless;
    std::vector<DirectionlessFieldFeature> penalty_spots;  // 2 per field
    std::vector<DirectionlessFieldFeature> goal_posts;     // 4 posts
    
    // Boundary lines:
    std::vector<FieldLineSegment> constantXLines;   // Vertical lines
    std::vector<FieldLineSegment> constantYLines;   // Horizontal lines
    
    Boundaryf edge;  // Field edge boundary
};

// Access in MultiModalCMKF:
FieldFeatureLocations *ffl = mmcmkf->getFieldFeatureLocations();
```

---

## 5. Processing Pipeline

### Main Localization Cycle (tick function)

Called every frame with observations:

```cpp
void Localiser::tick(
    const EstimatorInfoIn &estimatorInfoIn,
    EstimatorInfoMiddle &estimatorInfoMiddle,
    EstimatorInfoOut &estimatorInfoOut)
{
    // 1. Determine if we can localize in current game state
    fillCanLocaliseInState(estimatorInfoIn, estimatorInfoMiddle);
    
    // 2. Determine if observations are valid (not during animation)
    fillCanDoObservations(estimatorInfoIn, estimatorInfoMiddle);
    
    // 3. Run multi-modal Kalman filter
    mmcmkf->tick(estimatorInfoIn, estimatorInfoMiddle, estimatorInfoOut);
}
```

### MultiModalCMKF Processing Steps

For each frame, `MultiModalCMKF::tick()` performs:

#### **Step 1: Prediction Phase**

For each Kalman filter hypothesis (mode):

```
1. Apply odometry motion to state vector
2. Add process noise to covariance (scaled by motion magnitude)
3. Check anti-flip gate for heading consistency
```

**Process Noise Calculation**:
```cpp
float forwardMotion = sqrt(vx² + vy²);
float rotation = abs(omega);

covarianceNoise += [
    forwardMotion * odometryForwardMultiplyFactor,      0,
    0,                                                   forwardMotion * odometryLeftMultiplyFactor,
    0,                                                   0,
    rotation * odometryHeadingMultiplyFactor
]
```

#### **Step 2: Update Phase (if `canDoObservations = true`)**

For each observed field feature:

1. **Find matching candidates** in field feature database
2. **Create new hypothesis** for each candidate (mode splitting)
3. **Update hypothesis** using appropriate method:
   - Directional features (corners, junctions) → `update(expectedX, expectedY, expectedHeading)`
   - Directionless features (penalty spot, center circle) → `updateUsingDirectionlessObservation()`
   - Line features (boundaries) → `updateUsingConstantXLine()` or `updateUsingConstantYLine()`
4. **Adjust weight** using likelihood:
   ```cpp
   float mahalanobisDistance = innovation^T * inverseInnovationCovariance * innovation
   float weightMultiplier = exp(-0.5 * mahalanobisDistance)
   weightMultiplier = clamp(weightMultiplier, minWeightAdjustment, 1.0)
   kf.weight *= weightMultiplier
   ```

#### **Step 3: Merge Phase**

Combine similar modes to prevent explosion:

```cpp
for (each pair of modes) {
    if (headingDiff < 30° && 
        xDiff < 500mm && 
        yDiff < 500mm) {
        // Merge: combine weighted covariance, keep best weight
        merge(mode1, mode2)
    }
}
```

#### **Step 4: Normalization Phase**

Ensure weights sum to 1.0:

```cpp
float sumWeights = sum of all kf.weight
for (each mode) {
    kf.weight /= sumWeights
}
```

#### **Step 5: Pruning Phase**

Delete unlikely modes:

```cpp
// Remove if weight < 3% probability
for (each mode) {
    if (kf.weight < 0.03) {
        delete mode
    }
}

// Remove if off the field
for (each mode) {
    if (!fieldBoundary.contains(kf.state.x, kf.state.y)) {
        delete mode
    }
}
```

#### **Step 6: Best Mode Selection**

Choose output hypothesis:

```cpp
// Select highest-weight mode
bestCMKF = mode with highest weight

// Apply anti-flip gate
if (heading_change > 150°) {
    if (newMode.weight < 3.0 * lastMode.weight) {
        // Suppress flip, keep old heading
        bestCMKF->state.heading = lastBestHeading
    }
}
lastBestHeading = bestCMKF->state.heading
```

---

## 6. How to Use and Call

### Initialization in Localiser Constructor

```cpp
Localiser::Localiser(const EstimatorInfoInit &estimatorInfoInit)
    : Estimator(estimatorInfoInit)  // Store init config
{
    mmcmkf = new MultiModalCMKF(estimatorInfoInit);  // Initialize MMCMKF
}
```

### Main Processing Loop

```cpp
// Somewhere in perception pipeline:
Localiser localiser(estimatorInfoInit);

// Each frame:
EstimatorInfoIn in;
in.fieldFeatures = vision->detectedFeatures();  // From vision module
in.odometryDiff = odometry->getDelta();          // From odometry
in.dtInSeconds = 0.033f;                         // ~30Hz

EstimatorInfoMiddle middle;
EstimatorInfoOut out;

localiser.tick(in, middle, out);

// Use output:
float x = out.robotPos.x;
float y = out.robotPos.y;
float uncertainty = out.robotPosUncertainty;
```

### Accessing Parameters

```cpp
// Get parameter object:
MultiModalCMKFParams *params = localiser.getCMKFParams();

// Read parameter:
float cornerWeight = params->cornerModeSplitWeightMultiplyFactor;

// Modify at runtime:
params->cornerModeSplitWeightMultiplyFactor = 0.15f;  // Increase corner weighting

// Query field features:
FieldFeatureLocations *ffl = mmcmkf->getFieldFeatureLocations();
int numCorners = ffl->corners.size();  // Should be 4
```

### Accessing All Hypotheses

```cpp
// Access all competing hypotheses:
std::vector<AbsCoord> allPoses = out.allRobotPos;  // All hypothesis positions

// For debugging - iterate through all modes:
for (size_t i = 0; i < allPoses.size(); ++i) {
    float x = allPoses[i].x;
    float y = allPoses[i].y;
    float weight = /* would need to add to out struct */;
}
```

### Game State Integration

```cpp
// Localizer automatically disables updates in certain states:

// In Localiser::fillCanLocaliseInState():
if (estimatorInfoInit.skill == "Game") {
    uint8_t state = estimatorInfoIn.state;
    if (!(state == STATE_READY || state == STATE_SET || 
          state == STATE_PLAYING || estimatorInfoIn.seenRefGesture)) {
        estimatorInfoMiddle.canLocaliseInState = false;  // No updates
    }
}

// In Localiser::fillCanDoObservations():
if (action == PICKUP || action == DIVING || 
    action == DEAD || action == GETTING_UP) {
    estimatorInfoMiddle.canDoObservations = false;  // Observations ignored
}
```

---

## 7. Tuning Guide

### When to Adjust Parameters

| Issue | Adjustment | Why |
|-------|-----------|-----|
| Robot loses localization | Increase `odometryForwardMultiplyFactor` | More trust in features vs odometry |
| Too many hypotheses | Increase `minCMKFWeight` | Delete weak modes faster |
| Jittery heading | Increase `updateHeadingUncertainty` | Less confident in heading updates |
| Kidnapped robot recovery slow | Decrease mode split weights | More aggressive mode creation |
| Wrong corner hypotheses | Decrease `cornerModeSplitWeightMultiplyFactor` | Less likely to match wrong corner |
| Heading flips | Increase `antiFlipHeadingThreshold` | More aggressive flip suppression |

### Recommended Testing Procedure

1. Set `minCMKFWeight = 0.05` for fewer modes (faster)
2. Observe if robot drifts → increase `odometryForwardMultiplyFactor`
3. Check if heading oscillates → increase `updateHeadingUncertainty`
4. Verify corner matching accuracy → check field feature database
5. Reduce `minCMKFWeight` to 0.03 for better ambiguity handling

---

## 8. Field Coordinate System

Standard RoboCup SPL field:

```
                    BLUE GOAL
                   (opponents)
                        |
    +---------------------------------------------------+
    |   (0, 4500)                        (0, 4500)     |
    |     *---------*                      *---------*  |
    |     |         |                      |         |  |
    |     | Penalty |                      | Penalty |  |
    |     |  Spot   |                      |  Spot   |  |
    |     | (0,3900)|                      |(0,3900) |  |
    |     |         |                      |         |  |
    |     *---------*                      *---------*  |
    |                                                    |
    |                Center Circle                      |
    |                 Radius 750mm                      |
    |                  (0, 0)                           |
    |                    *                              |
    |                                                    |
    |     *---------*                      *---------*  |
    |     |         |                      |         |  |
    |     | Penalty |                      | Penalty |  |
    |     |  Spot   |                      |  Spot   |  |
    |     |(0,-3900)|                      |(0,-3900)|  |
    |     |         |                      |         |  |
    |     *---------*                      *---------*  |
    |  (0, -4500)                       (0, -4500)      |
    +---------------------------------------------------+
   X=-4500                             X=+4500

                     RED GOAL
                    (our goal)
```

- **X-axis**: -4500 to +4500 mm (left to right, field length)
- **Y-axis**: -3000 to +3000 mm (front to back, field width)
- **Origin (0,0)**: Center circle
- **Heading (θ)**: 0° = +X direction, π/2 = +Y direction

---

## 9. Debugging and Introspection

### Accessing Debugger (if enabled)

```cpp
#ifndef TMP_NDEBUG
StateEstimationDebuggerBlackboard *debugger = /* get from blackboard */;
localiser.setDebugger(debugger);  // Enable detailed logging
#endif
```

### Checking Mode Count

```cpp
EstimatorInfoOut out;
size_t numModes = out.allRobotPos.size();
printf("Active hypotheses: %zu\n", numModes);
```

### Uncertainty Check

```cpp
if (out.robotPosUncertainty > 1000.0f) {  // 1000mm = high uncertainty
    printf("Low confidence in position estimate\n");
}

if (out.robotHeadingUncertainty > 0.5f) {  // ~30 degrees
    printf("High heading uncertainty\n");
}
```

### Hypothesis Visualization

```cpp
for (size_t i = 0; i < out.allRobotPos.size(); ++i) {
    printf("Mode %zu: (%.1f, %.1f)\n", 
           i, out.allRobotPos[i].x, out.allRobotPos[i].y);
}
```

---

## Summary Table

| Aspect | Key Parameters | Default Values | Purpose |
|--------|---|---|---|
| **Weights** | Mode split factors | 0.05 - 0.5 | Control new hypothesis probability |
| **Noise** | Odometry factors | 0.5 - 20.0 | Uncertainty during prediction |
| **Measurement** | Distance/angle uncertainty | 100mm / 20° | Vision measurement trust |
| **Thresholds** | Merge/prune criteria | 30°, 500mm, 0.03 | Mode management |
| **Features** | 8 types | Corners, junctions, lines, spots | Observable landmarks |
| **State** | 3D (x, y, θ) | Continuous | Robot pose estimate |
| **Input** | FieldFeatureInfo | Multiple | Vision observations |
| **Output** | EstimatorInfoOut | Full structure | Best estimate + uncertainty |


