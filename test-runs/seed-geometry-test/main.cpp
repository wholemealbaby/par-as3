/**
 * @file main.cpp
 *
 * Standalone test for computeSeedPose() — the canonical seed-from-directional-
 * observation geometry shared with ParticleFilter.cpp.
 *
 * This file includes SeedGeometry.hpp directly via the -I flag (pointing to
 * the project's include root).  There is NO code duplication: the exact same
 * header is #included by ParticleFilter.cpp in the ROS 2 build.
 *
 * What this test does:
 *   1. Defines the LabLarge field geometry and known field features (matching
 *      FieldFeatureLocations.cpp).
 *   2. Creates synthetic test scenarios: robot at known poses looking at
 *      specific field features.
 *   3. For each scenario, generates a synthetic observation (distance, heading,
 *      orientation) that the robot would see — computed FORWARD from the
 *      known robot pose and feature location.
 *   4. Runs computeSeedPose() BACKWARD to compute the implied robot pose.
 *   5. Verifies geometric correctness (computed pose should match the known
 *      robot pose to numerical precision).
 *   6. Saves all results to seed_output.csv for Python visualisation.
 *
 * Compile (from this directory):
 *   g++ -std=c++17 -I../../soccer/Src/robot/include -o test_seed_geometry main.cpp -lm
 *
 * Run:
 *   ./test_seed_geometry
 *
 * Outputs:
 *   - seed_output.csv        — All test results (for Python visualiser)
 *   - Analysis printed to stdout
 *
 * Dependencies:
 *   - C++17 compiler (g++ or clang++)
 *   - libm (-lm for math functions)
 *   - SeedGeometry.hpp (via project include path)
 */

#include "perception/stateestimation/localiser/particlefilter/SeedGeometry.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>

// ═══════════════════════════════════════════════════════════════════════
//  1. Field Definitions  —  LabLarge  (matches LabLarge.hpp)
// ═══════════════════════════════════════════════════════════════════════

constexpr float FIELD_LENGTH = 6300.0f;        // mm
constexpr float FIELD_WIDTH  = 4050.0f;        // mm

constexpr float HALF_L = FIELD_LENGTH / 2.0f;  // 3150 mm
constexpr float HALF_W = FIELD_WIDTH  / 2.0f;  // 2025 mm

constexpr float GOAL_BOX_LENGTH = 450.0f;
constexpr float GOAL_BOX_WIDTH  = 1800.0f;
constexpr float PENALTY_BOX_LENGTH = 1350.0f;
constexpr float PENALTY_BOX_WIDTH  = 2700.0f;
constexpr float CENTER_CIRCLE_DIAMETER = 1350.0f;
constexpr float CORNER_ARC_RADIUS = 225.0f;

constexpr float M_PI_F = 3.14159265358979323846f;

// ═══════════════════════════════════════════════════════════════════════
//  2. Data Types  —  minimal, no project deps
// ═══════════════════════════════════════════════════════════════════════

struct FieldFeature {
    float x, y, orientation;
};

struct TestCase {
    std::string name;
    std::string featureType;
    float robotX, robotY, robotTheta;
    FieldFeature feature;
};

struct TestResult {
    TestCase tc;
    float computedX, computedY, computedTheta;
    float errorX, errorY, errorTheta;
    float errorPosition;           // Euclidean distance error (mm)
    float obsDistance, obsHeading, obsOrientation;
    bool passed;
};

// ═══════════════════════════════════════════════════════════════════════
//  3. Field Feature Database  —  matches FieldFeatureLocations.cpp
// ═══════════════════════════════════════════════════════════════════════

std::vector<FieldFeature> buildCorners() {
    return {
        {-HALF_L,                   HALF_W,                  -M_PI_F / 4.0f},
        {-HALF_L + GOAL_BOX_LENGTH,  GOAL_BOX_WIDTH / 2.0f,  -3.0f * M_PI_F / 4.0f},
        {-HALF_L + PENALTY_BOX_LENGTH, PENALTY_BOX_WIDTH / 2.0f, -3.0f * M_PI_F / 4.0f},
        {-HALF_L + GOAL_BOX_LENGTH, -GOAL_BOX_WIDTH / 2.0f,   3.0f * M_PI_F / 4.0f},
        {-HALF_L + PENALTY_BOX_LENGTH, -PENALTY_BOX_WIDTH / 2.0f, 3.0f * M_PI_F / 4.0f},
        {-HALF_L,                   -HALF_W,                  M_PI_F / 4.0f},
        { HALF_L,                    HALF_W,                  -3.0f * M_PI_F / 4.0f},
        { HALF_L - GOAL_BOX_LENGTH,   GOAL_BOX_WIDTH / 2.0f,  -M_PI_F / 4.0f},
        { HALF_L - PENALTY_BOX_LENGTH, PENALTY_BOX_WIDTH / 2.0f, -M_PI_F / 4.0f},
        { HALF_L - GOAL_BOX_LENGTH,  -GOAL_BOX_WIDTH / 2.0f,   M_PI_F / 4.0f},
        { HALF_L - PENALTY_BOX_LENGTH, -PENALTY_BOX_WIDTH / 2.0f, M_PI_F / 4.0f},
        { HALF_L,                   -HALF_W,                   3.0f * M_PI_F / 4.0f},
    };
}

std::vector<FieldFeature> buildTJunctions() {
    std::vector<FieldFeature> tj = {
        {-HALF_L,  GOAL_BOX_WIDTH / 2.0f,   0.0f},
        {-HALF_L,  PENALTY_BOX_WIDTH / 2.0f, 0.0f},
        {-HALF_L, -GOAL_BOX_WIDTH / 2.0f,   0.0f},
        {-HALF_L, -PENALTY_BOX_WIDTH / 2.0f, 0.0f},
        {0.0f,     HALF_W,                  -M_PI_F / 2.0f},
        {0.0f,    -HALF_W,                   M_PI_F / 2.0f},
        { HALF_L,  GOAL_BOX_WIDTH / 2.0f,   M_PI_F},
        { HALF_L,  PENALTY_BOX_WIDTH / 2.0f, M_PI_F},
        { HALF_L, -GOAL_BOX_WIDTH / 2.0f,   M_PI_F},
        { HALF_L, -PENALTY_BOX_WIDTH / 2.0f, M_PI_F},
    };
    // Corner arcs
    if (CORNER_ARC_RADIUS > 0) {
        tj.push_back({-HALF_L,  HALF_W - CORNER_ARC_RADIUS, 0.0f});
        tj.push_back({-HALF_L + CORNER_ARC_RADIUS, HALF_W, -M_PI_F / 2.0f});
        tj.push_back({-HALF_L, -HALF_W + CORNER_ARC_RADIUS, 0.0f});
        tj.push_back({-HALF_L + CORNER_ARC_RADIUS, -HALF_W, M_PI_F / 2.0f});
        tj.push_back({ HALF_L,  HALF_W - CORNER_ARC_RADIUS, M_PI_F});
        tj.push_back({ HALF_L - CORNER_ARC_RADIUS, HALF_W, -M_PI_F / 2.0f});
        tj.push_back({ HALF_L, -HALF_W + CORNER_ARC_RADIUS, M_PI_F});
        tj.push_back({ HALF_L - CORNER_ARC_RADIUS, -HALF_W, M_PI_F / 2.0f});
    }
    return tj;
}

std::vector<FieldFeature> buildXJunctions() {
    return {
        {0.0f,  CENTER_CIRCLE_DIAMETER / 2.0f, 0.0f},
        {0.0f, -CENTER_CIRCLE_DIAMETER / 2.0f, 0.0f},
    };
}

std::vector<FieldFeature> buildCentreCircles() {
    return {
        {0.0f, 0.0f,  M_PI_F / 2.0f},
        {0.0f, 0.0f, -M_PI_F / 2.0f},
    };
}

// ═══════════════════════════════════════════════════════════════════════
//  4. Forward Observation Generation
// ═══════════════════════════════════════════════════════════════════════

/**
 * Generate a synthetic observation that a robot at (robotX, robotY, robotTheta)
 * would see of a field feature at (ff.x, ff.y, ff.orientation).
 *
 * This is the FORWARD computation: given known robot pose and feature,
 * compute what the sensor would report.
 */
void generateObservation(
    float robotX, float robotY, float robotTheta,
    const FieldFeature &ff,
    float &outDistance, float &outHeading, float &outOrientation)
{
    float dx = ff.x - robotX;
    float dy = ff.y - robotY;
    outDistance = std::sqrt(dx * dx + dy * dy);

    // Heading: bearing of feature relative to robot's forward direction
    outHeading = seedNormaliseTheta(std::atan2(dy, dx) - robotTheta);

    // Orientation: angle from feature's known orientation to the direction
    // from feature to robot
    outOrientation = seedNormaliseTheta(
        std::atan2(robotY - ff.y, robotX - ff.x) - ff.orientation
    );
}

// ═══════════════════════════════════════════════════════════════════════
//  5. Test Scenarios
// ═══════════════════════════════════════════════════════════════════════

std::vector<TestCase> buildTestCases() {
    auto corners = buildCorners();
    auto tJunctions = buildTJunctions();
    auto xJunctions = buildXJunctions();
    auto centreCircles = buildCentreCircles();

    std::vector<TestCase> cases;

    // Test 1: Robot at origin looking at Left Top corner
    cases.push_back({"Robot at origin -> Left Top corner",
        "fCorner", 0.0f, 0.0f, 0.0f, corners[0]});

    // Test 2: Robot in bottom-left quadrant, facing right
    cases.push_back({"Robot in bottom-left -> Right Goal Top T-Junction",
        "fTJunction", -2000.0f, -1000.0f, 0.5f, tJunctions[6]});

    // Test 3: Robot near right goal, looking back at centre circle
    cases.push_back({"Robot near right goal -> Centre Circle (ori 1)",
        "fCentreCircle", 2500.0f, 500.0f, M_PI_F, centreCircles[0]});

    // Test 4: Robot at top-centre looking down at bottom X-junction
    cases.push_back({"Robot at top-centre -> Bottom X-Junction",
        "fXJunction", 0.0f, 1500.0f, -M_PI_F / 2.0f, xJunctions[1]});

    // Test 5: Robot slightly right of centre -> Left Penalty Bottom corner
    cases.push_back({"Robot right of centre -> Left Penalty Bottom corner",
        "fCorner", 500.0f, 300.0f, 2.0f, corners[4]});

    // Test 6: Robot near centre line, facing goal -> Centre Circle (ori 2)
    cases.push_back({"Robot at right side -> Centre Circle (ori 2)",
        "fCentreCircle", 1000.0f, -800.0f, 2.5f, centreCircles[1]});

    // Test 7: Robot near left penalty -> Left Goal Top corner
    cases.push_back({"Robot near left penalty -> Left Goal Top corner",
        "fCorner", -2500.0f, 500.0f, 1.2f, corners[1]});

    // Test 8: Robot at top sideline -> Centre Top T-Junction
    cases.push_back({"Robot at top sideline -> Centre Top T-Junction",
        "fTJunction", 800.0f, 1800.0f, -0.8f, tJunctions[4]});

    // Test 9: Robot at right-bottom corner -> Left top corner (long range)
    cases.push_back({"Robot at right-bottom -> Left Top corner (long range)",
        "fCorner", 3000.0f, -1900.0f, M_PI_F + 0.3f, corners[0]});

    // Test 10: Robot near centre facing right -> Right Goal Top corner
    cases.push_back({"Robot near centre facing right -> Right Goal Top corner",
        "fCorner", 200.0f, -300.0f, M_PI_F / 4.0f, corners[6]});

    return cases;
}

// ═══════════════════════════════════════════════════════════════════════
//  6. Run Tests
// ═══════════════════════════════════════════════════════════════════════

TestResult runTest(const TestCase &tc) {
    TestResult r;
    r.tc = tc;

    // Generate synthetic observation (forward)
    generateObservation(
        tc.robotX, tc.robotY, tc.robotTheta,
        tc.feature,
        r.obsDistance, r.obsHeading, r.obsOrientation
    );

    // Run computeSeedPose (backward — the function under test)
    SeedPose pose = computeSeedPose(
        tc.feature.x, tc.feature.y, tc.feature.orientation,
        r.obsDistance, r.obsHeading, r.obsOrientation
    );
    r.computedX = pose.x;
    r.computedY = pose.y;
    r.computedTheta = pose.theta;

    // Errors
    r.errorX = pose.x - tc.robotX;
    r.errorY = pose.y - tc.robotY;
    r.errorTheta = seedNormaliseTheta(pose.theta - tc.robotTheta);
    r.errorPosition = std::sqrt(r.errorX * r.errorX + r.errorY * r.errorY);

    // PASS if position error < 1 mm and heading error < 1e-6 rad
    r.passed = (r.errorPosition < 1.0f && std::abs(r.errorTheta) < 1e-6f);

    return r;
}

// ═══════════════════════════════════════════════════════════════════════
//  7. CSV Output
// ═══════════════════════════════════════════════════════════════════════

void writeCsv(const std::vector<TestResult> &results, const char *filename) {
    std::ofstream csv(filename);
    csv.precision(8);
    csv << std::fixed;

    csv << "test_idx,feature_type,"
        << "ff_x,ff_y,ff_orientation,"
        << "obs_distance,obs_heading,obs_orientation,"
        << "computed_x,computed_y,computed_theta,"
        << "robot_x,robot_y,robot_theta,"
        << "error_x,error_y,error_theta,error_position,passed\n";

    for (size_t i = 0; i < results.size(); ++i) {
        const auto &r = results[i];
        const auto &tc = r.tc;
        csv << i << ","
            << tc.featureType << ","
            << tc.feature.x << "," << tc.feature.y << "," << tc.feature.orientation << ","
            << r.obsDistance << "," << r.obsHeading << "," << r.obsOrientation << ","
            << r.computedX << "," << r.computedY << "," << r.computedTheta << ","
            << tc.robotX << "," << tc.robotY << "," << tc.robotTheta << ","
            << r.errorX << "," << r.errorY << "," << r.errorTheta << ","
            << r.errorPosition << "," << (r.passed ? 1 : 0) << "\n";
    }

    csv.close();
    std::printf("  CSV written: %s\n", filename);
}

// ═══════════════════════════════════════════════════════════════════════
//  8. Console Analysis
// ═══════════════════════════════════════════════════════════════════════

void printAnalysis(const std::vector<TestResult> &results) {
    int passedCount = 0;
    for (const auto &r : results) {
        if (r.passed) ++passedCount;
    }
    int total = static_cast<int>(results.size());

    std::printf("\n");
    std::printf("════════════════════════════════════════════════════════════════════════\n");
    std::printf("  computeSeedPose()  —  Geometry Verification Test\n");
    std::printf("  (Canonical implementation in SeedGeometry.hpp)\n");
    std::printf("════════════════════════════════════════════════════════════════════════\n");
    std::printf("  Field: LabLarge  %.0f x %.0f mm\n", FIELD_LENGTH, FIELD_WIDTH);
    std::printf("  Tests: %d  (%d passed, %d failed)\n\n",
                total, passedCount, total - passedCount);

    for (size_t i = 0; i < results.size(); ++i) {
        const auto &r = results[i];
        const auto &tc = r.tc;
        const char *status = r.passed ? "PASS" : "FAIL";

        std::printf("  ── Test %zu: %s  [%s]\n", i + 1, tc.name.c_str(), status);
        std::printf("     Feature: %s  @ (%.1f, %.1f, %.2f°)\n",
                    tc.featureType.c_str(),
                    tc.feature.x, tc.feature.y,
                    tc.feature.orientation * 180.0f / M_PI_F);
        std::printf("     Robot:   (%.1f, %.1f, %.2f°)\n",
                    tc.robotX, tc.robotY, tc.robotTheta * 180.0f / M_PI_F);
        std::printf("     Obs:     dist=%.2f  heading=%.4f°  orientation=%.4f°\n",
                    r.obsDistance,
                    r.obsHeading * 180.0f / M_PI_F,
                    r.obsOrientation * 180.0f / M_PI_F);
        std::printf("     Result:  (%.1f, %.1f, %.2f°)\n",
                    r.computedX, r.computedY, r.computedTheta * 180.0f / M_PI_F);
        std::printf("     Errors:  dx=%.4f  dy=%.4f  dtheta=%.8f°  pos=%.4f mm\n\n",
                    r.errorX, r.errorY, r.errorTheta * 180.0f / M_PI_F, r.errorPosition);
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  9. Main
// ═══════════════════════════════════════════════════════════════════════

int main() {
    std::printf("Building test cases...\n");
    auto testCases = buildTestCases();
    std::printf("  %zu test cases created\n\n", testCases.size());

    std::printf("Running tests...\n");
    std::vector<TestResult> results;
    results.reserve(testCases.size());
    for (const auto &tc : testCases) {
        results.push_back(runTest(tc));
    }
    std::printf("  All tests complete\n");

    // Write CSV
    writeCsv(results, "seed_output.csv");

    // Print analysis
    printAnalysis(results);

    return 0;
}
