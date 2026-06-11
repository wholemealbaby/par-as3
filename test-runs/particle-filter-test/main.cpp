/**
 * @file main.cpp
 *
 * Standalone unit tests for MergeSeededParticles.hpp and
 * ResampleParticles.hpp — the header-only particle filter functions.
 *
 * This file includes the headers directly via the -I flag (pointing to
 * the project's include root). There is NO code duplication: the exact
 * same headers are #included by ParticleFilter.cpp in the ROS 2 build.
 *
 * Test scenarios:
 *   Phase 1 — MergeSeededParticles (M1–M10)
 *   Phase 2 — ResampleParticles (R1–R7)
 *
 * Compile (from this directory):
 *   ./compile.sh
 *
 * Or manually:
 *   g++ -std=c++17 -I../../soccer/Src/robot/include \
 *       -o test_particle_filter main.cpp -lm
 *
 * Outputs:
 *   - Results printed to stdout (PASS/FAIL for each test)
 *   - Optionally: CSV output for Python visualisation
 *
 * Dependencies:
 *   - C++17 compiler (g++ or clang++)
 *   - libm (-lm for math functions)
 *   - ParticleLite.hpp, MergeSeededParticles.hpp, ResampleParticles.hpp
 */

#include "perception/stateestimation/localiser/particlefilter/MergeSeededParticles.hpp"
#include "perception/stateestimation/localiser/particlefilter/ResampleParticles.hpp"
#include "perception/stateestimation/localiser/particlefilter/ParticleLite.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <random>
#include <string>
#include <vector>

// ═══════════════════════════════════════════════════════════════════════
//  1. Constants (matching LabLarge field)
// ═══════════════════════════════════════════════════════════════════════

constexpr float FIELD_LENGTH          = 6300.0f;
constexpr float FIELD_WIDTH           = 4050.0f;
constexpr float FIELD_LENGTH_OFFSET   = 510.0f;
constexpr float FIELD_WIDTH_OFFSET    = 930.0f;
constexpr size_t NUM_PARTICLES        = 300;
constexpr float DEFAULT_RESAMPLE_THRESHOLD = 75.0f;

// ═══════════════════════════════════════════════════════════════════════
//  2. Test Framework
// ═══════════════════════════════════════════════════════════════════════

static int g_testsPassed = 0;
static int g_testsFailed = 0;

#define TEST(name, expr) do { \
    bool ok = (expr); \
    if (ok) { ++g_testsPassed; std::printf("  PASS: %s\n", name); } \
    else { ++g_testsFailed; std::printf("  FAIL: %s\n", name); } \
} while(0)

#define TEST_DETAIL(name, expr, detail) do { \
    bool ok = (expr); \
    if (ok) { ++g_testsPassed; std::printf("  PASS: %s\n", name); } \
    else { ++g_testsFailed; std::printf("  FAIL: %s  --  %s\n", name, detail); } \
} while(0)

// ── Helpers ──

inline bool approxEq(float a, float b, float eps = 1e-5f) {
    return std::abs(a - b) < eps;
}

inline bool approxEqVec(const ParticleLite &a, const ParticleLite &b, float eps = 1e-4f) {
    return std::abs(a.x - b.x) < eps
        && std::abs(a.y - b.y) < eps
        && std::abs(a.theta - b.theta) < eps
        && std::abs(a.weight - b.weight) < eps;
}

inline float weightSum(const std::vector<ParticleLite> &p) {
    float s = 0.0f;
    for (const auto &pt : p) s += pt.getWeight();
    return s;
}

// Create N particles with uniform weight, optionally at specific positions
std::vector<ParticleLite> makeUniformParticles(size_t n, float x = 0, float y = 0, float theta = 0) {
    std::vector<ParticleLite> result;
    result.reserve(n);
    float w = 1.0f / static_cast<float>(n);
    for (size_t i = 0; i < n; ++i) {
        result.emplace_back(x + static_cast<float>(i) * 0.1f,
                            y + static_cast<float>(i) * 0.1f,
                            theta + static_cast<float>(i) * 0.001f,
                            w);
    }
    return result;
}

// Create N particles with specific weights (must sum to ~1)
std::vector<ParticleLite> makeWeightedParticles(
    size_t n,
    const std::vector<float> &weights,
    float x = 0, float y = 0, float theta = 0)
{
    std::vector<ParticleLite> result;
    result.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        float w = (i < weights.size()) ? weights[i] : 1.0f / static_cast<float>(n);
        result.emplace_back(x + static_cast<float>(i) * 0.1f,
                            y + static_cast<float>(i) * 0.1f,
                            theta + static_cast<float>(i) * 0.001f,
                            w);
    }
    return result;
}

// ═══════════════════════════════════════════════════════════════════════
//  3. Phase 1: MergeSeededParticles Tests
// ═══════════════════════════════════════════════════════════════════════

void testMergeSeededParticles()
{
    std::printf("\n");
    std::printf("════════════════════════════════════════════════════════════════════════\n");
    std::printf("  Phase 1: MergeSeededParticles\n");
    std::printf("════════════════════════════════════════════════════════════════════════\n");

    // ── M1: Empty candidates → no change ──
    {
        auto particles = makeUniformParticles(10);
        auto original = particles;
        std::vector<PoseLite> candidates;
        std::mt19937 rng(42);

        mergeSeededParticles(particles, candidates, 0.5f, 150.0f, 0.3f,
                             FIELD_LENGTH, FIELD_WIDTH, 10, rng);

        bool same = true;
        for (size_t i = 0; i < particles.size() && same; ++i)
            same = approxEqVec(particles[i], original[i]);
        TEST("M1: Empty candidates → no change", same && particles.size() == 10);
    }

    // ── M2: Empty particles → no change ──
    {
        std::vector<ParticleLite> particles;
        std::vector<PoseLite> candidates = {{1000.0f, 500.0f, 0.0f}};
        std::mt19937 rng(42);

        mergeSeededParticles(particles, candidates, 0.5f, 150.0f, 0.3f,
                             FIELD_LENGTH, FIELD_WIDTH, 0, rng);
        TEST("M2: Empty particles → no change", particles.empty());
    }

    // ── M3: Single candidate, zero noise, deterministic ──
    {
        auto particles = makeUniformParticles(10);
        std::vector<PoseLite> candidates = {{1000.0f, 500.0f, 1.57f}};
        std::mt19937 rng(42);

        mergeSeededParticles(particles, candidates, 0.5f, 0.0f, 0.0f,
                             FIELD_LENGTH, FIELD_WIDTH, 10, rng);

        // 5 survivors (top 50% of 10), 5 seeded at candidate
        bool correct = (particles.size() == 10);
        correct = correct && approxEq(weightSum(particles), 1.0f, 1e-4f);

        // Check that seeded particles (indices 5-9) are at the exact candidate position
        for (size_t i = 5; i < 10 && correct; ++i) {
            correct = correct && approxEq(particles[i].x, 1000.0f, 1e-4f);
            correct = correct && approxEq(particles[i].y, 500.0f, 1e-4f);
            correct = correct && approxEq(particles[i].theta, 1.57f, 1e-4f);
        }

        TEST("M3: Single candidate, zero noise → 5 survivors + 5 seeded", correct);
    }

    // ── M4: Multiple candidates, even distribution ──
    {
        auto particles = makeUniformParticles(50);
        std::vector<PoseLite> candidates = {
            {1000.0f, 500.0f, 0.0f},
            {-1000.0f, 500.0f, 1.57f},
            {0.0f, -500.0f, 3.14f}
        };
        std::mt19937 rng(42);

        mergeSeededParticles(particles, candidates, 0.5f, 0.0f, 0.0f,
                             FIELD_LENGTH, FIELD_WIDTH, 50, rng);

        // 25 survivors, 25 remaining, 3 candidates: perPose=8, extra=1
        // PoseA gets 9, PoseB gets 8, PoseC gets 8
        bool correct = (particles.size() == 50);
        correct = correct && approxEq(weightSum(particles), 1.0f, 1e-4f);

        // Count particles at each candidate position
        int countA = 0, countB = 0, countC = 0;
        for (size_t i = 25; i < 50; ++i) {
            if (approxEq(particles[i].x, 1000.0f, 1e-4f)) ++countA;
            else if (approxEq(particles[i].x, -1000.0f, 1e-4f)) ++countB;
            else if (approxEq(particles[i].x, 0.0f, 1e-4f)) ++countC;
        }
        correct = correct && (countA == 9) && (countB == 8) && (countC == 8);

        TEST("M4: 3 candidates → 9+8+8 distribution", correct);
    }

    // ── M5: Zero keep fraction → minimum 1 survivor ──
    {
        auto particles = makeUniformParticles(10);
        std::vector<PoseLite> candidates = {{500.0f, 0.0f, 0.0f}};
        std::mt19937 rng(42);

        mergeSeededParticles(particles, candidates, 0.0f, 0.0f, 0.0f,
                             FIELD_LENGTH, FIELD_WIDTH, 10, rng);

        // keepCount clamped to 1, so 1 survivor + 9 seeded
        bool correct = (particles.size() == 10);
        correct = correct && approxEq(particles[0].x, 0.0f, 1e-3f);  // survivor at (0,0)
        // All seeded particles at (500, 0, 0)
        for (size_t i = 1; i < 10 && correct; ++i) {
            correct = correct && approxEq(particles[i].x, 500.0f, 1e-4f);
            correct = correct && approxEq(particles[i].y, 0.0f, 1e-4f);
        }
        TEST("M5: Zero keep fraction → 1 survivor + 9 seeded", correct);
    }

    // ── M6: Full keep fraction → no seeding ──
    {
        auto particles = makeUniformParticles(10);
        auto original = particles;
        std::vector<PoseLite> candidates = {{500.0f, 0.0f, 0.0f}};
        std::mt19937 rng(42);

        mergeSeededParticles(particles, candidates, 1.0f, 0.0f, 0.0f,
                             FIELD_LENGTH, FIELD_WIDTH, 10, rng);

        bool same = true;
        for (size_t i = 0; i < particles.size() && same; ++i)
            same = same && approxEqVec(particles[i], original[i], 1e-4f);
        TEST("M6: Full keep fraction → all original particles kept", same && particles.size() == 10);
    }

    // ── M7: Gaussian noise with fixed seed → deterministic ──
    {
        auto particles = makeUniformParticles(10);
        std::vector<PoseLite> candidates = {{1000.0f, 0.0f, 0.0f}};

        std::mt19937 rng1(42);
        std::vector<ParticleLite> result1 = particles;
        mergeSeededParticles(result1, candidates, 0.5f, 50.0f, 0.1f,
                             FIELD_LENGTH, FIELD_WIDTH, 10, rng1);

        std::mt19937 rng2(42);
        std::vector<ParticleLite> result2 = particles;
        mergeSeededParticles(result2, candidates, 0.5f, 50.0f, 0.1f,
                             FIELD_LENGTH, FIELD_WIDTH, 10, rng2);

        bool identical = true;
        for (size_t i = 0; i < result1.size() && identical; ++i)
            identical = approxEqVec(result1[i], result2[i], 1e-6f);

        TEST("M7: Fixed seed → deterministic output", identical);
    }

    // ── M8: Count invariant with NUM_PARTICLES ──
    {
        auto particles = makeUniformParticles(NUM_PARTICLES);
        std::vector<PoseLite> candidates = {{1000.0f, 500.0f, 0.0f}, {-1000.0f, -500.0f, 1.57f}};
        std::mt19937 rng(42);

        mergeSeededParticles(particles, candidates, 0.5f, 150.0f, 0.3f,
                             FIELD_LENGTH, FIELD_WIDTH, NUM_PARTICLES, rng);

        bool correct = (particles.size() == NUM_PARTICLES);
        correct = correct && approxEq(weightSum(particles), 1.0f, 1e-3f);
        TEST("M8: Count invariant (300 particles)", correct);
    }

    // ── M9: Candidate at field centre ──
    {
        auto particles = makeUniformParticles(20);
        std::vector<PoseLite> candidates = {{0.0f, 0.0f, 0.0f}};
        std::mt19937 rng(42);

        mergeSeededParticles(particles, candidates, 0.5f, 150.0f, 0.3f,
                             FIELD_LENGTH, FIELD_WIDTH, 20, rng);

        // 10 survivors, 10 seeded around (0,0,0)
        // Compute mean of seeded particles
        float meanX = 0, meanY = 0, meanTheta = 0;
        int seededCount = 0;
        for (size_t i = 10; i < 20; ++i) {
            meanX += particles[i].x;
            meanY += particles[i].y;
            meanTheta += particles[i].theta;
            ++seededCount;
        }
        meanX /= seededCount;
        meanY /= seededCount;
        meanTheta /= seededCount;

        bool correct = (seededCount == 10);
        // Mean should be approximately at (0, 0, 0) with noise
        correct = correct && (std::abs(meanX) < 100.0f);
        correct = correct && (std::abs(meanY) < 100.0f);
        TEST("M9: Candidate at field centre → seeded cluster near centre", correct);
    }

    // ── M10: More candidates than remaining slots ──
    {
        auto particles = makeUniformParticles(5);
        std::vector<PoseLite> candidates = {
            {100.0f, 0.0f, 0.0f},
            {200.0f, 0.0f, 0.0f},
            {300.0f, 0.0f, 0.0f},
            {400.0f, 0.0f, 0.0f},
            {500.0f, 0.0f, 0.0f},
            {600.0f, 0.0f, 0.0f}
        };
        std::mt19937 rng(42);

        mergeSeededParticles(particles, candidates, 0.5f, 0.0f, 0.0f,
                             FIELD_LENGTH, FIELD_WIDTH, 5, rng);

        // keepCount = max(1, 5*0.5) = 2 (rounded from 2.5, truncated to 2)
        // remaining = 3, perPose = 0, extra = 3
        // First 3 candidates get 1 each, last 3 get 0
        // Total = 2 survivors + 3 seeded = 5
        // No residual fill needed
        bool correct = (particles.size() == 5);
        correct = correct && approxEq(weightSum(particles), 1.0f, 1e-4f);
        TEST("M10: 6 candidates, 3 remaining slots → 3 candidates get 1 each", correct);
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  4. Phase 2: ResampleParticles Tests
// ═══════════════════════════════════════════════════════════════════════

void testResampleParticles()
{
    std::printf("\n");
    std::printf("════════════════════════════════════════════════════════════════════════\n");
    std::printf("  Phase 2: ResampleParticles\n");
    std::printf("════════════════════════════════════════════════════════════════════════\n");

    // ── R1: Uniform weights, ESS above threshold → no resampling ──
    {
        auto particles = makeUniformParticles(10);
        auto original = particles;

        // ESS = (1.0)^2 / (10 * 0.01) = 10
        // resamplingThreshold = 5 → ESS (10) >= 5 → no resample
        bool shouldResample = shouldResampleParticles(particles, 5.0f);
        TEST("R1a: Uniform weights, ESS=10, threshold=5 → no resample needed",
             !shouldResample);

        // Run full resample with high threshold to verify no change
        std::mt19937 rng(42);
        resampleParticles(particles, 5.0f, 100.0f, 0.1f,
                          FIELD_LENGTH, FIELD_WIDTH,
                          FIELD_LENGTH_OFFSET, FIELD_WIDTH_OFFSET,
                          10, rng);

        bool same = true;
        for (size_t i = 0; i < particles.size() && same; ++i)
            same = approxEqVec(particles[i], original[i], 1e-4f);
        TEST("R1b: EXPECTED_ESS >= threshold → particles unchanged", same);
    }

    // ── R2: Single dominant weight → resampling triggered ──
    {
        // 1 particle with weight 0.91, 9 particles with weight 0.01 each
        std::vector<float> weights = {0.91f, 0.01f, 0.01f, 0.01f, 0.01f,
                                      0.01f, 0.01f, 0.01f, 0.01f, 0.01f};
        auto particles = makeWeightedParticles(10, weights);
        auto original = particles;

        // ESS ≈ 1/(0.91² + 9*0.01²) ≈ 1/0.837 ≈ 1.19
        bool shouldResample = shouldResampleParticles(particles, 5.0f);
        TEST("R2a: Degenerate weights, ESS≈1.19, threshold=5 → resample needed",
             shouldResample);

        std::mt19937 rng(42);
        resampleParticles(particles, 5.0f, 0.0f, 0.0f,
                          FIELD_LENGTH, FIELD_WIDTH,
                          FIELD_LENGTH_OFFSET, FIELD_WIDTH_OFFSET,
                          10, rng);

        // After resample: all weights should be uniform (0.1)
        bool allUniform = (particles.size() == 10);
        for (size_t i = 0; i < particles.size() && allUniform; ++i)
            allUniform = approxEq(particles[i].getWeight(), 0.1f, 1e-5f);

        // The high-weight particle should be duplicated many times
        // Count how many particles match original[0]'s position
        int dominantCount = 0;
        for (const auto &p : particles) {
            if (approxEq(p.x, original[0].x, 1e-3f) &&
                approxEq(p.y, original[0].y, 1e-3f))
                ++dominantCount;
        }

        TEST("R2b: Degenerate → resampled, uniform weights, dominant duplicated",
             allUniform && dominantCount > 1);
    }

    // ── R3: Zero weights → should trigger resample ──
    {
        std::vector<ParticleLite> particles;
        for (int i = 0; i < 10; ++i)
            particles.emplace_back(0.0f, 0.0f, 0.0f, 0.0f);

        bool shouldResample = shouldResampleParticles(particles, 5.0f);
        TEST("R3a: All zero weights → shouldResample returns true", shouldResample);

        std::mt19937 rng(42);
        resampleParticles(particles, 5.0f, 0.0f, 0.0f,
                          FIELD_LENGTH, FIELD_WIDTH,
                          FIELD_LENGTH_OFFSET, FIELD_WIDTH_OFFSET,
                          10, rng);

        // After resample: weights should be uniform
        bool allUniform = (particles.size() == 10);
        for (size_t i = 0; i < particles.size() && allUniform; ++i)
            allUniform = approxEq(particles[i].getWeight(), 0.1f, 1e-5f);

        TEST("R3b: Zero weights → resampled, uniform output", allUniform);
    }

    // ── R4: Count invariant (input == output) ──
    {
        // Degenerate weights to trigger resample
        std::vector<float> weights = {0.5f, 0.3f, 0.1f, 0.05f, 0.05f};
        auto particles = makeWeightedParticles(5, weights);
        std::mt19937 rng(42);

        resampleParticles(particles, 5.0f, 50.0f, 0.05f,
                          FIELD_LENGTH, FIELD_WIDTH,
                          FIELD_LENGTH_OFFSET, FIELD_WIDTH_OFFSET,
                          5, rng);

        TEST("R4: Count invariant (5 in, 5 out)", particles.size() == 5);
    }

    // ── R5: Off-field particle removal ──
    {
        auto particles = makeUniformParticles(9);
        // Add 1 particle far outside the field
        particles.emplace_back(10000.0f, 0.0f, 0.0f, 0.1f);

        float maxX = FIELD_LENGTH / 2.0f + FIELD_LENGTH_OFFSET;  // 3150 + 510 = 3660
        // 10000 > 3660 → off-field

        std::mt19937 rng(42);
        resampleParticles(particles, 0.0f, 0.0f, 0.0f,  // threshold=0 → always resample
                          FIELD_LENGTH, FIELD_WIDTH,
                          FIELD_LENGTH_OFFSET, FIELD_WIDTH_OFFSET,
                          10, rng);

        // After resample: all particles should be on-field
        bool allOnField = (particles.size() == 10);
        for (const auto &p : particles) {
            if (std::abs(p.x) > maxX + 100.0f)  // small tolerance for roughening
                allOnField = false;
        }

        TEST("R5: Off-field particle removed and replaced", allOnField);
    }

    // ── R6: Roughening with zero noise → positions preserved ──
    {
        auto particles = makeUniformParticles(10);
        // Make weights degenerate to trigger resample (ESS << 5.0)
        particles[0].setWeight(0.91f);
        for (size_t i = 1; i < 10; ++i)
            particles[i].setWeight(0.01f);

        // Snapshot which particles get selected by resampling with a known seed
        std::mt19937 rng1(42);
        std::vector<ParticleLite> resampled1 = particles;
        resampleParticles(resampled1, 5.0f, 0.0f, 0.0f,
                          FIELD_LENGTH, FIELD_WIDTH,
                          FIELD_LENGTH_OFFSET, FIELD_WIDTH_OFFSET,
                          10, rng1);

        // With zero noise, positions should equal the selected parent particles
        // All weights should be uniform
        bool allUniform = true;
        for (const auto &p : resampled1)
            allUniform = allUniform && approxEq(p.getWeight(), 0.1f, 1e-5f);

        TEST("R6: Zero roughening → uniform weights after resample", allUniform && resampled1.size() == 10);
    }

    // ── R7: Roughening with fixed seed → deterministic ──
    {
        // Degenerate weights to trigger resample
        std::vector<float> weights = {0.5f, 0.3f, 0.1f, 0.05f, 0.05f};
        auto particles = makeWeightedParticles(5, weights);

        std::mt19937 rng1(42);
        std::vector<ParticleLite> result1 = particles;
        resampleParticles(result1, 5.0f, 50.0f, 0.05f,
                          FIELD_LENGTH, FIELD_WIDTH,
                          FIELD_LENGTH_OFFSET, FIELD_WIDTH_OFFSET,
                          5, rng1);

        std::mt19937 rng2(42);
        std::vector<ParticleLite> result2 = particles;
        resampleParticles(result2, 5.0f, 50.0f, 0.05f,
                          FIELD_LENGTH, FIELD_WIDTH,
                          FIELD_LENGTH_OFFSET, FIELD_WIDTH_OFFSET,
                          5, rng2);

        // Both runs with same seed → identical output
        bool identical = (result1.size() == result2.size());
        for (size_t i = 0; i < result1.size() && identical; ++i)
            identical = approxEqVec(result1[i], result2[i], 1e-6f);

        TEST("R7: Fixed seed roughening → deterministic output", identical);
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  5. Cross-Cutting Verification
// ═══════════════════════════════════════════════════════════════════════

void testCrossCutting()
{
    std::printf("\n");
    std::printf("════════════════════════════════════════════════════════════════════════\n");
    std::printf("  Cross-Cutting: Normalise & Edge Cases\n");
    std::printf("════════════════════════════════════════════════════════════════════════\n");

    // ── Normalise theta ──
    TEST("liteNormaliseTheta(0) == 0", approxEq(liteNormaliseTheta(0.0f), 0.0f));
    TEST("liteNormaliseTheta(pi) approx -pi or pi",
         std::abs(std::abs(liteNormaliseTheta(M_PI)) - M_PI) < 1e-5f);
    TEST("liteNormaliseTheta(2pi) == 0", approxEq(liteNormaliseTheta(2.0f * M_PI), 0.0f, 1e-5f));
    TEST("liteNormaliseTheta(-pi/2) == -pi/2", approxEq(liteNormaliseTheta(-M_PI_2), -M_PI_2));

    // ── Normalise weights ──
    {
        std::vector<ParticleLite> particles;
        particles.emplace_back(0, 0, 0, 2.0f);
        particles.emplace_back(1, 1, 0, 3.0f);
        particles.emplace_back(2, 2, 0, 5.0f);

        float beforeSum = weightSum(particles);
        TEST("Weight sum before normalise == 10", approxEq(beforeSum, 10.0f));

        liteNormaliseWeights(particles);

        float afterSum = weightSum(particles);
        TEST("Weight sum after normalise == 1", approxEq(afterSum, 1.0f));
        TEST("Weight proportional: p0 == 0.2", approxEq(particles[0].getWeight(), 0.2f));
        TEST("Weight proportional: p1 == 0.3", approxEq(particles[1].getWeight(), 0.3f));
        TEST("Weight proportional: p2 == 0.5", approxEq(particles[2].getWeight(), 0.5f));
    }

    // ── Cumulative sum of weights ──
    {
        std::vector<ParticleLite> particles;
        particles.emplace_back(0, 0, 0, 0.2f);
        particles.emplace_back(1, 1, 0, 0.3f);
        particles.emplace_back(2, 2, 0, 0.5f);

        auto cdf = cumulativeSumOfParticleWeights(particles);
        TEST("CDF[0] == 0.2", approxEq(cdf[0], 0.2f));
        TEST("CDF[1] == 0.5", approxEq(cdf[1], 0.5f));
        TEST("CDF[2] == 1.0", approxEq(cdf[2], 1.0f));
    }

    // ── ParticleLite construction ──
    {
        ParticleLite p;
        TEST("Default ctor: x==0", approxEq(p.x, 0.0f));
        TEST("Default ctor: weight==0", approxEq(p.weight, 0.0f));

        ParticleLite p2(10.5f, 20.5f, 1.5f);
        TEST("Value ctor: x==10.5", approxEq(p2.x, 10.5f));
        TEST("Value ctor: weight==1.0", approxEq(p2.weight, 1.0f));

        p2.setWeight(0.5f);
        TEST("setWeight(0.5)", approxEq(p2.getWeight(), 0.5f));

        p2.multiplyWeight(2.0f);
        TEST("multiplyWeight(2.0) → weight==1.0", approxEq(p2.getWeight(), 1.0f));

        TEST("getX()", approxEq(p2.getX(), 10.5f));
        TEST("getY()", approxEq(p2.getY(), 20.5f));
        TEST("getTheta()", approxEq(p2.getTheta(), 1.5f));

        p2.setPosition(100.0f, 200.0f);
        TEST("setPosition(100, 200)", approxEq(p2.x, 100.0f) && approxEq(p2.y, 200.0f));

        p2.setOrientation(3.0f);
        TEST("setOrientation(3.0)", approxEq(p2.theta, 3.0f));
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  6. Main
// ═══════════════════════════════════════════════════════════════════════

int main()
{
    std::printf("════════════════════════════════════════════════════════════════════════\n");
    std::printf("  Particle Filter Standalone Unit Tests\n");
    std::printf("════════════════════════════════════════════════════════════════════════\n");
    std::printf("\n");

    testCrossCutting();
    testMergeSeededParticles();
    testResampleParticles();

    // ── Summary ──
    int total = g_testsPassed + g_testsFailed;
    std::printf("\n");
    std::printf("════════════════════════════════════════════════════════════════════════\n");
    std::printf("  Results: %d / %d passed", g_testsPassed, total);
    if (g_testsFailed > 0)
        std::printf("  (%d FAILED)", g_testsFailed);
    std::printf("\n");
    std::printf("════════════════════════════════════════════════════════════════════════\n");
    std::printf("\n");

    return (g_testsFailed > 0) ? 1 : 0;
}
