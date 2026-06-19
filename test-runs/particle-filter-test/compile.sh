#!/usr/bin/env bash
#
# Compile and run the standalone particle filter unit tests.
#
# Tests:
#   - MergeSeededParticles.hpp  (Phase 1)
#   - ResampleParticles.hpp     (Phase 2)
#
# These headers have zero ROS/Eigen dependencies and compile with
# plain g++ -std=c++17.
#
# Usage:
#   ./compile.sh          # compile + run
#   ./compile.sh --no-run # compile only
#
# Dependencies: g++ (or clang++), libm
#

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_INCLUDE="${SCRIPT_DIR}/../../soccer/Src/robot/include"
PARTICLE_FILTER_INCLUDE="${PROJECT_INCLUDE}/perception/stateestimation/localiser/particlefilter"

echo "========================================"
echo " Particle Filter Standalone Unit Tests"
echo "========================================"
echo ""

# Verify headers exist
for header in ParticleLite.hpp MergeSeededParticles.hpp ResampleParticles.hpp; do
    HEADER_PATH="${PARTICLE_FILTER_INCLUDE}/${header}"
    if [ ! -f "$HEADER_PATH" ]; then
        echo "ERROR: ${header} not found at:"
        echo "  ${HEADER_PATH}"
        exit 1
    fi
    echo "  Found: ${header}"
done

echo ""
echo "All headers present in:"
echo "  ${PARTICLE_FILTER_INCLUDE}"
echo ""

# Detect C++ compiler
CXX="${CXX:-g++}"
echo "Compiler: ${CXX}"

OUTPUT="test_particle_filter"

echo ""
echo "Compiling..."
${CXX} -std=c++17 \
    -Wall -Wextra -Wpedantic \
    -O2 \
    -I "$PROJECT_INCLUDE" \
    -o "$OUTPUT" \
    "${SCRIPT_DIR}/main.cpp" \
    -lm

echo "Compilation successful: ./${OUTPUT}"
echo ""

if [ "${1:-}" != "--no-run" ]; then
    echo "Running..."
    echo ""
    ./"$OUTPUT"
fi
