#!/usr/bin/env bash
#
# Compile and run the standalone computeSeedPose() test.
#
# Usage:
#   ./compile.sh          # compile + run
#   ./compile.sh --no-run # compile only
#
# This compiles a single translation unit:
#   main.cpp
#     └─ #include "SeedGeometry.hpp"   ← inline header, no .cpp needed
#
# Dependencies: g++ (or clang++), libm
#

set -euo pipefail

# Path to the project include root (relative to this script's directory)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_INCLUDE="${SCRIPT_DIR}/../../soccer/Src/robot/include"

# Verify the header exists
HEADER="${PROJECT_INCLUDE}/perception/stateestimation/localiser/particlefilter/SeedGeometry.hpp"
if [ ! -f "$HEADER" ]; then
    echo "ERROR: SeedGeometry.hpp not found at:"
    echo "  $HEADER"
    echo "Make sure this script is in test-runs/seed-geometry-test/"
    exit 1
fi

echo "Found SeedGeometry.hpp at:"
echo "  $HEADER"
echo ""

# Detect C++ compiler
CXX="${CXX:-g++}"
echo "Using compiler: $CXX"

OUTPUT="test_seed_geometry"

echo "Compiling..."
$CXX -std=c++17 \
    -Wall -Wextra -Wpedantic \
    -O2 \
    -I "$PROJECT_INCLUDE" \
    -o "$OUTPUT" \
    main.cpp \
    -lm

echo "Compilation successful: ./${OUTPUT}"
echo ""

if [ "${1:-}" != "--no-run" ]; then
    echo "Running..."
    echo ""
    ./"$OUTPUT"
fi
