#!/usr/bin/env bash
#
# build_a.sh
#
# Wrapper script to configure and build the attTrackingError static library
# using CMake and the riscv32-elf toolchain. Prints each command before running it.

set -e

echo "=== attTrackingError Build Script ==="

BUILD_DIR="build"

# Helper function: echo the command, then execute it
run() {
  echo "  Command: $*"
  "$@"
}

echo
echo "Step 1: Creating (or verifying) build directory..."
run mkdir -p "${BUILD_DIR}"

echo
echo "Step 2: Entering build directory..."
run cd "${BUILD_DIR}"

echo
echo "Step 3: Running CMake (RISC-V toolchain)..."
run cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=../riscv32-toolchain.cmake ..

echo
echo "Step 4: Building the static library..."
run make

echo
echo "Build completed successfully!"
echo "The static library can be found at: $(pwd)/lib/libattTrackingError.a"
