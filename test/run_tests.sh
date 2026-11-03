#!/bin/sh
# Builds and runs the PC-side unit tests. Needs a C++ compiler (g++ or clang++).
set -e
cd "$(dirname "$0")/.."
mkdir -p test/build
CXX=${CXX:-g++}
FLAGS="-std=c++17 -Wall -Wextra -I firmware/teensy_ffb"
$CXX $FLAGS test/test_logic.cpp -o test/build/test_logic
./test/build/test_logic
$CXX $FLAGS test/test_ffb.cpp firmware/teensy_ffb/FfbEngine.cpp -o test/build/test_ffb
./test/build/test_ffb
