#!/bin/sh
# Builds and runs the PC-side unit tests. Needs a C++ compiler (g++ or clang++).
set -e
cd "$(dirname "$0")/.."
mkdir -p test/build
${CXX:-g++} -std=c++17 -Wall -Wextra -I firmware/teensy_ffb test/test_logic.cpp -o test/build/test_logic
./test/build/test_logic
