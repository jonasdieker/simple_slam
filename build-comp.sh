#!/bin/bash

rm -r build/
echo "--> Removed build dir!"
cmake -S . -B build
cmake --build build -j4
# cd build
# GTEST_COLOR=1 ctest --output-on-failure