#!/usr/bin/env bash
# Compile libfoil stand-alone and run the golden test against the Python model.
set -e
cd "$(dirname "${BASH_SOURCE[0]}")/../ros_ws/src/hydrofoil_physics"
g++ -std=c++17 -O2 -Wall -Wextra -Iinclude -Itest src/foil_model.cpp test/test_foil_model.cpp -o /tmp/test_foil
/tmp/test_foil
