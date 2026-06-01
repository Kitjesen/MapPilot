# Navigation Core C++ Tests

C++ unit tests for nav_core algorithm library: local planner, path follower, parameter sensitivity, edge cases, and benchmarks (96 tests across 7 suites).

Run via CMake:
```bash
cd src/nav/core && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)
./test_local_planner_core
./test_benchmark
./test_path_follower_core
./test_path_edge_cases
./test_param_sensitivity
```
No pytest markers needed (C++ test suite).
