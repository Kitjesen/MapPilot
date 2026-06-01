# Base Autonomy Local Planner Tests

C++ unit tests for local planner algorithms: path scoring, parameter sensitivity, and performance benchmarks.

Run via CMake:
```bash
cd src/nav/core && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)
./test_local_planner_core
./test_benchmark
```

No pytest markers needed (C++ test suite).
