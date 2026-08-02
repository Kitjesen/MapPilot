# Pose Graph Optimizer GTSAM Coverage

Status: migration coverage note; product optimization policy lives in `NATIVE_RUNTIME.md`
Audience: SLAM/map-optimization migration maintainers
Replaced by: `NATIVE_RUNTIME.md` for product defaults; this file remains detailed migration evidence

This note defines what the Rust `pose_graph_opt` kernel currently replaces,
what the separate camera-LiDAR Rust optimizer replaces, and what still remains
as explicit legacy GTSAM comparison surface.

## Covered Surface

| Area | Current path | GTSAM capability previously used | Rust coverage |
| --- | --- | --- | --- |
| SLAM PGO | `src/localization/pgo` | SE3 `Pose3`, prior factor, between factor, caller-shaped batch smoothing path | Covered by `src/kernels/slam/pose_graph_opt` through C ABI |
| HBA | `src/localization/hba` | SE3 `Pose3`, prior factor, between factor, batch LM smoothing | Covered by `src/kernels/slam/pose_graph_opt` through C ABI |
| Camera-LiDAR calibration | `calibration/camera_lidar/direct_visual_lidar_calibration` | CT-ICP/CT-GICP residuals, Hessian-style two-pose linearization, CT-GICP two-pose LM, dynamic integrator smoothing path | Covered by `src/kernels/calibration/camera_lidar_optimizer` through C ABI |

The covered tangent order is `[rx, ry, rz, tx, ty, tz]`. Information matrices
are full symmetric 6x6 matrices passed as packed upper triangles through the C
ABI.

## Not Covered Yet

| Area | Example paths | Missing capability |
| --- | --- | --- |
| PCT / GPMP global planning | `src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/src/trajectory_optimization/gpmp_optimizer` | Rust process runtime plus trajectory-chain sparse LM/GN exist; remaining Linux native golden parity, direct native `GPMPOptimizer` class API parity, and non-chain sparse solver coverage |
| PCT legacy native parity artifacts | `src/nav/services/plan/global_planner/algorithm/pct/runtime/native/<arch>` | Default runtime is Rust `rust_process`; Linux/GTSAM native artifacts are now explicit parity baselines only and are no longer loaded from bundled `planner/lib/x86_64` |
| Vendored GTSAM tree | `src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/3rdparty/gtsam-4.1.1` | Third-party source bundle, not an active replacement target by itself |

These remaining surfaces need separate kernels or process-level adapters. They
should not be treated as covered by `pose_graph_opt`, because their factor types
are not plain SE3 prior/between pose graph factors.

The remaining runtime/tooling migration targets are also tracked in
`src/kernels/catalog.py`:

- `gpmp_trajectory_optimizer`: PCT/GPMP vector-state trajectory optimization
  plus the explicit legacy native parity/runtime boundary.
- `camera_lidar_calibration_optimizer`: offline direct visual LiDAR
  calibration with CT-ICP/GICP/Hessian-style factors.

## Comparison Commands

Rust-only benchmark:

```bash
cargo run --release --manifest-path src/kernels/slam/pose_graph_opt/Cargo.toml --example benchmark -- 16 64 256
```

The benchmark emits two caller-shaped cases for each pose count:

- `pgo_loop`: hard prior on pose 0, odometry chain, and one loop-closure factor.
- `hba_full_info`: anchored between-factor graph with denser skip factors and
  off-diagonal 6x6 information blocks.

Rust benchmark plus optional C++/GTSAM baseline comparison:

```bash
python tools/bench/pose_graph_opt_compare.py --poses 16 64 256 --baseline path/to/cpp_gtsam_baseline.json
```

Neutral fixture generation and Rust ABI execution:

```bash
python tools/bench/pose_graph_opt_fixture.py generate --case pgo_loop --poses 64 --output artifacts/pgo_loop_64.json
python tools/bench/pose_graph_opt_fixture.py generate --case hba_full_info --poses 64 --output artifacts/hba_full_info_64.json
python tools/bench/pose_graph_opt_fixture.py run --fixture artifacts/pgo_loop_64.json
```

Suite-level fixture comparison:

```bash
python tools/bench/pose_graph_opt_fixture.py \
  generate-suite \
  --poses 16 64 256 \
  --output-dir artifacts/pose_graph_opt_suite/fixtures
python tools/bench/pose_graph_opt_fixture.py \
  run-suite \
  --fixture-dir artifacts/pose_graph_opt_suite/fixtures \
  --json-out artifacts/pose_graph_opt_suite/rust.json
```

Optional C++/GTSAM fixture baseline on an Ubuntu/GTSAM machine:

```bash
python tools/bench/pose_graph_opt_fixture.py \
  run \
  --fixture artifacts/pgo_loop_64.json \
  --json-out artifacts/pgo_loop_64_rust.json
python tools/bench/pose_graph_opt_gtsam_baseline.py \
  --fixture artifacts/pgo_loop_64.json \
  --json-out artifacts/pgo_loop_64_gtsam.json
python tools/bench/pose_graph_opt_compare.py \
  --rust-result artifacts/pgo_loop_64_rust.json \
  --baseline artifacts/pgo_loop_64_gtsam.json \
  --fixture artifacts/pgo_loop_64.json \
  --enforce
```

Suite-level optional C++/GTSAM baseline on an Ubuntu/GTSAM machine:

```bash
python tools/bench/pose_graph_opt_gtsam_baseline.py \
  run-suite \
  --fixture-dir artifacts/pose_graph_opt_suite/fixtures \
  --json-out artifacts/pose_graph_opt_suite/gtsam.json
python tools/bench/pose_graph_opt_compare.py \
  compare-suite \
  --rust-result artifacts/pose_graph_opt_suite/rust.json \
  --baseline artifacts/pose_graph_opt_suite/gtsam.json \
  --fixture-dir artifacts/pose_graph_opt_suite/fixtures \
  --json-out artifacts/pose_graph_opt_suite/compare.json \
  --enforce
```

`pose_graph_opt_gtsam_baseline.py` reads the neutral fixture in Python, generates
a temporary C++17/GTSAM runner with hardcoded poses and factors, builds it with
CMake, and emits benchmark JSON compatible with `pose_graph_opt_compare.py`.
This keeps JSON parsing out of the C++ runner and avoids adding a JSON library
or GTSAM requirement to Windows/lightweight development.

GTSAM coverage audit:

```bash
python tools/validate/validate_pose_graph_opt_coverage.py
```

The audit fails if GTSAM returns to `src/localization/pgo` or `src/localization/hba`, if a
removed PCT legacy runtime binary reappears under
`src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/x86_64`, or if a new GTSAM
reference appears outside a classified bucket. Known remaining
dependencies are reported but do not fail the default check; use
`--fail-on-remaining` only when intentionally driving the repo toward zero
GTSAM references. The audit scans both text references and runtime artifact
file names such as `libgtsam.so*` / `libmetis-gtsam.so*`. Its JSON output also
includes a `capability_surfaces` matrix: PGO/HBA are marked
`covered_by_pose_graph_opt`, camera-LiDAR calibration is marked
`covered_by_camera_lidar_optimizer`, and PCT/GPMP remains the explicit
`remaining_dependency` surface. The audit also emits
`remaining_dependency_surface_summary`, which groups remaining runtime/build
references by surface and includes representative paths plus hit counts. This
makes the next migration targets explicit instead of mixing PCT, calibration,
and support-script references into one undifferentiated list. The same JSON also
includes `dependency_subsurfaces` and `remaining_dependency_subsurface_summary`
so the remaining PCT/GPMP work is split into migration-sized chunks:

- PCT/GPMP: `pct_runtime_packaging` (P0), `pct_gpmp_optimizer_core` (P1),
  and `pct_build_packaging` (P2).

The audit deliberately ignores the local `gtsam_ext` namespace used by legacy
frame helpers; it is a historical name, not an external GTSAM dependency.
Real dependency hits still include `gtsam::`, `<gtsam/...>`, `libgtsam*`, and
`find_package(GTSAM ...)`.

The priority labels are migration order hints, not runtime severity. PCT/GPMP
is still the runtime deployment blocker; camera-LiDAR calibration is no longer
a remaining GTSAM dependency surface in the default build.

The first PCT runtime boundary is now explicit in
`src/nav/services/plan/global_planner/algorithm/pct/runtime/api.py`:
`load_pct_planner_runtime()` is the single planner-runtime seam, selected by
`LINGTU_PCT_PLANNER_RUNTIME`. The default runtime is now `rust_process`, which
is the Windows/lightweight path and the normal LingTu runtime path. The `native`
runtime preserves the Linux ELF `.so` behavior only for explicit legacy
GTSAM/Rust parity baselines. `rust_process` loads the tomogram in Python, runs
a portable A* seed path, and calls the same Rust JSON optimizer either through
the in-process C ABI dynamic library or, when that library is unavailable,
through the `gpmp_optimize` executable over stdin/stdout. `LINGTU_GPMP_OPTIMIZER_CALL`
can force `ffi` or `process`; the default `auto` mode prefers FFI and falls
back to the process path.
The old PCT `planner/build.sh`, `planner/build_thirdparty.sh`, and
`pct_runtime/build_legacy_native_x86_64.sh` entry points are now guarded by
`LINGTU_PCT_BUILD_LEGACY_GTSAM_NATIVE=1`, so ordinary builds do not silently
pull the Linux/GTSAM native stack back in. The old bundled
`src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/planner/lib/x86_64` runtime binaries have been
removed; any regenerated legacy native parity artifacts must be written under
`src/nav/services/plan/global_planner/algorithm/pct/runtime/native/<arch>` or supplied through
`LINGTU_PCT_LIB_DIR`.

PCT/GPMP also has a static migration contract:

```bash
python tools/validate/validate_pct_gpmp_migration_contract.py
```

This contract is Windows-safe because it does not import the native `.so`.
It proves from source that PCT/GPMP still requires Vector4/Vector6 nonlinear
graphs, custom GP prior/obstacle/heading-rate factors, LM-style batch solving,
and Python wrapper parity for `get_result_matrix()`, `get_layers()`,
`get_heights()`, `get_opt_init_value()`, and `get_opt_init_layer()`. That is
the minimum surface a Rust or process runtime must cover before replacing the
legacy C++/GTSAM optimizer.

The first Rust GPMP kernel slice now lives at
`src/kernels/planning/gpmp_trajectory_optimizer`. It covers the portable,
GTSAM-free part that can be verified on Windows:

- WNOJ Vector6 state order `[x, vx, ax, y, vy, ay]`.
- WNOA Vector4 active state order `[x, vx, y, vy]`.
- WNOJ/WNOA `Q`, `Q^-1`, and `Phi` process matrices.
- GP prior residuals `Phi * x_i - x_{i+1}`.
- GP interpolation `Lambda * x_i + Psi * x_{i+1}`.
- WNOJ heading-rate residual and Jacobian, including interpolated chain-rule
  Jacobians.
- DenseElevationMap safe layer/height selection and bilinear obstacle cost
  queries.
- WNOJ/WNOA obstacle residuals and analytic Jacobians, including interpolated
  chain-rule Jacobians.
- WNOJ/WNOA dense Levenberg-Marquardt / Gauss-Newton batch optimizers as the
  compatibility fallback.
- WNOJ/WNOA block-tridiagonal sparse Levenberg-Marquardt / Gauss-Newton solves for
  trajectory-chain graphs, selected by `linear_solver=auto` or
  `linear_solver=block_tridiagonal`.
- WNOJ/WNOA faer sparse Cholesky Levenberg-Marquardt / Gauss-Newton solves for
  non-chain graphs, selected by `linear_solver=sparse` or automatically for
  larger non-chain systems. The current assembly path still linearizes through
  the existing factor accumulators; this is a real sparse solve path, not a full
  GTSAM symbolic factor-graph engine.
- The nonlinear optimizer is selected with
  `LINGTU_PCT_RUST_NONLINEAR_OPTIMIZER=levenberg_marquardt` or
  `LINGTU_PCT_RUST_NONLINEAR_OPTIMIZER=gauss_newton`. The default remains
  Levenberg-Marquardt for compatibility.
- A C ABI JSON entrypoint and a `gpmp_optimize` process entrypoint used by the
  `rust_process` PCT runtime runtime.
- `rust_process` can call the Rust optimizer in-process through
  `lingtu_gpmp_optimizer_optimize_json`, avoiding process startup overhead when
  the dynamic library is present.
- `gpmp_optimize` returns both optimizer node states and native-like expanded
  trajectory states/layers/heights/costs. The Python runtime prefers the
  expanded trajectory for returned paths and native-style accessors.
- `LINGTU_PCT_RUST_INTERPOLATION_STEPS` controls that expanded trajectory
  density. The default runtime request is `8`, so the Rust path no longer
  exposes only sparse optimizer nodes to callers.
- `rust_process` planner accessors matching the native optimizer result surface:
  `get_result_matrix()`, `get_layers()`, `get_heights()`,
  `get_opt_init_value()`, and `get_opt_init_layer()`.
- `rust_process` native-like optimizer views:
  `get_trajectory_optimizer_wnoj()` returns a WNOJ view with
  `get_ceilings()` and `get_heading_rate()`, while
  `get_trajectory_optimizer()` returns a WNOA view with `gp_prior_test()`.

It does not yet cover a general sparse LM/GN solver for arbitrary non-chain
factor graphs, Linux native golden parity against the existing GTSAM optimizer,
or removal of the Linux native pybind `GPMPOptimizer` /
`GPMPOptimizerWnoa` implementation. Those remain part of the PCT/GPMP
remaining dependency surface and are still required before
`claims.full_gtsam_replacement` can become true.

The Windows-safe runtime smoke for the Rust process runtime is:

```bash
python tools/bench/pct_rust_process_smoke.py \
  --build \
  --compare \
  --enforce \
  --json-out artifacts/pct_rust_process_actual.json
python tools/validate/validate_kernel_migration_status.py \
  --pct-rust-process-actual-json artifacts/pct_rust_process_actual.json \
  --require-pct-rust-process-golden \
  --json
```

This builds `gpmp_optimize`, runs a synthetic PCT preview through
`LINGTU_PCT_PLANNER_RUNTIME=rust_process`, verifies that the runtime invoked
the Rust optimizer process, and checks block-tridiagonal solver diagnostics
and LM/GN nonlinear optimizer diagnostics against
`rust_process_synthetic_smoke.json`. The same golden also checks the
native wrapper optimizer accessor contract: result matrix, layers, heights,
ceilings, heading-rate, initial value matrix, and initial layer vector must have
the shapes expected by `planner_wrapper.py`. It proves the portable runtime path
is wired; it does not replace the Linux native `building2_9` parity gate.

The Windows-safe dense-vs-block-tridiagonal optimizer performance gate is:

```bash
python tools/bench/pct_gpmp_optimizer_compare.py \
  --build \
  --modes wnoj wnoa \
  --state-counts 96 \
  --repeats 3 \
  --json-out artifacts/pct_gpmp_optimizer_compare.json \
  --enforce
```

This feeds the same synthetic WNOJ/WNOA chain requests to the Rust
`gpmp_optimize` process with `linear_solver=dense` and
`linear_solver=block_tridiagonal`. It requires equal final cost within
tolerance, zero sparse fallback, and a minimum block-tridiagonal speedup. It is
a trajectory-chain performance gate; the separate sparse Cholesky path is
covered by Rust non-chain solver tests and runtime solver-report validation.

For the normal Windows/lightweight PCT migration gate, run the combined
runtime acceptance and feed that single artifact into the total migration
status:

```bash
python tools/bench/pct_rust_runtime_acceptance.py \
  --build \
  --json-out artifacts/pct_rust_runtime_acceptance.json \
  --enforce
python tools/validate/validate_kernel_migration_status.py \
  --pct-rust-runtime-acceptance-json artifacts/pct_rust_runtime_acceptance.json \
  --require-pct-rust-process-golden \
  --require-pct-rust-process-gn-golden \
  --require-pct-gpmp-optimizer-compare \
  --json
```

The acceptance artifact includes the LM rust_process preview, GN rust_process
preview, and dense-vs-block-tridiagonal optimizer comparison paths. The status
validator derives those three paths from the artifact and re-checks the
individual gates. This is the runtime-replacement and performance gate. It
still does not claim Linux native parity.

The Linux same-input native-vs-rust_process parity gate is:

```bash
python tools/bench/pct_native_rust_parity.py \
  --build \
  --tomogram src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/rsc/tomogram/building2_9.pickle \
  --start 2 3 0 \
  --goal 18 11 0 \
  --json-out artifacts/pct_native_rust_parity.json \
  --enforce
```

This runs `nav.services.plan.global_planner.algorithm.pct.runtime.preview` twice in isolated subprocesses: once with
`LINGTU_PCT_PLANNER_RUNTIME=native`, once with
`LINGTU_PCT_PLANNER_RUNTIME=rust_process`. It checks same-input identity,
path length/distance, start/goal error, first/last point, and arclength samples.
It is expected to require a Linux host with the native PCT `.so` runtime.

The GPMP math kernel also emits a solver-neutral JSON fixture for C++/legacy
comparison:

```bash
cargo run --manifest-path src/kernels/planning/gpmp_trajectory_optimizer/Cargo.toml \
  --example math_fixture > artifacts/pct_gpmp_math_rust.json
python tools/bench/pct_gpmp_math_cpp_baseline.py \
  --json-out artifacts/pct_gpmp_math_cpp.json \
  --json
python tools/bench/pct_gpmp_math_compare.py \
  --rust-result artifacts/pct_gpmp_math_rust.json \
  --baseline artifacts/pct_gpmp_math_cpp.json \
  --json-out artifacts/pct_gpmp_math_compare.json \
  --json \
  --enforce
```

The fixture currently covers WNOJ positive heading-rate limit, WNOJ negative
heading-rate limit, WNOJ inside-limit zero branch, and WNOA nominal math. Each
case exports process matrices, inverse process matrices, Phi matrices,
Lambda/Psi interpolation matrices, GP prior residuals and Jacobians,
interpolation Jacobians, path-point conversion, and a short interpolated
trajectory. WNOJ cases additionally export heading-rate residuals and
Jacobians, including interpolated heading-rate chain-rule Jacobians.

This is a pure-math parity gate. Passing it proves the Rust formulas match a
baseline JSON for that slice only. Obstacle, dense optimizer, and
block-tridiagonal sparse optimizer behavior are covered by Rust unit tests, not
by this C++ header baseline. The migration still does not prove Linux native
runtime trajectory parity, arbitrary-factor sparse convergence, or direct
native class wrapper output parity.

`pct_gpmp_math_cpp_baseline.py` is optional because it needs CMake, a C++17
compiler, and Eigen3. It does not require GTSAM. The generated runner includes
the existing legacy C++ `wnoj.hpp` / `wnoa.hpp` model headers for process-model
and interpolation math. The heading-rate branch is emitted as a standalone C++
formula because the production factor class is tied to GTSAM's
`NoiseModelFactor1` base class. On Windows, if Eigen is installed but CMake
cannot find `Eigen3Config.cmake`, pass the header root explicitly:
`--eigen-include C:/path/to/eigen3`.

The current PCT native C++ smoke comparison gate is:

```bash
python -m nav.services.plan.global_planner.algorithm.pct.runtime.preview \
  --tomogram src/nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/rsc/tomogram/building2_9.pickle \
  --start 2 3 0 \
  --goal 18 11 0 \
  --sample-count 9 \
  --json > artifacts/pct_building2_9_actual.json
python tools/bench/pct_preview_compare.py \
  --golden src/nav/tests/planning_backends/fixtures/pct_preview/building2_9_smoke.json \
  --actual-json artifacts/pct_building2_9_actual.json \
  --enforce
```

The first command still needs the native Linux PCT `.so` runtime. Its JSON uses
`lingtu.pct.preview.actual.v2` and includes aggregate metrics, index samples,
arclength-fraction samples, runtime metadata, stable wrapper diagnostics, and
tomogram input provenance (`sha256`, file size, `data_shape`, and `data_dtype`).
The second command is Windows-safe and validates the comparison contract without
loading native modules.

Camera-LiDAR calibration has a separate static contract:

```bash
python tools/validate/validate_camera_lidar_calibration_migration_contract.py
cargo test --manifest-path src/kernels/calibration/camera_lidar_optimizer/Cargo.toml
python tools/bench/camera_lidar_optimizer_abi_smoke.py
```

The Rust kernel at `src/kernels/calibration/camera_lidar_optimizer` covers the
fixed-correspondence CT-ICP point-to-plane and CT-GICP Mahalanobis
covariance-weighted slices: SE3 exp/log, continuous-time interpolation between
scan begin/end poses, endpoint Jacobians, singular fused-covariance rejection,
and two-pose Hessian/rhs assembly. The crate also exports C ABI entry points
for CT-ICP/CT-GICP linearization, Rust-owned CT-GICP nearest-neighbor
correspondence construction, and CT-GICP two-pose LM optimization with the same
prior/between constraints used by `dynamic_point_cloud_integrator`. The dynamic
CT-GICP scan optimizer entry point also owns the outer correspondence rebuild
and two-pose optimization loop so the C++ integrator no longer orchestrates
that algorithm sequence.

The `dynamic_point_cloud_integrator` runtime now builds CT-GICP
source/target point arrays and calls
`lingtu_camera_lidar_optimizer_optimize_ct_gicp_dynamic_two_pose` directly.
Rust rebuilds correspondences across optimization rounds and returns the scan
begin/end poses without going through any GTSAM factor wrapper. In the default Rust-enabled build,
`LINGTU_CAMERA_LIDAR_REQUIRE_RUST_OPTIMIZER=ON`, so dynamic integration fails
instead of silently falling back to the old GTSAM LM path on ABI,
correspondence, or solve failure. The dynamic integrator has no GTSAM LM
fallback source path in the Rust-required build. The legacy CT-ICP/CT-GICP
GTSAM factor headers, their impl headers, the Rust-to-GTSAM helper, and the old
GTSAM outlier support test source have been removed from the repository. The
visual calibration Nelder-Mead perturbation path no longer calls
`gtsam::Pose3::Expmap`; it uses
Sophus SE3 exp with an explicit reorder to preserve the historical
`[rx, ry, rz, tx, ty, tz]` optimization vector order. The dynamic point cloud
integrator header also no longer exposes `gtsam::Pose3`; it stores scan
begin/end poses as `Eigen::Isometry3d`. The pure Rust ABI header is
GTSAM-free, and the default `direct_visual_lidar_calibration` shared library
source audit rejects GTSAM headers, `gtsam::` APIs, and legacy CT factor
includes.

Unified migration status gate:

```bash
python tools/validate/validate_kernel_migration_status.py --json
```

The default status gate is Windows-safe. It aggregates the PGO/HBA coverage
audit, the PCT/GPMP static migration contract, and the classified remaining
GTSAM surfaces. It does not require a Linux PCT actual JSON by default. For a
release or claim gate, provide the actual JSON and require the golden compare:

```bash
python tools/validate/validate_kernel_migration_status.py \
  --pct-actual-json artifacts/pct_building2_9_actual.json \
  --require-pct-golden \
  --pct-rust-runtime-acceptance-json artifacts/pct_rust_runtime_acceptance.json \
  --require-pct-rust-process-golden \
  --require-pct-rust-process-gn-golden \
  --pct-gpmp-math-rust-json artifacts/pct_gpmp_math_rust.json \
  --pct-gpmp-math-baseline-json artifacts/pct_gpmp_math_cpp.json \
  --require-pct-gpmp-math \
  --require-pct-gpmp-optimizer-compare \
  --pct-native-rust-parity-json artifacts/pct_native_rust_parity.json \
  --require-pct-native-rust-parity \
  --json
```

`ok=true` means the bounded migration state is internally consistent:
PGO/HBA remain covered by `pose_graph_opt`, PCT/GPMP remains explicitly tracked
as not covered by that kernel, optional PCT/PCT-GPMP comparison artifacts pass
when requested, and no unclassified GTSAM usage has appeared.
`claims.pct_gpmp_math_kernel_parity=true` only means the pure math fixture
matched its baseline. `claims.pct_gpmp_obstacle_optimizer_kernel=true` means
the Rust GPMP kernel now contains the DenseElevationMap obstacle semantics,
WNOJ/WNOA dense fallback optimizers, WNOJ/WNOA block-tridiagonal sparse LM/GN
optimizers for trajectory-chain graphs, and faer sparse Cholesky LM/GN solves
for non-chain graphs, plus native-like `rust_process` optimizer result
accessors and expanded trajectory output.
`claims.pct_gpmp_rust_process_runtime=true` means `LINGTU_PCT_PLANNER_RUNTIME=rust_process`
can select a real Rust optimizer from the PCT runtime, use either the FFI or
process call path, consume expanded optimizer output, and report
`last_optimizer_input_states`, `last_optimizer_output_states`,
`last_optimizer_trajectory_expanded`, `last_optimizer_interpolation_steps`, and
`last_optimizer_call_mode` in preview diagnostics. It is still not a
full GTSAM-level replacement claim: Linux golden parity, direct native class
parity, and complete PCT legacy source removal are still open.
`claims.pct_gpmp_optimizer_performance=true` means the Rust process optimizer
passed the Windows-safe dense-vs-block-tridiagonal performance gate for
chain-structured WNOJ/WNOA requests. It does not prove Linux native PCT parity
or full GTSAM-level arbitrary sparse factor-graph performance.
`claims.pct_native_rust_parity=true` means a Linux same-input native-vs-rust_process
PCT preview artifact passed its parity checks. Without that artifact, PCT
runtime replacement remains unproven even if the Windows-safe Rust process
smoke and optimizer performance gates pass.
The full ROS workspace build script defaults to skipping the vendored
`gtsam` package through `LINGTU_BUILD_VENDORED_GTSAM=0`; legacy vendored GTSAM
builds must be requested explicitly. This keeps normal SLAM/PCT resource
packaging from pulling the old PCT native dependency back into lightweight
builds.
`claims.camera_lidar_dynamic_integrator_rust_runtime=true` means the
camera-LiDAR dynamic point cloud integrator has a direct Rust CT-GICP optimizer
path through `LINGTU_CAMERA_LIDAR_USE_RUST_OPTIMIZER`. The C++ runtime delegates
the CT-GICP correspondence rebuild plus two-pose optimization loop to
`lingtu_camera_lidar_optimizer_optimize_ct_gicp_dynamic_two_pose`, and the
default Rust-enabled CMake mode is non-silent-fallback through
`LINGTU_CAMERA_LIDAR_REQUIRE_RUST_OPTIMIZER`. The static camera-LiDAR contract
also audits the CMake source list for the default
`direct_visual_lidar_calibration` shared library and verifies that those
compiled sources do not include GTSAM headers, `gtsam::` APIs, or legacy CT
factor headers. `claims.camera_lidar_legacy_gtsam_factors_removed=true` means
the legacy CT factor headers, impl headers, Rust-to-GTSAM helper, and old
outlier support test source are absent from the repository.
`claims.full_gtsam_replacement=false` remains expected until those remaining
PCT/GPMP surfaces no longer depend on GTSAM.

Caller-level fixture exports:

- `SimplePGO::exportPoseGraphFixture(path)` writes the current PGO graph as a
  `pgo_loop` fixture. It includes current global key poses, prior factors,
  odometry/loop between factors, and any pending loop-cache pairs that the next
  smoothing pass would consume. Its metadata records `offsetR` / `offsetT` for
  map-frame write-back checks.
- `HBA::exportPoseGraphFixture(path)` writes the current HBA graph as an
  `hba_full_info` fixture. It exports `priors: []` and between-only full
  information factors collected through the same `getAllFactors()` path used by
  optimization.
- These export hooks are JSON contract-testable without ROS, GTSAM, or native
  SLAM compilation. They are intended to create real caller-shaped fixtures for
  Rust-vs-legacy comparison, not to run optimization themselves.

Runtime export switch:

- PGO and HBA nodes accept optional `fixture_export_path` as either a ROS
  parameter or YAML field. The default is empty, so normal runtime behavior is
  unchanged.
- When configured, PGO exports after each accepted key-pose update and HBA
  exports after each HBA optimization iteration. Parent directories are created
  automatically.
- The C++/GTSAM baseline runner mirrors Rust anchoring by adding an implicit
  fixed-pose anchor for `fixed_pose_index` / `auto_anchor` during optimization.
  That anchor is not counted in reported `factors`, `initial_cost`,
  `final_cost`, or residual statistics.

Baseline JSON should use the same shape as the Rust benchmark output:

```json
{
  "benchmark": "legacy_cpp_gtsam_pose_graph",
  "unit": "milliseconds",
  "cases": [
    {
      "case": "pgo_loop",
      "poses": 64,
      "factors": 65,
      "sparse_expected": true,
      "initial_cost": 1.0,
  "final_cost": 0.0,
  "final_residual_rms": 0.0,
  "final_residual_max": 0.0,
      "iterations": 5,
      "accepted_steps": 5,
      "rejected_steps": 0,
      "elapsed_ms": 1.25
    }
  ]
}
```

Rust fixture-run results also include `fixture_hash`, `fixture_identity`,
`final_residual_rms`, and `final_residual_max`, so the comparison gate can prove
that Rust and C++/GTSAM solved the same fixture and reached comparable residuals.
Suite-run results additionally include a top-level `fixtures` list and
case-level `fixture_path`; `compare-suite` checks the hash for each case against
the fixture directory and applies per-fixture `baseline_tolerances`.

`pose_graph_opt_compare.py` emits a `summary` object when a C++/GTSAM baseline is
provided. The summary records matched cases, missing Rust/baseline cases,
fixture hashes, status/convergence/write-count/factor-count checks, thresholds,
skipped checks, failed checks, and a top-level `verdict`. The default thresholds
come from `tools/bench/pose_graph_opt_compare.py`; fixture `baseline_tolerances`
can override them, and CLI flags can override both. Use `--enforce` when the
comparison should fail the command on a non-pass verdict.

Comparison results must include complete `optimized_poses` on both Rust and
C++/GTSAM sides. Missing pose outputs are treated as failed checks, not skipped
RMSE checks, because pose equality is the strongest evidence that both solvers
optimized the same graph to the same result.

Fixture JSON uses a solver-neutral schema:

- `schema`: `lingtu.pose_graph_opt.fixture.v1`
- `schema_version`: `1`
- `pose_format`: `t_xyz_q_wxyz`
- `poses`: initial SE3 poses as `{t_xyz, q_wxyz}`
- `priors`: optional prior factors with `index`, `pose`, and
  `information_upper`
- `betweens`: between factors with `from_index`, `to_index`, `pose_from_to`, and
  `information_upper`
- `config`: ABI solver controls, including `fixed_pose_index`
- `tangent_order`: `[rx, ry, rz, tx, ty, tz]`
- `information_upper_order`: `row_major_upper_6x6`
- `expected`: non-timing pass thresholds for status, final cost, iterations, and
  accepted steps
- `baseline_tolerances`: optional C++/GTSAM comparison thresholds

PGO caller-level exports should also record `offsetR/offsetT` after solve when
testing map-frame write-back behavior. HBA has no equivalent offset concept.

## Completion Bar

For the PGO/HBA replacement to stay accepted:

- `rg -n "gtsam|GTSAM" src/localization/pgo src/localization/hba` has no hits.
- `python tools/validate/validate_pose_graph_opt_coverage.py` passes and keeps
  remaining GTSAM use classified outside the migrated PGO/HBA surface.
- Rust unit tests cover SE3 exp/log, prior/between Jacobians, sparse solves,
  singular/gauge errors, and rotated loop closure.
- Python ABI tests cover library layout, Windows artifact names, rotated SE3
  solve, static library generation, and invalid input handling.
- Benchmark output exists for PGO-shaped loop graphs and HBA-shaped full
  information graphs, and can be compared against an optional C++/GTSAM
  baseline without adding GTSAM as a required dependency.
