# MATRiX reference integration validation

Date: 2026-10-12. This record covers local reference intake and one static model
compilation, not a playable UE product, controller or field-motion qualification.

## Delivered

- `sim/tools/matrix_reference.py`: public staging function and CLI, reusing
  SourceIntake. Existing destinations are preserved. Post-intake validation or
  receipt failures remove only the newly created, resolved owned destination.
- SourceIntake now reads bounded GNU `L` long-name records, as used by the real
  release. PAX, long-link, sparse, nonregular, unsafe path and size-budget
  restrictions are not relaxed.
- Research index in `research/matrix/`; raw runtime/assets/evidence in the
  existing Git-ignored `third_party/matrix/` seam.

## Observed results

| Check | Result |
| --- | --- |
| Test-first staging path | RED missing module → GREEN directory/binary/mesh intake |
| License inventory | RED missing receipt field → GREEN explicit file inventory, redistribution remains unverified |
| Empty README / cleanup | RED IndexError and retained failed destination → GREEN ValueError, owned cleanup and successful retry |
| Official GNU long-path format | RED rejected LongLink → GREEN bounded name handling; real full-release extraction succeeded |
| Focused tests | 42 passed, one existing pytest `cache_dir` configuration warning |
| Ruff on four touched Python/test files | Passed |
| git diff whitespace check on tracked changes | Passed |
| Independent code review | APPROVE, 0 current issues; code/Interface review only, not runtime certification |
| Real intake | exit=0, 845 original files, 6,605,465,703 unpacked bytes |
| Archive SHA256 | Matches official `baaeaf01817e90baf20e3fd6f74ae2fe885ef69f7d862fa36f50222f69cfea81` |
| UeSim SHA256 | Matches prior analysis input `ed98e648c3298d0efec44170f48b0ad528c9d3010ad743a0f0dacf20d52f7bc1` |
| Selected XGB scene | Compiled in Windows MuJoCo 3.10.0; nbody=14, njnt=13, nq=19, nv=18, nu=12, ngeom=31, nmesh=17 |

No original UeSim, UE, controller or simulation stepping was executed. No
production package, Product preset, DDS contract or motion-arbitration route
was changed. The installed SDK was used only to compile the selected model.

## Existing broader-suite failure

The broader import/promotion selection returned **127 passed, 1 failed**.
Failure:
`tests/sim/test_robot_importer.py::test_promoted_package_contained_visual_projection_resolves_in_fresh_catalog_and_composer`.

Its temporary repository copies `sim/packages/worlds` but omits the referenced
`sim/compat/engine/worlds/thunderv4_stair_showcase.xml`; CatalogResolver rejects
the missing `stair_showcase` world asset. The same failure was reproduced with
the original HEAD SourceIntake substituted in memory in a separate Python
process. No worktree reset, checkout or file rollback was used. This is not
attributed to the GNU name handling, and unrelated fixtures were not changed.

## Remaining qualification

- Applicable binary/model/map/third-party grants before redistribution or
  production package promotion; the release contains no standard license file.
- Per-asset frames, units, materials, collision proxies and visual projection.
- Controller compatibility and same-session movement/stop acceptance.
- UE loading/rendering, real stream rates, recorder/qualification and Windows
  distribution. Neither static decompilation nor successful intake proves these.

## Native sensor feature slice — 2026-10-13

Delivered code and configuration are described in
[FEATURE_MIGRATION.md](FEATURE_MIGRATION.md). The new feature is an independently
implemented experimental Mid360 range-error model, inspired by explicit sensor
error handling observed in MATRiX; it is not recovered MATRiX LiDAR source.

The native message decision is also explicit: LingTu keeps its own
`lingtu.dds.LivoxFrame` IDL and LTU1 process record. MATRiX PointCloud2 is an
optional future input Adapter, not a replacement for the LingTu contract.

- `RangeNoiseConfig` is consumed from the compiled SensorPlan by the existing
  production endpoint factory. Missing configuration remains disabled.
- `Mid360RaycastExtractor.set_noise_enabled()` affects only subsequent samples.
  Collision hits and truth remain unchanged; native Livox wire fields are retained.
- Effective noise configuration is disclosed by SessionSensorRuntime and the
  strict sensor summary. Strength/seed mismatch is rejected, while an explicit
  enabled-state override is represented in the next sample's evidence.
- A separate lidar-only, no-controller headless example is provided. Ordinary
  Thunder navigation Rig/Session/Bundle declarations were not modified.

Observed checks:

| Check | Result |
| --- | --- |
| New noise tests, including Catalog → Bundle write/load → SensorRuntime | 22 passed |
| Seven directly affected sensor/factory/session/coordinator test files | 96 passed |
| Wrong evidence strength/seed and effective switch-off assertions | Passed in the final 22-test rerun |
| Ruff on nine owned source files and the new test file | Passed |
| Scoped mypy, eight owned source files, no-incremental / follow-imports=skip | Passed |
| Independent scoped implementation review | APPROVE, 0 issues; independently ran 22 noise tests and scoped Ruff |
| CLI resolve of the experimental SessionSpec | exit 0, sensor.plan.json retains enabled=true, stddev_m=0.02, seed=17 |

The initial `follow-imports=silent` type check found two mixed-value JSON dict
inference diagnostics in the new evidence code. An explicit JSON document type
fixed them. A subsequent run of mypy 2.1.0 failed internally; that run is not
reported as PASS. The isolated eight-file check above completed successfully;
it does not constitute a repository-wide dependency/type check.

The pytest runs retain the existing `cache_dir` configuration warning when the
cache plugin is disabled. No live UE, original UeSim, hardware motion, complete
recorder/replay or enabled-noise throughput qualification is claimed here.
