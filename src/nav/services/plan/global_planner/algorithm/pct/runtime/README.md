# Legacy PCT Runtime

PCT runtime is kept for benchmarks, parity checks, and historical GPMP work.
It is not registered by the product plugin catalog and is not the normal global
planner.

Legacy tool flow:

```text
bench/manual PCT tools
  -> nav.services.plan.global_planner.algorithm.pct.runtime.api
  -> src/kernels/planning/gpmp_trajectory_optimizer
```

Files:

| File | Role |
| --- | --- |
| `api.py` | Public API facade. |
| `common.py` | Shared constants, runtime selector, Rust artifact lookup. |
| `native_runtime.py` | Legacy native/GTSAM runtime path, parity-only. |
| `rust_gpmp.py` | Native-like optimizer result adapters. |
| `tomogram_planner.py` | Rust-process tomogram planner implementation. |
| `loader.py` | Runtime factory that chooses Rust or native. |
| `preview.py` | CLI/report helper for direct tomogram planning checks. |
| `build_legacy_native_x86_64.sh` | Parity-only native/GTSAM build helper. |

Main commands:

```bash
python -m nav.services.plan.global_planner.algorithm.pct.runtime.preview --tomogram <tomogram.pickle> --start 0 0 0 --goal 5 5 0 --json
python scripts/build/build_rust_kernels.py --target gpmp_trajectory_optimizer --release
```
