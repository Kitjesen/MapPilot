# Runnable PCT Planner

This directory is the runnable isolation layer for the original
`src/global_planning/pct_planner` tree. It does not fork the planner algorithm,
and it must not replace the original native library tree.

## Directory Purpose

The original PCT wrapper imports native extensions with:

```python
from lib import a_star, ele_planner, traj_opt
```

That import works on S100P/aarch64 when extension modules live directly under
`pct_planner/planner/lib`. It is fragile on host Linux/x86_64 because runnable
`.so` files may live under an architecture-specific directory, while Python
still resolves `lib` as the original package.

This runnable layer fixes only runtime loading:

- selects a host-compatible native directory for the current architecture and
  Python ABI
- prefers `pct_planner_runnable/native/<arch>` over bundled fallback binaries
- preloads dependent shared libraries with `ctypes.CDLL(..., RTLD_GLOBAL)`
- exposes the selected native directory as the `lib` package path
- imports and runs the original `planner/scripts/planner_wrapper.py`
- prepares LingTu tomograms for the original PCT axis convention without
  mutating the source `tomogram.pickle`

The expected host output directory is:

```text
src/global_planning/pct_planner_runnable/native/x86_64/
```

Do not copy rebuilt host `.so` files into:

```text
src/global_planning/pct_planner/planner/lib/
```

That directory remains the upstream/original PCT native tree. Overwriting it can
break S100P/aarch64 runtime assumptions and makes host rebuilds hard to audit.

## When To Rebuild

Rebuild the runnable x86_64 native artifacts when any of these changes:

- the host OS or C/C++ runtime changes, especially across Ubuntu releases
- Python changes ABI, for example CPython 3.10 to CPython 3.11
- GTSAM, OSQP, Eigen, compiler, or CMake inputs change
- files under `pct_planner/planner/lib/src/` or its CMake configuration change
- imports fail with native linker errors such as `GLIBC`, `GLIBCXX`, or missing
  shared-library messages
- `scripts/planning/plan_preview.py --planner pct --strict` reports
  `backend_available=false`

Do not rebuild just because a map changed. New maps require a fresh
`tomogram.pickle`, not new native libraries.

## Build On Ubuntu 22.04 x86_64

Run this from the repository root on the target host. The known-good host is
Ubuntu 22.04, Python 3.10, x86_64.

```bash
cd /path/to/lingtu
python3 --version
bash src/global_planning/pct_planner_runnable/build_host_x86_64.sh
```

Optional parallelism control:

```bash
JOBS=8 bash src/global_planning/pct_planner_runnable/build_host_x86_64.sh
```

By default the host build uses `PCT_CPU_ARCH_FLAGS=-march=x86-64-v2` so the
result is not tied to the exact CPU that compiled it. Override only when the
target fleet baseline is known:

```bash
PCT_CPU_ARCH_FLAGS=-march=native bash src/global_planning/pct_planner_runnable/build_host_x86_64.sh
```

The script builds or reuses third-party dependencies, builds PCT native modules,
and writes the runnable artifacts to:

```text
src/global_planning/pct_planner_runnable/native/x86_64/
```

Expected files include CPython extension modules and dependent shared
libraries, for example:

```text
a_star.cpython-310-x86_64-linux-gnu.so
ele_planner.cpython-310-x86_64-linux-gnu.so
traj_opt.cpython-310-x86_64-linux-gnu.so
py_map_manager.cpython-310-x86_64-linux-gnu.so
liba_star_search.so
libcommon_smoothing.so
libele_planner_lib.so
libgpmp_optimizer.so
libgtsam.so*
libmap_manager.so
libmetis-gtsam.so*
```

The build helper may create temporary build/install directories below the
original third-party source tree, but the runnable output remains under
`pct_planner_runnable/native/x86_64/`.

## Validate The Runnable Loader

Use `run_pct_preview` for a direct PCT smoke test against a real tomogram:

```bash
cd /path/to/lingtu
export PYTHONPATH="$PWD/src:$PWD"
python3 -m global_planning.pct_planner_runnable.run_pct_preview \
  --tomogram /path/to/tomogram.pickle \
  --start 2 3 0 \
  --goal 18 11 0 \
  --json
```

A passing response has:

```json
{
  "ok": true,
  "planner": "pct",
  "path_count": 55,
  "goal_error_m": 0.11
}
```

Small differences in `path_count`, `path_distance_m`, and `goal_error_m` are
normal when the tomogram, start, or goal changes. The important gates are
`ok=true`, finite path points, and a goal error appropriate for the map
resolution.

## Validate The System Entry

Use `scripts/planning/plan_preview.py` to validate the same runtime through the
NavigationModule preview path. This is the gate to run before a real navigation
session:

```bash
cd /path/to/lingtu
python3 scripts/planning/plan_preview.py --planner pct --strict --compact
```

If the active map is not under the default map root, pass it explicitly:

```bash
python3 scripts/planning/plan_preview.py \
  --planner pct \
  --tomogram /path/to/tomogram.pickle \
  --start 2 3 0 \
  --goal 18 11 0 \
  --strict \
  --compact
```

A passing strict response has:

```json
{
  "ok": true,
  "planner": "pct",
  "cases": [
    {
      "feasible": true,
      "backend_available": true
    }
  ]
}
```

`plan_preview.py` is offline and non-motion: it does not start
`lingtu.service`, does not call Gateway, does not send `/api/v1/goal`, and does
not publish `cmd_vel`.

## Known-Good Validation

PCT has been rebuilt and validated on a remote Ubuntu 22.04 / Python 3.10 /
x86_64 host with artifacts in:

```text
src/global_planning/pct_planner_runnable/native/x86_64/
```

Direct PCT smoke test against a real building tomogram:

```text
run_pct_preview ok=true
path_count=55
path_distance_m~=17.78
goal_error_m~=0.11
```

System preview entry:

```text
scripts/planning/plan_preview.py --planner pct --strict
ok=true
feasible=true
backend_available=true
```

## Common Failures

### `GLIBC` Or `GLIBCXX` Version Not Found

Typical errors:

```text
ImportError: /lib/x86_64-linux-gnu/libc.so.6: version `GLIBC_2.xx' not found
ImportError: /usr/lib/x86_64-linux-gnu/libstdc++.so.6: version `GLIBCXX_3.4.xx' not found
```

Cause: the native modules were built on a newer Linux or C++ runtime than the
host that runs them.

Fix:

```bash
cd /path/to/lingtu
bash src/global_planning/pct_planner_runnable/build_host_x86_64.sh
ldd src/global_planning/pct_planner_runnable/native/x86_64/a_star*.so
```

Build on the same Ubuntu release and Python ABI that will run the preview. Do
not fix this by copying random system libraries into the repo.

### `libmetis-gtsam.so` Missing

Typical error:

```text
ImportError: libmetis-gtsam.so: cannot open shared object file: No such file or directory
```

Cause: GTSAM's bundled METIS shared library was not built, copied, or visible to
the runtime loader.

Fix:

```bash
find src/global_planning/pct_planner_runnable/native/x86_64 \
  -name 'libmetis-gtsam.so*' -o -name 'libgtsam.so*'
bash src/global_planning/pct_planner_runnable/build_host_x86_64.sh
```

The runnable loader preloads shared libraries from the selected native
directory. `libmetis-gtsam.so*` should sit next to the PCT extension modules in
`native/x86_64/`.

### Tomogram Axes Or Bounds Look Wrong

Symptoms:

- PCT returns no path while the same map looks connected
- `plan_preview.py` reports out-of-bounds start or goal unexpectedly
- path endpoints or dimensions look swapped
- goal error is much larger than the map resolution

Cause: LingTu's offline tomogram builder stores grid axes for preview code as
`x,y`, while the original PCT wrapper expects `y,x` and derives map dimensions
from the last two array axes.

Fix:

- Use this runnable loader instead of importing the original wrapper directly.
- Do not edit the source `tomogram.pickle` by hand.
- Let `prepare_tomogram_for_pct()` create its cached transposed copy. It uses
  `$LINGTU_PCT_CACHE_DIR` or `$LINGTU_CACHE_DIR` when configured, otherwise it
  tries `<tomogram-dir>/.pct_runnable_cache/` and then the system temp directory
  when the map directory is read-only.
- If a tomogram is already in PCT axis order, mark it with
  `pct_axes_transposed=True` in the pickle metadata before handing it to this
  loader.

If the source tomogram changes, remove stale local cache files only after
confirming they were generated from the old file. The cache filename includes
the source size and mtime.

### Backend Unavailable In `plan_preview.py`

Typical output fields:

```json
{
  "ok": false,
  "pct_runtime_libs": {
    "ok": false
  },
  "cases": [
    {
      "backend_available": false,
      "backend_load_error": "PCT runtime libraries are missing"
    }
  ]
}
```

Fix in order:

1. Rebuild with `build_host_x86_64.sh` on the target host.
2. Confirm `native/x86_64/` contains the CPython extension modules for the
   running Python version.
3. Confirm `libgtsam.so*` and `libmetis-gtsam.so*` are present beside the
   extension modules.
4. Rerun `run_pct_preview` directly before retrying `plan_preview.py`.

For diagnosis only, `LINGTU_PCT_LIB_DIR=/absolute/path/to/native_dir` can point
the loader at a custom native directory. Do not use that as the normal
deployment path.

## Non-Negotiable Boundary

Keep runnable host builds in:

```text
src/global_planning/pct_planner_runnable/native/x86_64/
```

Do not overwrite, clean, or repopulate:

```text
src/global_planning/pct_planner/planner/lib/
```

The runnable layer exists so host-specific fixes can be tested without changing
the original PCT planner tree or the S100P/aarch64 runtime surface.
