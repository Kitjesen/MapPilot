# CPU PCT experiment

`pct_cpu.py` ports the point-cloud tomography, terrain costs, inflation and
slice simplification equations from `byangw/PCT_planner`, pinned at
`35cd73fd82bcd51bc538429294af7646b2a09815`. Upstream authors: Bowen Yang and
contributors; upstream source license: GPL-2.0. This is experimental benchmark
code, not a production planner or a new field dependency.

Requires NumPy and SciPy. It does not import CUDA, CuPy, ROS or Open3D. Native
PCT A* and GPMP trajectory optimization are compiled separately from upstream.
The CPU kernel tests use small scalar/analytic fixtures; they do not certify
bitwise equivalence to GPU execution. Coordinate quantization preserves the
upstream half-precision input conversion and C round-away-from-zero behavior.
Tomograms retain the upstream float16 export format.

The building-scene defaults are retained. Unless explicitly supplied, the
scene's lower height is taken from the complete input point cloud rather than
assuming the demo's zero datum. Both the original slice indices and raw
pre-inflation costs are retained as diagnostic data. No missing ground is
filled. Uniform unknown regions retain upstream cost semantics; finite ground
must be examined independently when interpreting a result.

The offline dataset experiment and build scripts are under
`build/go2-map-save-v6-20260930/pct/`. It first selects ground/ceiling intervals
containing the original body Z, then picks the finite ground nearest the
original body height minus 0.35 m. Equal-height layers use
the lower cost. XY is rounded to the nearest grid cell, never relocated to an
easier target. Each search uses a fresh upstream planner, avoiding retained
search state across independent benchmark cases. These input adaptations are
reported separately from unchanged native search code.

The isolated WSL build uses Python 3.12, current pybind11, and the bundled
GTSAM 4.1.1/OSQP sources. Both GTSAM and its PCT consumers must agree on Eigen
alignment: this experiment disables `-march=native` in both, rather than
mixing the upstream native-vectorized consumer with generic GTSAM. The build
scripts and dependency library paths are recorded with the experiment.

Validate the CPU port with:

```powershell
python -m pytest tests/tools/test_pct_cpu.py -q
```

CPU map construction, native A* success, optimization output, trajectory
diagnostics, simulation and field motion are distinct evidence levels. Do not
report an offline path as successful robot navigation.
