# LingTu Pose Graph Optimizer

Portable Rust kernel for the GTSAM subset LingTu currently uses in PGO/HBA:

- SE3 `Pose3` nodes with SO3/SE3 exp/log, compose, inverse, and between.
- Prior factors.
- Between-pose factors.
- Full 6x6 information matrices.
- SE3 local Jacobian linearization with block-sparse normal equations.
- Sparse Cholesky LM solves for larger systems, with dense LU fallback.
- Rust API plus stable C ABI probes and lifecycle functions.

`nalgebra` owns SE3/quaternion/small-matrix math, while `faer` solves the
dynamic LM linear systems.

This is not a full GTSAM replacement. It replaces the LingTu usage surface:
`Pose3`, prior factors, between factors, and batch SE3 pose-graph smoothing.
PGO uses this as a batch smoother with warm-started poses instead of iSAM2 so
the SLAM post-processing path can build without GTSAM on Windows and other
lightweight deployments.

The current kernel no longer recomputes a prior or between residual for every
pose perturbation in the hot path. Each factor uses the SE3 residual's local
right Jacobian inverse and adjoint blocks, then assembles into a block-sparse
upper-triangular normal equation structure. Larger systems use `faer` sparse
Cholesky first and fall back to dense LU if sparse factorization fails.

This is the first GTSAM-style performance step. Remaining performance work is
closed-form SE3 right-Jacobian formulas, symbolic sparsity/order caching, and
benchmark gates for LingTu-sized PGO/HBA graphs, not a full GTSAM API clone.

Run the portable benchmark with:

```bash
cargo run --release --manifest-path src/kernels/slam/pose_graph_opt/Cargo.toml --example benchmark -- 16 64 256
```

The benchmark emits `pgo_loop` and `hba_full_info` cases for each pose count.
Use `tools/bench/pose_graph_opt_compare.py` when a C++/GTSAM baseline JSON is
available from a Linux robot/dev machine. The Rust benchmark itself does not
require GTSAM.
