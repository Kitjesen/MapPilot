# Map Optimization

`src/localization/opt` is the product-facing C++ entry surface for map
optimization.

Short names are intentional:

| File | Role |
| --- | --- |
| `map.*` | Resolve and check saved-map artifacts: `map.pcd`, `poses.txt`, `patches/*.pcd`. |
| `graph.*` | Shared non-ROS pose graph, PCD patch, and bundle writer used by PGO. |
| `pose_math.hpp` | Shared, explicit `T_parent_child` pose algebra used by loop verification. |
| `cloud.hpp` | Portable PCD input contract shared by map optimization and loop verification. |
| `loop_constraints.*` | Deterministic saved-map loop candidate, 4DoF verification, and audit report. |
| `assembly.*` | Build a complete measured graph from adjacent patches plus verified loops. |
| `loop_cli.cpp` | Read-only `lt_loop_verify` verification command. |
| `pgo.*` | ROS-free save-time pose graph optimization runner. |
| `constraints.*` | Strict dependency-free parser for independent full-information constraints. |

Runtime rule:

- the former ROS2 PGO and HBA packages remain deleted;
- `lt_pgo` is a short-lived native SaveMap helper, not a resident service;
- Fast-LIO2 freezes `map.pcd`, `poses.txt`, body-local `patches/*.pcd`, and
  `patch_bundle.manifest`; it does not publish `pose_graph.constraints`;
- `lt_pgo --auto-constraints` measures every adjacent single-patch edge, runs
  loop verification once, and optimizes only when the complete N-1 chain and at
  least one verified loop are available;
- `lt_loop_verify` reports verified constraints but never calls PGO
  and never rewrites `map.pcd`, `poses.txt`, or patches;
- a rejected adjacent registration or absence of verified loops is a structured
  successful skip: no optimization output or partial constraint file is left;
- `pose_graph.constraints` is an atomically written, strictly re-read private
  optimizer input; automatic mode deletes it before publishing any output;
- Gateway/Web do not implement optimization logic or expose an optimizer switch.

Native commands:

```bash
lt_pgo --map MAP_DIR --out NEW_BUNDLE_DIR --constraints CONSTRAINTS.txt
lt_pgo --map MAP_DIR --out NEW_BUNDLE_DIR --auto-constraints
lt_loop_verify --map MAP_DIR --report /tmp/loop-$(date +%s).json
```

`lt_pgo` reads `poses.txt`, `patches/*.pcd`, and an explicit independent
constraint file containing the complete factor graph (including adjacent
odometry and loop factors), calls the portable `src/kernels/slam/pose_graph_opt` C ABI,
and writes a new self-contained bundle containing `map.pcd`, `poses.txt`,
`patches/`, `patch_bundle.manifest`, and `map_optimization.json`. In-place output
and any output below the source map directory are rejected. The executable is
included in the native release; no systemd unit or DDS endpoint is created for it.

`poses.txt` provides initial estimates only. PGO fixes pose 0 as the sole gauge
anchor and never turns pose differences into graph weights. In automatic mode,
each adjacent factor is instead measured from its two local patches with the
same bidirectional trimmed 4DoF registration and point-to-plane information
model used by loop verification. Its information comes from the measured
`J^T J / sigma^2` and is converted to the body-right tangent. Each input factor
carries `T_from_to` and a full packed
upper-21 information matrix in the right tangent order
`[omega_x omega_y omega_z upsilon_x upsilon_y upsilon_z]`.

`lt_loop_verify` is deliberately separate. It requires one basename-only,
unique `patch_name` per `poses.txt` row, exact pose/patch cardinality, finite
unit quaternions, and a report path outside the source map directory. Reports
are written through a same-directory temporary file and published by rename;
an existing report is never overwritten.

The verifier uses:

1. planar XY candidate radius plus independent Z and traveled-path gates;
2. local multi-patch submaps transformed into gravity-aligned anchor frames;
3. directional descriptors with distinct yaw-peak and historical-place
   ambiguity margins;
4. deterministic trimmed 4DoF ICP in gravity coordinates, converted back to
   body-frame `T_from_to`;
5. forward/reverse independent registration; every yaw hypothesis must pass
   overlap, residual, span, and observability gates before coverage-first
   ranking, so a low-overlap repeated fragment cannot win on RMSE alone;
6. target-normal point-to-plane Fisher/Hessian observability gates that reject
   single-wall and open-corridor sliding modes; accepted measurements carry a
   full packed 6x6 right-tangent information matrix;
7. consensus requiring different history and current anchors that advance in
   the same direction; one current scan matching two adjacent history scans
   is not independent evidence;
8. bounded spatial-history sampling and hard physical parameter limits before
   descriptor/ICP work, plus pre/post fingerprints so a changing map is
   rejected.

The implementation is independent and informed by public loop-closure
literature; it does not copy STD/LTA-OM code or claim their closed-source
Fast-LIO2-BA implementation. Automatic assembly now produces a complete graph
only from measured adjacent factors and accepted loop factors; it never uses a
fixed information matrix or a keyframe covariance substitute. Field replay,
false-loop, trajectory-quality, and S100P performance acceptance remain
outstanding.
