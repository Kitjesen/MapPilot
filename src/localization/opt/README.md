# Map Optimization

`src/localization/opt` is the product-facing C++ entry surface for map
optimization.

Short names are intentional:

| File | Role |
| --- | --- |
| `map.*` | Resolve and check saved-map artifacts: `map.pcd`, `poses.txt`, `patches/*.pcd`. |
| `graph.*` | Shared non-ROS pose graph, PCD patch, and report writer used by PGO/HBA. |
| `pose_math.hpp` | Shared, explicit `T_parent_child` pose algebra used by loop verification. |
| `cloud.hpp` | Portable PCD input contract shared by map optimization and loop verification. |
| `loop_constraints.*` | Deterministic saved-map loop candidate, 4DoF verification, and audit report. |
| `loop_cli.cpp` | Read-only `lt_loop_verify` shadow command. |
| `pgo.*` | Product default save-time pose graph optimization runner. |
| `hba.*` | Optional high-quality/offline save-time map refinement runner. |

Product rule:

- product save-map optimization remains `off` until field-accepted independent
  loop constraints and transactional PGO application exist;
- `lt_loop_verify` is shadow-only: it reports constraints but never calls PGO
  and never rewrites `map.pcd`, `poses.txt`, or patches;
- HBA is an optional high-quality/offline refinement;
- `none` is allowed for explicit diagnostics or emergency save;
- Gateway/Web should call native binaries after they exist, not implement
  optimization logic.

Product commands:

```bash
lt_pgo --map MAP_DIR --out MAP_DIR
lt_hba --map MAP_DIR --out MAP_DIR
lt_loop_verify --map MAP_DIR --report /tmp/loop-$(date +%s).json
```

Both commands read `poses.txt` plus `patches/*.pcd`, call the portable
`src/kernels/slam/pose_graph_opt` C ABI, rebuild `map.pcd`, replace
`poses.txt`, keep `.preopt` backups for in-place runs, and write
`map_optimization.json`.

`lt_loop_verify` is deliberately separate. It requires one basename-only,
unique `patch_name` per `poses.txt` row, exact pose/patch cardinality, finite
unit quaternions, and a report path outside the source map directory. Reports
are written through a same-directory temporary file and published by rename;
an existing report is never overwritten.

The shadow verifier uses:

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
   single-wall and open-corridor sliding modes; eigenvalues and the weakest
   mode are written to the v3 report;
7. consensus requiring different history and current anchors that advance in
   the same direction; one current scan matching two adjacent history scans
   is not independent evidence;
8. bounded spatial-history sampling and hard physical parameter limits before
   descriptor/ICP work, plus pre/post fingerprints so a changing map is
   rejected.

The implementation is independent and informed by public loop-closure
literature; it does not copy STD/LTA-OM code or claim their closed-source
Fast-LIO2-BA implementation. Shadow constraints deliberately carry an all-zero
graph information diagonal: gravity-frame left-tangent observability has not
yet been converted into the pose graph's body-frame right-tangent full 6x6
information matrix. Existing graph validation therefore rejects accidental
consumption. Product `MapOpt` stays `off` until a moving MID-360 loop replay,
false-loop corpus, 3000-patch S100P time/RSS gate, full information-frame
conversion, and transactional map commit all pass.
