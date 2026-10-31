# Map Cleaning Core

This folder defines the product pipeline contract for saved-map cleaning. It is
the LingTu-owned `lingtu_field_v2` path for S100P/MID-360 saved maps.

The product path is:

```text
load -> label -> submap -> evidence -> protect -> score -> split -> save
```

Current files:

| File | Role |
| --- | --- |
| `types.hpp` | Shared point, pose, voxel, and evidence structs. |
| `text.hpp` / `text.cpp` | Small string, tokenization, and JSON/path helpers. |
| `io.hpp` / `io.cpp` | PCD and LingTu `poses.txt` loading/writing. |
| `evidence.hpp` / `evidence.cpp` | Pose transform, voxelization, and report-only support classification. |
| `visibility.hpp` / `visibility.cpp` | Chronological free-ray contradictions, calibrated origin, per-frame endpoint priority, and persistent planar-neighbor preservation. |
| `score.hpp` / `score.cpp` | Instance-level moving-object score summary. |
| `save.hpp` / `save.cpp` | Clean/removed output, backup, and apply semantics. |
| `flow.hpp` / `flow.cpp` | Canonical stage names, implementation state, and JSON report shape. |

Reserved stage names (not implementation requirements):

| File | Role |
| --- | --- |
| `label.hpp` / `label.cpp` | Unused in the saved-map path; standalone ground segmentation lives in `src/maps/ground`. |
| `submap.hpp` / `submap.cpp` | Multi-frame submap accumulation. |
| `protect.hpp` / `protect.cpp` | Static structure protection. |
| `split.hpp` / `split.cpp` | Static/dynamic point split. |

Do not add ERASOR2 headers or source includes here. ERASOR2 remains a research
reference and optional external comparison path.

The saved-map path evaluates planar support from final multi-frame evidence.
It does not use cached ground labels or link Patchwork++/Eigen. The legacy JSON
ground-protection count remains zero; supported surfaces have their own count.
