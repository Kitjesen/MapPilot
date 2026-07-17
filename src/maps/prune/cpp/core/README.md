# Map Cleaning Core

This folder defines the product pipeline contract for saved-map cleaning. It is
the LingTu-owned `lingtu_field_v1` path for S100P/MID-360 saved maps.

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
| `evidence.hpp` / `evidence.cpp` | Pose transform, voxelization, and current static protection rule. |
| `score.hpp` / `score.cpp` | Instance-level moving-object score summary. |
| `save.hpp` / `save.cpp` | Clean/removed output, backup, and apply semantics. |
| `flow.hpp` / `flow.cpp` | Canonical stage names, implementation state, and JSON report shape. |

Planned product files:

| File | Role |
| --- | --- |
| `label.hpp` / `label.cpp` | Ground and instance labeling. |
| `submap.hpp` / `submap.cpp` | Multi-frame submap accumulation. |
| `protect.hpp` / `protect.cpp` | Static structure protection. |
| `split.hpp` / `split.cpp` | Static/dynamic point split. |

Do not add ERASOR2 headers or source includes here. ERASOR2 remains a research
reference and optional external comparison path.
