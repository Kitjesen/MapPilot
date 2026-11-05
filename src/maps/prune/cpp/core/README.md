# Saved-map cleaning core

The default prune evaluates recorded scans in chronological order, finds
multi-frame free-ray contradictions, preserves currently supported surfaces,
and writes the kept/removed point split. It does not perform semantic motion
recognition or run ERASOR2 stages.

| File | Responsibility |
| --- | --- |
| `types.hpp` | Point, pose and spatial-index key types. |
| `text.hpp` / `text.cpp` | Tokenization, JSON escaping and path text. |
| `io.hpp` / `io.cpp` | PCD, recorded pose order and calibrated sensor-origin input. |
| `evidence.hpp` / `evidence.cpp` | Pose transform and voxel-key calculation shared by visibility and offline replay. |
| `visibility.hpp` / `visibility.cpp` | Per-point chronological hit/free evidence, same-frame endpoint priority and current planar-neighbor support. |
| `surface.hpp` | Local plane support geometry shared with the offline incremental experiment. |
| `save.hpp` / `save.cpp` | Kept/removed output, backup and apply semantics. |

`cleaner.cpp` owns the complete read, observe, classify and save flow.
Classification reports the actual decision as uncontradicted, supported surface
or removed. It does not accumulate a second voxel-hit map for support scores.
Ground segmentation experiments live in `research/maps/ground`. Build instructions
and offline comparisons are maintained in the [map research guide](../../../../../research/maps/README.md).

Do not add ERASOR2 headers or source includes here. External research comparisons
remain separate from the product build and default saved-map cleaning.
