# Saved-map cleanup core

`cleaner.cpp` owns read, validate, observe, classify and save. The default classifier
uses the pinned DUFOMap/UFOMap integration, with no legacy visibility fallback.

| File | Responsibility |
| --- | --- |
| `types.hpp` | Source point and saved pose types. |
| `text.hpp` / `text.cpp` | Tokenization, JSON escaping and paths. |
| `io.hpp` / `io.cpp` | PCD, poses, calibrated origin and original-record partitioning. |
| `evidence.hpp` / `evidence.cpp` | Rigid pose transform shared with offline replay. |
| `dufo.hpp` / `dufo.cpp` | DUFOMap integration and final seen-free classification. |
| `save.hpp` / `save.cpp` | Kept/removed output, first backup and apply semantics. |

The old planar helper belongs only to `research/maps/cleaning/surface.hpp`.
Research incremental cleanup is not a production fallback. Output point counts
describe the partition, not semantic dynamic-object labels.
