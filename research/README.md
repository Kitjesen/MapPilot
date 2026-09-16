# Research

`research/` contains experiments, archived implementations, upstream references,
and integration candidates. Nothing here belongs to the shipped LingTu runtime
until it is intentionally migrated into an owned `src/`, `config/`, `scripts/`,
or `web/` surface with tests and a license review.

## Directory map

| Path | Ownership | Purpose |
| --- | --- | --- |
| `coreplanner/` | Git submodule | Upstream CORE Planner exploration research. |
| `far/` | Archived reference | Historical ROS FAR Planner source retained for comparison. |
| `perception/` | LingTu experiment | Prototype perception and path-planning code. |
| `semantic/` | Git submodules | Upstream semantic-map implementations. |
| `parking/` | LingTu candidate | RDK no-parking validation package; not the shipped Inspection Workbench analyzer. |
| `vla/` | LingTu experiment | ROS 2 VLA navigation training and deployment prototype. |

## Naming

- Top-level names use concise domain or project labels; status words such as
  `archive`, `experimental`, `refs`, and `package` belong here, not in paths.
- LingTu-owned directories and ordinary files use lowercase `snake_case`.
- `README.md`, `CMakeLists.txt`, `COLCON_IGNORE`, ROS message types such as
  `Graph.msg`, and other tool-defined names keep their required spelling.
- Submodule contents and project directories inside `semantic/` keep upstream
  names so that upstream identity, imports, and documentation remain traceable.
- Generated model receipts and manifests keep the names recorded by their
  producing tool.

The short top-level names are workspace labels only. Internal identities such
as the `vla_nav` ROS/Python package and `vehicle_parking_detection` module do not
change with their parent directory.

## External references

| Path or name | Source | Purpose |
| --- | --- | --- |
| `coreplanner/` | https://github.com/BBD00/core_planner | Contextual-memory RL exploration. |
| `semantic/concept-graphs/` | https://github.com/concept-graphs/concept-graphs | 3D scene-graph construction. |
| `semantic/DualMap/` | https://github.com/Eku127/DualMap | Dual semantic-geometric mapping. |
| `semantic/HOV-SG/` | https://github.com/hovsg/HOV-SG | Hierarchical open-vocabulary 3D scene graphs. |
| `semantic/OVO/` | https://github.com/tberriel/OVO | Open-vocabulary occupancy. |
| `semantic/vlmaps/` | https://github.com/vlmaps/vlmaps | Visual-language maps. |
| `dimos` | https://github.com/dimensionalOS/dimos | Optional local reference for semantic maps, replay, simulation, and MCP. |

## Restore references

Restore the registered references at their pinned commits:

```bash
git submodule update --init --recursive
```

Optional, unregistered references may be cloned locally under
`research/semantic/`; `research/semantic/dimos/` is ignored and all research
content is excluded from LingTu releases.
