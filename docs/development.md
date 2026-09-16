# Development

**Status:** Current contributor guide
**Audience:** Contributors, maintainers, and integration engineers
**Runs on:** Local development, native builds, simulation, and field-targeted builds

Make the smallest change in the layer that owns the behavior. A correct
implementation in the wrong layer creates a second source of truth.

## Find the owner first

| Change | Primary owner |
| --- | --- |
| Product roles, capabilities, or logical topics | `config/runtime_graph/products/` |
| Real or simulation implementation mapping | `config/runtime_graph/envs/` |
| Product resolution and RunPlan | `src/lingtu/run_plan.py` and `src/lingtu/assembly/` |
| Product lifecycle | `src/lingtu/control.py` |
| Host Module framework and shared messages | `src/runtime/` |
| Global/local planning, safety, or tracking hot path | `src/nav/cpp/` |
| Saved maps and live map services | `src/maps/` |
| Localization and SLAM | `src/localization/` |
| Hardware and simulation I/O | `src/drivers/` |
| Perception, decision, and memory | Their matching `src/` package |
| REST, SSE, WebSocket, and MCP | `src/gateway/` |
| Simulation packages and runtime | `sim/` |

Package-local READMEs explain deeper placement. The repository-level map is in
[Architecture](./architecture.md).

## Dependency direction

```text
All Modules -> runtime core, contracts, messages, registry, and utilities

nav/        must not import perception/, decision/, drivers/, gateway/
perception/ must not import nav/, decision/, drivers/, gateway/
decision/   consumes perception through messages, not nav/gateway imports
drivers/    must not import nav/ or semantic code except lazy registration
gateway/    must not import nav/, semantic/, or drivers/
```

Machine-enforced ownership is defined in
[`config/architecture_layers.yaml`](../config/architecture_layers.yaml).

## Ordinary change loop

1. State the user-visible contract and the evidence level it affects.
2. Find the current owner, interface, configuration, and focused tests.
3. Reuse an existing message, registry, factory, or native boundary.
4. Implement one behavior slice without adding a parallel lifecycle.
5. Wire the change through the owning Blueprint or Product declaration.
6. Run the narrowest test that could disprove the intended behavior.
7. Update one of these ten docs only when a maintained public contract changes.

Before a check, name the concrete failure it can reveal and what you would
change if it fails.

## Modules and Blueprints

A Module is one typed, in-process Host runtime unit. Keep construction cheap;
open devices, threads, subscriptions, and external resources in the declared
lifecycle. A Module cleans up only resources it owns.

Use typed `In` and `Out` ports for Host dataflow. Critical command, map,
localization, and external-boundary wires must be explicit. `auto_wire()` is
only suitable for unique, unambiguous local matches.

Blueprint constructs one Host graph. It may choose Modules, configuration,
aliases, wires, and boundary adapters. It must not resolve a Product, manage
systemd, start native endpoints, or hide domain algorithms.

Adding a Python class does not make it Product behavior. The Product compiler,
assembly stack, critical wires, and focused graph tests must agree.

## Backends and adapters

Use the existing runtime registry and the owning factory when an implementation
is intentionally selectable. Business logic depends on a stable interface,
not a concrete backend import.

An adapter belongs at a real boundary:

| Boundary | Required design information |
| --- | --- |
| Hardware SDK | Device owner, reconnect behavior, calibration, and failure state |
| Typed DDS | Producer, consumer, schema, frame, timestamp, QoS, and stale behavior |
| Shared memory | Ownership, generation, payload lifetime, and readiness metadata |
| Simulator or replay | Source identity, timing, reset, and claim boundary |
| ROS 2 compatibility | Explicit opt-in path that cannot become a default Product dependency |

Do not add a Module merely to wrap a pure function, or a transport abstraction
where an existing adapter already expresses the seam.

## Native code

Python owns the Host framework and semantic/API layer. C++ owns field LiDAR,
SLAM, maps, terrain, navigation, and driver hot paths.

Rust is appropriate for a bounded portable kernel when its ownership and build
path are explicit. It is not automatically faster than optimized C++.

Native executables, libraries, configuration, and assets must provide CMake
install rules for `bin`, `lib`, `etc/lingtu`, and `share/lingtu`. Production
code must not assume a component build directory.

Do not add dependencies without explicit approval. For third-party code, record
its license, provenance, supported target, and actual Product role.

## Configuration and parameters

Keep source-of-truth values with their owner:

- Product behavior in `config/runtime_graph/products/*.yaml`.
- Environment mapping in `config/runtime_graph/envs/*.yaml`.
- Physical devices and calibration in RobotConfig and `config/devices.yaml`.
- Algorithm defaults at the owning algorithm boundary.
- Operator overrides only for bounded, documented tuning surfaces.

Never embed a robot address, credential, map choice, or runtime env in Product
source. Do not solve a field issue by weakening a production threshold without
representative evidence.

## Tests and style

Start with the owning unit or contract test, then run only directly affected
lint, type, native-build, simulation, or field gates. See
[Testing](./testing.md) for claim levels.

Useful repository checks include:

```bash
python tools/validate/validate_architecture_boundaries.py
python tools/validate/validate_topics.py
python tools/validate/validate_docs.py
python -m pytest tests/docs/test_documentation_navigation.py -q
```

Run Ruff only on touched Python files. Run `bash -n` on changed shell scripts.
Use a parser-only PowerShell check for changed `.ps1` files.

## Commit and review

Keep unrelated user changes intact. Prefer small, reversible commits with one
clear purpose. A commit message should describe the owned behavior, not the
editing activity.

Before pushing:

1. Review the exact diff and staged paths.
2. Run the narrow validation that proves the change.
3. Record known baseline failures separately from new failures.
4. Do not claim field behavior without fresh S100P evidence.

Git history is the archive. Do not keep completed plans, transcripts, or dated
implementation diaries in `docs/`.
