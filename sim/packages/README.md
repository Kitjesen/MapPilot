# Simulation Packages

`sim/packages/` is the single Catalog root for versioned simulation manifests
and every resource owned by those manifests.

## Contents

- `robots/`: robot bodies, articulated assets, frames, and visual projections.
- `controllers/`: control policies and robot or actuator binding requirements.
- `sensors/`: reusable sensor models, stream contracts, and package-owned assets.
- `sensor_rigs/`: sensor instances, mounts, frames, and calibration references.
- `worlds/`: static physics, visual, generated, and provenance resources.
- `scenarios/`: dynamic actors, events, stop conditions, and qualification rules.
- `payloads/`: attachable payload physics and visual resources.

## Boundary

`CatalogResolver` discovers packages only below this directory. A package owns
its manifest-referenced assets and must not escape its package root. Session
selection belongs in `sim/sessions/`; process allocation and execution belong
in `sim/runtime/`. Controller packages may contain the adapter or policy code
selected by their manifest; `sim/runtime/` still owns when and how it runs.
