# Gateway schema implementation

gateway._schemas contains the domain-grouped implementation behind the stable
gateway.schemas public interface. Production callers, tests, plugins, and
external tools must continue to import public names from gateway.schemas.

## Module map

| Module | Owns |
| --- | --- |
| common.py | response base, frame constants, command/error receipts, shared server/realtime primitives |
| diagnostics.py | routecheck, runtime evidence, benchmark, and runtime-contract responses |
| navigation.py | goals, path and plan-preview request/response models |
| control.py | velocity, stop, cancel, mode, safety, visual-servo, and lease contracts |
| voice.py | voice turn request/result contracts |
| maps.py | saved-map requests and lifecycle/point responses |
| session.py | product session start/state/transition contracts |
| semantic.py | scene graph, locations, and temporal-memory contracts |
| operations.py | SLAM, bag, exploration, and go2rtc operation contracts |
| runtime_contracts.py | runtime frame, dataflow, format, source, and manifest contracts |
| runtime.py | runtime switch, readiness, liveness, device, and health contracts |
| auth.py | authentication contracts |
| inspection.py | acceptance and inspection-route contracts |
| navigation_status.py | localization and navigation status aggregates |
| app.py | app links, media, capabilities, traffic, bootstrap, and state aggregates |
| system.py | driver-swap contracts |

## Dependency rules

- Internal schema modules may import common and lower-level schema modules.
- They must not import Gateway routes, Gateway service helpers, or
  GatewayModule.
- gateway.schemas is the only supported public seam. It owns __all__ and
  restores every model's historical gateway.schemas pickle identity.
- Model class names, aliases, validators, defaults, and OpenAPI component names
  are compatibility contracts.

When adding or moving a model, update its internal module __all__, re-export it
from gateway.schemas, and run test_gateway_schema_modules.py plus the Gateway
OpenAPI contract tests.
