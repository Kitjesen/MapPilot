# Thunder Runtime Refactor Plan

## Why This Refactor Exists

The current repository has the right foundation: Module-First runtime units,
Blueprint orchestration, typed ports, and explicit critical wiring. The part
that still feels unprofessional is ownership mixing:

- Product names, board names, and runtime endpoints appear in the same layer.
- Normal module code still has historical ROS-shaped names or assumptions.
- Lightweight deployment and full endpoint navigation share too many defaults.
- Compatibility bridges are not always visibly separated from product logic.

The target is not to delete all advanced capability. The target is to make the
default product path clear, small, and deployable, while keeping ROS and other
external stacks as explicit compatibility endpoints.

Communication policy is recorded in
`docs/architecture/LINGTU_RUNTIME_BUS_DECISION.md`. Future runtime, endpoint,
and topic-contract work should treat that file as the project memory for the
ROS-to-LingTu Runtime Bus transition.

Current scope: Thunder endpoint/server/simulation communication. `thunder_field`
is a legacy endpoint id, not evidence that the current target is real hardware
motion.

## Target Shape

```text
lingtu.py
  -> runtime resolver
  -> product profile catalog
  -> endpoint catalog
  -> product blueprint
  -> module stacks
  -> compat endpoint adapters
```

### Product Layer

Product profiles answer: what should Thunder do?

Owned by:

- `src/runtime/blueprints/catalog/products.py`
- `src/runtime/blueprints/products/thunder.py`

Rules:

- Use product names such as `thunder-lite`, `thunder-nav`, and
  `thunder-explore`.
- Do not encode board names such as `s100p` into new product-facing APIs.
- Keep robot presets split into canonical names and compatibility aliases.
  `thunder` and `thunder_remote` are canonical physical robot presets;
  `s100p` and `navigate` remain compatibility aliases only.
- A product profile may choose SLAM, semantic, maps, gateway, and planner
  defaults, but it must not directly own ROS launch or transport details.

### Endpoint Layer

Endpoints answer: where does runtime data come from and how does it cross a
process or stack boundary?

Owned by:

- `src/runtime/blueprints/catalog/endpoints.py`
- `src/runtime/blueprints/runtime_endpoint.py`

Rules:

- `module_transport` describes internal ModulePort connections.
- `endpoint_transport` describes cross-stack or cross-process data exchange.
- `thunder_lite` stays `local + local`.
- `thunder_field` uses `local + lcm` and advertises the
  `thunder_field_lcm_v1` endpoint contract.
- Product-facing endpoint selectors should use aliases such as
  `thunder-field` or `field`; `real_s100p` and `s100p` remain compatibility
  aliases only.
- `localization_adapter=lcm_endpoint` is the endpoint localization ingress for
  Thunder LCM odometry, map cloud, registered cloud, and health streams.
- `nav_out_adapter=lcm_nav_output` is the endpoint navigation egress for
  Thunder LCM global path, local path, active waypoint, and muxed command
  velocity streams.
- `command_output_mode=endpoint_only` means endpoint navigation does not keep an
  in-process `ThunderDriver`; the endpoint source owns hardware actuation.
- `nav_in_adapter=lcm_nav_input` is the endpoint navigation command
  ingress for Thunder LCM goal, cancel, and semantic instruction streams.
- LCM payloads use a versioned JSON envelope for normal Module messages; high
  bandwidth streams such as point clouds should use explicit binary schemas.
- ROS, LCM, simulator, and replay endpoints must be visible here, not hidden
  inside normal modules.

### Module Layer

Modules answer: what computation happens inside LingTu?

Owned by:

- `src/runtime/`
- `src/nav/`
- `src/semantic/`
- `src/memory/`
- `src/drivers/`
- `src/gateway/`

Rules:

- Normal modules should depend on `core`, not on ROS packages.
- Normal modules should use typed ports and registry-selected backends.
- Critical fan-in/fan-out stays explicit in blueprint wiring.
- Hot-path native code remains optional behind backend selection.

### Compatibility Layer

Compatibility adapters answer: how does LingTu talk to legacy or external
systems?

Owned by:

- `src/*/adapters/ros2/`
- `src/runtime/adapters/lcm/`
- future endpoint-specific adapter packages

Rules:

- ROS imports live in compat code only.
- Legacy import paths may remain as small shims while tests migrate.
- LCM endpoint contracts live in compat code and must use product-neutral
  schemas rather than ROS message type names.
- LCM localization ingress is implemented by
  `src/runtime/adapters/lcm/localization_adapter.py` and keeps the legacy
  `SlamBridgeModule` blueprint alias only as a wiring compatibility name.
- LCM path, waypoint, and velocity command egress is implemented by
  `src/runtime/adapters/lcm/nav_output.py`. The product graph should expose it
  as `nav.out`; legacy path bridge selector values are accepted only by the
  binding policy and resolve to `lcm_nav_output`.
- LCM goal, cancel, and semantic instruction ingress is implemented by
  `src/runtime/adapters/lcm/nav_input.py`. The product graph should expose it
  as `nav.in`; legacy command bridge selector values are accepted only by the
  binding policy and resolve to `lcm_nav_input`.
- Standalone Thunder endpoint processes should use
  `src/runtime/adapters/lcm/endpoint_service.py` and the runnable
  `src/runtime/adapters/lcm/endpoint_runner.py` entrypoint to publish normalized
  sensor/localization streams and consume LingTu path/cmd_vel outputs without
  importing the LCM package directly.
- Endpoint sources must implement the
  `src/runtime/adapters/lcm/source.py` plugin protocol. Real hardware adapters and
  smoke/replay sources attach there, not inside product modules.
- `--source thunder_field` is the product source group for endpoint deployment.
  It expands to the Brainstem command sink and automatically includes the
  JSONL sensor/localization provider when JSONL path or command configuration
  is present.
- The endpoint runner accepts comma-separated sources. Use this to compose
  independent endpoint duties such as Brainstem command sinking, localization
  publishing, sensor publishing, or replay without merging those concerns into
  one large adapter.
- `src/runtime/adapters/lcm/sources/smoke.py` is the built-in no-ROS source for endpoint
  data-flow smoke checks. It is not a hardware adapter.
- `src/runtime/adapters/lcm/sources/jsonl.py` is the built-in no-ROS normalized
  sensor/localization source. It reads JSONL records from a file or external
  process stdout and publishes them through the Thunder LCM endpoint contract.
  Use it for replay, external non-ROS SLAM/localization sidecars, and endpoint
  provider bring-up.
- `tools/validate/validate_lcm_jsonl_feed.py` is the deployment gate for JSONL
  endpoint providers. It decodes records through the same Thunder LCM contract,
  checks endpoint-to-LingTu direction, validates runtime frame ids, and can
  enforce the minimal endpoint sensor inputs with `--require-field-inputs`.
- Brainstem command sinks are no longer built into the driver adapter package.
  Field deployments should provide command sinks as endpoint `module:factory`
  sources outside the LingTu module graph.
- Endpoint bridge module names should be transport-neutral and short in the
  product graph: use `nav.in` and `nav.out` for navigation command ingress and
  navigation output egress. Long class names stay hidden behind adapter code and
  legacy aliases.

## Deployment Targets

### `thunder-lite`

Purpose: fast local endpoint bring-up and cross-platform smoke deployment.

Contract:

- No ROS.
- No SLAM.
- No semantic stack.
- No gateway.
- No map layers.
- Pure-Python autonomy defaults: `simple` local planning and `pid` following.
- Command path still goes through `SafetyRingModule -> CmdVelMux -> ThunderDriver`.

### `thunder-nav`

Purpose: full endpoint navigation on Thunder.

Contract:

- Module graph remains in-process local transport.
- Cross-stack endpoint transport is explicit.
- SLAM/localization can be provided by a compatibility endpoint while LingTu
  consumes canonical odometry, map cloud, health, path, and command topics.
- Safety and command arbitration remain inside LingTu.
- The resolved `thunder-nav` graph does not include `ThunderDriver` by
  default. Final motion output is
  `nav.velocity_mux.driver_cmd_vel -> nav.out.cmd_vel`, then the
  endpoint source translates that command to the robot hardware.
- The endpoint process runs separately through
  `scripts/deploy/thunder/run_lcm_endpoint_service.py` or the
  `lingtu-thunder-lcm-endpoint.service` systemd unit. Real sensor/SLAM
  ownership attaches as a source plugin at this endpoint boundary, not inside
  normal LingTu modules.
- The default endpoint command source is `thunder_brainstem`. It owns the
  Brainstem motion sink only. The default service now names the product source
  group `thunder_field`, which expands to that sink and optional configured
  no-ROS localization/sensor providers.
- `lingtu-thunder-lcm-endpoint.service` uses `LINGTU_ENDPOINT_SOURCES` for the
  comma-separated endpoint source list and keeps `LINGTU_ENDPOINT_SOURCE` only
  as a compatibility fallback.
- The service exposes `LINGTU_ENDPOINT_JSONL_PATH`,
  `LINGTU_ENDPOINT_JSONL_COMMAND`, and `LINGTU_ENDPOINT_JSONL_RATE_HZ` so a
  no-ROS localization/sensor provider can be attached without changing the
  LingTu module graph.
- `tools/validate/validate_thunder_field_deployment.py` is the deployment
  gate for the default endpoint path. It checks the resolved `thunder-nav`
  runtime spec, endpoint-only graph shape, LCM endpoint contract, systemd unit,
  and deploy script defaults.
- Cross-platform endpoint smoke test:
  `python scripts/deploy/thunder/run_lcm_endpoint_service.py --transport local --source smoke --once --json`.

## Refactor Phases

### Phase 1: Lock Product Boundaries

- Keep Thunder product blueprints separate from legacy full-stack entrypoints.
- Keep `lite` out of heavyweight snapshot groups.
- Add tests that fail if `thunder-lite` pulls in ROS, SLAM, semantic, gateway,
  map, or endpoint bridge modules.

### Phase 2: Contain ROS Completely

- Move remaining ROS-facing implementations into `src/*/adapters/ros2/`.
- Keep only temporary shim files at legacy paths.
- Expand AST boundary checks so `nav`, `semantic`, `drivers`, and `gateway`
  cannot import ROS directly.

### Phase 3: Make LCM The Field Endpoint

- Keep generic LCM ModulePort traffic on the JSON envelope, not pickle.
- Keep `thunder_field_lcm_v1` as the visible endpoint contract for Thunder
  odometry, clouds, localization health, paths, active waypoint, command
  velocity, goal, cancel, and semantic instruction streams.
- Use `LCMLocalizationAdapterModule` for odometry, map cloud, registered cloud,
  localization quality, and localization health ingress.
- Use `LCMNavOutModule` for global path, local path, active
  waypoint, and muxed command velocity egress.
- Keep `command_output_mode=endpoint_only` for Thunder endpoints so the
  module graph has one command sink and no duplicate in-process hardware
  driver.
- Use `LCMNavInModule` for goal, cancel, and semantic
  instruction ingress.
- Use `LCMEndpointService` as the standalone endpoint-side publisher/consumer
  for Thunder LCM sensor, localization, map, path, waypoint, and cmd_vel
  streams.
- Use `python -m runtime.adapters.lcm.endpoint_runner` or
  `scripts/deploy/thunder/run_lcm_endpoint_service.py` as the supervised
  process entrypoint for that endpoint service.
- Use `--source smoke` for no-ROS endpoint data-flow checks and
  `--source thunder_field` for the default product endpoint source group. Use
  `--source thunder_brainstem` or `--source jsonl` only when debugging those
  endpoint duties directly. Configure `LINGTU_ENDPOINT_JSONL_PATH` or
  `LINGTU_ENDPOINT_JSONL_COMMAND` when the endpoint source group should include
  normalized no-ROS sensor/localization ingress.
- Validate any JSONL endpoint provider before deployment:
  `python tools/validate/validate_lcm_jsonl_feed.py <feed.jsonl> --require-field-inputs`.
- Keep message normalization at the endpoint boundary.
- Preserve ModulePort wiring inside LingTu.

### Phase 4: Lightweight Packaging

- Keep `config/thunder_lite_package.yaml` as the single contract for the
  lightweight package boundary.
- Keep `requirements-lite.txt` as the no-ROS, no-Gateway, no-semantic-model
  Python dependency target.
- Keep `tools/validate/validate_thunder_lite_package.py` in CI/manual checks
  so core dependencies, Lite deploy files, and the resolved `thunder-lite`
  runtime spec cannot drift back into ROS or Gateway coupling.
- Build releaseable Lite directories with
  `python tools/package_thunder_lite.py --output artifacts/thunder-lite-package --force`;
  the packager must read the same manifest, apply `exclude_paths` and
  `omit_paths`, and audit the output for forbidden runtime imports.
- Keep `scripts/deploy/thunder/runtime-env.sh`,
  `scripts/deploy/thunder/lingtu-thunder-lite.service`, and
  `scripts/deploy/thunder/install_lite_service.sh` as the minimal no-ROS
  install target for `thunder-lite`.
- Keep `scripts/deploy/thunder/install_services.sh` defaulting to
  `lcm-endpoint`. Use `lite` for the minimal local service and
  `ros-compat` / `legacy` only when the old ROS service chain is explicitly
  required.
- Keep `deploy_thunder.sh` on the lightweight runtime environment and skip the
  ROS compatibility environment by default for Thunder product profiles.
- Exclude ROS, SLAM, semantic models, gateway, native planner builds, and
  simulator assets from the lightweight package by default.
- Keep optional extras for full endpoint navigation and simulation.

### Phase 5: Delete Legacy Surface

- Remove compatibility aliases only after tests and docs have migrated.
- Keep product aliases stable for operators.
- Prefer deletion over another naming layer.

## Quality Gates

Each phase should leave these checks passing:

- Product graph tests for selected profiles.
- Runtime resolver tests for endpoint contracts.
- Architecture boundary validator.
- Thunder endpoint deployment validator.
- Thunder Lite package validator and packager dry-run.
- JSONL endpoint feed validator for any no-ROS endpoint provider.
- Focused module tests for touched components.
- `runtime-spec thunder-lite --json`.
- `runtime-spec thunder-nav --json`.
