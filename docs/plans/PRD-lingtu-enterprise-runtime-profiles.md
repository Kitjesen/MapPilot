# PRD: LingTu Enterprise Runtime Profiles

## 1. Summary

This PRD defines how LingTu runtime profiles should become product-grade. A
profile must be a clear product entrypoint, not a loose pile of flags. Operators
should be able to choose a profile and know what robot, endpoint, planner,
safety policy, map source, and validation gate will run.

## 2. Contacts

| Name | Role | Comment |
| --- | --- | --- |
| Product owner | Runtime profile owner | Owns profile names and product modes. |
| Navigation owner | Global/local planning owner | Owns planner, safety policy, and map contract. |
| Runtime owner | Blueprint and resolver owner | Owns config merge, endpoints, and startup checks. |
| Field operator | Primary user | Needs safe, repeatable commands on Thunder. |
| QA owner | Release gate owner | Owns profile matrix tests and audit reports. |

## 3. Background

LingTu now has product profiles, simulation profiles, endpoint configs, robot
presets, and planner selection spread across several files. This is workable
for development, but not good enough for a field product.

The main problem is not one bad file. The problem is unclear ownership:
product intent, hardware endpoint, compatibility adapter, planner choice, and
test expectation can all override each other. This makes it hard to answer:
"What exactly runs when I choose `nav` on Thunder?"

The planner path was recently simplified. Profiles now need the same treatment:
less magic, fewer aliases, clear validation, and a product matrix that can be
checked before field use.

Existing runtime audit and runtime-spec tools already cover part of this. The
work should extend those checks instead of adding a parallel profile framework.

## 4. Objective

Make profiles the trusted product entrypoint for LingTu.

The result should let a user pick a product profile such as `lite`, `map`,
`nav`, `explore`, or `tare_explore` and get a predictable runtime. Engineers
should be able to add or change a profile without guessing which layer owns a
field.

### Key Results

| Key result | Target |
| --- | --- |
| Product profile audit | Every product profile resolves with zero implicit ROS2 runtime violations unless the endpoint explicitly allows compatibility. |
| Profile explainability | One command/report shows product intent, endpoint, robot preset, planner, map source, safety policy, autonomy backend, and gateway state. |
| Planner clarity | Product profiles use one canonical planner field. Legacy `planner_backend` is removed from product configs or marked compatibility-only. |
| Endpoint clarity | Every product profile declares supported runtime endpoints, or resolves to exactly one default endpoint. |
| Field safety | `map`, `nav`, `explore`, and `tare_explore` block startup when required map/localization/planner artifacts are missing. |
| Test coverage | A profile matrix test covers all product profiles and supported endpoints. |

## 5. Market Segments

| Segment | Job |
| --- | --- |
| Field operators | Start mapping, navigation, or exploration on Thunder without reading code. |
| Robotics engineers | Change planner, SLAM, or endpoint settings without breaking unrelated profiles. |
| QA/release | Prove each profile is valid before a robot-side release. |
| Integration partners | Understand which runtime endpoint and data contract a deployment uses. |

## 6. Value Propositions

| Need | Value |
| --- | --- |
| Safe field startup | Bad profile combinations fail before motion. |
| Clear product modes | `lite`, `map`, `nav`, `explore`, and `tare_explore` mean one thing each. |
| Fast replacement | Planner and endpoint can be swapped at defined seams. |
| Lower debug cost | Runtime reports show which layer selected each critical setting. |
| Release confidence | CI can validate the profile matrix without hardware. |

## 7. Solution

### 7.1 User Flow

```text
Operator chooses profile
  -> resolver merges product + robot + endpoint + overrides
  -> profile audit validates the resolved runtime
  -> blueprint builds the module graph
  -> Navigation creates planner service
  -> planner runs through GlobalPlanRequest / GlobalPlanResult
```

### 7.1.1 Runtime Decision Contract

These rules are binding:

1. `profile` answers what the robot should do.
2. `endpoint` answers where runtime data and commands come from.
3. `robot_preset` answers which robot defaults to apply.
4. `blueprint` only assembles modules from a resolved config.
5. `planner` is the only product planner selector.
6. `planner_backend` is compatibility-only and must not appear in product
   profiles or product endpoint overrides.
7. User overrides apply last, but they do not bypass product endpoint checks.

Examples:

| Command | Expected result |
| --- | --- |
| `python lingtu.py nav` | Resolves to profile `nav` on endpoint `thunder_field`. |
| `python lingtu.py nav --endpoint replay` | Allowed no-actuation replay runtime. |
| `python lingtu.py nav --endpoint cmu_unity` | Fails before blueprint build. |
| `python lingtu.py lite --endpoint thunder_field` | Fails before blueprint build. |

### 7.2 Requirements

#### P0: Profile ownership rules

- Product profiles own "what the robot should do".
- Runtime endpoints own "where data and commands come from".
- Robot presets own hardware identity and connection defaults.
- Blueprint stack owns module assembly only.
- Planner implementation is not selected through registry at runtime.

#### P0: Canonical product profiles

Keep these product profiles:

| Profile | Purpose | Default planner | Endpoint expectation |
| --- | --- | --- | --- |
| `lite` | Minimal local robot shell | `direct` | `thunder_lite` |
| `map` | Build/save map | `octoplanner3d` | `thunder_field` or simulation endpoint |
| `nav` | Navigate saved map | `octoplanner3d` | `thunder_field` |
| `explore` | Wavefront/traversable frontier exploration | `octoplanner3d` | `thunder_field` or simulation endpoint |
| `tare_explore` | TARE-style exploration | `octoplanner3d` | `thunder_field`, `mujoco_live`, or `cmu_unity` |
| `super_lio` | Super-LIO evaluation | `octoplanner3d` | explicit evaluation endpoint |
| `super_lio_relocation` | Super-LIO relocation evaluation | `octoplanner3d` | explicit evaluation endpoint |

#### P0: Allowed endpoint matrix

Use this matrix as the product contract:

| Profile | Allowed endpoints |
| --- | --- |
| `lite` | `thunder_lite` |
| `map` | `thunder_field`, `mujoco_live`, `replay` |
| `nav` | `thunder_field`, `replay` |
| `explore` | `thunder_field`, `mujoco_live`, `replay`, `gazebo` |
| `tare_explore` | `thunder_field`, `mujoco_live`, `replay`, `cmu_unity` |
| `super_lio` | `thunder_field` |
| `super_lio_relocation` | `thunder_field` |

Unsupported pairs must fail during profile resolution or runtime-spec
generation, before blueprint build.

#### P0: Canonical fields

Product profiles should use these fields:

- `planner`
- `tomogram`
- `plan_safety_policy`
- `fallback_planner_name`
- `slam_profile`
- `localization_adapter`
- `exploration_backend`
- `enable_frontier`
- `enable_traversable_frontier`
- `enable_gateway`
- `enable_map_modules`
- `planning_frame_id`
- `expected_saved_map_frame_id`
- `python_autonomy_backend`
- `python_path_follower_backend`

Compatibility fields such as `planner_backend` may remain in simulation or
legacy endpoint code for one release, but product profiles should not need them.
Product-specific endpoint overrides should also stop writing `planner_backend`;
simulation-only profiles can keep it during the compatibility window.

Field ownership:

| Owner | Owns | Must not own |
| --- | --- | --- |
| Product profile | Product mode, planner family, safety policy, map artifact need, semantic/exploration flags | Endpoint transport, command bridge, hardware source |
| Runtime endpoint | Data source, command sink, adapter transport, endpoint contract, launcher args | Product intent names, product-only planner aliases |
| Robot preset | Robot identity and hardware connection defaults | Product behavior, planner family |
| Resolver | Merge order, endpoint validation, public resolved metadata | Module construction |
| Blueprint | Module graph from resolved config | Profile policy or endpoint policy |

#### P0: Profile audit

Add or tighten checks for:

- Product and simulation profile names do not overlap.
- Product profiles do not select ROS2 global planner wrappers.
- Product profiles do not rely on implicit ROS2 localization fallback.
- Every product profile has a supported endpoint path.
- `nav`, `explore`, and `tare_explore` require a valid saved-map planner input.
- Planner name resolves to a direct `GlobalPlanner` mapping.
- Endpoint overrides cannot silently change planner family without audit output.

Extend the existing audit surface:

- `cli/runtime_audit.py::_check_profile_runtime_specs`
- `src/runtime/runtime_validation_gates.py`
- `src/runtime/tests/test_runtime_evidence.py`
- `src/runtime/tests/test_runtime_catalogs.py`
- `src/runtime/tests/test_runtime_binding_policy.py`

Do not create a second audit command.

#### P1: Runtime explain report

Reuse existing CLI surfaces:

- `python lingtu.py show-config <profile> --endpoint <endpoint> --json`
- `python lingtu.py runtime-spec <profile> --endpoint <endpoint> --json`

The human-readable runtime output should include:

```text
profile=nav
endpoint=thunder_field
robot=thunder
planner=octoplanner3d
map=<resolved map path>
safety=reject
localization=lcm_endpoint
local_planner=nanobind
path_follower=nav_kernel
gateway=true
startup_gates=map_artifact, localization, planner_runtime
```

This can reuse existing resolver and runtime display code. Do not add a new
reporting framework or a new command unless the existing commands cannot show
the needed fields cleanly.

#### P1: Enterprise profile matrix

Extend the existing profile runtime specs test for:

- Product profile alone.
- Product profile with default endpoint.
- Product profile with every supported endpoint.
- CLI override of planner, robot, and endpoint where allowed.

The test should report blockers, not only pass/fail import status.

#### P1: Implementation tasks

The first implementation pass is limited to this table:

| Task | File(s) | Required behavior | Required check |
| --- | --- | --- | --- |
| Define product endpoint matrix | `src/runtime/profiles/catalog/endpoints.py` | Add one `PRODUCT_PROFILE_ENDPOINTS` mapping matching the table above. | Catalog test compares the mapping to `RUNTIME_ENDPOINTS`. |
| Enforce matrix | `src/runtime/profiles/resolver.py` | `resolve_runtime_config(profile, runtime_endpoint_name=...)` rejects unsupported product/endpoint pairs. | Resolver test proves `nav + cmu_unity` fails even if endpoint catalog drifts. |
| Remove product `planner_backend` | `product_intents.py`, `endpoint_adapter_configs.py`, `endpoints.py` | Product profiles and product endpoint overrides use `planner` only. | Catalog/audit tests fail if product paths contain `planner_backend`. |
| Keep compatibility | `simulation_profiles.py`, planner alias helpers | Simulation and legacy tests may still use `planner_backend` for one release. | Existing simulation tests keep passing. |
| Explain resolved runtime | `cli/ui.py`, `runtime_switch.py`, `runtime_display.py` | `show-config --json` exposes public runtime metadata; `runtime-spec` shows planner, autonomy backends, and startup gates. | CLI/display tests assert these fields. |
| Extend audit | `cli/runtime_audit.py` | `profile_runtime_specs` reports endpoint matrix and blocks matrix drift. | `profile_runtime_specs.ok=true` for current repo state. |

Do not add new abstractions for this pass.
Do not add a new command.
Do not move profile catalogs.

#### P2: Compatibility cleanup

After P0/P1:

- Remove `planner_backend` from product profiles.
- Keep aliases like `s100p` and `octplanner` as compatibility only.
- Add deprecation warnings for compatibility names in CLI output.

### 7.3 Technology

Use existing files:

- `src/runtime/profiles/catalog/product_intents.py`
- `src/runtime/profiles/catalog/simulation_profiles.py`
- `src/runtime/profiles/catalog/endpoints.py`
- `src/runtime/profiles/catalog/robots.py`
- `src/runtime/profiles/catalog/robot_runtime_defaults.py`
- `src/runtime/profiles/resolver.py`
- `src/runtime/profiles/binding_policy.py`
- `src/runtime/blueprints/stacks/navigation_core.py`
- `src/nav/services/plan/contracts.py`

Do not add a new profile framework. The current dict catalog is enough if we
add clear rules and tests.

### 7.4 Assumptions

- Thunder product modes are the first enterprise target.
- ROS2 compatibility remains allowed at endpoints, but not as hidden fallback
  inside product profiles.
- OctoPlanner3D remains the default map-backed global planner.
- PCT remains an explicit evaluation/legacy planner.
- Direct planner remains only for `lite` and mapless tests.

## 8. Release

### V1: Profile contract and audit

- Define canonical product profile fields.
- Remove duplicate planner fields from product profiles.
- Add profile matrix audit.
- Add explain output for resolved profile.
- Keep compatibility aliases working.

V1 is complete only when these commands work:

```bash
python lingtu.py show-config nav --json
python lingtu.py runtime-spec nav --json
python lingtu.py runtime-spec nav --endpoint replay --json
python lingtu.py runtime-audit --json
```

`runtime-audit` may fail on unrelated checks, but
`checks.profile_runtime_specs.ok` must be true.

### V2: Endpoint hardening

- Make each product profile declare allowed endpoints.
- Block unsupported profile/endpoint pairs early.
- Tighten map/localization/planner artifact gates.

### V3: Compatibility cleanup

- Remove product use of compatibility fields.
- Add warnings for old aliases.
- Keep simulation profiles separate from product profiles.

## Non-Goals

- No new registry system.
- No new profile DSL.
- No new product UI.
- No new profile/audit CLI command unless existing commands cannot expose the
  required data.
- No planner algorithm change.
- No removal of simulation profiles.

## Acceptance Criteria

- `resolve_profile_config(profile)` works for every product profile.
- Default endpoint resolution is deterministic for every hardware product profile.
- Every allowed profile/endpoint pair resolves through `runtime-spec`.
- `checks.profile_runtime_specs.ok=true` for supported product profile/endpoint pairs.
- Unsupported profile/endpoint pairs fail with a clear error.
- Product profile catalogs and product endpoint overrides do not use
  `planner_backend`.
- `Navigation` still receives only resolved constructor kwargs.
- Global planning still uses `GlobalPlanRequest` and `GlobalPlanResult`.
- `python lingtu.py show-config nav --json` includes `runtime_endpoint`,
  `endpoint_data_source`, `runtime_contract`, and `endpoint_contract`.
- `python lingtu.py runtime-spec nav --json` includes `global_planner`,
  `autonomy_backends`, `plan_safety_policy`, and `startup_gates`.
- Existing planner, profile resolver, runtime catalog, and runtime binding
  tests pass.
