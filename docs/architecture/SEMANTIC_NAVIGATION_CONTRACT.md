# Semantic Navigation Contract

Status: current semantic-navigation contract

This document defines how natural-language destinations become executable
navigation goals without allowing a language model to invent coordinates.

## Boundary

```text
AskMe FINAL text
  -> Gateway voice turn
  -> symbolic semantic intent
  -> native map-bound place lookup
  -> map-bound lingtu.building_goal.v1 mission request
  -> navigation ownership and safety gates
```

Language understanding owns only symbolic fields:

- `action`
- `target_query`
- `floor_id`
- `tour_id`
- `travel_mode`
- `confidence`
- `needs_clarification`
- `reason`

Coordinates, poses, paths, velocities, and map identity never come from an
LLM. They come from native saved-map records after version, point-cloud hash,
and frame validation.

The production slow path is asynchronous and symbolic-only. The
`SemanticPlannerModule` first runs the deterministic grammar; recognized
first-release commands never call an LLM. Only unrecognized utterances with weak
navigation, floor, connector, or tour-control features are sent through the
existing `LLMModule` `request`/`response` ports. The planner records a bounded
pending `request_id`, accepts only the current matching response, strips common
JSON code fences, and validates the payload with
`HybridSemanticIntentParser.from_symbolic_mapping`. Bad JSON, unknown fields,
coordinate fields, stale responses, and LLM errors fail closed without
publishing a goal or navigation command. `needs_clarification: true` produces a
clarifying reply only.

## Voice request

AskMe submits only admitted final turns:

```http
POST /api/v1/voice/turns
Authorization: Bearer <LINGTU_API_KEY>
X-Request-Id: <stable turn id>
Idempotency-Key: <stable turn id>
Content-Type: application/json
```

```json
{
  "text": "坐电梯去6楼某公司",
  "operator_id": "askme.voice",
  "session_id": "voice-session-1",
  "channel": "voice",
  "robot_id": "dog-01",
  "site_id": "site-a",
  "submit": true,
  "metadata": {"locale": "zh-CN"}
}
```

LingTu returns `handled: true` with a non-empty `turn.spoken_reply`, including
when safety or motion ownership rejects execution. This prevents AskMe from
locally processing the same final turn a second time.

The AskMe `lingtu_voice` deployment must explicitly set
`runtime.voice_bridge.enabled: true`, point `ASKME_EDGE_SERVICE_URL` at the
LingTu Gateway, and use the same API key on both sides. The repository default
remains disabled so unrelated AskMe profiles cannot acquire this handoff by
accident.

## Place truth

`PlaceCatalog` is a typed projection over the native Maps POI service. A place
is executable only when all of the following are present and current:

- stable `place_id` and canonical display name;
- aliases, kind, building, and floor identity;
- caller-surveyed `x/y/z/yaw` in the saved-map frame;
- `map_id`, integer map version, version ID, `map.pcd` SHA-256, and frame ID;
- provenance source and confidence.

Name or stable-ID rebinding is rejected. Alias collisions produce an
ambiguous result and require a filter or clarification. A map rebuild or hash
change makes the old place non-executable until it is deliberately surveyed
or rebound again.

Natural floor labels normalize to the existing building identity convention:
`6楼`, `六层`, `floor_6`, and `floor-6` become `floor-6`; `负一楼` becomes
`floor-b1`.

For every resolved `PlaceCatalog` destination, including an `any`-mode target
on the active map, the semantic planner publishes a
`lingtu.building_goal.v1` `building_navigate` command only when both
`nav.building` and `nav.goals` are mounted. The command contains the symbolic
travel preference plus the complete target binding: place/building/floor/map
identity, frame and surveyed pose, map version, version ID, and point-cloud
hash. If either runtime capability or any binding field is missing, the request
fails closed. The language parser never supplies these coordinates.

## How the map learns names

Geometry does not discover business names by itself. Deployment provisions
them through an explicit evidence workflow:

1. Save and validate each floor map.
2. An operator, BIM import, or confirmed perception observation proposes a
   place at a measured pose.
3. `PlaceCatalog` records it through the native POI API and binds it to the
   current map version/hash/frame.
4. A human-confirmed place becomes executable. Unconfirmed visual/LLM/vector
   evidence remains search evidence only.
5. Elevator and stair entrances are registered as connector endpoint places;
   the native MapGraph owns inter-map topology.
6. A tour name is the native inspection `route_id`; the inspection store owns
   its ordered, map-version-bound route points.

Gateway exposes the minimal provisioning surface for this workflow:

- `POST /api/v1/places` registers or updates one canonical POI-backed place.
  The body supplies `place_id`, `name`, `map_id`, finite `x/y/z`, and `source`,
  with optional aliases/kind/building/floor/yaw/connector/confidence/frame
  metadata. Callers cannot send `map_version`, `version_id`, or
  `map_pcd_sha256`; those binding fields are read from the native maps service.
- `GET /api/v1/places?map_id=...` lists places with executable status, reason,
  and stored/current map binding.
- `GET /api/v1/places/resolve?q=...&map=...&building=...&floor=...&kind=...`
  resolves exact names or aliases and reports `resolved`, `ambiguous`,
  `not_found`, or `stale_map`. Stale places are returned as candidates but are
  not executable.

## Persistence decision

Version 1 does not add a new database. Native POIs and MapGraph remain the
single executable source of truth. Chroma/vector memory, scene graphs, tagged
location JSON, and semantic voxels may suggest candidates, but they cannot
authorize motion.

SQLite becomes appropriate only if the product later needs independent
multi-writer transactions, large alias/provenance queries, BIM synchronization,
or observation history. Even then it should store catalog workflow and audit
data; executable poses remain bound to native map artifacts.

## Delivery status

| Capability | Status |
| --- | --- |
| First-release Chinese intent grammar | Implemented |
| Async symbolic LLM slow path | Implemented through existing `LLMModule` ports; rules skip LLM |
| Strict symbolic LLM-result schema | Implemented; coordinates and unknown fields fail closed |
| AskMe final-turn/idempotency/Bearer API | Implemented |
| Native map-bound PlaceCatalog | Implemented |
| Same-active-map named-place dispatch | Routed through the same map-bound `BuildingMission` ingress; no raw POI pose bypass |
| Inspection tour start/pause/resume/cancel | Routed through native inspection commands |
| Explicit elevator/stair intent | Routed to the optional existing `BuildingMission` ingress; otherwise `CONNECTOR_RUNTIME_REQUIRED` |
| Cross-map named-place dispatch | Routed as `lingtu.building_goal.v1` only with a complete map binding and mounted building runtime |
| Physical floor transition | BuildingMission integration is present; site connector adapters/catalogs and Z-aware stair execution still require deployment configuration |

No resolved `PlaceCatalog` coordinate is sent through the plain `PoseStamped`
path. `BuildingMission` owns same-map admission as well as floor transitions
and target-map handoff. `PoseStamped` remains available to the legacy
scene-graph and visual-servo resolution paths. A deployment without a verified
connector catalog, lift adapter, or stair executor must leave that capability
disabled so named-place requests fail closed.

The current native `list_map_graph` response does not yet expose a stable
`connector_id` or `lift_id`. `BuildingService` may use a route `edge_id` only
to correlate graph records; it never promotes that graph-edge identity to an
executable lift identity. Explicit elevator/stair requests therefore fail
closed until the native connector contract or a verified site adapter supplies
the real connector identity.

The Python service is opt-in with `enable_building=True` and the short runtime
alias `nav.building`. Its `building_mission_module` must name the deployed,
site-verified mission owner. This does not add a native process, so
`config/runtime_graph/products/nav.yaml` stays unchanged; add a Product process
role and Endpoint mapping only if a real lift controller is later deployed as
an independent process.

## Upstream patterns considered

- [That-nav](https://github.com/real-lsy/That-nav) for integrated mapping,
  localization, planning, control, and robot-bridge process closure.
- [LingBot-Map](https://github.com/robbyant/lingbot-map) as an optional offline
  geometry/reconstruction producer, not a place database.
- [Hydra](https://github.com/MIT-SPARK/Hydra) and
  [Spark-DSG](https://github.com/MIT-SPARK/Spark-DSG) for layered scene-graph
  identities and relations.
- [VLMaps](https://github.com/vlmaps/vlmaps) and
  [ConceptGraphs](https://github.com/concept-graphs/concept-graphs) for
  open-vocabulary semantic recall that remains non-authoritative.
- [Open-RMF Traffic Editor](https://github.com/open-rmf/rmf_traffic_editor)
  for level, lift, door, and topology modeling patterns.

These projects inform schemas and interfaces. Their ROS/CUDA-heavy runtimes
are not added to the S100P production dependency set.
