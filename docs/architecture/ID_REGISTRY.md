# LingTu ID Registry

Status: current guardrail
Audience: API, runtime, domain-service, SDK, and persistence maintainers
Replaced by: not replaced

This registry assigns a canonical meaning to each cross-boundary identifier and
records current domain-qualified exceptions. Identical string values do not
make two identifier kinds interchangeable.

## Registry

| ID | Meaning | Owner | Generator | Scope | Lifecycle | Retry rule | Parent | Exposure | Deprecated / compatibility |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `request_id` | One logical interface request, including exact re-delivery after transport uncertainty. | The originating caller at the first command boundary. | Caller, SDK, or a permitted ingress adapter. The canonical automatically generated form is a 26-character uppercase ULID; caller-supplied values remain accepted where the API allows them for compatibility. | The owning service's request/idempotency namespace, normally further scoped by caller and command kind. It is not globally unique by implication. | Generated before submission, stable for exact re-deliveries, and retained through the boundary's correlation or deduplication window. | Reuse only when the outcome is unknown and the exact request is replayed. A changed payload, changed intent, or new application-level attempt gets a new value. | None. A resulting task or operation may record it as causal correlation. | Request fields, receipts, status correlation, audit, and logs where allowed. | No alias. It is not an `operation_id`, `task_id`, or native `job_id`; native navigation keeps `task_id` stable while each command delivery attempt has its own `request_id`. |
| `task_id` | One admitted, finite Task execution from planning through exactly one terminal result. | The domain runtime that owns lifecycle truth; native `nav` owns field navigation, while a composite mission orchestrator owns its parent Task. | The ingress application service generates a canonical 26-character uppercase ULID before native dispatch; downstream endpoints preserve it byte-for-byte. | One robot's domain Task namespace. | Stable across plan, execute, pause, resume, recovery, cancellation, Gateway restart, and evidence replay. It remains queryable after terminal according to retention policy. | Exact transport replay keeps the same `request_id` and `task_id`. An explicit application-level retry creates a new `task_id` and records `retry_of_task_id`. | Optional `parent_task_id` for a child execution; that value is another registered `task_id`, not a new ID kind. | HTTP receipts and queries, typed DDS commands/events/state, TaskLedger, SSE, CLI/Web, logs, and MCAP/evidence manifests. | Explore currently uses the registered domain-qualified `exploration_run_id`; no second alias is stored for the same run. A product read model may label either registered domain ID as the execution ID without minting another identity. |
| `operation_id` | One server-owned, durable asynchronous operation. | The domain service executing and persisting the operation. | The canonical target is a service-generated 26-character uppercase ULID. During the map-save migration, Gateway still projects the native identifier into this field. | One service and operation kind; consumers must not infer fleet-wide uniqueness. | Created when work is admitted and stable through queued/running/terminal states and crash recovery. It remains addressable until explicit service cleanup; map save currently publishes no expiry guarantee. | Query, cancel, and retry the admitted operation with this value; map-save retry reuses the same operation. A submission replay still uses its `request_id`, and clients never mint an `operation_id`. | No identity parent. The admitting `request_id`, when recorded, is causal metadata only. | Opaque public response/path identifier and internal operation lookup key. | Public map `job_id` and `/maps/save-jobs/...` are replaced by `operation_id` and `/maps/operations/...`. Native C++ `job_id` remains an internal implementation name during migration. |
| `exploration_run_id` | One finite execution of autonomous Explore, from admitted start through a native terminal result. | The native Explore endpoint owns lifecycle truth; Gateway owns durable admission and projects ordered native facts. | Gateway generates a canonical 26-character uppercase ULID before native dispatch. Clients never mint it. | One robot's Explore run namespace. | Created on start admission, stable across pause/resume/finish commands and Gateway restarts, and retained for bounded audit/query after terminal. An endpoint restart does not silently create a replacement run. | Exact replay of the same start `request_id` and immutable Product/map binding returns the same value. Changed intent conflicts; a newly requested execution gets a new `request_id` and new `exploration_run_id`. | No identity parent. `product_session_id` and map identity are immutable execution bindings, not identity parents. | Explore start receipt, `/api/v1/explore/runs/{exploration_run_id}`, native typed event stream, operator status, and logs. | Replaces the unshipped generic Explore `task_id` draft. Bodyless legacy start remains accepted only by having ingress generate a `request_id`; no public `task_id` or unqualified `run_id` alias is added. |
| `map_id` | One logical saved map and its current native content directory. | The Maps domain (`MapStore` / Maps service). | A caller supplies the public name, or Gateway generates the permitted default; Maps validates, normalizes, and commits it. | One map-store namespace. | Created, updated in place by a successful save, optionally renamed, activated, retired, and eventually deleted under Maps-domain transactions. Rename rekeys the resource; it does not create a second live identity. | Never use it as an idempotency key. Repeated operations may target the same `map_id` with distinct request and operation identities. | None. `content_epoch` accompanies its current content state but is not a child resource. | Map resources, bindings, active-map state, artifacts, and map-aware tasks. | Current API-local `name` or `map` fields may project the same logical ID. They are supported spellings, not additional identities; avoid adding new cross-domain names for `map_id`. |
| `content_epoch` | The positive numeric content-state marker for a saved map. It detects replacement of the content behind a stable `map_id`; it is not an immutable version resource. | Native C++ `MapStore`. | Derived from the committed native content state. Python, Gateway, and DDS only transport it. | Meaningful with its `map_id`; saved-map DDS and navigation fields use `content_epoch` or `map_content_epoch`. | Changes when committed content changes. There is no version directory, current-version pointer, or version rollback lifecycle. | An exact replay observes the committed epoch. A later successful content replacement exposes its new native epoch. | Associated map: `map_id`. | Map records, active-map identity, map-bound tasks/places, DDS, status, and planning contracts. | `map_version`, `version_id`, `map:v...`, and `map:e...` are not supported compatibility identities. Do not parse or emit them. |
| `map_save.job_id` (internal) | One persisted native C++ SaveMap job backing the transitional public operation. | Native `SaveMapEngine`. | The current engine copies `request_id`; this is implementation coupling, not the public generation rule. | One map-store root and its `.save_jobs` journal. | Durable from admission through terminal state and restart recovery; no current expiry guarantee is published. | Exact admission replay and failed/cancelled retry reuse the same job. | No identity parent; it records the causal `request_id`. | Native C++, C ABI, and private Python adapter only; Gateway translates it at the public seam. | Not deprecated internally yet. Public `job_id` and `/maps/save-jobs/...` are removed. |

## Qualified current profiles

- Map save uses `request_id` as its idempotency token within one map-store root.
  An exact payload replay returns the existing native job; different content
  with the same value conflicts.
- Native navigation uses `request_id` for one command delivery attempt and keeps
  `task_id` stable across higher-level retries. This domain scope is mandatory;
  neither value aliases the other.
- One navigation or inspection execution must carry the same `task_id` through
  HTTP admission, TaskLedger, typed DDS request/ACK, native state and events,
  Gateway/SSE projection, path/command/stop evidence, recording manifest, and
  replay. A missing link marks correlation or evidence incomplete; it does not
  authorize Gateway or RecordingManager to invent a Task terminal result.
- A composite inspection or mission has one parent `task_id`. Each independently
  cancellable native navigation leg has its own child `task_id` and references
  the parent. Pause/resume/recovery never create a child; an explicit retry does.
- `map_save.operation_id` is the durable public query/cancel/retry handle defined
  above. ProductControl currently also calls its switch correlation value
  `operation_id` and may derive it from `request_id`, but it does not expose the
  same map-operation query lifecycle. Treat that as a compatibility overload.
- Saved-map identity is the pair `(map_id, content_epoch)`. C++ owns both
  values; Python and Gateway transport the numeric epoch without deriving it
  from a path or encoded string.

## Non-equivalence rules

- Treat every ID as opaque. Do not derive resource kind, time, parent, or
  routing policy from its prefix or current textual shape.
- `request_id` answers "which submission attempt?"; `operation_id` answers
  "which persisted asynchronous execution?". Equality of their bytes never
  establishes equality of meaning.
- An operation may produce or update a `map_id`, and a successful map commit may
  report a `content_epoch`; neither value identifies the operation.
- Persist map-bound references as `(map_id, content_epoch)` whenever stale-content
  detection matters. A bare epoch does not identify a map.

## Map-save migration boundary

The public HTTP and SDK contract has migrated to:

```text
POST /api/v1/map/save
  request:  request_id
  response: operation_id

GET  /api/v1/maps/operations/{operation_id}
POST /api/v1/maps/operations/{operation_id}/cancel
POST /api/v1/maps/operations/{operation_id}/retry
```

The native C++ SaveMap engine still persists and accepts `job_id`. Gateway
currently translates public `operation_id` to that internal field and translates
native status back at the HTTP boundary. The current engine may initialize
`job_id` from `request_id`, so all three strings can be transitionally equal.
That coincidence is not an API guarantee and does not mean the semantics are
fully separated in the implementation. Clients must store the returned
`operation_id`, must not send or parse native `job_id`, and must not assume
`request_id == operation_id`.

## Registry changes

Before adding or exposing another identifier, register its owner, generator,
scope, lifecycle, retry behavior, parent, exposure, and deprecation path here.
Renaming a field is insufficient when ownership or lifecycle semantics change;
the boundary needs an explicit translation and migration note.
