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
| `operation_id` | One server-owned, durable asynchronous operation. | The domain service executing and persisting the operation. | The canonical target is a service-generated 26-character uppercase ULID. During the map-save migration, Gateway still projects the native identifier into this field. | One service and operation kind; consumers must not infer fleet-wide uniqueness. | Created when work is admitted and stable through queued/running/terminal states and crash recovery. It remains addressable until explicit service cleanup; map save currently publishes no expiry guarantee. | Query, cancel, and retry the admitted operation with this value; map-save retry reuses the same operation. A submission replay still uses its `request_id`, and clients never mint an `operation_id`. | No identity parent. The admitting `request_id`, when recorded, is causal metadata only. | Opaque public response/path identifier and internal operation lookup key. | Public map `job_id` and `/maps/save-jobs/...` are replaced by `operation_id` and `/maps/operations/...`. Native C++ `job_id` remains an internal implementation name during migration. |
| `exploration_run_id` | One finite execution of autonomous Explore, from admitted start through a native terminal result. | The native Explore endpoint owns lifecycle truth; Gateway owns durable admission and projects ordered native facts. | Gateway generates a canonical 26-character uppercase ULID before native dispatch. Clients never mint it. | One robot's Explore run namespace. | Created on start admission, stable across pause/resume/finish commands and Gateway restarts, and retained for bounded audit/query after terminal. An endpoint restart does not silently create a replacement run. | Exact replay of the same start `request_id` and immutable Product/map binding returns the same value. Changed intent conflicts; a newly requested execution gets a new `request_id` and new `exploration_run_id`. | No identity parent. `product_session_id` and map identity are immutable execution bindings, not identity parents. | Explore start receipt, `/api/v1/explore/runs/{exploration_run_id}`, native typed event stream, operator status, and logs. | Replaces the unshipped generic Explore `task_id` draft. Bodyless legacy start remains accepted only by having ingress generate a `request_id`; no public `task_id` or unqualified `run_id` alias is added. |
| `map_id` | One logical map across its committed versions. | The Maps domain (`MapStore` / Maps service). | A caller supplies the public name, or Gateway generates the permitted default; Maps validates, normalizes, and commits it. | One map-store namespace. | Created, versioned, optionally renamed, activated, retired, and eventually deleted under Maps-domain transactions. Rename rekeys the resource; it does not create a second live identity. | Never use it as an idempotency key. Repeated operations may target the same `map_id` with distinct request and operation identities. | None; `map_version` is scoped beneath it. | Map resources, bindings, active-map state, artifacts, and map-aware tasks. | Current API-local `name` or `map` fields may project the same logical ID. They are supported spellings, not additional identities; avoid adding new cross-domain names for `map_id`. |
| `map_version` | One immutable committed version of a logical map. | The native Maps service and version store. | The Maps service allocates the next numeric version under the map write lock and commits it only after verification. | Unique only within its parent `map_id`; the durable reference is `(map_id, map_version)`. | Immutable after commit. Rollback changes the current-version pointer; it does not rewrite the selected version. | Retries must not mutate a committed version. A later successful content commit receives another version. | Required parent: `map_id`. | Map records, operation results, map-bound tasks/places, version listing, and rollback APIs. | Current API-local `version` path parameters and internal `MapRecord.version` project this same identity and remain supported; avoid new unqualified version fields. |
| `map_save.job_id` (internal) | One persisted native C++ SaveMap job backing the transitional public operation. | Native `SaveMapEngine`. | The current engine copies `request_id`; this is implementation coupling, not the public generation rule. | One map-store root and its `.save_jobs` journal. | Durable from admission through terminal state and restart recovery; no current expiry guarantee is published. | Exact admission replay and failed/cancelled retry reuse the same job. | No identity parent; it records the causal `request_id`. | Native C++, C ABI, and private Python adapter only; Gateway translates it at the public seam. | Not deprecated internally yet. Public `job_id` and `/maps/save-jobs/...` are removed. |

## Qualified current profiles

- Map save uses `request_id` as its idempotency token within one map-store root.
  An exact payload replay returns the existing native job; different content
  with the same value conflicts.
- Native navigation uses `request_id` for one command delivery attempt and keeps
  `task_id` stable across higher-level retries. This domain scope is mandatory;
  neither value aliases the other.
- `map_save.operation_id` is the durable public query/cancel/retry handle defined
  above. ProductControl currently also calls its switch correlation value
  `operation_id` and may derive it from `request_id`, but it does not expose the
  same map-operation query lifecycle. Treat that as a compatibility overload.
- Persistent Maps `map_version` is numeric. The global-planning contract still
  has a string `map_version` token; that is a current compatibility inconsistency
  and must be translated or normalized before claiming the same identity.

## Non-equivalence rules

- Treat every ID as opaque. Do not derive resource kind, time, parent, or
  routing policy from its prefix or current textual shape.
- `request_id` answers "which submission attempt?"; `operation_id` answers
  "which persisted asynchronous execution?". Equality of their bytes never
  establishes equality of meaning.
- An operation may produce or update a `map_id`, and a successful map commit may
  report a `map_version`; neither value identifies the operation.
- Persist map-bound references as `(map_id, map_version)` whenever stale-version
  detection matters. A bare `map_version` is ambiguous.

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
