# LingTu Documentation

**Status:** Current documentation index
**Audience:** Developers, integrators, operators, and reviewers
**Runs on:** Local development, `env=sim`, and `env=real`

Start with [Product](product.md): what LingTu delivers, how an operator uses it,
and which capabilities still need qualification. The shared delivery workflow is
**mapping -> map save/reload -> localization -> autonomous navigation -> safe stop**.

## Choose one page

| Task | Read |
| --- | --- |
| Understand product scope, user workflow, and delivery limits | [Product](product.md) |
| Install and make a first run | [Getting started](getting-started.md) |
| Map, save/reload, navigate, take over, and stop | [Operations](operations.md) |
| Find saved point clouds, pre-cleaning backups, failed saves and retry data | [Map files and save tasks](operations/map-files.md) |
| Test the current dialogue/follow work on this workstation and Go2 | [Dialogue test entry, 2026-10-21](operations/dialogue-test-20261021.md) |
| Decide whether the workflow has passed | [Testing](testing.md) |
| Run MuJoCo or inspect simulation evidence | [Simulation](simulation.md) |
| Inspect the fresh navigation physics tests and the unresolved crossing failure | [MuJoCo navigation, 2026-10-26](operations/mujoco-navigation-20261026.md) |
| Find a responsibility and its source of truth | [Architecture](architecture.md) |
| See the current responsibility map and verified boundary gaps | [System responsibility map](design/system-responsibility-map.md) |
| Find what each design document records and when to use it | [Design document index](design/README.md) |
| Understand the current twin-cylinder envelope, geometry sources, turning posture and historical Body decisions | [Collision model and motion posture](design/navigation-body-model-and-motion-posture.md) |
| Trace confirmed navigation errors, their dates, fixes and deployment limits | [Error record](operations/error_record.md) |
| Follow Product lifecycle and native dataflow | [Runtime](runtime.md) |
| Change code or simplify a directory | [Development](development.md) |
| Integrate REST, MCP, or SDK clients | [API](api.md) |
| See the current task, next action, evidence and delivery gates | [Roadmap](roadmap.md#current-delivery-cycle) |
| Understand persistent semantic mapping and paper references | [Persistent semantic mapping](research/semantic-map.md) |
| Design the camera, semantic-task, and motion handoffs | [Camera-to-motion design](design/autonomy.md) |

For an operator, read **Product -> Getting started -> Operations**. For delivery,
continue with **Testing -> Roadmap**. Developers use Architecture, Runtime,
Development and API as references; simulation procedures belong in Simulation.

## Sources of truth

Schemas, validated configuration, and owning code define behavior. These
pages explain how to use it; package READMEs hold implementation detail.
Start from [runtime declarations](../config/system/README.md),
[source ownership](../src/README.md), [simulation](../sim/README.md), or
[build/deploy scripts](../scripts/README.md).

## Documentation policy

The root pages form the maintained handbook and are bundled into the Web
`/guide/`. FastAPI `/docs` remains the live OpenAPI interface. The directory
boundaries are:

| Location | Purpose | Update when |
| --- | --- | --- |
| Root handbook | Current product scope, procedures, contracts and qualification policy | Supported behavior or operator workflow changes |
| [Roadmap](roadmap.md#current-delivery-cycle) | One current task, remaining work and delivery gates | Evidence or priorities change |
| `operations/` | Dated field incidents, handoffs and experiment records | A named run needs its context recorded |
| [Design](design/README.md) | Design decisions and proposed interactions; history is marked separately | A design is considered or revised |
| [Research](research/README.md) | External references, comparisons and experiments | Research evidence changes |
| `assets/` | Reviewed illustrations and their provenance READMEs for published documentation | A published figure or its evidence description changes |
| Package READMEs | Implementation, build and configuration details owned by that package | The owning package changes |

Historical records and design notes are not proof of current readiness and are
not bundled as handbook chapters. Published capabilities must refer to the
relevant evidence, not simply to a design document or an old successful run.

Keep one explanation per responsibility and link to it. Keep generated API
inventories in sync with source. Store new run output in `artifacts/` or an
external evidence store, and preserve active field records when simplifying.
The [planning gallery](assets/planning/README.md) retains reviewed PDF figure
exports alongside their previews. These published illustrations and the papers
in `research/papers/` are the PDF collections maintained under `docs/`.
Handbook snapshots and unreviewed run exports belong in generated artifacts.
Generated evidence kept only in local `build/` or `artifacts/` directories is
listed as a code-formatted repository path, rather than a checkout-local link.
Retain its provenance and evidence limits; a path alone is not published proof.
Use [evidence levels](testing.md#evidence-levels) for every capability claim.
Before publishing, review the local material and obtain confirmation as recorded
in [the current delivery rules](roadmap.md#current-delivery-cycle).
