# LingTu Runtime

LingTu separates the outer `env` from the Product it runs. Assembly resolves
one Product inside one fixed `env` into a RunPlan; that RunPlan is
the only executable artifact consumed by runtime owners.

## Language

**env**:
The outer runtime environment. Public values are exactly `real` and `sim`; a
simulation backend is internal env configuration.
_Avoid_: field endpoint, target endpoint

**Product**:
An immutable, env-independent operating-mode declaration: Host graph, logical
native roles, topics, and capabilities.
_Avoid_: field Profile, product profile, deployment target

**RunPlan**:
The immutable resolution of one Product inside one `env`,
including concrete processes and Host configuration. ProductControl publishes
one RunPlan; its internal SystemdRunner and the Host consume that exact artifact.
_Avoid_: ProductManifest, combined names such as `map@real`

**ProductControl**:
The owner of switch, quiesce, restart, stop, transient session, readiness,
rollback, and current RunPlan commit inside its fixed `env`.
_Avoid_: env switching, domain algorithms, Bash/Gateway product policy

**Blueprint**:
The builder and wire owner for in-process Host Modules. It is active inside
every Python Host, but it is not the product process orchestrator.
_Avoid_: Product-specific Host lifecycle, systemd ownership, native endpoints

**RobotConfig**:
Static physical robot, device, and calibration data referenced internally by the
`real` env. It is not a runtime selector or operating mode.
_Avoid_: treating physical configuration as a Product, env, or Profile selector

**Endpoint**:
A concrete HTTP, DDS, or native-service communication access point. It is not an
env and does not select a Product.
_Avoid_: using endpoint to mean real or simulated deployment or Product identity

**Navigation runtime**:
The authoritative in-process owner of one native `navd` navigation lifecycle,
including goal, recovery, stop, rolling-segment, inspection, and terminal state.
_Avoid_: a separate state-machine process, DDS-owned state, status projection as state ownership

**Global planning**:
Map-scale search from an admitted start and goal to a map-frame route. It does
not inspect live near-field motion on every control tick or produce velocity.
_Avoid_: global navigation package, command controller

**Local planning**:
Short-horizon selection from current pose, route target, obstacles, and terrain
to a local path or a verified stop/recovery result. It does not track the path.
_Avoid_: path following, mission lifecycle

**Path following**:
Geometric tracking of an already-selected local path into pre-safety body-frame
velocity intent. It does not search obstacles or choose a path.
_Avoid_: local planner, final motion authority

**Motion execution**:
Stateful progression of an admitted route or assisted intent through local
planning and path following. It does not admit goals, compute global routes, or
publish the final robot command.
_Avoid_: navigation runtime, global planner, driver

**Profile**:
A local-development configuration input for building a Host graph. It is not a
field Product, env, endpoint identity, or ownership category.
_Avoid_: using Profile for field runtime ownership

## Simulation Platform

**SimCatalog**:
The read-only view of available simulation packages and
their dependency and qualification state.
_Avoid_: package installer, runtime launcher, second resolver

**SessionIntent**:
A user or application request selecting a world, robot instances, scenario,
and required simulation facets before validation and compilation.
_Avoid_: SessionSpec, RunPlan, executable session

**SessionSpec**:
The validated deterministic source declaration consumed once by the simulation
resolver to compile a session.
_Avoid_: SessionIntent, mutable runtime state, allocation

**ResolvedSessionBundle**:
The immutable, validated set of compiled simulation plans sharing one
`session_id` across simulation runtime owners.
_Avoid_: source package catalog, RunAllocation, runtime state

**RunAllocation**:
The ephemeral resources assigned to one execution of a ResolvedSessionBundle,
such as ports, shared-memory names, process IDs, and log paths.
_Avoid_: session identity, package metadata, deterministic content

**QualificationRecord**:
Evidence stating which declared package-version capabilities have passed
defined checks; absence or a different package version means unqualified, not
unsupported.
_Avoid_: self-declared capability, package availability, runtime readiness
