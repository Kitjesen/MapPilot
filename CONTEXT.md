# LingTu Runtime

LingTu separates the outer `env` from the Product it runs. Assembly resolves
one Product inside one fixed `env` into a fingerprinted RunPlan; that RunPlan is
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
The immutable, fingerprinted resolution of one Product inside one `env`,
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

**Profile**:
A local-development configuration input for building a Host graph. It is not a
field Product, env, endpoint identity, or ownership category.
_Avoid_: using Profile for field runtime ownership
