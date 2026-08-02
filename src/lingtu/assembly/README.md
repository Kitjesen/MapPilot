# LingTu Assembly

`src/lingtu/assembly` is the product assembly layer. It chooses which Modules
form a LingTu application graph and passes that declaration to
`runtime.blueprint.Blueprint`.

This package is deliberately separate from `src/runtime/blueprint.py`:

```text
runtime.blueprint       = generic Module graph mechanism
lingtu.assembly         = LingTu product recipes
runtime.graph.ProcessSpec = env-resolved process data
lingtu.control          = Product operation boundary
lingtu.systemd          = internal ProductControl process executor
```

## Main Path

```text
env + Product
  -> profile_builder.compile_run_plan()
       -> products/ -> stacks/ + wires/ -> Blueprint
       -> Runtime Graph env -> concrete ProcessSpec values
  -> RunPlan(identity, launch, host, checks)
       -> ProductControl -> fingerprinted RunPlan + transient session
            -> Host verifies RunPlan -> Blueprint.build() -> SystemHandle
            -> internal SystemdRunner.apply(RunPlan) -> native processes
```

| Path | Owns |
| --- | --- |
| `profile_builder.py` | Build a local Profile graph or resolve one Product+env into a RunPlan. |
| `products/` | Top-level product variants and mode composition. |
| `stacks/` | Small reusable groups of Module declarations and constructor config. |
| `wires/` | Explicit cross-stack port connections. |
| `graph.py` | Static product graph inspection and snapshots. |
| `full_stack_wiring.py` | Collector for critical explicit full-product wires. |

## Allowed

- choose Module classes;
- pass constructor configuration;
- assign stable runtime aliases;
- declare explicit wires and route contracts;
- select adapters at known product boundaries;
- export product graph metadata.

## Forbidden

- map building, planning, SLAM, filtering, or control algorithms;
- HTTP route implementations;
- direct map artifact mutation;
- opening hardware or network sockets during graph declaration;
- installing or supervising native services;
- hiding product policy inside Module constructors.

Native processes such as LiDAR, SLAM, traversability, navigation, and the
hardware driver are selected by Product+env contracts and executed through
`lingtu.control.ProductControl` and its internal `lingtu.systemd` runner. Assembly may add typed
DDS adapters to the Module graph, but only ProductControl performs process lifecycle
side effects.

## Dependency Rule

Dependencies point in one direction:

```text
lingtu.assembly -> runtime + domain Module APIs
runtime core    -X-> lingtu.assembly
domain code     -X-> lingtu.assembly
```

Only product entrypoints, CLI/deployment tooling, graph tests, and this package
should import `lingtu.assembly`.
