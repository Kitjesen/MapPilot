# LingTu Assembly

`src/lingtu/assembly` is the product assembly layer. It chooses which Modules
form a LingTu application graph and passes that declaration to
`runtime.blueprint.Blueprint`.

This package is deliberately separate from `src/runtime/blueprint.py`:

```text
runtime.blueprint       = generic Module graph mechanism
lingtu.assembly         = LingTu product recipes
runtime.graph.RuntimePlan = native process lifecycle plan
lingtu.launcher         = external RuntimePlan executor
```

## Main Path

```text
resolved profile
  -> profile_builder.compile_product()
       -> products/ -> stacks/ + wires/ -> Blueprint
       -> Runtime Graph -> RuntimePlan
  -> Product(Blueprint, RuntimePlan)
       -> Product.build() -> Blueprint.build() -> SystemHandle
       -> Launcher.apply() -> native processes
```

| Path | Owns |
| --- | --- |
| `profile_builder.py` | Compile one resolved profile and endpoint into one Product containing Blueprint + RuntimePlan. |
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
hardware driver are selected by product/endpoint contracts and executed through
`lingtu.launcher`. `RuntimePlan` itself is data and never executes a command.
Assembly may add typed DDS adapters to the Module graph, but it does not own
those process lifecycles.

## Dependency Rule

Dependencies point in one direction:

```text
lingtu.assembly -> runtime + domain Module APIs
runtime core    -X-> lingtu.assembly
domain code     -X-> lingtu.assembly
```

Only product entrypoints, CLI/deployment tooling, graph tests, and this package
should import `lingtu.assembly`.
