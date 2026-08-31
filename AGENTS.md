# AGENTS.md

This file contains only repository-wide instructions that must stay in context
while changing LingTu. Operational commands, component catalogues, and detailed
architecture live in the linked documentation.

## HERO Anti-OverDefense

=== 范围约束(约束你提议什么修法,不约束你找什么)===
凡是这里真的有问题,都要报——包括听起来罕见但本项目确实会产生的情况。
然后把修法收在范围内:
1. 这不是一篇安全攻防论文。可以校验,禁止过度防御。除非本项目另有说明,默认操作者是
   自己机器上的合作者;如果它真有对手,它会写明,以那个范围为准。
2. 不要加哈希/校验和/指纹,除非它替代了一个实质上更贵的操作,并且结果会改变下一步做什么。
3. 禁止防御性脚手架:不为这里不会发生的情况加 feature flag、迁移框架、兼容层、包装层。
4. 禁止钻牛角尖:冷门编码、符号链接竞态、RTL 文本、毫秒级竞态一律不在范围内,
   除非该情况经由本项目**受支持的用法**可达——它的文档示例、它公开的接口、它真实的
   数据。可达即可,不需要你复现出来;但"理论上构造得出"不算。
5. 该判断的地方就判断,不要换成评分表、检查清单,或对已经定论的东西再跑一遍校验。
6. 以上都不覆盖用户、本项目自己的约定、或更高优先级规则明确要求的安全、迁移、校验与
   审阅。那些是被要求的,是活儿本身,不算范围外。
已经见过的形状,供你校准。是例子不是清单——一个真问题不会因为"长得像其中一条"就被驳回:
  H  为了比对两个表格的差异,给每一行都算哈希——直接比单元格就能回答
  H  写下一堆校验和文件,而没有任何代码会去读它们
  E  给一个没有用户、没有部署的应用做账号安全加固
  R  用一整夜对自己的补丁反复审计,而功能一行没写
  R  一个对任何提交都给不通过的审阅者
  O  一层守卫的理由是上一层守卫,而不是需求
另有两种长得像上面、但不是的。这些要报:
  ✓  用摘要比对来跳过重读一个你已经有的大文件
  ✓  本项目自己的文档示例就会产生的那种"听起来罕见"的输入
跑任何检查之前先回答:这次运行会检测出什么具体的失败?真出现了我下一步会做什么不同的事?
答不上来就别跑。
对的就说对。不要为了交差硬找问题。

## Project Boundary

LingTu is an autonomous-navigation system for quadruped robots. The field
target is the S100P/RDK X5 (`aarch64`, Ubuntu, native CycloneDDS). Python owns
the Host framework and semantic/API layer; C++ owns field LiDAR, SLAM, maps,
terrain, navigation, and driver hot paths.

Use the following terms with exactly one meaning:

| Term | Owns | Does not own |
| --- | --- | --- |
| `env` | The outer runtime environment. Public values are exactly `real` and `sim`. | Product behavior or an endpoint. |
| `Product` | Immutable, env-independent operating-mode declaration. | Deployment targets or runtime side effects. |
| `RunPlan` | Resolved Product inside one env, including concrete processes and Host configuration. | User intent or mutable runtime state. |
| `ProductControl` | The complete lifecycle transaction inside one fixed env. | Env switching or domain algorithms. |
| `Host` | Managed Python application containing Gateway, Agent, adapters, and selected Modules. | Native process ownership. |
| `Blueprint` | Construction and wiring of Modules inside one Host. | Product switching, systemd, or native endpoint lifecycle. |
| `Module` | Typed in-process Host runtime unit. | Cross-process lifecycle management. |
| `DDS` | Native typed cross-process data plane. | Product policy or process lifecycle. |
| `RobotConfig` | Static physical robot, device, and calibration data used by `real`. | Runtime mode selection. |
| `Endpoint` | Concrete HTTP, DDS, or native-service access point. | An env or Product identity. |

## Product And Process Ownership

- `lingtu.assembly` resolves Product + env without side effects. Resolve once;
  do not independently resolve the same Product in CLI, systemd, Gateway, or
  Host.
- `ProductControl` is the only public Product lifecycle entry. It publishes one
  exact RunPlan, owns the mutation lock/current record/session/readiness, and
  routes by the resolved execution contract:
  - `real + systemd` -> `real_switch` -> `SystemdRunner`.
  - `sim + subprocess` -> `sim_switch` -> direct-child process management.
- One Product run is identified only by `product_session_id`. A RunPlan path is
  an internal file location, while process `launch_id` and protocol `boot_id`
  values stay private to their owning runtime boundary.
- Real and simulation switches share only `SwitchRequest`, `SwitchReport`,
  `SwitchFailed`, `MapIdentity`, and RunPlan/ProcessReport semantics. They do
  not share rollback implementation, process operations, session/map
  transports, or runtime identity evidence. Do not introduce `BaseSwitch` or a
  generic phase engine.
- Field lifecycle commands must call `python -m lingtu.control` directly or use
  the thin `scripts/lingtu` adapter. Do not put Product policy, ordering, map
  activation, or readiness loops in Bash or Gateway.
- Blueprint is active inside every Host, but it is not the Product process
  orchestrator. Do not add Product-specific Host lifecycle implementations
  beside Blueprint.

## Dependency Direction

High layers may depend on lower layers only. Waypoint/path dispatch is message
flow, not a package dependency.

```text
All Modules -> core/ (Module, In/Out, Registry, utils, msgs)

nav/        must not import perception/, decision/, drivers/, gateway/
perception/ must not import nav/, decision/, drivers/, gateway/
decision/   consumes perception through runtime messages, not nav/gateway imports
drivers/    must not import nav/ or semantic code except lazy registration
gateway/    must not import nav/, semantic/, or drivers/
```

Use existing factories, registries, and `runtime.registry` before direct backend
imports. Machine-enforced ownership lives in
[`config/architecture_layers.yaml`](config/architecture_layers.yaml).

## Where Changes Belong

| Change | Source of truth |
| --- | --- |
| Product capabilities, logical roles, topics | `config/runtime_graph/products/*.yaml` |
| Real or sim implementation mapping | `config/runtime_graph/envs/*.yaml` |
| Product resolution and immutable artifact | `src/lingtu/run_plan.py`, `src/lingtu/assembly/` |
| Public lifecycle routing and locking | `src/lingtu/control.py` |
| Shared switch values | `src/lingtu/switch_contracts.py` |
| Real/systemd transaction and rollback | `src/lingtu/real/switch.py` |
| Sim/subprocess transaction and rollback | `src/lingtu/sim/switch.py`, `src/lingtu/sim/journal.py` |
| systemd process execution/readiness | `src/lingtu/real/systemd.py` |
| Sim direct-child execution | `src/lingtu/sim/process.py`, `src/lingtu/sim/identity.py` |
| Host Module graph | `src/lingtu/assembly/`, `src/runtime/blueprint.py` |
| Physical devices and calibration | `config/robots/<vendor>/<model>/robot.yaml`, `config/devices.yaml` |
| Native algorithms/services | Their owning C++ tree under `src/` |

## Runtime Safety Boundaries

- Product sources are ROS-free. Use native typed DDS at process boundaries and
  direct calls inside one C++ service. ROS surfaces are explicit compatibility
  paths only.
- In `real`, native navigation owns final motion arbitration. Only
  `lingtu-driver` may forward the checked command to Brainstem. Do not bypass
  planning, safety, or the declared command owner.
- Field `mapd` owns live map state, standalone traversability owns
  `/nav/traversability`, and save-time pruning owns persistent-map cleanup.
  Python map layers are development/simulation fallbacks and must not compete
  with field owners.
- The real `nav` Product consumes native C++ SLAM status through its adapter.
  Do not switch it to a managed localizer without intentionally changing
  process and sensor ownership.
- A local test does not prove simulation; simulation does not prove the field;
  a running process does not prove readiness for motion.
- Preserve calibration and bounded tuning knobs at hardware boundaries. S100P
  has no CUDA; keep `aarch64` performance and native-memory behavior in mind.

## Working Rules

- Prefer deletion and existing utilities over new abstractions or dependencies.
- Keep diffs small, reversible, and behavior-preserving unless behavior change
  is explicitly requested.
- Do not add dependencies without explicit approval.
- Preserve unrelated user or teammate changes in a dirty worktree.
- New Python comments are English; legacy Chinese comments may remain.
- Treat trust-boundary validation, data-loss prevention, motion safety, and
  explicitly required migration/compatibility work as required work, not
  over-defense.
- Critical shared surfaces require extra care: runtime Module/Blueprint/stream/
  registry, Product compilation and control, navigation safety, robot/device
  configuration, and explicit full-stack wiring.

## Verification

- Before running a check, name the concrete failure it can reveal and what
  would change if it fails.
- Run the narrowest test that proves the modified behavior, then only the
  directly affected lint/type/build checks.
- Do not claim hardware behavior without S100P evidence. Keep local contract,
  native MuJoCo, field no-motion, and supervised field-motion evidence distinct.
- Known repository-wide baseline failures do not excuse new diagnostics on the
  touched surface; report both separately.

## Current References

- Architecture index and sources of truth:
  [`docs/architecture/README.md`](docs/architecture/README.md)
- Current document/evidence status: [`docs/CURRENT.md`](docs/CURRENT.md)
- Product/runtime-graph contract:
  [`config/runtime_graph/README.md`](config/runtime_graph/README.md)
- ProductControl package guide: [`src/lingtu/README.md`](src/lingtu/README.md)
- Products, environments, and lifecycle commands: [`docs/QUICKSTART.md`](docs/QUICKSTART.md)
- Build and platform instructions:
  [`docs/01-getting-started/BUILD_GUIDE.md`](docs/01-getting-started/BUILD_GUIDE.md)
- Robot-side operations: [`docs/04-deployment/lingtu_cli.md`](docs/04-deployment/lingtu_cli.md)
- Validation levels and gates: [`docs/07-testing/README.md`](docs/07-testing/README.md)
- Simulation architecture: [`sim/ARCHITECTURE.md`](sim/ARCHITECTURE.md)
- External/Gateway integration boundary:
  [`docs/09-integrations/README.md`](docs/09-integrations/README.md)
