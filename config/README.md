# Configuration

`config/` contains committed inputs that are consumed by LingTu runtime,
deployment, simulation, or validation code. It is not a general storage area:
secrets, site overrides, generated maps, logs, and evidence belong elsewhere.

## Mental model

Read configuration in this order:

1. `robots/<vendor>/<model>/` describes one physical robot and its sensors.
2. `runtime_graph/products/` declares what the selected Product needs.
3. `runtime_graph/envs/` binds those needs to exactly one `real` or `sim` Env.
4. `lingtu.assembly` resolves the inputs once into an immutable RunPlan.
5. `acceptance/` describes how to qualify a built runtime; it is not part of
   Product resolution and does not itself constitute field evidence.

```text
config/
├── acceptance/
│   └── mujoco/               # Local MuJoCo acceptance manifests
├── robots/
│   └── <vendor>/<model>/     # Model, physical config, and sensor calibration
├── runtime_graph/
│   ├── products/             # Env-independent operating modes
│   ├── envs/                 # real/sim implementation bindings
│   └── topics.yaml           # Cross-process topic contract
├── architecture_layers.yaml  # Machine-enforced source/import ownership
├── cyclonedds.xml            # Native DDS participant defaults
├── devices.yaml              # Hardware inventory and camera startup input
├── go2rtc.yaml               # Browser camera-stream sidecar template
└── semantics/
    ├── scoring.yaml           # Decision scoring weights
    └── taxonomy.json          # Stable semantic class identifiers
```

## Ownership

| Path | Owns | Does not own |
| --- | --- | --- |
| `robots/<vendor>/<model>/model.yaml` | Model identity and paths to model-specific sensor inputs. | Product or Env selection. |
| `robots/<vendor>/<model>/robot.yaml` | Physical geometry, calibrated transforms, device endpoints, and hardware safety limits. | Gateway ports, planner policy, or Product lifecycle. |
| `robots/<vendor>/<model>/sensors/` | Sensor and estimator calibration for that installation. | Product support policy. |
| `runtime_graph/products/*.yaml` | Immutable Product capabilities, topics, and logical process roles. | Hardware implementation or launch side effects. |
| `runtime_graph/envs/*.yaml` | Concrete `real`/`sim` processes, transports, and backend bindings. | Product intent. |
| `../src/message/topics/*.yaml` | Canonical logical topic, DDS type, QoS profile, and ownership by domain. | Generated Python/C++ tables or per-site tuning. |
| `acceptance/mujoco/*.json` | Reproducible local qualification orchestration and thresholds. | Runtime configuration or S100P field claims. |
| `semantics/` | Shared scoring weights and stable class taxonomy. | Detector model files or Product policy. |
| Root files | Cross-cutting configuration with one direct consumer family. | Component-private configuration. |

Gateway HTTP and MCP ports are Host/deployment values, not robot properties.
The code defaults are `5050` and `8090`; field overrides use the deployed
environment. Doso's Brainstem endpoint remains under `driver.target` because it
selects the physical motion adapter endpoint:

```yaml
driver:
  backend: doso
  target: REMOTE_BRAINSTEM_IP:13145
  tls_ca_file: /opt/lingtu/config/tls/brainstem-ca.crt
  tls_cert_file: /opt/lingtu/config/tls/lingtu-driver.crt
  tls_key_file: /opt/lingtu/config/tls/lingtu-driver.key
```

Do not commit private TLS material.

## Naming

- Use lowercase `snake_case` for repository-owned directory and file names.
- Use plural directory names for collections (`robots`, `products`, `envs`).
- Let the parent directory carry context. For example,
  `acceptance/mujoco/navigation.json` does not repeat `mujoco`, `native`, or
  `acceptance` in its file name.
- Keep public identifiers stable: Product names, `real`/`sim`, schema versions,
  DDS topics, vendor/model identifiers, and systemd units are contracts rather
  than cosmetic file names.
- Preserve tool-defined conventional names such as `README.md` and
  `cyclonedds.xml`.
- A path rename must update every active consumer in the same change. Historical
  evidence may retain the command or path that produced it.

## Calibration write path

Use the calibration tools instead of hand-editing measured values after a
calibration session:

1. Camera intrinsics -> selected `robot.yaml` camera fields.
2. IMU noise -> selected `sensors/mid360_fastlio2.yaml` `na/ng/nba/nbg`.
3. LiDAR-IMU extrinsics -> selected sensor file `r_il/t_il` and time offset.
4. Camera-LiDAR extrinsics -> selected `robot.yaml` camera transform.

Verification:

```bash
python tools/calibration/verify.py
python -m lingtu.control status --robot unitree/go2 --env real --json
```

## Editing rules

- Keep site-specific addresses that are not model defaults, API keys, tokens,
  passwords, TLS keys, local map paths, and runtime evidence out of committed
  configuration.
- Define every logical Topic once in `../src/message/topics/*.yaml`. Generate the
  Python and C++ views with `python tools/generate_topic_contracts.py`.
- Change a DDS domain or interface only as a deployment-wide decision; every
  participant must agree.
- Use ignored environment files or systemd drop-ins for site-local overrides.
- Do not add a field until a named consumer reads it. Delete fields whose
  supported consumer has been retired instead of preserving false interfaces.
- Run the narrowest parser or contract test that can detect a mistake in the
  file being changed.
