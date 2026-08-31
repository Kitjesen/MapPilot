# config/ - LingTu configuration

`config/` stores committed defaults and runtime contracts shared by development,
simulation, packaging, and field robots. Do not store secrets, one-off evidence,
local logs, generated maps, or site-private overrides here.

## Current runtime facts

- Product field transport is native typed DDS on CycloneDDS domain `0` unless a
  deployment override explicitly changes `LINGTU_DDS_DOMAIN_ID` for every
  participant.
- Gateway HTTP/WS/SSE listens on port `5050`; MCP JSON-RPC listens on port
  `8090`.
- The `real` environment uses native field processes. Read-only diagnostics use
  ProductControl `status`; they are not a separate runtime identity.
- `lt-driver.service` is the unique LingTu field speed exit. It consumes
  `rt/nav/cmd_vel` and loads the adapter selected by `driver.backend`.
- Go2 uses `driver.network_interface` for SDK2 discovery. Doso uses
  `driver.target` and the TLS file paths compiled into the Product run.

## Important files

| File | Purpose |
| --- | --- |
| `robots/<vendor>/<model>/robot.yaml` | Physical robot geometry, calibration, and control limits. |
| `robots/` | Company/model/function index for supported robot models; it does not duplicate the active physical config. |
| `devices.yaml` | Read-only hardware inventory plus camera startup configuration; native Product services own device dataflow. |
| `runtime_graph/` | Product declarations plus canonical topics and `real`/`sim` process and DDS/SHM boundary mappings. |
| `go2rtc.yaml` | go2rtc WebRTC/WHEP camera stream template. |
| `cyclonedds.xml` | CycloneDDS configuration used by native DDS participants. |
| `semantic_scoring.yaml`, `semantic_taxonomy.json` | Active semantic scoring and taxonomy data. |
| `architecture_layers.yaml` | Architecture ownership and dependency boundaries. |

## Ports and remote control

The selected model's `robot.yaml` contains its physical configuration. Field
runtime values may be overridden by systemd environment files.

```yaml
gateway:
  port: 5050
  mcp_port: 8090
brainstem:
  host: "127.0.0.1"   # development/local placeholder only
  port: 13145
```

For Doso field deployment, configure the remote Brainstem endpoint and TLS file
paths in `RobotConfig`; ProductControl writes them into the transient Product
session consumed by `lt-driver.service`:

```yaml
driver:
  backend: doso
  target: REMOTE_BRAINSTEM_IP:13145
  tls_ca_file: /opt/lingtu/config/tls/brainstem-ca.crt
  tls_cert_file: /opt/lingtu/config/tls/lingtu-driver.crt
  tls_key_file: /opt/lingtu/config/tls/lingtu-driver.key
```

Do not commit private TLS material.

## Calibration write path

Use the calibration tools instead of hand-editing geometry after a calibration
session:

1. Camera intrinsics -> `camera.fx/fy/cx/cy + dist_*`.
2. IMU noise -> selected robot `mid360_fastlio2.yaml` `na/ng/nba/nbg`.
3. LiDAR-IMU extrinsics -> selected robot `mid360_fastlio2.yaml` `r_il/t_il` and time offset.
4. Camera-LiDAR extrinsics -> `camera.position_* + roll/pitch/yaw`.

Verification:

```bash
python tools/calibration/verify.py
python -m lingtu.control status --robot unitree/go2 --env real --json
```

## Editing rules

- Keep field-private IPs, API keys, tokens, passwords, TLS keys, local map paths,
  and runtime evidence out of `config/`.
- Keep `runtime_graph/topics.yaml` transport-neutral. Detailed runtime formats,
  frames, and aliases are owned by `runtime.runtime_interface`.
- Change DDS domain/interface settings only as a deployment-wide decision; all
  native DDS participants must agree.
- Prefer ignored environment files or deployment-specific systemd drop-ins for
  site overrides.
- After changing committed defaults, run the narrowest relevant config or
  route-contract test before deploying.
