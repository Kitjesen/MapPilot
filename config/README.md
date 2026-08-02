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
- The `real` environment uses native field processes; the local `lite` Profile
  uses its `thunder_lite` ProfileAdapter and the in-process driver boundary.
- `lingtu-driver` is the unique speed exit. It consumes `rt/nav/cmd_vel` and
  calls a remote Brainstem gRPC endpoint configured by
  `/opt/lingtu/config/brainstem.env`.
- Brainstem loopback (`127.0.0.1`) is not valid for field deployment; use a
  protected deployment environment file for the remote host/TLS settings.

## Important files

| File | Purpose |
| --- | --- |
| `robot_config.yaml` | Physical robot geometry, calibration, control limits, and default ports. |
| `devices.yaml` | Hardware registry for camera, LiDAR, IMU, GNSS, and control devices. |
| `endpoints.yaml` | Concrete HTTP, DDS, native-service, and stream access points. |
| `topic_contract.yaml` | Canonical runtime stream/topic contract shared by ModulePorts, Gateway, and endpoint adapters. |
| `runtime_graph/` | Product declarations plus `real`/`sim` process and DDS/SHM boundary mappings. |
| `go2rtc.yaml` | go2rtc WebRTC/WHEP camera stream template. |
| `cyclonedds.xml` | CycloneDDS configuration used by native DDS participants. |
| `fastdds_no_shm.xml` | FastDDS no-shared-memory compatibility profile for Docker or legacy tests. |
| `dufomap.toml` | Legacy DUFOMap map-cleaning configuration kept for comparison. |
| `decision.yaml`, `perception.yaml`, `semantic_*` | Semantic navigation and perception defaults. |

## Ports and remote control

`robot_config.yaml` contains development defaults for service ports. Field
runtime values may be overridden by systemd environment files.

```yaml
gateway:
  port: 5050
  mcp_port: 8090
brainstem:
  host: "127.0.0.1"   # development/local placeholder only
  port: 13145
```

For Thunder field deployment, the driver installer writes the real remote
Brainstem settings to `/opt/lingtu/config/brainstem.env`:

```bash
LINGTU_BRAINSTEM_HOST=REMOTE_BRAINSTEM_IP
LINGTU_BRAINSTEM_PORT=13145
LINGTU_BRAINSTEM_TLS_CA_FILE=/opt/lingtu/config/tls/brainstem-ca.crt
LINGTU_BRAINSTEM_TLS_CERT_FILE=/opt/lingtu/config/tls/lingtu-driver.crt
LINGTU_BRAINSTEM_TLS_KEY_FILE=/opt/lingtu/config/tls/lingtu-driver.key
```

Do not commit that environment file or private TLS material.

## Calibration write path

Use the calibration tools instead of hand-editing geometry after a calibration
session:

1. Camera intrinsics -> `camera.fx/fy/cx/cy + dist_*`.
2. IMU noise -> `pointlio.yaml` `na/ng/nba/nbg`.
3. LiDAR-IMU extrinsics -> `lidar.offset_* + roll/pitch/yaw`.
4. Camera-LiDAR extrinsics -> `camera.position_* + roll/pitch/yaw`.

Verification:

```bash
python calibration/verify.py
python lingtu.py doctor
```

## Editing rules

- Keep field-private IPs, API keys, tokens, passwords, TLS keys, local map paths,
  and runtime evidence out of `config/`.
- Keep `topic_contract.yaml` transport-neutral. Canonical stream tokens are not
  ROS-only topic names.
- Change DDS domain/interface settings only as a deployment-wide decision; all
  native DDS participants must agree.
- Prefer ignored environment files or deployment-specific systemd drop-ins for
  site overrides.
- After changing committed defaults, run the narrowest relevant config,
  route-contract, or profile test before deploying.
