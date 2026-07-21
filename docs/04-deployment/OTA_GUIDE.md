# OTA Guide — LingTu Deployment Updates

## 1. Design contract

OTA updates must satisfy three invariants. Violating any one blocks release:

| Invariant                       | Meaning                                                       | Mechanism                                  |
|---------------------------------|---------------------------------------------------------------|--------------------------------------------|
| Verifiable authenticity         | Robot confirms artifact came from the official release        | Ed25519 signed `manifest.json`             |
| Atomic install + crash recovery | Power loss / Wi-Fi drop / disk full never bricks the robot    | Symlink swap + transaction log + auto-rollback |
| Safety-state gating             | Updates affecting motion don't apply while the robot is moving | `safety_level` (hot/warm/cold) gating      |

## 2. Architecture

The current OTA path is a **standalone agent** running as `ota-agent.service` on the
robot. There is no in-process OTA gRPC service; the previous `data_service.cpp` /
`grpc_gateway` implementation has been removed.

```
+------------------+    HTTPS    +------------------+   local actions   +------------------+
| Release Server   |<------------|  ota-agent       |------------------>| /opt/lingtu/     |
| (GitHub or       |             |  (Python daemon  |                   |   releases/      |
|  internal HTTP)  |             |   on the robot)  |                   |   current ->     |
|                  |             |                  |                   |                  |
| manifest.json    |             | 1. poll          |                   | 1. stage release |
| (Ed25519 signed) |             | 2. verify sig    |                   | 2. atomic swap   |
| release tarballs |             | 3. verify sha256 |                   |    symlink       |
+------------------+             | 4. stage         |                   | 3. systemctl     |
                                 | 5. swap + restart|                   |    restart lingtu|
                                 +------------------+                   +------------------+
```

Lifecycle:

```
1. Check       agent polls server for newer release vs ota_state.json
2. Pre-flight  disk space (>=2x release size), battery, hw_compat,
               dependency versions, safety mode, stale transaction logs
3. Safety gate hot  -> install while running
               warm -> ModeManager -> IDLE, install, resume
               cold -> robot must sit + disable motors/locomotion first
4. Download    direct from server -> /opt/lingtu/staging/<version>/
5. Verify      Ed25519 manifest signature; SHA256 every artifact
6. Install     write transaction log, swap /opt/lingtu/current, restart lingtu.service
7. Health      curl http://127.0.0.1:5050/api/v1/health on the robot (must report ok within 60 s)
8. Recover     on next boot, any leftover txn_*.json triggers auto-rollback
```

## 3. Artifact safety levels

Every artifact in the manifest declares `safety_level`. The agent enforces the gate
before applying:

| Level | Required state                          | Typical artifacts                |
|-------|-----------------------------------------|----------------------------------|
| hot   | none — install while running            | maps (PCD), config (YAML)        |
| warm  | mission paused, controllers in IDLE     | ONNX models, BPU `.hbm`          |
| cold  | sit + motors disabled + maintenance hold | Python source / `.deb` / firmware |

`lingtu` itself ships as a cold artifact today: the agent stops
`lingtu.service`, swaps `/opt/lingtu/current`, then starts it again. Native
field services such as `lingtu-livox-dds`, `lingtu-slam-dds`,
`lingtu-nav-dds`, and `lingtu-driver` are managed explicitly by the deployment
or operations procedure for the selected release.

## 4. Manifest

Schema v2 example (signed):

```json
{
  "schema_version": "2",
  "release_version": "v2.0.0",
  "release_date": "2026-04-25T12:00:00Z",
  "min_system_version": "1.0.0",
  "channel": "stable",
  "signature": "a1b2c3d4...",
  "public_key_id": "ota-signing-key-01",
  "artifacts": [
    {
      "name": "lingtu",
      "category": "package",
      "version": "2.0.0",
      "filename": "lingtu_v2.0.0.tar.gz",
      "sha256": "f6e5d4c3b2a1...",
      "target_path": "/opt/lingtu/releases/v2.0.0",
      "apply_action": "swap_symlink",
      "safety_level": "cold",
      "owner_module": "system",
      "rollback_safe": true,
      "min_battery_percent": 20,
      "dependencies": []
    },
    {
      "name": "campus_map",
      "category": "map",
      "version": "1.0.0",
      "filename": "campus_map.pcd",
      "sha256": "1234abcd...",
      "target_path": "/home/sunrise/data/nova/maps/campus_map.pcd",
      "apply_action": "copy_only",
      "safety_level": "hot",
      "owner_module": "navigation",
      "rollback_safe": true,
      "min_battery_percent": 0,
      "dependencies": []
    }
  ]
}
```

Field summary:

| Field                            | Required | Notes                                                   |
|----------------------------------|----------|---------------------------------------------------------|
| `schema_version`                 | yes      | `"2"`                                                   |
| `release_version`                | yes      | semver                                                  |
| `signature`                      | yes      | Ed25519 hex over canonical JSON minus `signature` field |
| `public_key_id`                  | yes      | identifies which key to use; supports rotation          |
| `channel`                        | no       | `stable` / `canary` / `dev`                             |
| `artifacts[].safety_level`       | yes      | `hot` / `warm` / `cold`                                 |
| `artifacts[].apply_action`       | yes      | `copy_only` / `swap_symlink` / `reload_model` / `install_deb` / `install_script` |
| `artifacts[].owner_module`       | yes      | `system` / `navigation` / `brain` / `config_service` / `mcu` |
| `artifacts[].dependencies`       | no       | `[{artifact_name, min_version, max_version}]`           |
| `artifacts[].sha256`             | yes      | hex                                                     |
| `artifacts[].min_battery_percent`| no       | default 0                                               |
| `artifacts[].rollback_safe`      | no       | default true; affects whether agent retains backup      |

## 5. Signing

### Generate keys (once)

```bash
python3 scripts/ota/generate_manifest.py --generate-keys --key-dir ./keys/
# keys/ota_private.pem  -- keep secret, used only by CI
# keys/ota_public.pem   -- shipped to every robot
```

### Deploy public key to robot

```bash
scp keys/ota_public.pem sunrise@<robot>:/opt/lingtu/nav/ota/ota_public.pem
```

### CI signing step

```yaml
- name: Sign release manifest
  run: |
    python3 scripts/ota/generate_manifest.py \
      --version ${{ github.ref_name }} \
      --artifacts-dir ./dist/ \
      --signing-key ${{ secrets.OTA_SIGNING_KEY_PATH }} \
      --key-id ota-signing-key-01 \
      --channel stable \
      --output ./dist/manifest.json
```

The agent rejects unsigned manifests when `LINGTU_OTA_REQUIRE_SIGNATURE=1` (the
production setting). With it unset, an unsigned manifest is logged with a `WARN` and
processed — useful for development.

## 6. Transaction log + rollback

Every install writes `/opt/lingtu/nav/ota/backup/txn_<artifact>.json` before
touching files:

```json
{
  "artifact": "lingtu",
  "version": "2.0.0",
  "status": "installing",
  "staged_path": "/opt/lingtu/staging/v2.0.0",
  "target_path": "/opt/lingtu/releases/v2.0.0",
  "previous_symlink": "/opt/lingtu/releases/v1.9.3",
  "started_at": "2026-04-25T12:00:00Z"
}
```

On install success the file is deleted. On failure it's retained with
`status=failed`. On next agent start, any `installing` entry triggers automatic
rollback (symlink reverts to `previous_symlink`).

Manual rollback:

```bash
ssh sunrise@<robot>
ls /opt/lingtu/releases/
sudo ln -sfn /opt/lingtu/releases/v1.9.3 /opt/lingtu/current
sudo systemctl restart lingtu
```

## 7. Releasing

### Manual

```bash
# 1. Build artifact
make build
tar -czf dist/lingtu_v2.0.0.tar.gz install/ src/ lingtu.py config/ launch/

# 2. SHA256
sha256sum dist/*

# 3. Manifest
python3 scripts/ota/generate_manifest.py \
    --version v2.0.0 \
    --artifacts-dir ./dist/ \
    --signing-key ./keys/ota_private.pem \
    --channel stable \
    --output ./dist/manifest.json

# 4. Publish
gh release create v2.0.0 dist/* --title v2.0.0 --notes "Release notes"
```

### CI (`.github/workflows/release.yml`)

Tag push triggers build, signs the manifest, uploads to a GitHub Release. The robot's
`ota-agent` polls the configured server URL at the interval in
`/opt/lingtu/nav/ota/config.yaml` (typically 30 s). New deployments should keep
the active application symlink at `/opt/lingtu/current`; `/opt/lingtu/nav/ota`
is retained as the legacy OTA agent state path.

## 8. Configuration on the robot

`/opt/lingtu/nav/ota/config.yaml`:

```yaml
server_url: "https://releases.example.com/api"
check_interval: 30
channel: "stable"
deploy_dir: "/opt/lingtu/releases"
current_symlink: "/opt/lingtu/current"
public_key: "/opt/lingtu/nav/ota/ota_public.pem"
require_signature: true
state_file: "/opt/lingtu/nav/ota/installed_manifest.json"
history_file: "/opt/lingtu/nav/ota/upgrade_history.jsonl"
```

`upgrade_history.jsonl` is append-only; one JSON object per install / rollback for
post-mortem auditing.

## 9. Threat model

| Threat                     | Mitigation                                            | Status      |
|----------------------------|-------------------------------------------------------|-------------|
| Manifest forgery           | Ed25519 signature verified by agent                   | implemented |
| Artifact tampering         | SHA256 verified per artifact                          | implemented |
| Path traversal             | Reject `..` components in `target_path`               | implemented |
| Wrong-hardware flash       | `hw_compat` whitelist                                 | implemented |
| Interrupted install        | Transaction log + auto-rollback                       | implemented |
| Post-install regression    | Health check (`/api/v1/health`) within 60 s, else roll back | implemented |
| Apply during motion        | `safety_level` gate                                   | implemented |
| Battery brown-out          | `min_battery_percent` precheck                        | implemented |
| Version downgrade race     | Integer semver compare (no string compare)            | implemented |
| Audit                      | `upgrade_history.jsonl` append-only                   | implemented |
| Key compromise             | `public_key_id` allows rotation                       | available, not yet exercised |

## 10. Roadmap

| Item                                          | Priority | Notes                                          |
|-----------------------------------------------|----------|------------------------------------------------|
| Force-strict signature mode by default        | P1       | Currently warn-only when no public key present |
| Delta updates (bsdiff/zstd-frame-diff)        | P2       | Current full tarballs are ~50 MB each          |
| A/B partition for system-level packages       | P2       | Out of scope for `lingtu` itself; relevant if we ship `.deb`s |
| Smoke-test step after install                 | P1       | Run native Gateway/service/dataflow readiness before declaring success |
| Fleet dashboard                               | P3       | Needed once robot count > 5                    |

## 11. Related

- `docs/04-deployment/README.md` — deployment layout, services
- `docs/04-deployment/lingtu_cli.md` — operations CLI
- `scripts/ota/generate_manifest.py` — manifest tool
- `scripts/ota/manifest_template.json` — schema template
