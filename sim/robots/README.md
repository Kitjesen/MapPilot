# Simulation Robot Assets

`sim/robots/` contains compatibility robot models and optional local policy files
used by simulation entrypoints. Keep source-controlled model definitions and
manifests here, but keep heavyweight or externally supplied policies out of git.

## Layout

| Path | Purpose | Git policy |
| --- | --- | --- |
| `thunder.urdf` | Thunder v3 URDF compatibility path with mesh references adjusted to `sim/assets/meshes/`. | Tracked. |
| `nova_dog/robot_with_camera.xml` | Legacy NOVA Dog path that mirrors the current Thunder v3 simulation adapter. | Tracked. |
| `nova_dog/policy_manifest.json` | Contract and hash metadata for the simulation-only NOVA/Thunder policy. | Tracked. |
| `nova_dog/policy.onnx` | Optional local simulation policy referenced by the manifest. | Ignored local asset. |
| `thunderv4/mjcf/thunderv4.xml` | Current default LingTu MuJoCo robot model with local mesh assets. | Tracked. |
| `thunderv4/policy/policy_manifest.json` | Hash metadata for the imported Thunder v4 TorchScript policy. | Tracked. |
| `thunderv4/policy/*.pt` | Optional local Thunder v4 TorchScript policy. | Ignored local asset. |
| `go1_playground/` | Placeholder for optional external Go1 demo assets. | README tracked; `.onnx`/XML external assets stay local. |
| `omni_cart/` | Omnidirectional cart model used by selected simulation experiments. | Tracked. |

## Cleanup / organization rules

- Do not delete tracked robot definitions when cleaning local data; simulation
  profiles and tests rely on stable paths under `sim/robots/`.
- Keep `*.onnx` and `*.pt` files local unless there is a deliberate model-release process.
- If `go1_policy.onnx` is present for the legacy Go1 demo, it belongs at
  `sim/robots/go1_playground/go1_policy.onnx`, not a separate ad-hoc `go2/`
  directory.
- If local policy files need to be removed to save disk space, move them to a
  quarantine/cache location first so they can be restored without re-downloading.
