# MATRiX integration reference

Target: official MATRiX v1.1.2 Linux x86-64 release, repository commit
`55ad3fec77deaf1b589d27fa6cc4bfca4553754e`.

This is the project-owned research index. It is not a second simulator or an
active Product. LingTu's Windows Physics/Visual/Sensor Modules and typed DDS
contracts remain authoritative.

## Local material

- Runtime and original assets: `third_party/matrix/runtime-v1.1.2/MATRiX_v1.1.2/`.
- Intake receipt: `third_party/matrix/runtime-v1.1.2/matrix-reference.json`.
- Raw static-analysis evidence: `third_party/matrix/analysis/v1.1.2/`.
- Analysis working directory retained outside the repository:
  `D:/inovxio/tools/matrix-analysis/runtime-v1.1.2/`.

The runtime directory is local-only and ignored by Git. The receipt identifies
the source archive, entry binary, file counts and license-file inventory. It
does not certify redistribution rights, runnable Windows support or performance.

The real intake completed on 2026-10-12: 845 original files / 6,605,465,703
unpacked bytes. Its archive SHA256 matches the official release:
`baaeaf01817e90baf20e3fd6f74ae2fe885ef69f7d862fa36f50222f69cfea81`.
The entry binary SHA256 also matches the prior static-analysis input:
`ed98e648c3298d0efec44170f48b0ad528c9d3010ad743a0f0dacf20d52f7bc1`.
Asset counts are 138 STL, 32 OBJ, 17 FBX, 199 uasset, 24 PNG and 52 XML.
No standard license/notice file was found in the release inventory; the
repository license is retained separately at `third_party/matrix/repository-LICENSE`
with repository-only scope, not as a grant for all release material.

The original XGB `scene.xml` compiled with the installed Windows MuJoCo 3.10.0
SDK: nbody=14, njnt=13, nq=19, nv=18, nu=12, ngeom=31, nmesh=17. This was model
compilation only: no mj_step, controller, renderer, UE or original UeSim was
started. It proves this selected MJCF/STL graph can be read by that SDK, not
locomotion, UE asset import or field behavior.

To stage a fresh copy from the already-downloaded release, from the repository root:

```powershell
& .venv/Scripts/python.exe -m sim.tools.matrix_reference `
  --source D:/inovxio/tools/matrix-analysis/runtime-v1.1.2/downloads/verified/MATRiX_v1.1.2.tar.gz `
  --destination D:/inovxio/brain/lingtu/third_party/matrix/runtime-v1.1.2 `
  --version 1.1.2
```

An existing destination is rejected. A failed release validation removes only
the new destination created by this intake. No executable is launched and no
package is registered in `sim/packages/`.

## Evidence already obtained

- [2026-10-21 sensor investigation and experimental GNSS slice](SENSORS_20261021.md):
  sensor inventory, exact native calls, successful REA layout plus preserved
  Ghidra timeout, and the implemented/remaining integration boundaries.

The prior analysis retained 67 distinct functions' complete machine-code
records and 10 REA/Ghidra local decompilation results. The latter used a
source-mapped, byte-identical focused ELF projection, not whole-program recovery.
The official core/debug Build ID is `28d46d067b635be708a2e7e6ce9f5128e78bbe6c`.

Confirmed static paths include embedded MuJoCo stepping, UE robot physics being
disabled while visuals follow snapshots, two state-derived odometry paths,
the 26-byte Livox PointCloud2 serializer, tagged box/cylinder collision proxies,
camera capture/readback paths and GPU LiDAR generation checks. Camera Hz,
complete dynamic-actor collision behavior, GPU shader semantics and complete
transport backpressure remain unqualified. The interrupted independent review
was not recorded as a PASS.

## What enters production

The bounded [2026-10-16 UI/runtime study](UI_RUNTIME_STUDY_20261016.md)
inspects a focused ELF layout with REA and one complete config-apply function
with static instruction records. It separates validated config submission from
session, physics pause/reset, input and camera entry points. This is not a full
widget recovery or a new runtime/performance claim; the current Slate task-page
work reuses LingTu's existing native control and report owners.

The 2026-10-13 continuation is recorded in
[RUNTIME_REVERSE_20261013.md](RUNTIME_REVERSE_20261013.md): 14 bounded
machine-code function records and 10 new REA/Ghidra decompilation results cover
command freshness/safe hold, latest geom snapshots, and synchronized sensor
triggers. It also traces the current Windows demo's first camera failure and
the directly supported deferred-capture promotion fix. These are mechanism and
regression evidence, not a complete MATRiX source recovery or a new frame-rate claim.

| Material or behavior | Intended owner | Required work |
| --- | --- | --- |
| Original Linux UeSim and its libraries | Local third-party reference only | It is ELF, not a Windows DLL/UE plugin; do not place it in the Windows runtime load path. |
| Raw MJCF, STL, OBJ and FBX candidates | Existing RobotImporter/WorldImporter, then versioned `sim/packages/` | Applicable grants, units, frames, geometry, materials, collision and package qualification. |
| Cooked UE assets and map DLCs | Local reference until separately qualified | Applicable grants and target UE/platform compatibility; do not assume a Linux cooked asset is an editable Windows asset. |
| Physics/visual synchronization improvements | `sim/runtime/physics/`, `sim/runtime/visual/RobotSimUE/` | Independent implementation through existing plan/snapshot Interfaces; preserve one physics authority. |
| Sensor timing/readback improvements | `sim/runtime/sensors/`, RobotSimUE render-sensor implementation | Use the existing scheduler and generation semantics; measure same-run rates. |
| MATRiX transport compatibility | Explicit Adapter, if selected | Convert fields/time/frame/tag semantics; do not replace the native typed DDS Product contract. |

The repository's BSD-3-Clause license does not settle every release asset.
The release README explicitly warns that binaries, models, DLCs and third-party
components may have different terms. Preserve upstream notices and keep
redistribution marked unverified until the applicable grants are established.

## Next implementation slice

The active feature mapping, delivered Mid360 noise experiment and next vertical
slices are maintained in [FEATURE_MIGRATION.md](FEATURE_MIGRATION.md). This
feature is independently implemented in the existing Windows Sensor Runtime;
it does not execute or copy the original Linux core.

1. Reference intake and evidence retention are complete; preserve the original files.
2. Select one robot or prop with an established asset grant; run the existing
   importer instead of creating a MATRiX-specific Robot Runtime.
3. Verify its MuJoCo geometry and UE visual projection agree in frames/scale;
   visual quality does not prove contact geometry.
4. Improve one measured native sensor/physics seam and prove it in the existing
   single-RunPlan session. Keep real/field validation separate.

Public sources: [repository](https://github.com/zsibot/matrix/tree/55ad3fec77deaf1b589d27fa6cc4bfca4553754e),
[runtime release](https://github.com/zsibot/matrix/releases/tag/v1.1.2),
[repository license](https://github.com/zsibot/matrix/blob/55ad3fec77deaf1b589d27fa6cc4bfca4553754e/LICENSE).
