# MATRiX v1.1.2 UI/runtime sidecar study — 2026-10-16

This bounded pass asks one narrow question: what does the shipped MATRiX UI
actually own at the config/session boundary, and what should remain with
LingTu's existing runtime owners? It does not recover the complete MATRiX UI.

## Scope and stop condition

- Source: the retained official MATRiX v1.1.2 Linux x86-64 `UeSim` and matching
  debug sidecar already present under the external analysis working directory.
- New REA work: one metadata/symbol-string inspection of a 55,328-byte,
  non-runnable projection. No whole-binary Ghidra import or re-index completed.
- Focused instruction evidence: one function,
  `UMujocoSensorSchemeManagerWidget::ApplySchemeToRobotConfig(FString&)` at
  `0xddbac00` (673 bytes, all 673 decoded).
- No target execution, UE/UBT, MuJoCo, Blender, game, download, DRM/encryption
  work, asset extraction, or source/asset copying was performed.
- Stop condition reached: the config transaction and its owner boundary are
  distinguishable from session, input, camera, pause, and reset entry points;
  deeper UI recovery is not needed for the current owner mapping.

The projection's selected bytes came from the verified original executable
whose SHA-256 is
`ed98e648c3298d0efec44170f48b0ad528c9d3010ad743a0f0dacf20d52f7bc1`.
The projection SHA-256 is
`6b348723f79741f761587bd785c0f2fcb655375015a5a7e416a3674554fa330f`.
Its manifest records the copied source ranges and the limitation that it is an
analysis artifact, not a runnable product binary.

## Exact commands and outputs

### Bounded REA metadata/symbol-string inspection

Command (PowerShell, repository root):

```powershell
& 'C:/Program Files/nodejs/node.exe' `
  D:/inovxio/tools/matrix-analysis/runtime-v1.1.2/rea-session.mjs `
  D:/inovxio/brain/lingtu/build/research/matrix-ui-20261016/rea-metadata-plan.json `
  D:/inovxio/brain/lingtu/build/research/matrix-ui-20261016/rea-metadata-run
```

Exact stdout:

```text
{"phase":"CONNECTED","tools":170,"pid":68356}
{"phase":"CALL_BEGIN","index":0,"name":"inspect_binary_layout"}
{"phase":"CALL_END","index":0,"name":"inspect_binary_layout","isError":false}
```

Result summary read from `00-inspect_binary_layout.json`:

```text
evidence_id: ev_10f1c23e4cec91eb4055b7ccba4a3e53b23ea8ea2b114474ff41133b44ffbbe6
subject: ui-runtime-projection.elf
format/architecture: elf / x86_64
sections: 32
segments: 28
symbol rows: 751
relocations: 0
```

Most symbol rows are retained PLT entries. The nine selected project symbols
are exact names/addresses imported from the matching official debug sidecar:

```text
0xdd7adb0  AMujocoSceneActor::PausePhysicsSimulation()                       size 0x7
0xdd7adc0  AMujocoSceneActor::ResetPhysicsSimulation()                       size 0x276
0xddb50b0  UMujocoSensorSchemeManagerWidget::HandleSensorSyncModeChanged(bool) size 0x229
0xddbac00  UMujocoSensorSchemeManagerWidget::ApplySchemeToRobotConfig(FString&) size 0x2a1
0xde52740  AMatrixMotionControlActor::BeginPlay()                             size 0x3fa
0xde53fc0  AMatrixMotionControlActor::Tick(float)                             size 0xf6
0xde56f80  AMatrixMotionControlActor::SetExternalInputEnabled(bool)           size 0x1db
0x11a56b20 ARobotCameraSensorActor::StartSensor()                             size 0x114
0x11a5aa30 ARobotCameraSensorActor::StopSensor()                              size 0x245
```

These names establish distinct entry points. They do not by themselves prove
their complete behavior, call order, or that a UI button invokes them.

### One-function instruction inspection

Command used to create the retained focused instruction records:

```powershell
wsl.exe -d Ubuntu --exec `
  /home/thunder/matrix-analysis-tools/elf-venv/bin/python `
  /mnt/d/inovxio/tools/matrix-analysis/runtime-v1.1.2/focused-disassembly.py `
  /mnt/d/inovxio/tools/matrix-analysis/runtime-v1.1.2/inputs/MATRiX_v1.1.2/UeSim/Binaries/Linux/UeSim `
  /mnt/d/inovxio/brain/lingtu/build/research/matrix-ui-20261016/disassembly `
  --addresses 0xdd7adb0 0xdd7adc0 0xde52740 0xde53fc0 0xde56f80 `
  0xddbac00 0xddb50b0 0x11a56b20 0x11a5aa30
```

Exact meaningful stdout lines (apart from WSL's local networking warning):

```text
{"address": "0xdd7adb0", "name": "AMujocoSceneActor::PausePhysicsSimulation()", "size": 7, "decoded_bytes": 7}
{"address": "0xdd7adc0", "name": "AMujocoSceneActor::ResetPhysicsSimulation()", "size": 630, "decoded_bytes": 630}
{"address": "0xde52740", "name": "AMatrixMotionControlActor::BeginPlay()", "size": 1018, "decoded_bytes": 1018}
{"address": "0xde53fc0", "name": "AMatrixMotionControlActor::Tick(float)", "size": 246, "decoded_bytes": 246}
{"address": "0xde56f80", "name": "AMatrixMotionControlActor::SetExternalInputEnabled(bool)", "size": 475, "decoded_bytes": 475}
{"address": "0xddbac00", "name": "UMujocoSensorSchemeManagerWidget::ApplySchemeToRobotConfig(FString&)", "size": 673, "decoded_bytes": 673}
{"address": "0xddb50b0", "name": "UMujocoSensorSchemeManagerWidget::HandleSensorSyncModeChanged(bool)", "size": 553, "decoded_bytes": 553}
{"address": "0x11a56b20", "name": "ARobotCameraSensorActor::StartSensor()", "size": 276, "decoded_bytes": 276}
{"address": "0x11a5aa30", "name": "ARobotCameraSensorActor::StopSensor()", "size": 581, "decoded_bytes": 581}
```

The one retained record interpreted in this study reports:

```text
address: 0xddbac00
name: UMujocoSensorSchemeManagerWidget::ApplySchemeToRobotConfig(FString&)
size: 673
decoded_bytes: 673
code_sha256: c94f2613859c375c7316c4d8413ee32d0069abe3b872c7bdfb662230c2eb063f
```

This local extraction completed before the memory-thin constraint was
tightened. Only the config function above is used as behavioral evidence here;
the other records are retained as raw sidecar artifacts and are not interpreted
below.

### Distributed config value check

Exact shipped values read from
`UeSim/Content/model/config/config.json` include:

```json
{
  "network_mode": "standalone",
  "sensor_sync_mode": true,
  "mujoco_running": false,
  "sensors": {}
}
```

This proves only the distributed default file values. It does not prove the
post-load runtime state or a user session's selected configuration.

## Observed config transaction

The following statements are direct instruction/call evidence from the one
inspected function:

1. It clears the output `FString`, calls `CommitEditorToSelected`, tests the
   boolean result, and branches to the failure status path when false.
2. It then calls `ValidateScheme`, tests that result, and uses the same failure
   path when false.
3. It calls `UMujocoConfigLibrary::WriteMujocoConfigSensors`; the returned byte
   is tested at `0xddbacca`, and false branches to `0xddbad2a`.
4. The common failure path calls `UpdateStatus(..., false, true)` and returns
   false. Exact enum/boolean parameter names are unavailable; only the literal
   values and call are observed.
5. On success it calls `ResolveMujocoConfigPath`, then
   `RefreshRuntimeContext`, formats the UTF-16 string
   `Applied %d sensors to the robot configuration.`, calls
   `UpdateStatus(..., true, false)`, and invokes the
   `FMujocoSensorSchemeAppliedSignature` delegate wrapper.

Therefore the inspected widget function is a validated config-write and
notification transaction. No session creation, physics pause/reset, motion
input arbitration, or camera frame operation is present in this function.
That final sentence is complete only within this 673-byte function; it is not
a claim about every MATRiX UI callback.

## Mechanism mappings to existing LingTu owners

| Concern | MATRiX evidence | Evidence class | Existing LingTu owner / decision |
| --- | --- | --- | --- |
| Sensor-config apply | The inspected widget commits editor state, validates, writes sensor config, refreshes runtime context, updates status, and broadcasts success. | **Observed** in the complete 673-byte function. | Keep configuration compilation/validation in `sim/catalog/resolver.py` and immutable SessionBundle artifacts. UI may request an apply and display status, but must not become a second config compiler or mutate a live RunPlan ad hoc. |
| Session/config boundary | Config apply has its own widget transaction, while the projection separately names pause/reset, input, and camera entry points. The shipped file says `network_mode=standalone`, not a LingTu Product/session identity. | Entry-point separation is **observed**; lifecycle separation is an **inference**. | `LingTuSimSessionModule.cpp` / `FSessionBundleLoader` / `FSessionService` own UE session binding and generation publication; Python `ProductControl` owns Product lifecycle and `product_session_id`. A UI config success event should not mint or replace those identities. |
| Pause and reset | Exact symbols exist for `AMujocoSceneActor::PausePhysicsSimulation` and `ResetPhysicsSimulation`; this pass did not behaviorally inspect them. | Symbol existence is **observed**; semantics are **unresolved**. | Route operator pause/resume/restart through the existing coordinator request seam in `sim/runtime/coordinator/playable_control.py`. Keep UI pause-menu mode in `LingTuSimRuntimeUIModel.cpp` distinct from physics/session pause. Reset must continue to advance the authoritative model/reset generation rather than being a display-only action. |
| Input and camera | Exact, separate symbols exist for motion `BeginPlay`/`Tick`/external-input enable and camera `StartSensor`/`StopSensor`. No call relation to config apply was observed. | Symbol separation is **observed**; ownership mapping is an **inference** checked against current LingTu code. | Keep drive focus/deadman/release handling in `LingTuSimRuntimeUIInputProcessor.cpp` and `LingTuSimRobotDriveInput.cpp`. Keep sensor capture/readback/session-reset rejection in `LingTuSimCameraCaptureSubsystem.cpp`. UI camera view controls may publish view intent, but must not own camera sensor lifecycle or bypass capture generation checks. |

## Independent judgment

The useful pattern to adopt is not MATRiX's mutable config file or its widget
class. It is the narrow transactional shape: validate before write, expose one
success/failure result, refresh the consumer context only after a successful
write, and emit one completion notification. LingTu should implement that
shape through its existing SessionBundle/RunPlan owners.

The risky interpretation would be to equate the pause menu with physics pause,
or to let a UI sensor/config screen restart camera/runtime components directly.
The static evidence shows those are separate MATRiX entry-point families, and
LingTu already has stronger session/reset-generation and command-admission
boundaries. Preserve those boundaries rather than cloning MATRiX's UI/runtime
coupling.

## Artifacts and limitations

All new artifacts are under `build/research/matrix-ui-20261016/`:

- `ui-runtime-projection.elf` and `.manifest.json`: byte-mapped, non-runnable
  analysis projection.
- `rea-metadata-plan.json` and `rea-metadata-run/`: successful REA metadata
  request, complete response, schemas, and server stderr.
- `disassembly/`: raw bounded static instruction records; only
  `0xddbac00.json` is interpreted in this note.
- `make_projection.py`: task-local projection builder; it does not modify the
  retained runtime or release evidence directories.

One attempted Ghidra-backed string query was stopped after metadata/open
succeeded but the string call made no progress within the bounded window. Its
task-owned Ghidra process was terminated, and no result from that attempt is
cited. A follow-up process check found no remaining `rea.mjs`, Ghidra, or
`analyzeHeadless` process. The successful REA evidence ID above comes only from
the non-indexing layout inspection.

This pass does not prove button bindings, complete widget composition, camera
or physics runtime behavior, pause/resume timing, reset ordering, config file
atomicity, or session recovery. Those would require additional selected
functions or controlled runtime evidence and are intentionally outside this
sidecar.
