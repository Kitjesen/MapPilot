# MATRiX camera binding follow-up — bounded parser attempt, 2026-10-22

## Outcome

**Production Blueprint binding remains unresolved. No new dispatch, spawn, property-assignment, mount or body-follow behavior was recovered.** The actually new result is a concrete current-tool boundary: connected REA **6.6.0** rejects the existing uncompressed `BP_PTZCamera.uexp` with `target_unavailable / unsupported_target`, `constraint=file_format_rejected`, `retryable=false`, before Blueprint decoding. This is not an Oodle failure, an encryption failure, an analysis timeout or proof that the packaged Blueprint lacks the requested behavior.

This lane selected **Blueprint production binding**, not infrared material/shader reconstruction. The one target-opening attempt ended at the format gate; no alternate target, whole-executable import or further overview loop was attempted after that gate. The requested bounded stop condition has been reached, not the production-chain completion condition.

## Authority and data boundary

- Read repository `AGENTS.md`, parent `D:/inovxio/AGENTS.md`, the requested REA skill and its complete native/artifact guide; read the connection/recovery guide after the observed rejection. Followed the skill's evidence/unknown distinction and stopped at the unsupported route without installing or repairing tools.
- Authoritative prior implementation report: [CAMERA_CHAIN_20261021.md](CAMERA_CHAIN_20261021.md). Its 48 complete instruction records, 29 decompilations and P1 package evidence are **reused authority, not new findings** here.
- Original selected target remains `D:/inovxio/tools/matrix-analysis/runtime-v1.1.2/inputs/MATRiX_v1.1.2/UeSim/Binaries/Linux/UeSim` with `UeSim.debug`. Existence/length observed locally: 350,856,080 / 1,473,671,040 bytes. Prior report binds both to Build ID `28d46d067b635be708a2e7e6ce9f5128e78bbe6c` and executable SHA256 `ed98e648c3298d0efec44170f48b0ad528c9d3010ad743a0f0dacf20d52f7bc1`; these identities were not rehashed or re-established in this lane. No original executable/debug analysis was repeated.
- Exact source archive/member attribution is inherited from P1: `downloads/verified/MATRiX_v1.1.2.tar.gz`, member `MATRiX_v1.1.2/UeSim/Content/Paks/pakchunk0-Linux.pak`, tar **uncompressed data** offset 2,449,957,376, member length 1,640,238,903. Tar offsets below are not seek offsets into the compressed gzip file.
- Only this new report and [the new evidence directory](../../build/research/matrix-camera-binding-20261022/) were written. Shared dirty-tree changes were not reverted or edited. No UE/game/MuJoCo execution, compilation, installation, MCP configuration change, network upload, asset promotion/copy into production, decryption or key/DRM work occurred. Existing extracted resources were read in place; no new asset copies were made.

## Exact attempt and retained artifacts

Evidence root: `build/research/matrix-camera-binding-20261022/`.

1. `rea-session-before.json`: actual `binary_session({})` response. `open=false`, `cleanup_pending=false`, `analysis_provider_binding=null`, `analysis_run=null`. Server package/root: `rea-agents@6.6.0`, `D:/inovxio/tools/rea`; skill version expected by server: 42. No matching active native session existed to reuse.
2. `rea-tool-catalog-names.json`: 207 connected REA tool names. No Unreal/Blueprint/PAK/Oodle-specific decoder is advertised. `inspect_asset_catalog` is explicitly the Apple `assetutil` route (`unsupported_host` on this Windows host), not a generic UE asset decoder. `inspect_artifact` and `extract_artifact` require a supported opened target. These are catalog/session observations, not claims about every tool installed elsewhere on the computer.
3. `rea-blueprint-open.json`: exact request and complete returned error. Request: `open_binary({path: "D:/inovxio/brain/lingtu/build/research/matrix-camera-chain-20261021/material-records-win/UeSim__Content__Sensors__BP_PTZCamera.uexp"})`. Message: `Cannot open artifact: unsupported binary format.` Error fields: `code=target_unavailable`, `category=unsupported_target`, `details.reason=unsupported binary format`, `details.constraint=file_format_rejected`, `retryable=false`. No Blueprint interpretation or Evidence ID was returned. **Do not label this error a REA behavioral Evidence record.**
4. `verify-record-anchors.ps1`: local read-only verifier for exactly the four prior Blueprint records below. It compares original stored header size/unpacked size/compression method with P1's record table, checks full cached record extent, and byte-compares each uncompressed loose `.uexp` with its cached record payload. Run completed exit 0, `status=PASS`, `selected_records=4`; all four `header_and_export_checks=PASS`. P1's prior SHA1/encryption attribution is reused, not independently reauthenticated by this script.
5. `rea-session-after.json`: actual final session response, again `open=false`, `cleanup_pending=false`, no provider binding or analysis run. The rejected open created no owned native session to close. No pre-existing session was closed.
6. `verification.json`: retained final check output, including all four source-byte anchors, the actual rejection and final session state. Record/header/export checks, report/anchor consistency and report whitespace checks all passed. The storage checks do not claim a successful Blueprint decode.

Additional local source corroboration, not target-behavior evidence: `D:/inovxio/tools/rea/src/domain/binaryTarget.ts:79-100` admits ELF/MZ/Mach-O executable headers and returns `unsupported binary format` at line 99. `src/application/BinaryTargetResolver.ts:290-391` handles the archive/plist/ASAR/JavaScript-related format routing; no Unreal route was found in the bounded `src` search for `oodle|uasset|uexp|unreal|pak-v|pakfile`. The source search is bounded corroboration, not a universal nonexistence proof. One diagnostic search also named the nonexistent `D:/inovxio/tools/rea/lib` and reported that directory absent; no conclusion relies on its absence. The actual MCP rejection is the controlling result.

## Source-byte anchors

All offsets are decimal, zero-based. Ranges are identified by start plus length. Record/header locations originate in prior P1 `material-record-results.json`; this lane freshly checked cached header fields and loose-export equality, not the whole archive. `index` is the offset into prior `pakchunk0-encoded-entries.bin`, not an ELF address. `tar payload` is the uncompressed tar-member data base plus PAK payload offset.

| Prior packaged resource | Index | PAK record | Header | PAK payload | Tar payload | Stored / unpacked | Method |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `BluePrint/Robot/BP_MujocoSceneActor.uasset` | 94360 | 650778870 | 73 | 650778943 | 3100736319 | 1582 / 4178 | 1 |
| `BluePrint/Robot/BP_MujocoSceneActor.uexp` | 94376 | 650780525 | 53 | 650780578 | 3100737954 | 984 / 984 | 0 |
| `Sensors/BP_PTZCamera.uasset` | 277520 | 1474878204 | 73 | 1474878277 | 3924835653 | 1641 / 4846 | 1 |
| `Sensors/BP_PTZCamera.uexp` | 277536 | 1474879918 | 53 | 1474879971 | 3924837347 | 967 / 967 | 0 |

Paths are relative to `UeSim/Content/`. Cached source records live in `build/research/matrix-camera-chain-20261021/material-records-win/` under slash-to-double-underscore names, suffixed `.pak-record`. Record header fields checked at local offsets 8 (stored u64), 16 (unpacked u64), 24 (method u32); payload starts at the listed header size. Method 1 maps to Oodle according to P1's footer, not by guessing from payload bytes. All four have `encrypted=false` in P1.

The actual rejected 967-byte PTZ export begins at PAK **1474879971**, with these first 32 bytes:

```text
08 02 01 03 10 00 00 00 03 00 00 00 00 00 00 00
f3 ff ff ff 04 00 00 00 04 00 00 00 06 00 00 00
```

The scene actor's 984-byte export at PAK **650780578** begins:

```text
08 02 01 03 0c 00 00 00 03 00 00 00 00 00 00 00
f5 ff ff ff 03 00 00 00 05 00 00 00 04 00 00 00
```

These byte anchors establish precisely which retained payload was offered to REA and permit later reuse without scanning the multi-gigabyte archive. They **do not** identify properties, function opcodes, object indices, class ancestry or executed nodes. A split `.uexp` is not a self-contained executable; format rejection does not diagnose corruption or establish whether a future parser can decode it with the matching `.uasset` and schema.

## What this means for LingTu mount/calibration

**No new calibration fact is authorized by this lane.** The concrete gap has two layers: the observed current REA route cannot admit this split Blueprint payload; additionally the matching `.uasset` records remain Oodle-compressed in existing evidence, and a matching cooked package/property/bytecode schema has not been demonstrated. Fixing decompression alone would not establish the requested production binding.

The parent sensor Product chain should therefore keep these distinctions from the prior report:

- Configuration parsing and actor constructor attachment do not prove the production Blueprint forwards `sensor_type`, pose, intrinsics or `sensor_attach` into the right instance.
- `GetMujocoMeshRelativeSpawnTransform` having a native body does not prove that the selected production spawn path calls it or maintains body-follow updates.
- No new evidence here settles mount units/signs/order, parent selection, body-relative updates, reset behavior or exposure-time gimbal extrinsics. A `frame_id` string and a configured pose are not measured/validated camera calibration.
- Implement LingTu's own declared mount and capture-time pose contracts through its existing owners and tests; do not mark them MATRiX-compatible from these package names/bytes. No overlapping parent implementation was changed.

## Closure and verification

Verification was intentionally narrow: detect a wrong/truncated cached record or wrong loose export before citing its source location; any mismatch would invalidate the corresponding anchor. The four-record verifier passed. The real REA request failed before decoding and is preserved as a failure, not converted to a semantic PASS. Final session confirms no owned analysis cleanup remains. Documentation/evidence-only lane: application tests, build/typecheck and simulated capture were not run and would not resolve this parser gap.

To continue this exact question in a separately scoped lane requires a **demonstrated existing local decoder** for the matching Oodle stream and cooked Blueprint package/schema, consuming the paired headers/exports and returning referenced class/function/property/script offsets. Until then, dispatch/spawn/property assignment/mount/body-follow remain **unknown**; infrared MI parent/parameters/shader law was **not attempted** here. No further overview work is queued by this report.
