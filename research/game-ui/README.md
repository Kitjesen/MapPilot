# Installed-game UI reference study

Current continuation: [Hogwarts audio/UI study, 2026-10-20](HOGWARTS_AUDIO_20261020.md)
records three newly extracted audio/accessibility packages and a bounded source
reader. REA MCP is now 6.6.0; Ghidra is not configured in that MCP process.
Package references are not presented as native function decompilation. The
original LingTu audio-preference implementation and its tests are tracked in
`docs/design/product-shell-and-audio.md` separately from these research facts.

Latest: the [2026-10-17 front-end study](FRONTEND_STUDY_20261017.md) updates
the existing REA CLI/source to 6.5.0 and examines 15 targeted main-menu, camera,
scene and button files from the installed games. It records REA's cooked-package
limitation separately from successful PE-resource inspection and UnrealPak evidence.

Continuation: the [2026-10-16 task/journal study](TASK_JOURNAL_STUDY_20261016.md)
records a real, unsuccessful REA cooked-package probe, a transparent strings
fallback, and original interaction mappings. It recommends separate execution
and evidence/acceptance rails before adding any chronological history. The
current task projection is a summary, not a reconstructed event journal.

Date: 2026-10-13. Scope: a light restyle of RobotSimUE's task/status HUD,
not game-content migration. The two installations were supplied by the user.
No game was launched, patched or modified; no encryption key was supplied or
protection bypassed. Only the existing official UnrealPak utility and the
existing Pillow decoder were used.

## Hogwarts Legacy: archive and real texture evidence

Installation: `D:/software/steam/steamapps/common/Hogwarts Legacy`.
Relevant archive directory: `Phoenix/Content/Paks/`.

The installed UE 5.8 UnrealPak successfully listed
`pakchunk0-WindowsNoEditor.pak` (26,546 entries), then separately listed the
IoStore directory index in `pakchunk0-WindowsNoEditor.utoc` to a CSV. A PAK
listing alone does not enumerate all the cooked UI textures in this game.

Generated evidence lives under
`build/game-ui-research-20261013/hogwarts/`:

- `pakchunk0-list.log`: actual PAK entries, including 11 entries under
  `Phoenix/Content/UI/` (mainly raw `.ufont` files).
- `pakchunk0-iostore.csv`: actual container directory entries; 7,791 unique
  filenames under `Phoenix/Content/UI/`, including 360 under `HUD/`,
  111 under `Common/`, and 268 under `Menus/`.
- `mission-extract.log`: successful targeted extraction using
  `*/UI/HUD/Mission/*`, not whole-container extraction.
- `mission-cooked/`: exactly 9 selected container files, including 3 widget
  packages and 6 texture packages.
- `mission-png/`: 6 decoded PNGs plus `decode-report.json`. These are local
  research references and are **not** copied into RobotSimUE Content.

Useful verified mission entries:

| Entry under `Phoenix/Content/UI/HUD/Mission/` | Observed evidence |
| --- | --- |
| `UI_BP_MissionBanner_New.uasset` | Imported-name strings reference body/header fonts, mission-card/font materials and the three ornament textures. This is not a decoded widget tree. |
| `UI_BP_MissionCompleteBanner.uasset` | Extracted cooked package; full widget properties/animations not decoded. |
| `UI_BP_ChallengeHud.uasset` | Extracted cooked package; full widget properties/animations not decoded. |
| `UI_T_MissionBanner_BorderOrnament.uasset` | One inline `PF_BC7` mip, 256x128, decoded and visually inspected. |
| `UI_T_MissionBanner_DpadOrnament.uasset` | One inline `PF_BC7` mip, decoded to PNG. |
| `UI_T_MissionBanner_KBDOrnament.uasset` | One inline `PF_BC7` mip, decoded to PNG. |
| `UI_T_ActiveMission_Icon.uasset` | One inline `PF_BC7` mip, 128x128, decoded and visually inspected. |
| `UI_T_check_gradientBack.uasset` | One inline `PF_BC7` mip, decoded to PNG. |
| `UI_T_CheckBox.uasset` | One inline `PF_BC7` mip, decoded to PNG. |

Strings inside the mission-banner package reference:

```text
/Game/UI/Common/Labels/UI_BP_MissionBannerCheckbox
/Game/UI/Fonts/UI_FNT_Body
/Game/UI/Fonts/UI_FNT_Header
/Game/UI/FX/MI_UI_FontGoldleaf
/Game/UI/FX/MI_UI_GLF_MissionCard
```

These names support a component/material composition hypothesis; they do not
prove its exact hierarchy, parameters, opacity or animation timing. The installed
game uses an older cooked package format; extracted bytes must not be treated
as editable UE 5.8 source assets. UE 5.8 CSV chunk-type labels also are not used
here to infer the old game's object semantics.

### Targeted extraction and decoding

```powershell
& 'D:/Program Files/Epic Games/UE_5.8/Engine/Binaries/Win64/UnrealPak.exe' IoStore `
  '-FindAndExtract=D:/software/steam/steamapps/common/Hogwarts Legacy/Phoenix/Content/Paks/pakchunk0-WindowsNoEditor.utoc' `
  '-PackageFilter=*/UI/HUD/Mission/*' `
  '-OutPath=D:/inovxio/brain/lingtu/build/game-ui-research-20261013/hogwarts/mission-cooked'

& .venv/Scripts/python.exe research/game-ui/decode_inline_bc7.py `
  build/game-ui-research-20261013/hogwarts/mission-cooked/Phoenix/Content/UI/HUD/Mission `
  build/game-ui-research-20261013/hogwarts/mission-png
```

Use a fresh output directory for a repeat decode. The helper deliberately
supports only the observed single-mip, inline-BC7 shape. All six successful
decodes checked pixel length, footer dimensions and RGBA output dimensions;
all six truncated-payload probes were rejected. Ruff passed. This is not a
general Unreal object parser or a reconstruction of the original UMG widgets.

Field-order references:
[CUE4Parse platform data](https://github.com/FabianFG/CUE4Parse/blob/master/CUE4Parse/UE4/Assets/Exports/Texture/FTexturePlatformData.cs),
[CUE4Parse mip data](https://github.com/FabianFG/CUE4Parse/blob/master/CUE4Parse/UE4/Assets/Exports/Texture/FTexture2DMipMap.cs).
Pixel decoding uses the installed Pillow DDS reader;
[upstream format documentation](https://pillow.readthedocs.io/en/stable/handbook/image-file-formats.html#dds).

## Palworld: manifest and archive cross-check

Installation: `D:/software/steam/steamapps/common/Palworld`.
Main archive: `Pal/Content/Paks/Pal-Windows.pak`.
The root `Manifest_UFSFiles_Win64.txt` exists (20,810,392 bytes). The HUD root,
quest display and key-guide widget paths were cross-checked against the actual
archive listing, saved in `build/game-ui-research-20261013/palworld/`.

Verified path families:

| Path under `Pal/Content/` | Archive evidence |
| --- | --- |
| `Pal/Blueprint/UI/BP_PalHUD_InGame.uasset` | HUD root, with separate input-listener and general-dispatch receiver packages. Relationships are inferred from names until object properties are decoded. |
| `Pal/Blueprint/UI/Quest/WBP_Quest_ForDisplay.uasset` | Quest-display entry; tracking-icon/canvas packages also exist. |
| `Pal/Blueprint/UI/PalStatus/WBP_PalStatus.uasset` | Status package; popup, hunger and player-side-info packages are separate. |
| `Pal/Blueprint/UI/CommonWidget/ActionWidget/WBP_PalKeyGuideIconAndText.uasset` | Key-and-text entry; separate hold-progress material is present. |
| `Others/KeyboardIcons/key_black/` | 84 `.uasset` plus 84 `.uexp` entries. |
| `Pal/Blueprint/UI/` | 1,043 `.uasset` plus 1,043 `.uexp` entries; this family includes materials/other objects and must not be called 1,043 widgets. |
| `Pal/Texture/UI/` | 1,326 `.uasset`, 1,326 `.uexp`, 4 `.ubulk` entries. |
| `Pal/Blueprint/UI/CommonWidget/RadialMenu/` | Plate/background/selection-border material instances. |

No Palworld cooked textures were decoded in this pass. Archive paths are not
standalone PNG files and their presence does not establish their render layout.

## Independent RobotSimUE adaptation

- Use Hogwarts' layered task/title/marker treatment as a visual reference;
  avoid importing its parchment, crest or ornate fantasy artwork into a robot HUD.
- Use Palworld's separate key-and-text/hold-feedback component concept for
  future control prompts, without changing the existing no-Shift keyboard contract.
- Keep Drive minimal and technical details in Tactical. Session, recording,
  readiness and unavailable/unverified states stay owned by existing runtime data.
- Keep commercial-game font/texture files out of product packages. Recreate
  simple geometry with Slate and existing project fonts; obtain a separate
  usable asset license before distributing any original game artwork.

This pass delivers real file-level references, not a new HUD revision. The
previous compact HUD build and its 50/50 Automation result are unchanged.
