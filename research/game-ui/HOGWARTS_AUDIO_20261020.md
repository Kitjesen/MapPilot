# Hogwarts audio/UI reference — 2026-10-20

## Scope and tooling

The user-selected Hogwarts Legacy installation was read without executing or
modifying the game. The existing UE 5.8 UnrealPak extracted exactly three cooked
packages (90,115 bytes) from its IoStore container into
`D:/inovxio/tools/game-ui-analysis/run-20261020-hogwarts-audio-06/`.
The exact commands, exit codes, selected strings and byte offsets are in
`audio-reference-evidence.json`; all three extraction commands exited zero.

REA MCP is 6.6.0, but its current process does not have a configured Ghidra
installation. These new package observations came from UnrealPak and the bounded
reader in `inspect_hogwarts_audio.py`, not REA native decompilation. No game code,
sound recordings, fonts or artwork were copied into the product. This pass does
not establish the original game's runtime callbacks, timing or audio routing.

## Observations

| Cooked package | Bytes | Source-bound observations |
| --- | ---: | --- |
| `UI_BP_Settings_Audio` | 67,955 | Wwise UI/menu and options-adjusted event references at offsets 326, 369, 434, 498 and 565; distinct slider-changed/finished delegate names; mute, effect/music/voice volume getter/setter names |
| `BP_HUD_Audio` | 9,891 | Audio-cue activation/deactivation, accessibility manager, audio visualizer and directional-arrow widget references |
| `UI_DT_AccessibilityAudioCueIconTable` | 12,269 | Declared alert, interaction, door, footsteps and other event categories, with UI accessibility audio-cue identifiers |

`SettingsSliderChanged` and `SettingsSliderFinished` appear as distinct names at
offsets 11181 and 11204 of the audio-settings package. The object hierarchy,
delegate binding behavior and executable control flow were not decoded. Names
support a separation-of-responsibilities design inference, not execution proof.

## Original LingTu implementation decision

- Keep an explicit audio settings page alongside display, controls and interface.
- Master volume belongs to the selected UE world's audio device; UI level scales
  only LingTu UI cues. Do not modify Windows system volume.
- Volume edits are immediately previewable, with mute retaining the two levels.
- Reuse the existing player-preference checkpoint for save, undo and discard;
  preserve the independent graphics apply/confirm/rollback transaction.
- Use the existing original short PCM cues and an explicit preview action. Do not
  add unsupported music, voice-chat or audio-device selectors.
- Accessibility feedback must also remain visible text; sound is supplementary.
  Directional environmental sound visualization is a future feature requiring
  actual world events, not a HUD decoration to imitate now.

Product implementation and UE test results are separate from these static facts.
The completed original audio settings and verification results are recorded in
`docs/design/product-shell-and-audio.md`; its final focused suite passed 48/48.
