# LingTu menu music drafts

Two original procedural compositions, authored for this project on 2026-10-20:

- `base-arrival.wav` / 基地序曲: slow pads and sparse bell notes.
- `departure.wav` / 启程: a second chord/melody arrangement.

Each is 32 seconds, stereo PCM16 at 24 kHz, with overlapping notes across its
loop seam. No third-party samples, commercial game audio, vocals, or external
generation service are used. These are editable composition drafts, not a
claim of a recorded orchestral score or a listening-quality approval.

Regenerate with `sim/tools/assets/author_menu_music.py`; import with the game's
`Scripts/import_menu_music.py`. The game owns one music component while in the
front end, shares it across menu pages, crossfades track changes, and stops it
when entering simulation or destroying the HUD. Master/mute, music and UI-cue
levels are separate player preferences, not acoustic-sensor controls.
