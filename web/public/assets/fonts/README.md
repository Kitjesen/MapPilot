# Self-hosted interface fonts

LingTu serves these fonts from `/assets/fonts/`; the application does not make
runtime font requests to third-party services. This path uses the Gateway's
existing cacheable asset route.

## Inter Variable

- Source: https://rsms.me/inter/font-files/InterVariable.woff2
- Project: https://github.com/rsms/inter
- License: SIL Open Font License 1.1, stored in `Inter-OFL.txt`
- File: `InterVariable.woff2` (normal variable font; CSS uses weights 400–700)

## Noto Sans SC Variable

- Source: https://github.com/google/fonts/raw/main/ofl/notosanssc/NotoSansSC%5Bwght%5D.ttf
- License source: https://github.com/google/fonts/raw/main/ofl/notosanssc/OFL.txt
- License: SIL Open Font License 1.1, stored in `NotoSansSC-OFL.txt`
- File: `NotoSansSC-GB2312-VF.woff2`

The Noto source was restricted to the 400–700 weight axis and converted to
WOFF2 with FontTools 4.61.1 and Brotli 1.2.0. Its glyph set contains all 6,763
GB2312 Hanzi, supported non-ASCII characters found under `web/src`, CJK
punctuation, and fullwidth forms. Characters outside the subset continue
through the local CJK system-font fallbacks in `--font-ui`.
