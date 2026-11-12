# Astryx UI reference

Reviewed on 2026-11-10 for DOSO App and LingTu interface work.

## Source and local checkout

| Item | Value |
| --- | --- |
| Official source | https://github.com/meta/astryx |
| Pinned release | v0.7.11 |
| Commit | `df8a86a3ae9d0b2d6337c5cf3369488a6f5ac838` |
| License | MIT, with Meta copyright and license notice retained in the checkout |
| Local source | `third_party/research_ui/astryx/` |
| Current delivery | Local source reference and integration assessment |

The source checkout follows LingTu's existing `third_party/` convention. The
manifest records its location and revision. The source directory is local-only
under the existing ignore rules.

The official README describes the system as Beta. The core package at this
release declares React and React DOM >=19.0.0, plus StyleX ^0.19.0 as peer
dependencies. LingTu's `web/package.json` currently declares React and React DOM
^19.2.4, making its Vite web application a suitable integration candidate.
Runtime integration and a LingTu component build remain separate verification
steps. [Official release](https://github.com/meta/astryx/releases/tag/v0.7.11)

## Fit within DOSO

| Surface | Use |
| --- | --- |
| LingTu Web settings and equipment panels | Evaluate forms, cards, spacing, status feedback and themes |
| LingTu Web maps and 3D scene | Evaluate surrounding toolbars, selection menus and inspector panels alongside the existing Three.js scene |
| DOSO React Native App | Reference typography, spacing, interaction and motion rules when refining the existing native components |

Astryx's core renders React DOM and CSS. Direct component adoption belongs to
LingTu Web. The DOSO native App needs platform-specific implementations using
its current Expo and React Native components.

## Useful source entry points

All paths below are relative to `third_party/research_ui/astryx/`.

| Topic | Source |
| --- | --- |
| Setup, CSS import order and Theme provider | `packages/core/README.md` |
| Card composition | `packages/core/src/Card/`, `packages/core/src/ClickableCard/` |
| Consistent spacing and layout | `packages/core/src/Stack/`, `packages/core/src/Grid/`, `packages/core/src/FormLayout/` |
| Text hierarchy | `packages/core/src/Text/`, `packages/core/src/Heading/` |
| Menus and sheets | `packages/core/src/Popover/`, `packages/core/src/DropdownMenu/`, `packages/core/src/BottomSheet/` |
| Action feedback | `packages/core/src/Button/`, `packages/core/src/Toast/`, `packages/core/src/FieldStatus/` |
| Motion | `packages/core/src/motion/` |
| Theme definitions and tokens | `packages/core/src/theme/`, `packages/themes/neutral/` |
| Documentation and interactive examples | `apps/docsite/`, `apps/storybook/`, `apps/sandbox/` |

For the next Web component trial, use one equipment-settings form and one status
card. Evaluate the rendered spacing, keyboard focus, loading, error and disabled
states against LingTu's existing components. A successful trial should include
the Web type check and a browser interaction check before wider adoption.

The three stylesheet imports documented upstream are ordered as follows.

```css
@import '@astryxdesign/core/reset.css';
@import '@astryxdesign/core/astryx.css';
@import '@astryxdesign/theme-neutral/theme.css';
```

## Reproduce the reference checkout

Run from the LingTu repository root when the target directory is absent.

```powershell
git -c core.longpaths=true clone --depth 1 --branch v0.7.11 --single-branch https://github.com/meta/astryx.git third_party/research_ui/astryx
git -C third_party/research_ui/astryx rev-parse HEAD
```

The expected revision is recorded in the source table above. Full Astryx
Storybook installation, a production dependency integration, and target-device
performance testing remain pending.
