# LingTu motion options

Review URL: http://127.0.0.1:47682/

This is an isolated, local interactive prototype: twelve product scenarios with
three per-scenario choices. It uses LingTu's existing font files and restrained
white/neutral-gray styling with blue focus and selection accents. CSS and Web Animations illustrate the proposed interactions;
there are no backend, model, robot, or production API calls. Local browser storage
holds selections; the copy button exports a readable selection list.

## References cloned on 2026-11-12

| Repository | Local checkout | Commit |
| --- | --- | --- |
| https://github.com/ibelick/motion-primitives | D:/inovxio/references/motion-primitives | 120f64f6ca60348e251f929e9c81f11ccbe45eda |
| https://github.com/assistant-ui/assistant-ui | D:/inovxio/references/assistant-ui | 53ab9cbae076b569b37189795b93b71af0a40117 |
| https://github.com/motiondivision/motion | D:/inovxio/references/motion | cdced3d0696601ea4b3eb1dc7205be1426f74c13 |
| https://github.com/benjitaylor/skills | C:/Users/99563/.codex/skills/stageshow | 299140fafd08df55a9c234e610c3f64af3f45531 |

Source inspection: Motion Primitives `animated-group.tsx`, `transition-panel.tsx`,
and `morphing-dialog.tsx`; assistant-ui `docs/tools/tool-ui.mdx`; Motion
`components/Reorder/Group.tsx`; Stageshow motion/craft guidance. This prototype
implements the comparative examples independently; none of the three new repos'
dependencies or runtimes were installed into LingTu. Motion Primitives README
declares MIT but this checkout has no top-level LICENSE file; verify component
license provenance before copying its source into production.

## Recommended applications

- B: tool disclosure, returned cards, queue reordering, message insertion,
  tab changes, and diagnostic side panels.
- A: frequent metric updates, connection status, and other operational signals.
- C: sparingly for menu entry or successful noncritical confirmations.
- Never defer emergency stop, control loss, failures, or safety interlocks for
  animation. Approval and execution remain different states.

## Verification

`node check.mjs` checked all 36 combinations, rapid switching/abort, selection
persistence, clipboard export, filtering, reduced motion, and 390px overflow.
No browser runtime errors were observed. `overview.png`, `results.png`, and
`compact.png` capture the page. These checks are prototype checks, not hardware
acceptance or evidence that the cloned libraries have been integrated.

Serve again from the LingTu repo using:
`.venv/Scripts/python.exe -m http.server 47682 --bind 127.0.0.1 --directory designs/motion-options`
