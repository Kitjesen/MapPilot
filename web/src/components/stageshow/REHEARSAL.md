# LingTu motion integration — 2026-11-12

## Source and scope

Stageshow commit `299140fafd08df55a9c234e610c3f64af3f45531`.
The installed skill contains all 372 upstream files, including 71 example folders.
Copied `kit/motion/Roll.tsx`, its CSS, `kit/reduced-motion.ts`, and MIT license.
Adaptations: outgoing text is hidden from assistive technology; a scoped CSS
reduced-motion rule stops transitional animation immediately. Upstream React
hook code is retained as vendor code; TypeScript and browser checks cover it.

`examples/reorder` informed queue travel and easing. Production queue movement
uses measured row positions and Web Animations, follows server order, and keeps
real controls. No demo Stage, fictional replies, cursor playback, model runtime,
or new npm dependency was added to LingTu. No drag gesture was added.

## Checks

- Production TypeScript/Vite build passed; application changes passed ESLint.
- 42 existing conversation UI tests passed.
- `artifacts/ui/campus-geography-20261111/conversation-motion-check.mjs`:
  server-returned reordering animates, edits/cancel keep semantics, tool receipt
  changes animate through Roll, unchanged polling does not replay animations,
  reduced motion suppresses both, and 390px viewport has no card overflow.
- Browser requests were intercepted; no paid model or real robot action ran.
- Screenshots: `conversation-motion.png`, `conversation-motion-compact.png`.
- Recording: `conversation-motion.webm` in the same artifact directory.

The upstream reorder still was captured with Stageshow film.mjs as
`stageshow-reorder-reference.png`; its report flagged unequal title-bar insets.
That upstream still is not a motion or rehearsal-gate pass. LingTu is a live
application rather than a scripted Stage; verification here uses actual UI
event/animation checks and recorded controlled responses, not a demo gate.

## Limits

No G20 hardware or real-model latency acceptance. Remaining example effects are
installed references, not automatically integrated production features.
