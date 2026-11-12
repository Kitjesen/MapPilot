# Grok avatar renderer provenance

Vendored from https://github.com/RongleCat/grok-bot-avatar-engine at commit
`a995c0675e36ba113de41ead3a5d9be3fcd567e3` (2026-11-05).
The reference package identifies its extracted renderer as Grok Bot 0.78.1.
Its MIT license, copyright 2026 RongleCat, is retained in LICENSE. That license
does not automatically establish or replace rights to upstream xAI code or assets.
The engine/render/mount interface files were adapted from that repository.
The core data/extracted/src/avatar-engine.js is instead an avatar-only declaration
closure from the user-supplied Windows Grok Bot 1.0.1 installation at
D:/Program Files/Grok Bot/resources/app.asar, renderer asset
dist/renderer/assets/index.eager-common-1bSx8o0X.js. TypeScript AST symbol references
selected 777 declarations with no external module imports. Source character
offsets are retained, and the export footer aliases the 1.0.1 symbols to the
reference interface. App bootstrap, account/network clients, and native modules
are excluded. Upstream rights remain with their respective holders.

The runtime closure consists of src/index.mjs, src/engine.mjs, src/render.mjs,
src/mount.mjs and data/extracted/src/avatar-engine.js. There are no runtime npm,
network, or filesystem dependencies. Geometry remains unchanged.
LingTu's local render.mjs patch calls driver.holdFace(options.expression) and
settles after the original seek procedure when a static expression is supplied.
One local addition in the core host controller exposes holdExpression(name):
it holds a selected face, or restores the current native state's default hold
when passed null. Automatic idle/work/streaming retain their native expression
and blink tracks. These explicit additions are not claimed as upstream APIs.
The reference 0.78.1 closure was not used as the final core because actual render
checks exposed missing free symbols; no guessed constants were substituted.
src/index.d.mts is a local, corrected declaration of the public entry.
GrokAvatar.tsx is the LingTu React integration and lifecycle/state adapter.
