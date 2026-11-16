# Follow conversation preview

Reference: Stageshow `ai-chat` question / tool / reply rhythm. The rendering uses
LingTu's existing AgentConversation and InstructionTaskBar instead of copied UI.

Scope: preview files only; the production components are unchanged. All fetches
are answered by fixture data or rejected. No Host, model or robot is contacted.
The status and stop controls use the existing task presentation code.

Beats: selecting, following, lost, cancellation pending, cancelled. Playback is
15 seconds, once, with an explicit replay button; stages can also be selected.
The final cancellation changes the task bar, not the historical chat reply.

Checks: Chrome at 1000×920 and 390×844; selection, stop, confirmation, replay,
console errors, outbound requests, and horizontal overflow. Recording and stills
are in `build/follow-preview/`; `build/check-follow-preview.mjs` repeats the check.
The screenshot revealed narrow-screen stage labels wrapping; the controls now
use a separate row and keep each label whole. TypeScript checks include this
preview entry and its imported production components.

Stageshow gate: film.mjs does not recognize this ordinary product-component tree
as a Stage (`no visible stages`). It did not produce a passing gate; the custom
browser capture supplies the interaction evidence instead.

Not checked: real model interpretation, live perception, robot movement or NX
deployment. Proactive loss notifications are not simulated as implemented.
