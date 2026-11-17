# Follow conversation preview

Open `http://127.0.0.1:47685/tests/follow-preview.html` with the web Vite server.
The preview renders the production `AgentConversation` and `FollowSelectionCard`.
All fetches are answered by fixtures or rejected: no Host, model or robot is contacted.

## In-chat flow

- Switch between original proposals (8) and the reviewed result (7) on one frozen frame.
- Open exclusion reasons and click candidate 8 to locate the advertising portrait in the original image.
- Candidate numbering stays unchanged. Original proposals, rejected candidates and unknown candidates cannot authorize following.
- Select a retained person and confirm in the card. Use chat messages `跟随状态`, `模拟目标丢失`, `停止跟随` and `重新选人` for the remaining simulated stages.
- Click the conversation title to rename it; Enter saves, Escape cancels. A failed save retains the draft for retry.
- The Bot menu uses a fixed 32 px avatar lane and compact text. Bot naming remains under `Bot 名称与外观`.

The image is MOT17-09 frame 0. `fixtures/follow-review.json` contains the saved
real Kimi K3 response: candidates 1–7 retained, candidate 8 rejected as an
advertising portrait. The call took 25.41 seconds. The preview translates that
exclusion reason into Chinese for display; the original response is retained.
Control receipts and lifecycle changes are simulated. This frame has no measured
RGB-D data; do not interpret the model's wording about depth as sensor evidence.

## Checks

`tests/followSelectionUi.test.ts` checks stable numbering, shared image/box
geometry, and confirmation gates in original, reviewed, unknown and terminal states.
`build/check-follow-selection.mjs` exercises the real browser at 1100×1120 and
390×920: comparison, exclusion lookup, title cancel/save/failure retry, compact Bot
menu, person confirmation, in-chat loss/cancel, network isolation and overflow.
Screenshots, recording and results are in `build/follow-preview/`.

Not checked: current live perception, robot movement or NX deployment. A single
reviewed frame does not establish general detection accuracy or liveness.
