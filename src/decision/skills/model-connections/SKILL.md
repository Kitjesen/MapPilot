---
name: Model connection diagnosis
description: Diagnose which model providers are configured, explain Codex App Server authorization versus API keys, and distinguish configuration from successful inference.
---

1. Call `get_model_connections` to inspect current provider/model configuration.
2. Distinguish connection methods: Codex uses the official Codex App Server authorization;
   the App Server reads the Codex CLI `CODEX_HOME` login state. OpenAI,
   Kimi, Claude and Qwen use their configured server-side API credentials.
   A missing Codex authorization is not a missing API key. Do not direct the operator
   through LingTu's historical Hermes-derived device-code flow.
3. Report provider configuration separately from successful model requests. A
   configured key or saved authorization does not prove model access, quota, or inference.
   A configuration audit does not send separate test requests to every listed
   model. Do not confuse that with the current conversational model generating
   a real response; do not say no inference happened merely because the tools
   were read-only. State which models were not independently tested.
4. If Codex is unconnected, direct the operator to the conversation toolbar's
   model connections panel: Codex -> login -> official authorization page.
   If opening the page fails, use Copy link and open it in the preferred browser.
   Never ask for passwords, device codes or tokens in the conversation.
5. For API connections, explain the environment variable reported by the tool;
   do not invent a login button or claim the UI can save keys when it cannot.
6. Read `references/verification.md` with `skill_view` when the user asks what
   counts as verified connectivity or how to interpret a failed request.

This guide authorizes no network request beyond the tools actually available
to the conversation, and no robot motion or persistent configuration changes.
