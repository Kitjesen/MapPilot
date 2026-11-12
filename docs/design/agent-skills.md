# Agent skills: configuration and live verification

Updated 2026-11-12. This adds task guides to the existing LingTu conversation
loop. It does not run Hermes as another service and does not change robot control.

## Hermes comparison

Reference version: Hermes v0.23.1,
`3b421578588243f920886e6fe79a6faa87ea026e`.

| Responsibility | Hermes | LingTu today |
| --- | --- | --- |
| Runtime/model settings | `$HERMES_HOME/config.yaml`, normally `~/.hermes/config.yaml` | Product default connections in `config/system/models.yaml`; conversation choices in `decision/llm/conversation_models.py` |
| API credentials | Profile `.env`, credential management | Host environment; never put values in model YAML or chat |
| Account authentication | Auth/credential storage owned by the selected provider | LingTu Codex account store under `$XDG_STATE_HOME/lingtu/model_accounts` |
| Task guides | Profile `skills/**/SKILL.md`, plus configured/plugin sources | Packaged `decision/skills/` and `$XDG_CONFIG_HOME/lingtu/skills/` |
| Persistent assistant/user memory | Profile `memories/` and its memory tool | Curated preferences in the existing conversation SQLite store; see [agent-preferences.md](agent-preferences.md). `query_memory` remains robot spatial memory |

Hermes supports `hermes config`, `hermes config get <key>`,
`hermes config set <key> <value>`, `hermes config edit`, `hermes config check`,
`hermes config path` and `hermes config env-path`.
These commands configure Hermes, not LingTu. Do not mix either application's credentials.

Sources: upstream `hermes_cli/config.py`, `tools/skills_tool.py`,
`tools/memory_tool.py`. Protocol scope is documented in
[`SKILLS_NOTICE.md`](../../src/decision/llm/SKILLS_NOTICE.md).

## Install a local guide

Default on this Windows workstation:
`C:/Users/99563/.config/lingtu/skills/<folder>/SKILL.md`.
Linux defaults to `~/.config/lingtu/skills/<folder>/SKILL.md`.
`XDG_CONFIG_HOME` changes the config root. No restart is needed after editing a guide.

```markdown
---
name: Inspection preparation
description: Review named inspection points before starting a task.
---

Read the selected map and existing inspection points using the available tools.
Ask for missing location information. Apply the normal tool confirmations;
loading this document does not authorize robot motion.
```

Use `skills_list()` to get metadata, then `skill_view(name)` to read a guide.
IDs are explicit, for example `builtin/map-review` or `local/inspection-preparation`.
A local folder cannot silently replace a bundled guide. Names follow the folder
path; the metadata name is a display title.

Use `skill_view(name, file_path="references/check.md")` for a Markdown/text file
inside that guide's folder. Files must be UTF-8 and no larger than 64 KiB; the
reader rejects traversal outside the selected skill, missing files, invalid
metadata and oversized documents rather than returning partial instructions.
Discovery reports invalid entries separately so one bad guide does not hide good ones.

Loading does not run scripts, interpolate shell commands, install packages,
download skills, save credentials or grant permissions. Hermes supports additional
activation behavior that this lightweight implementation deliberately does not migrate.

## Bundled workflows

- `builtin/model-connections`: inspect actual configuration, distinguish Codex
  device authorization from API keys, and separate configured/reachable/inference/tool-use evidence.
- `builtin/map-review`: inspect map identity and content epoch, require observed
  bounds, preview edits, save a new copy and report actual build/navigation readiness.

These are Markdown instructions. The existing runtime `@skill` mechanism still
discovers callable Python functions; `query_memory` still reads spatial memory.

## Verification

- 49 backend skill/conversation tests passed. Ruff passed.
- ESLint passed for the tool-label service; the production web build passed.
  Existing chunk-size and mixed static/dynamic import warnings remain.
- Wheel build succeeded; both bundled guides, the reference document and provenance
  notice were verified inside `lingtu-2.3.0-py3-none-any.whl`.
- Real `kimi-k3`, low reasoning, read-only conversation completed in 82.83 seconds:
  `skills_list` (92 ms) -> `skill_view` (60 ms) -> reference `skill_view` (55 ms)
  -> `get_model_connections` (1466 ms) -> model reply.
- The first reply incorrectly described the read-only configuration audit as no
  real inference. The guide was clarified; without restarting Host, a second real
  request reread the changed document (36 ms) and corrected this distinction.
  That request completed in 21.83 seconds. This is evidence of on-demand reload,
  not a guarantee that every future model answer is accurate.
- Evidence: `artifacts/ui/20261112-agent/skills-real-conversation.json` and
  `skills-real-hot-read.json`. Conversation title: `真实技能验证 · 模型连接诊断`.
- The same persisted conversation was opened in the production web console:
  `artifacts/ui/20261112-agent/skills-first-turn.png` shows the actual tool chain;
  `skills-real-ui.png` shows the follow-up correction. The attempted PrintWindow
  recording captured stale browser content and is not valid workflow evidence;
  use the screenshots and persisted turn results above.
- These are local production-module checks, not robot field verification.
  Codex is still unconnected. Tavily live search still needs its server credential.
