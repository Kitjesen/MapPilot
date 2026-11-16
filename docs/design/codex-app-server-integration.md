# LingTu Codex App Server integration

LingTu uses the official Codex App Server as the production Codex provider
boundary. The Host launches `codex app-server --stdio`; Codex owns
authentication, `CODEX_HOME`, account state, threads and turns. LingTu reads
the JSON-RPC account/thread/turn events and keeps its own conversation store.

```text
LingTu AgentPlanner
    ↓
CodexAppServer (`codex app-server --stdio`)
    ↓ JSON-RPC
Codex CLI / CODEX_HOME / official Codex service
```

The current slice implements `initialize`, `account/read`,
`account/login/start`, `account/login/cancel`, `account/logout`,
`thread/start`, `turn/start`, `item/agentMessage/delta`, `turn/completed`,
and the main item/plan/thread/MCP status events. It starts an ephemeral thread
per LingTu turn and keeps Codex separate from LingTu motion tools.

The old Hermes-derived device-code module remains in `codex.py` as a migration
reference and fallback implementation. It is not used by the Codex connection
routes or conversation provider.

## Connection boundary

`base_url` remains supported for the ordinary API-backed providers and Product
LLM configuration. The OpenAI-compatible clients pass that value to the SDK;
the server keeps the API key in the configured environment variable. The Web
model picker does not accept arbitrary URLs or keys, so changing a provider's
endpoint remains a Host configuration change.

Codex is intentionally different: its production conversation path does not
send requests to `base_url` and does not read a LingTu token. It starts the
official App Server, which reads the Codex CLI login state from `CODEX_HOME`.
ACP agents likewise use their configured executable and stdio protocol rather
than an OpenAI-compatible URL.

## File and command permissions

Permission mode is mapped explicitly:

- `read_only`: `sandbox=read-only`, no external writes;
- `default`: `sandbox=workspace-write`, `approvalPolicy=on-request`, with
  Codex-native file/command requests bound to the current LingTu approval;
- `full_access`: `sandbox=workspace-write`, `approvalPolicy=never`, allowing
  the explicitly selected workspace to be edited without another approval.

The ACP adapter forwards text and `session/request_permission`; its permission
responses use the same selected conversation mode. LingTu's own map and robot
tools still pass through the existing conversation approval and motion-safety
boundary. Workspace edits are controlled by the selected conversation
permission mode, not by a hidden provider switch.
