# Assistant preference memory

Updated 2026-11-12. This is curated user preference storage in LingTu's existing
conversation runtime. It does not change robot spatial memory or navigation.

## Operator workflow

- “请记住：回复先给结论，再给必要依据。” invokes `remember_preference`.
- “你记住了我的哪些偏好？” invokes `read_assistant_memory`.
- “把那条回复偏好改成……” reads entries, then invokes `update_preference`.
- “忘记这条偏好。” reads entries, then invokes `forget_preference`.

Default mode presents the existing tool approval card before writing; read-only
mode refuses writes; full-access mode can execute the requested write directly.
All calls retain tool timing and conversation history, without creating a motion
task. This change adds tool labels, not a new UI layout.

The model receives a fresh preference snapshot at each turn, including new
conversations. Writes during a turn return their result through the tool; they
do not rewrite the already-submitted prompt. Current user instructions take
priority. The assistant is instructed to save only explicitly requested durable
preferences, never credentials, temporary observations or motion permissions.

## Storage and ownership

`ConversationStore` owns the `assistant_preferences` table in the existing
conversation SQLite database. The default is
`$XDG_STATE_HOME/lingtu/conversations.sqlite3`, falling back to
`~/.local/state/lingtu/conversations.sqlite3`. Existing
`LINGTU_CONVERSATION_DB_PATH` or the Agent's `conversation_db_path` selects the
database, including test isolation. No additional configuration is required.

All conversations using that Host/database share one operator's preferences.
This is not multi-user account isolation. Deleting a conversation does not remove
preferences. Forgetting a preference does not erase old messages, exports, share
snapshots, or an in-flight model request. The model must not restore a forgotten
entry from history without a new explicit request.

Entries have stable IDs. Updates/deletes require exact previous content, so a
pending approval cannot overwrite a preference changed in another conversation.
Repeated identical adds return the existing entry. Limits are 100 entries,
1000 characters per entry and 12000 total characters. A full store returns an
error without dropping old entries; the operator chooses what to shorten/remove.

## Upstream and scope

See [MEMORY_NOTICE](../../src/decision/llm/MEMORY_NOTICE.md) for the pinned Hermes
reference and differences. There is no background extraction, autonomous memory
rewriting, vector store, Markdown mirror, second Agent loop or new dependency.

This is separate from `query_memory`, which queries observed robot locations.
Preferences cannot establish a map position, physical readiness or permission
to move. Real field behavior is outside this local conversation verification.

## Verification

- 57 preference and Agent conversation tests passed. They cover persistence,
  exact repeated adds, stale edits/deletes, capacity without eviction, read-only
  refusal, default approval/denial for all three writes, and fresh cross-chat context.
- The diagnostic Host contract test passed; its RPC listener now uses an OS-assigned
  port so a second isolated HTTP diagnostic does not conflict with the running Host.
- Ruff, the changed frontend service's ESLint, production web build and wheel build
  passed. The preference module and provenance notice were verified inside the wheel.
- Real `kimi-k3`/low requests used the production Gateway and Agent at port 4330,
  with an isolated conversation database: remember 19.55 s, new-chat read 16.45 s,
  update 26.67 s, forget 22.62 s. Each step used a separate conversation. All three
  writes requested default-mode approval; the verification driver approved only
  the expected preference tool. No physical tasks were created.
- SQL observations matched the actual tool results; the isolated preference table
  was empty after the final delete. The operator's usual database was not modified
  by these test preferences. This does not validate every provider or robot deployment.
- Reproduction driver and full results:
  `artifacts/agent-preferences-verification/verify_live.py` and `live-results.json`.
- The usual local console at port 4326 was restarted after verifying zero active
  turns; its preference table is ready. The isolated port-4330 verification Host
  was stopped after validation. No robot Product was started or restarted.
