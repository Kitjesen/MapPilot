# Curated preference memory reference

Reference: NousResearch/hermes-agent v0.23.1, commit
`3b421578588243f920886e6fe79a6faa87ea026e`,
[`tools/memory_tool.py`](https://github.com/NousResearch/hermes-agent/blob/3b421578588243f920886e6fe79a6faa87ea026e/tools/memory_tool.py).
Upstream MIT terms are retained in `LICENSE.hermes.txt`.

LingTu adapts curated persistent entries, bounded prompt context, read-before-edit
and approved-write semantics. This is a local implementation, not the complete
Hermes memory store. Existing ConversationStore owns persistence, locking and
transactions in its SQLite database; no Hermes profile or process is loaded.

Differences: one Host operator preference list rather than MEMORY.md/USER.md;
exact entry ID and previous content rather than substring patching; no batch
edits, autonomous extraction, background consolidation or automatic eviction.
The prompt snapshot refreshes each turn rather than once per Hermes session.
User-requested writes use LingTu's existing default/read_only/full_access gates.
The tools are read_assistant_memory, remember_preference, update_preference and
forget_preference. Robot spatial query_memory is unchanged.

Limits: 100 entries, 1000 characters per entry and 12000 total characters.
Preferences belong to the local Host operator, not a browser or login identity.
Forgetting removes the live preference; it does not erase conversation history,
exports, shares or a request already sent to a model. Credentials and robot
authorization must not be stored here. No new dependencies were added.
