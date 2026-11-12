# Context management provenance

Reference: NousResearch/hermes-agent v0.23.1, commit
`3b421578588243f920886e6fe79a6faa87ea026e`.
Reviewed source: `agent/context_compressor.py`, `context_compressor_summary.py`,
`context_compressor_tail.py`, `context_compressor_window.py`,
`context_compressor_prompt_rules.py`, and
`website/docs/developer-guide/context-compression-and-caching.md`.
Upstream MIT license is retained in `LICENSE.hermes.txt`.

LingTu implements a capability adaptation, not a drop-in copy of ContextCompressor:
summarize old context, keep a recent raw tail, preserve uncertainty explicitly,
commit only a completed summary, retain original history for recovery, and expose
progress. It uses the existing SDK model adapter, ConversationStore and tool events.
It does not run another Hermes Agent or import Hermes private modules.

Intentional differences:
- Text-character policy (48000 including turn instructions/tools), not Hermes
  real-token usage anchors or a fraction of provider context windows.
- Four recent conversation turns retained whole; tool calls/results are not split.
- Tool-free summaries through the same selected model, 24000-character source
  pages and a 6000-character structured summary, with a 120-second total deadline.
- SQLite checkpoint at a completed-turn boundary, not Hermes session lineage.
- No in-loop compression, provider-native compaction, cache tuning, background
  preparation, autonomous memory extraction or automatic overflow retry.
- Images/large geometry payloads stay in the original application records and
  are omitted from textual summaries; the summary cannot verify their contents.

Summarization is lossy. Schema checks do not prove semantic fidelity. Completed
source records, tool evidence, approvals and recovery tools remain available;
failure/cancellation leaves the last checkpoint and source history intact.
