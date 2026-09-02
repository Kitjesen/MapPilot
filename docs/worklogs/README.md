# Session worklogs

Session worklogs preserve task continuity when a long Codex conversation is
compacted or the desktop task index omits older messages. They are navigation
aids only: code, tests, architecture contracts, and `docs/CURRENT.md` remain the
sources of truth.

## Current Worklogs

| Worklog | Recovered transcript |
| --- | --- |
| [Explore session, 2026-08-24](./2026-08-24-explore-session.md) | Linked from the worklog when detailed visible history is needed |

## What to keep

- One short dated worklog for decisions, completed edits, verification evidence,
  blockers, and the next concrete action.
- A recovered visible transcript only when missing UI history matters. Do not
  routinely commit full chats.
- Links to existing plans, diffs, or evidence instead of copied snapshots.

Use `YYYY-MM-DD-<task>.md` names. Put machine-recovered transcripts under
`recovered/`.

## Recover a local Codex task

Run the repository tool with either the task/thread ID or an exact rollout path:

```powershell
.venv\Scripts\python.exe tools/docs/export_codex_thread.py `
  --thread-id <thread-id> `
  --output docs/worklogs/recovered/<date>-<task>-visible-transcript.md `
  --title "Codex visible transcript: <task>"
```

The tool reads active and archived rollouts under `CODEX_HOME` (or `~/.codex`).
It exports only completed `UserMessage` and `AgentMessage` text, deduplicates
identical completed item IDs, and ignores system/developer instructions,
reasoning, and tool records. A live file may have one unfinished final JSON
record; that record is reported and skipped so the export can be rerun after the
turn completes. An invalid complete/interior record, conflicting item ID, mixed
thread identity, or source/output collision stops the export instead of creating
a misleading recovery copy.

Review the visible text for credentials or private data before staging it.
Deleting Codex databases or `.omx` state is not a recovery step.

## Working rhythm

1. Update the short worklog after a meaningful implementation or verification
   milestone.
2. If the desktop view loses history, regenerate the visible transcript from the
   same thread ID.
3. When moving to a new task, start with the worklog; open the full transcript
   only for details that the summary intentionally omitted.
