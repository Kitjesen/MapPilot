"""SQLite conversation history, independent of motion ownership and SDK internals."""

from __future__ import annotations

import json
import math
import os
import secrets
import sqlite3
import threading
import time
import uuid
from pathlib import Path

from decision.tasks.conversation_inputs import conversation_user_input


class ConversationError(ValueError):
    pass


def validate_tool_arguments(schema: dict, args: dict) -> str | None:
    properties = schema.get("properties", {})
    if args.keys() - properties.keys():
        return "unknown tool argument"
    for name in schema.get("required", []):
        if name not in args:
            return f"missing required argument {name}"
    for name, value in args.items():
        kind = properties[name].get("type")
        if kind == "number" and (isinstance(value, bool) or not isinstance(value, (float, int))
                                  or not math.isfinite(value)):
            return f"{name} must be a finite number"
        if kind == "string" and (not isinstance(value, str) or not value.strip()):
            return f"{name} must be a nonempty string"
    return None


class ConversationStore:
    def __init__(self, path: str | Path | None = None):
        if path is None:
            state = Path(os.environ.get("XDG_STATE_HOME") or Path.home() / ".local" / "state")
            path = os.environ.get("LINGTU_CONVERSATION_DB_PATH") or state / "lingtu" / "conversations.sqlite3"
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._db = sqlite3.connect(path, check_same_thread=False)
        self._db.row_factory = sqlite3.Row
        with self._db:
            self._db.executescript("""
                CREATE TABLE IF NOT EXISTS conversations (id TEXT PRIMARY KEY);
                CREATE TABLE IF NOT EXISTS assistant_preferences (
                    id TEXT PRIMARY KEY, content TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS conversation_context (
                    conversation_id TEXT PRIMARY KEY, through_ordinal INTEGER NOT NULL,
                    summary TEXT NOT NULL, model TEXT, updated_at REAL NOT NULL
                );
                CREATE TABLE IF NOT EXISTS turns (
                    ordinal INTEGER PRIMARY KEY AUTOINCREMENT,
                    conversation_id TEXT NOT NULL, turn_id TEXT UNIQUE NOT NULL,
                    message_id TEXT NOT NULL, input TEXT NOT NULL, reply_to TEXT,
                    state TEXT NOT NULL, kind TEXT, text TEXT, run_id TEXT, failure_reason TEXT,
                    model_items TEXT NOT NULL DEFAULT '[]',
                    UNIQUE(conversation_id, message_id)
                );
                CREATE TABLE IF NOT EXISTS messages (
                    conversation_id TEXT NOT NULL, id TEXT NOT NULL,
                    seq INTEGER NOT NULL, role TEXT NOT NULL, text TEXT NOT NULL,
                    turn_id TEXT NOT NULL, run_id TEXT, kind TEXT,
                    PRIMARY KEY(conversation_id, seq)
                );
                CREATE TABLE IF NOT EXISTS attachments (
                    conversation_id TEXT NOT NULL, id TEXT NOT NULL,
                    name TEXT NOT NULL, mime_type TEXT NOT NULL,
                    data BLOB NOT NULL, content TEXT NOT NULL,
                    PRIMARY KEY(conversation_id, id)
                );
                CREATE TABLE IF NOT EXISTS approvals (
                    conversation_id TEXT NOT NULL, approval_id TEXT PRIMARY KEY,
                    turn_id TEXT NOT NULL, tool_call_id TEXT NOT NULL,
                    tool_name TEXT NOT NULL, arguments TEXT NOT NULL,
                    state TEXT NOT NULL DEFAULT 'pending',
                    UNIQUE(turn_id, tool_call_id)
                );
                CREATE TABLE IF NOT EXISTS conversation_shares (
                    conversation_id TEXT PRIMARY KEY, token TEXT UNIQUE NOT NULL,
                    created_at REAL NOT NULL, payload TEXT NOT NULL
                );
            """)
            conversation_columns = {row["name"] for row in self._db.execute("PRAGMA table_info(conversations)")}
            for name, declaration in {
                "title": "TEXT NOT NULL DEFAULT '新对话'", "created_at": "REAL NOT NULL DEFAULT 0",
                "updated_at": "REAL NOT NULL DEFAULT 0", "archived": "INTEGER NOT NULL DEFAULT 0",
                "forked_from": "TEXT",
            }.items():
                if name not in conversation_columns:
                    self._db.execute(f"ALTER TABLE conversations ADD COLUMN {name} {declaration}")
            migrated_at = time.time()
            old = self._db.execute("SELECT id FROM conversations WHERE created_at=0").fetchall()
            for row in old:
                first = self._db.execute("""SELECT text FROM messages WHERE conversation_id=? AND role='user'
                    ORDER BY seq LIMIT 1""", (row["id"],)).fetchone()
                title = " ".join(first["text"].split())[:80] if first else ""
                self._db.execute("UPDATE conversations SET title=?,created_at=?,updated_at=? WHERE id=?",
                                 (title or "新对话", migrated_at, migrated_at, row["id"]))
            columns = {row["name"] for row in self._db.execute("PRAGMA table_info(turns)")}
            for name, declaration in {
                "model": "TEXT", "resolved_model": "TEXT", "reasoning": "TEXT NOT NULL DEFAULT 'default'",
                "attachment_ids": "TEXT NOT NULL DEFAULT '[]'",
                "permission_mode": "TEXT NOT NULL DEFAULT 'default'",
                "map_context": "TEXT", "persona": "TEXT NOT NULL DEFAULT ''",
                "started_at": "REAL", "finished_at": "REAL", "reply_quote": "TEXT",
                "queue_order": "INTEGER NOT NULL DEFAULT 0",
            }.items():
                if name not in columns:
                    self._db.execute(f"ALTER TABLE turns ADD COLUMN {name} {declaration}")
            message_columns = {row["name"] for row in self._db.execute("PRAGMA table_info(messages)")}
            self._db.execute("UPDATE turns SET queue_order=ordinal WHERE queue_order=0")
            if "metadata" not in message_columns:
                self._db.execute("ALTER TABLE messages ADD COLUMN metadata TEXT")
            if "created_at" not in message_columns:
                self._db.execute("ALTER TABLE messages ADD COLUMN created_at REAL")
            if "revision" not in message_columns:
                self._db.execute("ALTER TABLE messages ADD COLUMN revision INTEGER NOT NULL DEFAULT 1")
            self._db.execute("UPDATE turns SET state='interrupted', failure_reason='host_restarted' WHERE state IN ('running','queued')")
            self._db.execute("UPDATE approvals SET state='cancelled' WHERE state='pending'")
            for row in self._db.execute("""SELECT m.*,t.state,t.text AS final_text,t.kind AS final_kind
                FROM messages m JOIN turns t ON m.turn_id=t.turn_id
                WHERE m.role='assistant' AND m.metadata LIKE '%"streaming"%'""").fetchall():
                metadata = json.loads(row["metadata"] or "null")
                if metadata and metadata.get("status") == "streaming":
                    metadata["status"] = row["state"]
                    self._db.execute("""UPDATE messages SET text=?,kind=?,metadata=?,revision=revision+1
                        WHERE conversation_id=? AND id=?""",
                        (row["final_text"] or row["text"], row["final_kind"] or row["kind"], json.dumps(metadata),
                         row["conversation_id"], row["id"]))

    def preferences(self) -> dict:
        with self._lock:
            entries = [dict(row) for row in self._db.execute(
                "SELECT id,content FROM assistant_preferences ORDER BY rowid")]
        return {"entries": entries, "scope": "host_operator", "characters": sum(len(e["content"]) for e in entries),
                "character_limit": 12000}

    def edit_preference(self, action: str, *, content: str = "", entry_id: str = "", old_content: str = "") -> dict:
        if action not in {"remember", "update", "forget"}:
            return {"error": "unsupported_preference_action"}
        if action != "forget":
            if not isinstance(content, str) or not content.strip() or len(content) > 1000:
                return {"error": "preference_content_must_contain_1_to_1000_characters"}
            content = content.strip()
        with self._lock, self._db:
            entries = self.preferences()["entries"]
            previous = None
            if action != "remember":
                previous = next((entry for entry in entries if entry["id"] == entry_id), None)
                if previous is None:
                    return {"error": "preference_not_found"}
                # Approval may wait while another conversation edits this same preference.
                if previous["content"] != old_content:
                    return {"error": "preference_changed", "hint": "Read memory again before requesting a new change."}
            if action == "forget":
                self._db.execute("DELETE FROM assistant_preferences WHERE id=?", (entry_id,))
                return {"removed": True, "id": entry_id, "history_erased": False}
            if action == "remember":
                existing = next((entry for entry in entries if entry["content"] == content), None)
                if existing:
                    return {"saved": True, "created": False, "entry": existing}
            size = sum(len(entry["content"]) for entry in entries) + len(content)
            if previous:
                size -= len(previous["content"])
            if size > 12000 or (action == "remember" and len(entries) >= 100):
                return {"error": "preference_memory_full", "hint": "Ask which existing preferences to shorten or forget. Nothing was evicted."}
            if action == "remember":
                entry_id = "preference-" + uuid.uuid4().hex
                self._db.execute("INSERT INTO assistant_preferences(id,content) VALUES (?,?)", (entry_id, content))
            else:
                self._db.execute("UPDATE assistant_preferences SET content=? WHERE id=?", (content, entry_id))
            return {"saved": True, "created": action == "remember", "entry": {"id": entry_id, "content": content}}

    def create(self) -> dict:
        conversation_id = f"conversation-{uuid.uuid4().hex}"
        with self._lock, self._db:
            created_at = time.time()
            self._db.execute("INSERT INTO conversations (id,created_at,updated_at) VALUES (?,?,?)",
                             (conversation_id, created_at, created_at))
        return {"conversation_id": conversation_id}

    @staticmethod
    def _metadata(row) -> dict:
        metadata = {"conversation_id": row["id"], "title": row["title"], "created_at": row["created_at"],
                    "updated_at": row["updated_at"], "archived": bool(row["archived"])}
        if row["forked_from"]:
            metadata["forked_from"] = json.loads(row["forked_from"])
        return metadata

    def fork(self, conversation_id: str, message_id: str) -> dict:
        with self._lock, self._db:
            self._require(conversation_id)
            source = self._db.execute("SELECT * FROM conversations WHERE id=?", (conversation_id,)).fetchone()
            point = self._db.execute("""SELECT m.*,t.state,t.finished_at FROM messages m
                JOIN turns t ON t.turn_id=m.turn_id WHERE m.conversation_id=? AND m.id=?""",
                (conversation_id, message_id)).fetchone()
            if point is None:
                raise ConversationError("message_not_found")
            if (point["role"] != "assistant" or point["state"] not in {"completed", "failed", "cancelled", "interrupted"}
                    or self._db.execute("""SELECT 1 FROM messages WHERE turn_id=? AND role='assistant'
                        AND seq>?""", (point["turn_id"], point["seq"])).fetchone()):
                raise ConversationError("fork_point_conflict")
            # Streamed answers keep their first seq even when later tool rows arrive.
            turns = self._db.execute("""SELECT t.* FROM turns t JOIN messages m ON m.turn_id=t.turn_id
                WHERE t.conversation_id=? AND t.state IN ('completed','failed','cancelled','interrupted')
                AND m.role IN ('user','assistant')
                AND (? IS NULL OR t.finished_at IS NULL OR t.finished_at<=?)
                GROUP BY t.turn_id HAVING MAX(m.seq)<=?
                ORDER BY COALESCE(t.started_at,t.finished_at,0),t.ordinal""",
                (conversation_id, point["finished_at"], point["finished_at"], point["seq"])).fetchall()
            turn_ids = {row["turn_id"]: f"turn-{uuid.uuid4().hex}" for row in turns}
            by_turn = {row["turn_id"]: [] for row in turns}
            for row in self._db.execute("""SELECT * FROM messages
                WHERE conversation_id=? AND role IN ('user','assistant') ORDER BY seq""", (conversation_id,)):
                if row["turn_id"] in by_turn:
                    by_turn[row["turn_id"]].append(row)
            messages = [message for turn in turns for message in by_turn[turn["turn_id"]]]
            message_ids = {row["id"]: f"message-{uuid.uuid4().hex}" for row in messages}
            target_id = f"conversation-{uuid.uuid4().hex}"
            created_at = time.time()
            suffix = " · 分支"
            title = source["title"][:80 - len(suffix)].rstrip() + suffix
            origin = {"conversation_id": conversation_id, "message_id": message_id,
                      "title": source["title"], "seq": point["seq"]}
            self._db.execute("""INSERT INTO conversations (id,title,created_at,updated_at,forked_from)
                VALUES (?,?,?,?,?)""", (target_id, title, created_at, created_at, json.dumps(origin, ensure_ascii=False)))
            attachment_ids = set()
            for row in turns:
                reply_to = message_ids.get(row["reply_to"])
                attachment_ids.update(json.loads(row["attachment_ids"]))
                self._db.execute("""INSERT INTO turns
                    (conversation_id,turn_id,message_id,input,reply_to,reply_quote,state,kind,text,failure_reason,
                     model,resolved_model,reasoning,attachment_ids,started_at,finished_at)
                    VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)""",
                    (target_id, turn_ids[row["turn_id"]], message_ids[row["message_id"]], row["input"], reply_to,
                     row["reply_quote"], row["state"], row["kind"], row["text"], row["failure_reason"],
                     row["model"], row["resolved_model"], row["reasoning"], row["attachment_ids"],
                     row["started_at"], row["finished_at"]))
            for seq, row in enumerate(messages, start=1):
                original_metadata = json.loads(row["metadata"] or "null") or {}
                metadata = {key: original_metadata[key] for key in ("status", "choices", "plan") if key in original_metadata}
                self._db.execute("""INSERT INTO messages
                    (conversation_id,id,seq,role,text,turn_id,kind,created_at,metadata)
                    VALUES (?,?,?,?,?,?,?,?,?)""",
                    (target_id, message_ids[row["id"]], seq, row["role"], row["text"], turn_ids[row["turn_id"]],
                     row["kind"], row["created_at"], json.dumps(metadata) if metadata else None))
            for attachment_id in attachment_ids:
                self._db.execute("""INSERT INTO attachments (conversation_id,id,name,mime_type,data,content)
                    SELECT ?,id,name,mime_type,data,content FROM attachments WHERE conversation_id=? AND id=?""",
                    (target_id, conversation_id, attachment_id))
            return self._metadata(self._db.execute("SELECT * FROM conversations WHERE id=?", (target_id,)).fetchone())

    def metadata(self, conversation_id: str) -> dict:
        with self._lock:
            self._require(conversation_id)
            row = self._db.execute("SELECT * FROM conversations WHERE id=?", (conversation_id,)).fetchone()
            return self._metadata(row)

    def list(self, archived: bool = False) -> dict:
        with self._lock:
            rows = self._db.execute("SELECT * FROM conversations WHERE archived=? ORDER BY updated_at DESC,id",
                                    (int(archived),)).fetchall()
            return {"conversations": [self._metadata(row) for row in rows]}

    def search(self, query: str, archived: bool = False) -> dict:
        query = query.strip()
        if not query:
            return {"results": []}
        with self._lock:
            rows = self._db.execute("""SELECT c.id AS conversation_id,c.title,m.id AS message_id,
                m.seq,m.role,m.text FROM messages m JOIN conversations c ON c.id=m.conversation_id
                WHERE c.archived=? AND m.role IN ('user','assistant') AND instr(lower(m.text),lower(?))>0
                ORDER BY c.updated_at DESC,m.seq DESC LIMIT 30""", (int(archived), query)).fetchall()
            results = []
            for row in rows:
                body = row["text"]
                start = max(0, body.lower().find(query.lower()) - 40)
                snippet = ("…" if start else "") + " ".join(body[start:start + 240].split())
                if start + 240 < len(body):
                    snippet += "…"
                results.append({**{key: row[key] for key in (
                    "conversation_id", "title", "message_id", "seq", "role")}, "snippet": snippet})
            return {"results": results}

    def update(self, conversation_id: str, *, title: str | None = None, archived: bool | None = None) -> dict:
        if title is not None:
            title = title.strip()
            if not title or len(title) > 80:
                raise ConversationError("invalid_conversation_title")
        with self._lock, self._db:
            current = self.metadata(conversation_id)
            if archived and self._db.execute(
                "SELECT 1 FROM turns WHERE conversation_id=? AND state IN ('running','queued')", (conversation_id,)
            ).fetchone() is not None:
                raise ConversationError("conversation_active")
            self._db.execute("UPDATE conversations SET title=?,archived=?,updated_at=? WHERE id=?",
                             (title if title is not None else current["title"],
                              int(archived if archived is not None else current["archived"]),
                              time.time(), conversation_id))
            return self.metadata(conversation_id)

    def _require(self, conversation_id: str):
        if self._db.execute("SELECT 1 FROM conversations WHERE id=?", (conversation_id,)).fetchone() is None:
            raise ConversationError("conversation_not_found")

    @staticmethod
    def _turn(row) -> dict:
        turn = {key: row[key] for key in (
            "conversation_id", "message_id", "turn_id", "state", "kind", "text", "run_id", "failure_reason",
            "model", "reasoning", "permission_mode", "persona", "started_at", "finished_at",
        ) if row[key] is not None}
        if row["resolved_model"] is not None:
            turn["model"] = row["resolved_model"]
        if row["reply_to"] is not None:
            turn["reply_to_message_id"] = row["reply_to"]
        if row["reply_quote"] is not None:
            turn["reply_quote"] = row["reply_quote"]
        if row["state"] == "queued":
            turn["queue_position"] = row["queue_order"]
        return turn

    def quoted_text(self, conversation_id: str, turn_id: str) -> str | None:
        with self._lock:
            row = self._db.execute("""SELECT COALESCE(t.reply_quote,m.text) AS text FROM turns t LEFT JOIN messages m
                ON m.conversation_id=t.conversation_id AND m.id=t.reply_to
                WHERE t.conversation_id=? AND t.turn_id=?""", (conversation_id, turn_id)).fetchone()
            return row["text"] if row is not None else None

    def attachment(self, conversation_id: str, attachment_id: str) -> dict | None:
        with self._lock:
            self._require(conversation_id)
            row = self._db.execute("SELECT * FROM attachments WHERE conversation_id=? AND id=?",
                                   (conversation_id, attachment_id)).fetchone()
            if row is None:
                return None
            return {"id": row["id"], "name": row["name"], "mime_type": row["mime_type"],
                    "size": len(row["data"]), "data": row["data"], "content": json.loads(row["content"])}

    @staticmethod
    def attachment_receipt(attachment: dict) -> dict:
        return {key: attachment[key] for key in ("id", "name", "mime_type", "size")}

    def save_attachment(self, conversation_id: str, attachment_id: str, attachment: dict) -> dict:
        with self._lock, self._db:
            old = self.attachment(conversation_id, attachment_id)
            if old is not None:
                if any(old[key] != attachment[key] for key in ("name", "mime_type", "data")):
                    raise ConversationError("attachment_id_conflict")
                return self.attachment_receipt(old)
            self._db.execute("INSERT INTO attachments VALUES (?,?,?,?,?,?)", (
                conversation_id, attachment_id, attachment["name"], attachment["mime_type"],
                attachment["data"], json.dumps(attachment["content"], ensure_ascii=False)))
            return self.attachment_receipt({**attachment, "id": attachment_id})

    def attachments(self, conversation_id: str, attachment_ids: list[str]) -> list[dict]:
        with self._lock:
            self._require(conversation_id)
            result = []
            for attachment_id in attachment_ids:
                attachment = self.attachment(conversation_id, attachment_id)
                if attachment is None:
                    raise ConversationError("attachment_not_found")
                result.append(attachment)
            return result

    def accept(self, conversation_id: str, message_id: str, text: str, reply_to: str | None,
               *, control: bool = False, model: str | None = None, reasoning: str = "default",
               attachment_ids: list[str] | None = None, resolved_model: str | None = None,
               permission_mode: str = "default", map_context: dict | None = None, persona: str = "",
               reply_quote: str | None = None, enqueue: bool = False) -> tuple[dict, bool]:
        if permission_mode not in {"read_only", "default", "full_access"}:
            raise ConversationError("invalid_permission_mode")
        if not isinstance(persona, str) or len(persona) > 2000:
            raise ConversationError("invalid_persona")
        if reply_quote is not None and (not reply_to or not isinstance(reply_quote, str) or not reply_quote.strip()):
            raise ConversationError("invalid_reply_quote")
        attachment_ids = attachment_ids or []
        with self._lock, self._db:
            self._require(conversation_id)
            old = self._db.execute("SELECT * FROM turns WHERE conversation_id=? AND message_id=?",
                                   (conversation_id, message_id)).fetchone()
            if old is not None:
                if (old["input"] != text or old["reply_to"] != reply_to or old["reply_quote"] != reply_quote or old["model"] != model
                        or old["reasoning"] != reasoning or json.loads(old["attachment_ids"]) != attachment_ids
                        or old["permission_mode"] != permission_mode or old["persona"] != persona
                        or json.loads(old["map_context"] or "null") != map_context):
                    raise ConversationError("message_id_conflict")
                return self._turn(old), False
            if self.metadata(conversation_id)["archived"]:
                raise ConversationError("conversation_archived")
            queued = enqueue and not control and self._db.execute(
                "SELECT 1 FROM turns WHERE conversation_id=? AND state IN ('running','queued')",
                (conversation_id,)
            ).fetchone() is not None
            if not control and not enqueue and self._db.execute(
                "SELECT 1 FROM turns WHERE conversation_id=? AND state='running' AND run_id IS NULL",
                (conversation_id,)
            ).fetchone() is not None:
                raise ConversationError("turn_active")
            if reply_to and self._db.execute(
                "SELECT 1 FROM messages WHERE conversation_id=? AND id=?", (conversation_id, reply_to)
            ).fetchone() is None:
                raise ConversationError("reply_not_found")
            turn_id = f"turn-{uuid.uuid4().hex}"
            self._db.execute("""INSERT INTO turns
                (conversation_id,turn_id,message_id,input,reply_to,state,model,reasoning,attachment_ids,resolved_model,
                 permission_mode,map_context,persona,started_at,reply_quote) VALUES (?,?,?,?,?,'running',?,?,?,?,?,?,?,?,?)""",
                (conversation_id, turn_id, message_id, text, reply_to, model, reasoning,
                 json.dumps(attachment_ids), resolved_model, permission_mode, json.dumps(map_context), persona, time.time(), reply_quote))
            self.append(conversation_id, turn_id, "user", text, message_id=message_id)
            self._db.execute("UPDATE turns SET queue_order=ordinal WHERE turn_id=?", (turn_id,))
            if queued:
                self._db.execute("UPDATE turns SET state='queued',started_at=NULL WHERE turn_id=?", (turn_id,))
            return self.turn(conversation_id, turn_id), True

    def edit_queued(self, conversation_id: str, turn_id: str, text: str | None) -> dict:
        if text is not None and (not text.strip() or len(text) > 8000):
            raise ConversationError("invalid_queue_text")
        with self._lock, self._db:
            turn = self.turn(conversation_id, turn_id)
            if turn["state"] != "queued":
                raise ConversationError("queue_closed_conflict")
            if text is None:
                self.finish(conversation_id, turn_id, state="cancelled", kind="queue",
                            text="已取消待执行消息。", model_items=[])
            else:
                self._db.execute("UPDATE turns SET input=? WHERE turn_id=?", (text, turn_id))
                self._db.execute("UPDATE messages SET text=?,revision=revision+1 WHERE conversation_id=? AND id=?",
                                 (text, conversation_id, turn["message_id"]))
            return self.turn(conversation_id, turn_id)

    def claim_queued(self, conversation_id: str, turn_id: str) -> str | None:
        with self._lock, self._db:
            if self.turn(conversation_id, turn_id)["state"] != "queued":
                return None
            if self._db.execute("SELECT 1 FROM turns WHERE conversation_id=? AND state='running'",
                                (conversation_id,)).fetchone():
                return None
            first = self._db.execute("""SELECT turn_id,input,state FROM turns WHERE conversation_id=?
                AND state IN ('running','queued') ORDER BY queue_order,ordinal LIMIT 1""", (conversation_id,)).fetchone()
            if first["turn_id"] != turn_id:
                return None
            self._db.execute("UPDATE turns SET state='running',started_at=? WHERE turn_id=?", (time.time(), turn_id))
            return first["input"]

    def promote_queued(self, conversation_id: str, turn_id: str) -> dict:
        with self._lock, self._db:
            if self.turn(conversation_id, turn_id)["state"] != "queued":
                raise ConversationError("queue_closed_conflict")
            position = self._db.execute("SELECT MIN(queue_order)-1 FROM turns WHERE conversation_id=? AND state='queued'",
                                        (conversation_id,)).fetchone()[0]
            self._db.execute("UPDATE turns SET queue_order=? WHERE turn_id=?", (position, turn_id))
            return self.turn(conversation_id, turn_id)

    def append(self, conversation_id: str, turn_id: str, role: str, text: str,
               *, message_id: str | None = None, run_id: str | None = None, kind: str | None = None, metadata: dict | None = None) -> dict:
        with self._lock, self._db:
            seq = self._db.execute("SELECT COALESCE(MAX(seq),0)+1 FROM messages WHERE conversation_id=?",
                                   (conversation_id,)).fetchone()[0]
            message = dict(conversation_id=conversation_id, id=message_id or f"message-{uuid.uuid4().hex}",
                           seq=seq, role=role, text=text, turn_id=turn_id, run_id=run_id, kind=kind,
                           created_at=time.time())
            self._db.execute("""INSERT INTO messages
                (conversation_id,id,seq,role,text,turn_id,run_id,kind,created_at,metadata) VALUES (?,?,?,?,?,?,?,?,?,?)""",
                             (*message.values(), json.dumps(metadata)))
            self._db.execute("UPDATE conversations SET updated_at=? WHERE id=?", (message["created_at"], conversation_id))
            if role == "user" and seq == 1:
                title = " ".join(text.split())[:80]
                if title:
                    self._db.execute("UPDATE conversations SET title=? WHERE id=? AND title='新对话'",
                                     (title, conversation_id))
            if metadata is not None:
                message["metadata"] = metadata
            message["revision"] = 1
            return message

    def update_message(self, conversation_id: str, message_id: str, text: str, metadata: dict) -> dict:
        with self._lock, self._db:
            self._db.execute("UPDATE messages SET text=?,metadata=?,revision=revision+1 WHERE conversation_id=? AND id=?",
                             (text, json.dumps(metadata), conversation_id, message_id))
            row = self._db.execute("SELECT * FROM messages WHERE conversation_id=? AND id=?",
                                   (conversation_id, message_id)).fetchone()
            return {**dict(row), "metadata": metadata}

    def bind_run(self, conversation_id: str, turn_id: str, run_id: str):
        with self._lock, self._db:
            self._db.execute("UPDATE turns SET run_id=? WHERE conversation_id=? AND turn_id=?",
                             (run_id, conversation_id, turn_id))

    @staticmethod
    def _approval(row) -> dict:
        return {"approval_id": row["approval_id"], "turn_id": row["turn_id"],
                "tool_name": row["tool_name"], "arguments": json.loads(row["arguments"]), "state": row["state"]}

    def request_approval(self, conversation_id: str, turn_id: str, tool_call_id: str,
                         tool_name: str, arguments: dict) -> dict:
        with self._lock, self._db:
            if self.turn(conversation_id, turn_id)["state"] != "running":
                raise ConversationError("approval_closed")
            approval_id = f"approval-{uuid.uuid4().hex}"
            self._db.execute("""INSERT INTO approvals
                (conversation_id,approval_id,turn_id,tool_call_id,tool_name,arguments)
                VALUES (?,?,?,?,?,?)""",
                (conversation_id, approval_id, turn_id, tool_call_id, tool_name,
                 json.dumps(arguments, ensure_ascii=False)))
            return self.approval(conversation_id, approval_id)

    def approval(self, conversation_id: str, approval_id: str) -> dict:
        with self._lock:
            self._require(conversation_id)
            row = self._db.execute("SELECT * FROM approvals WHERE conversation_id=? AND approval_id=?",
                                   (conversation_id, approval_id)).fetchone()
            if row is None:
                raise ConversationError("approval_not_found")
            return self._approval(row)

    def decide_approval(self, conversation_id: str, approval_id: str, approved: bool) -> dict:
        with self._lock, self._db:
            approval = self.approval(conversation_id, approval_id)
            state = "approved" if approved else "denied"
            if approval["state"] == state:
                return approval
            if approval["state"] != "pending":
                raise ConversationError("approval_decision_conflict")
            if self.turn(conversation_id, approval["turn_id"])["state"] != "running":
                raise ConversationError("approval_closed")
            self._db.execute("UPDATE approvals SET state=? WHERE approval_id=?", (state, approval_id))
            return {**approval, "state": state}

    def finish(self, conversation_id: str, turn_id: str, *, state: str, kind: str,
               text: str, model_items: list, failure_reason: str = "", metadata: dict | None = None):
        with self._lock, self._db:
            self._db.execute("""UPDATE turns SET state=?,kind=?,text=?,model_items=?,failure_reason=?,
                finished_at=COALESCE(finished_at,?)
                WHERE conversation_id=? AND turn_id=?""",
                (state, kind, text, json.dumps(model_items, ensure_ascii=False), failure_reason,
                 time.time(), conversation_id, turn_id))
            self._db.execute("UPDATE approvals SET state='cancelled' WHERE conversation_id=? AND turn_id=? AND state='pending'",
                             (conversation_id, turn_id))
            rows = self._db.execute("""SELECT id,metadata FROM messages
                WHERE conversation_id=? AND turn_id=? AND role='assistant'""", (conversation_id, turn_id)).fetchall()
            final_message = None
            for row in rows:
                details = json.loads(row["metadata"] or "null")
                if details and details.get("status") == "streaming":
                    details.update(metadata or {})
                    details["status"] = state
                    self._db.execute("""UPDATE messages SET text=?,kind=?,metadata=?,revision=revision+1,
                        run_id=(SELECT run_id FROM turns WHERE turn_id=?) WHERE conversation_id=? AND id=?""",
                        (text, kind, json.dumps(details), turn_id, conversation_id, row["id"]))
                    final_message = dict(self._db.execute("SELECT * FROM messages WHERE conversation_id=? AND id=?",
                                                         (conversation_id, row["id"])).fetchone())
                    final_message["metadata"] = details
            return final_message

    def context_checkpoint(self, conversation_id: str) -> dict:
        with self._lock:
            row = self._db.execute("SELECT * FROM conversation_context WHERE conversation_id=?", (conversation_id,)).fetchone()
            return dict(row) if row else {"through_ordinal": 0, "summary": ""}

    def _context_record(self, row) -> dict:
        approvals = [self._approval(item) for item in self._db.execute(
            "SELECT * FROM approvals WHERE turn_id=?", (row["turn_id"],))]
        record = {**{key: row[key] for key in ("ordinal", "turn_id", "input", "state", "kind", "text", "failure_reason", "reply_quote")},
                "model_items": json.loads(row["model_items"]), "approvals": approvals,
                "pending_approval": any(item["state"] == "pending" for item in approvals)}
        # Cancelled/interrupted turns can have durable tool receipts but no SDK replay items.
        if not record["model_items"]:
            record["messages"] = []
            for message in self._db.execute("SELECT role,kind,text FROM messages WHERE turn_id=? ORDER BY seq", (row["turn_id"],)):
                content = message["text"]
                if message["role"] == "tool":
                    try:
                        content = json.loads(content)
                    except json.JSONDecodeError:
                        pass
                record["messages"].append({"role": message["role"], "kind": message["kind"], "content": content})
        return record

    def context_turns(self, conversation_id: str, exclude_turn_id: str, after_ordinal: int) -> list:
        with self._lock:
            return [self._context_record(row) for row in self._db.execute("""SELECT * FROM turns
                WHERE conversation_id=? AND turn_id!=? AND ordinal>?
                ORDER BY COALESCE(started_at,finished_at,0),ordinal""", (conversation_id, exclude_turn_id, after_ordinal))]

    def save_context(self, conversation_id: str, turn_id: str, previous: int, through: int, summary: str, model: str | None) -> bool:
        with self._lock, self._db:
            if (self.turn(conversation_id, turn_id)["state"] != "running"
                    or self.context_checkpoint(conversation_id)["through_ordinal"] != previous):
                return False
            if self._db.execute("""SELECT 1 FROM turns WHERE conversation_id=? AND ordinal<=?
                AND (state IN ('running','queued') OR EXISTS
                    (SELECT 1 FROM approvals WHERE approvals.turn_id=turns.turn_id AND approvals.state='pending'))""",
                (conversation_id, through)).fetchone():
                return False
            self._db.execute("""INSERT INTO conversation_context VALUES (?,?,?,?,?)
                ON CONFLICT(conversation_id) DO UPDATE SET through_ordinal=excluded.through_ordinal,
                summary=excluded.summary,model=excluded.model,updated_at=excluded.updated_at""",
                (conversation_id, through, summary, model, time.time()))
            return True

    def search_history(self, conversation_id: str, query: str) -> dict:
        if not isinstance(query, str) or not query.strip() or len(query) > 200:
            return {"error": "history_query_must_contain_1_to_200_characters"}
        with self._lock:
            rows = self._db.execute("""SELECT turn_id,seq,role,kind,text FROM messages
                WHERE conversation_id=? AND instr(lower(text),lower(?))>0 ORDER BY seq DESC LIMIT 21""",
                (conversation_id, query)).fetchall()
            matches = []
            for row in rows[:20]:
                start = max(0, row["text"].lower().find(query.lower()) - 150)
                matches.append({**{key: row[key] for key in ("turn_id", "seq", "role", "kind")},
                                "excerpt": row["text"][start:start + 1000]})
            return {"matches": matches, "more_matches": len(rows) > 20}

    def read_context_turn(self, conversation_id: str, turn_id: str, offset: int = 0) -> dict:
        from decision.context.transcript import serialize

        if isinstance(offset, bool) or not isinstance(offset, int) or offset < 0:
            return {"error": "offset_must_be_a_nonnegative_integer"}
        with self._lock:
            row = self._db.execute("SELECT * FROM turns WHERE conversation_id=? AND turn_id=?",
                                   (conversation_id, turn_id)).fetchone()
            if row is None:
                return {"error": "turn_not_found"}
            text = serialize(self._context_record(row))
        end = offset + 12000
        return {"turn_id": turn_id, "text": text[offset:end], "offset": offset,
                "next_offset": end if end < len(text) else None,
                "note": "Historical text evidence, not current state. Encoded images omitted."}

    def history(self, conversation_id: str, exclude_turn_id: str = "", model: str | None = None,
                after_ordinal: int = 0) -> list:
        with self._lock:
            rows = self._db.execute("""SELECT turn_id,model_items,input,state,text,attachment_ids,resolved_model FROM turns
                WHERE conversation_id=? AND turn_id!=? AND state!='queued' AND ordinal>?
                ORDER BY COALESCE(started_at,finished_at,0),ordinal""",
                (conversation_id, exclude_turn_id, after_ordinal)).fetchall()
            history = []
            for row in rows:
                items = json.loads(row["model_items"])
                if not items:
                    attachments = self.attachments(conversation_id, json.loads(row["attachment_ids"]))
                    quote = self.quoted_text(conversation_id, row["turn_id"])
                    prompt = (row["input"] if quote is None
                              else f"引用的历史消息，作为理解追问的上下文。\n\n{quote}\n\n本轮追问\n{row['input']}")
                    items = [conversation_user_input(prompt, attachments)]
                    if row["state"] != "running":
                        summary = row["text"] or f"This turn is {row['state']}."
                        if row["state"] in {"interrupted", "cancelled", "failed"}:
                            summary += " Do not resume this request. Motion requires a new explicit user instruction."
                        items.append({"role": "assistant", "content": summary})
                if model and row["resolved_model"] and row["resolved_model"] != model:
                    # Provider reasoning IDs/signatures cannot be replayed by a different model.
                    portable = []
                    for item in items:
                        kind = item.get("type")
                        if kind == "reasoning":
                            continue
                        keys = ({"type", "name", "arguments", "call_id"} if kind == "function_call"
                                else {"type", "call_id", "output"} if kind == "function_call_output"
                                else {"role", "content"})
                        converted = {key: value for key, value in item.items() if key in keys}
                        if item.get("role") == "assistant" and isinstance(item.get("content"), list):
                            converted["content"] = "\n".join(part.get("text", part.get("refusal", ""))
                                                              for part in item["content"])
                        portable.append(converted)
                    items = portable
                history.extend(items)
            return history

    def turn(self, conversation_id: str, turn_id: str) -> dict:
        with self._lock:
            self._require(conversation_id)
            row = self._db.execute("SELECT * FROM turns WHERE conversation_id=? AND turn_id=?",
                                   (conversation_id, turn_id)).fetchone()
            if row is None:
                raise ConversationError("turn_not_found")
            return self._turn(row)

    def snapshot(self, conversation_id: str, *, before_seq: int | None = None, limit: int | None = None) -> dict:
        with self._lock:
            self._require(conversation_id)
            messages = self._db.execute("""SELECT * FROM messages WHERE conversation_id=?
                AND (? IS NULL OR seq<?) ORDER BY seq DESC LIMIT ?""",
                (conversation_id, before_seq, before_seq, limit if limit is not None else -1)).fetchall()[::-1]
            cursor = messages[0]["seq"] if messages and self._db.execute(
                "SELECT 1 FROM messages WHERE conversation_id=? AND seq<?", (conversation_id, messages[0]["seq"])
            ).fetchone() else None
            if limit is not None and before_seq is None:
                queued = self._db.execute("""SELECT m.* FROM messages m JOIN turns t ON m.turn_id=t.turn_id
                    WHERE m.conversation_id=? AND t.state='queued' AND m.role='user'""", (conversation_id,)).fetchall()
                messages = sorted({row["seq"]: row for row in [*messages, *queued]}.values(), key=lambda row: row["seq"])
            turns = self._db.execute("SELECT * FROM turns WHERE conversation_id=? ORDER BY ordinal",
                                     (conversation_id,)).fetchall()
            turn_attachments = {row["turn_id"]: json.loads(row["attachment_ids"]) for row in turns}
            receipts = {row["id"]: dict(row) for row in self._db.execute(
                "SELECT id,name,mime_type,length(data) AS size FROM attachments WHERE conversation_id=?",
                (conversation_id,))}
            visible_messages = []
            for row in messages:
                message = dict(row)
                message["metadata"] = json.loads(row["metadata"]) if row["metadata"] else None
                if row["role"] == "user":
                    message["attachments"] = [receipts[attachment_id]
                                              for attachment_id in turn_attachments[row["turn_id"]]]
                visible_messages.append(message)
            approvals = self._db.execute("SELECT * FROM approvals WHERE conversation_id=? ORDER BY rowid",
                                         (conversation_id,)).fetchall()
            return {"conversation_id": conversation_id, "messages": visible_messages, "history_cursor": cursor,
                    "turns": [self._turn(row) for row in turns], "approvals": [self._approval(row) for row in approvals],
                    "metadata": self.metadata(conversation_id), "share": self.share_receipt(conversation_id),
                    "server_time": time.time()}

    def share_receipt(self, conversation_id: str) -> dict | None:
        with self._lock:
            row = self._db.execute("SELECT token,created_at,payload FROM conversation_shares WHERE conversation_id=?",
                                   (conversation_id,)).fetchone()
            if row is None:
                return None
            return {"token": row["token"], "created_at": row["created_at"],
                    "message_count": len(json.loads(row["payload"])["messages"])}

    def create_share(self, conversation_id: str) -> dict:
        with self._lock, self._db:
            self._require(conversation_id)
            existing = self.share_receipt(conversation_id)
            if existing is not None:
                return existing
            messages = self._db.execute("""SELECT m.role,m.text,m.seq FROM messages m
                JOIN turns t ON m.turn_id=t.turn_id
                WHERE m.conversation_id=? AND m.role IN ('user','assistant') AND t.state NOT IN ('running','queued')
                ORDER BY m.seq""", (conversation_id,)).fetchall()
            if not messages:
                raise ConversationError("share_empty")
            created_at = time.time()
            payload = {"title": self.metadata(conversation_id)["title"], "created_at": created_at,
                       "messages": [dict(row) for row in messages]}
            token = secrets.token_urlsafe(32)
            self._db.execute("INSERT INTO conversation_shares VALUES (?,?,?,?)",
                             (conversation_id, token, created_at, json.dumps(payload, ensure_ascii=False)))
            return {"token": token, "created_at": created_at, "message_count": len(messages)}

    def revoke_share(self, conversation_id: str) -> dict:
        with self._lock, self._db:
            self._require(conversation_id)
            self._db.execute("DELETE FROM conversation_shares WHERE conversation_id=?", (conversation_id,))
            return {"revoked": True}

    def shared(self, token: str) -> dict:
        with self._lock:
            row = self._db.execute("SELECT payload FROM conversation_shares WHERE token=?", (token,)).fetchone()
            if row is None:
                raise ConversationError("share_not_found")
            return json.loads(row["payload"])

    def close(self):
        with self._lock:
            self._db.close()
