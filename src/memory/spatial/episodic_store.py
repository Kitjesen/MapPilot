"""Persistent SQLite backend for EpisodicMemory.

Drop-in store that survives process restarts.  The in-memory list in
EpisodicMemory is still used as a write-through cache; on startup the
cache is re-hydrated from the DB.

Usage::

    store = SqliteEpisodicStore("~/.lingtu/episodic.db")
    store.open()
    store.save(record)
    records = store.load_recent(n=50)
    store.close()
"""
from __future__ import annotations

import json
import logging
import pickle
import sqlite3
import threading
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from memory.spatial.episodic import MemoryRecord

logger = logging.getLogger(__name__)

_SCHEMA = """
CREATE TABLE IF NOT EXISTS episodic (
    ts       REAL    PRIMARY KEY,
    position BLOB    NOT NULL,
    labels   TEXT    NOT NULL DEFAULT '[]',
    room     TEXT    NOT NULL DEFAULT '',
    desc     TEXT    NOT NULL DEFAULT '',
    emb      BLOB
);
CREATE INDEX IF NOT EXISTS idx_episodic_ts ON episodic (ts);
"""


class SqliteEpisodicStore:
    """Thread-safe SQLite store for MemoryRecord objects.

    All public methods are safe to call from any thread.  Internally the
    connection is created once per open() call and protected by a lock.
    """

    # Write-buffer: accumulate records and flush in batch to reduce fsync pressure.
    # Worst-case data loss window on crash = _FLUSH_INTERVAL seconds.
    _BUFFER_LIMIT: int = 20
    _FLUSH_INTERVAL: float = 2.0

    def __init__(self, db_path: str | Path) -> None:
        self._path = Path(db_path).expanduser().resolve()
        self._conn: sqlite3.Connection | None = None
        self._lock = threading.Lock()
        self._write_buf: list = []  # MemoryRecord objects pending flush
        self._last_flush: float = 0.0

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def open(self) -> None:
        """Open (or create) the SQLite database."""
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._conn = sqlite3.connect(
            str(self._path),
            check_same_thread=False,
            timeout=10.0,
        )
        self._conn.execute("PRAGMA journal_mode=WAL;")
        self._conn.executescript(_SCHEMA)
        self._conn.commit()
        logger.info("EpisodicStore opened: %s (%d records)", self._path, self.count())

    def flush(self) -> None:
        """Force-flush the write buffer to disk (call before shutdown)."""
        with self._lock:
            if self._write_buf and self._conn:
                self._flush_locked()

    def _flush_locked(self) -> None:
        """Flush write buffer — caller must hold self._lock."""
        if not self._write_buf:
            return
        import time as _time
        rows = [
            (
                r.timestamp,
                pickle.dumps(r.position),
                json.dumps(r.labels, ensure_ascii=False),
                r.room_type,
                r.description,
                pickle.dumps(r.embedding) if r.embedding is not None else None,
            )
            for r in self._write_buf
        ]
        self._conn.executemany(
            "INSERT OR REPLACE INTO episodic(ts, position, labels, room, desc, emb)"
            " VALUES (?, ?, ?, ?, ?, ?)",
            rows,
        )
        self._conn.commit()
        self._write_buf.clear()
        self._last_flush = _time.time()

    def close(self) -> None:
        """Flush pending writes and close the database."""
        with self._lock:
            if self._conn:
                self._flush_locked()
                self._conn.close()
                self._conn = None

    # ------------------------------------------------------------------
    # Write
    # ------------------------------------------------------------------

    def save(self, record: MemoryRecord) -> None:
        """Buffer a single MemoryRecord; flushes automatically when the
        buffer fills or the flush interval elapses."""
        import time as _time
        with self._lock:
            self._write_buf.append(record)
            now = _time.time()
            if (len(self._write_buf) >= self._BUFFER_LIMIT
                    or (now - self._last_flush) >= self._FLUSH_INTERVAL):
                self._flush_locked()

    def save_batch(self, records: list[MemoryRecord]) -> None:
        """Persist multiple records in one transaction."""
        rows = [
            (
                r.timestamp,
                pickle.dumps(r.position),
                json.dumps(r.labels, ensure_ascii=False),
                r.room_type,
                r.description,
                pickle.dumps(r.embedding) if r.embedding is not None else None,
            )
            for r in records
        ]
        with self._lock:
            self._conn.executemany(
                "INSERT OR REPLACE INTO episodic(ts, position, labels, room, desc, emb)"
                " VALUES (?, ?, ?, ?, ?, ?)",
                rows,
            )
            self._conn.commit()

    def prune_oldest(self, keep: int) -> int:
        """Delete all but the most recent *keep* records.  Returns deleted count."""
        with self._lock:
            deleted = self._conn.execute(
                """DELETE FROM episodic WHERE ts NOT IN (
                       SELECT ts FROM episodic ORDER BY ts DESC LIMIT ?
                   )""",
                (keep,),
            ).rowcount
            self._conn.commit()
        return deleted

    # ------------------------------------------------------------------
    # Read
    # ------------------------------------------------------------------

    def load_recent(self, n: int) -> list[MemoryRecord]:
        """Return the *n* most recent records, oldest-first."""

        with self._lock:
            cur = self._conn.execute(
                "SELECT ts, position, labels, room, desc, emb"
                " FROM episodic ORDER BY ts DESC LIMIT ?",
                (n,),
            )
            rows = cur.fetchall()
        records = [_row_to_record(r) for r in rows]
        records.reverse()  # oldest first, matching in-memory order
        return records

    def load_all(self) -> list[MemoryRecord]:
        """Return all records, oldest-first."""
        with self._lock:
            cur = self._conn.execute(
                "SELECT ts, position, labels, room, desc, emb"
                " FROM episodic ORDER BY ts ASC"
            )
            rows = cur.fetchall()
        return [_row_to_record(r) for r in rows]

    def load_range(self, start_ts: float, end_ts: float) -> list[MemoryRecord]:
        """Return records in the [start_ts, end_ts] range, oldest-first."""
        with self._lock:
            cur = self._conn.execute(
                "SELECT ts, position, labels, room, desc, emb"
                " FROM episodic WHERE ts >= ? AND ts <= ? ORDER BY ts ASC",
                (start_ts, end_ts),
            )
            rows = cur.fetchall()
        return [_row_to_record(r) for r in rows]

    def count(self) -> int:
        """Return total number of stored records."""
        with self._lock:
            return self._conn.execute("SELECT COUNT(*) FROM episodic").fetchone()[0]


# ------------------------------------------------------------------
# Internal helper
# ------------------------------------------------------------------

def _row_to_record(row: tuple) -> MemoryRecord:
    from memory.spatial.episodic import MemoryRecord
    ts, pos_b, labels_j, room, desc, emb_b = row
    return MemoryRecord(
        timestamp=ts,
        position=pickle.loads(pos_b),  # noqa: S301  # trusted local episodic store
        labels=json.loads(labels_j),
        room_type=room,
        description=desc,
        embedding=pickle.loads(emb_b) if emb_b is not None else None,  # noqa: S301  # trusted local episodic store
    )
