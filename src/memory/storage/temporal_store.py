"""TemporalStore — flat SQLite backend for per-entity temporal observations.

Flat schema optimised for label + time + spatial queries, unlike the
generic SqliteStore (pickle BLOB). Both TemporalMemoryModule (writer)
and GatewayModule (reader) instantiate this class pointing at the same
file — SQLite's WAL mode keeps concurrent access safe.

Schema:
    entity_observations(id, label, confidence,
                        pos_x, pos_y, robot_x, robot_y,
                        ts, session_id)
"""

from __future__ import annotations

import sqlite3
import time
import uuid
from pathlib import Path
from typing import Any

from core.msgs.numpy_compat import np

_DDL = """
CREATE TABLE IF NOT EXISTS entity_observations (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    label       TEXT    NOT NULL,
    confidence  REAL    NOT NULL DEFAULT 1.0,
    pos_x       REAL,
    pos_y       REAL,
    robot_x     REAL,
    robot_y     REAL,
    ts          REAL    NOT NULL,
    session_id  TEXT    NOT NULL DEFAULT '',
    embedding   BLOB                          -- CLIP feature vector (float32 numpy array)
);
CREATE INDEX IF NOT EXISTS idx_eo_ts    ON entity_observations(ts);
CREATE INDEX IF NOT EXISTS idx_eo_label ON entity_observations(label);
"""

DEFAULT_DB_PATH = "~/.lingtu/temporal_memory.db"

# One UUID per process lifetime — groups all observations from this run.
_SESSION_ID: str = str(uuid.uuid4())


class TemporalStore:
    """Flat SQLite store for per-entity observations.

    Thread-safe: SQLite WAL + check_same_thread=False.

    Usage::

        store = TemporalStore()                          # default path
        store = TemporalStore("/data/temporal.db")       # explicit path
        store = TemporalStore(":memory:")                # in-memory (tests)

        store.insert("person", 0.92, pos_x=3.1, pos_y=1.8,
                     robot_x=2.0, robot_y=1.5)

        rows = store.query(label="person", since_ts=time.time()-3600,
                           near_x=3.0, near_y=2.0, radius=2.0)
    """

    def __init__(self, db_path: str = "") -> None:
        raw = db_path or DEFAULT_DB_PATH
        self._path = raw if raw == ":memory:" else str(Path(raw).expanduser())
        self._conn: sqlite3.Connection | None = None

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------

    def _get_conn(self) -> sqlite3.Connection:
        if self._conn is None:
            if self._path != ":memory:":
                Path(self._path).parent.mkdir(parents=True, exist_ok=True)
            conn = sqlite3.connect(self._path, check_same_thread=False)
            conn.execute("PRAGMA journal_mode=WAL")
            conn.executescript(_DDL)
            conn.commit()
            self._conn = conn
        return self._conn

    def close(self) -> None:
        if self._conn is not None:
            try:
                self._conn.close()
            except Exception:
                pass
            self._conn = None

    def __del__(self) -> None:
        self.close()

    # ------------------------------------------------------------------
    # Write
    # ------------------------------------------------------------------

    def insert(
        self,
        label: str,
        confidence: float = 1.0,
        *,
        pos_x: float = 0.0,
        pos_y: float = 0.0,
        robot_x: float = 0.0,
        robot_y: float = 0.0,
        ts: float | None = None,
        session_id: str = "",
        embedding: np.ndarray | None = None,
    ) -> None:
        """Insert one entity observation.

        Args:
            embedding: Optional CLIP feature vector (float32 ndarray). When
                       stored, enables semantic similarity queries via
                       ``query_semantic()``.
        """
        ts = ts if ts is not None else time.time()
        sid = session_id or _SESSION_ID
        emb_blob: bytes | None = None
        if embedding is not None:
            try:
                emb_blob = np.asarray(embedding, dtype=np.float32).tobytes()
            except Exception:
                pass
        conn = self._get_conn()
        conn.execute(
            """INSERT INTO entity_observations
               (label, confidence, pos_x, pos_y, robot_x, robot_y, ts, session_id, embedding)
               VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)""",
            (label, float(confidence), float(pos_x), float(pos_y),
             float(robot_x), float(robot_y), float(ts), sid, emb_blob),
        )
        conn.commit()

    # ------------------------------------------------------------------
    # Query
    # ------------------------------------------------------------------

    def query(
        self,
        label: str | None = None,
        since_ts: float | None = None,
        near_x: float | None = None,
        near_y: float | None = None,
        radius: float | None = None,
        limit: int = 100,
    ) -> list[dict[str, Any]]:
        """Query observations with optional label / time / spatial filters.

        Spatial filtering is done in Python after the SQL WHERE clause because
        SQLite lacks sqrt(). The SQL limit is multiplied by 4 so we have enough
        rows to apply the radius filter before trimming to `limit`.
        """
        conditions: list[str] = []
        params: list[Any] = []

        if label:
            conditions.append("label = ?")
            params.append(label)
        if since_ts is not None:
            conditions.append("ts >= ?")
            params.append(float(since_ts))

        where = (" WHERE " + " AND ".join(conditions)) if conditions else ""
        sql_limit = limit * 4 if (near_x is not None) else limit
        sql = (
            f"SELECT label, confidence, pos_x, pos_y, robot_x, robot_y, ts"
            f" FROM entity_observations{where}"
            f" ORDER BY ts DESC LIMIT ?"
        )
        params.append(sql_limit)

        rows = self._get_conn().execute(sql, params).fetchall()
        results: list[dict[str, Any]] = []
        for row in rows:
            lbl, conf, px, py, rx, ry, ts_val = row
            # Spatial post-filter
            if near_x is not None and near_y is not None and radius is not None:
                dx = (px or 0.0) - near_x
                dy = (py or 0.0) - near_y
                if (dx * dx + dy * dy) ** 0.5 > radius:
                    continue
            results.append({
                "label": lbl,
                "confidence": conf,
                "pos_x": px,
                "pos_y": py,
                "robot_x": rx,
                "robot_y": ry,
                "ts": ts_val,
            })
            if len(results) >= limit:
                break
        return results

    def query_semantic(
        self,
        query_embedding: np.ndarray,
        top_k: int = 10,
        since_ts: float | None = None,
        label: str | None = None,
    ) -> list[dict[str, Any]]:
        """Find observations whose CLIP embedding is closest to query_embedding.

        Uses cosine similarity (in-Python dot product after L2 normalisation).
        Only rows with a stored embedding are considered.

        Args:
            query_embedding: 1-D float32 numpy array (CLIP feature vector).
            top_k:           Number of results to return.
            since_ts:        Optional unix timestamp lower bound.
            label:           Optional label filter.

        Returns:
            List of dicts (same schema as ``query()``), ordered by similarity
            descending.  Each dict has an additional ``similarity`` key (0-1).
        """
        if np is None:
            return []

        conditions = ["embedding IS NOT NULL"]
        params: list[Any] = []
        if label:
            conditions.append("label = ?")
            params.append(label)
        if since_ts is not None:
            conditions.append("ts >= ?")
            params.append(float(since_ts))

        where = " WHERE " + " AND ".join(conditions)
        sql = (
            f"SELECT label, confidence, pos_x, pos_y, robot_x, robot_y, ts, embedding"
            f" FROM entity_observations{where}"
            f" ORDER BY ts DESC LIMIT ?"
        )
        params.append(top_k * 20)  # fetch more candidates than needed

        rows = self._get_conn().execute(sql, params).fetchall()
        if not rows:
            return []

        q = np.asarray(query_embedding, dtype=np.float32)
        q_norm = q / (np.linalg.norm(q) + 1e-8)

        scored: list[tuple[float, dict[str, Any]]] = []
        emb_dim = len(q)
        for row in rows:
            lbl, conf, px, py, rx, ry, ts_val, emb_blob = row
            if not emb_blob:
                continue
            try:
                vec = np.frombuffer(emb_blob, dtype=np.float32)
                if vec.shape[0] != emb_dim:
                    continue
                sim = float(np.dot(q_norm, vec / (np.linalg.norm(vec) + 1e-8)))
            except Exception:
                continue
            scored.append((sim, {
                "label": lbl, "confidence": conf,
                "pos_x": px, "pos_y": py,
                "robot_x": rx, "robot_y": ry,
                "ts": ts_val, "similarity": sim,
            }))

        scored.sort(key=lambda x: x[0], reverse=True)
        return [d for _, d in scored[:top_k]]

    def count(self, label: str | None = None) -> int:
        """Return total observation count (optionally filtered by label)."""
        if label:
            row = self._get_conn().execute(
                "SELECT COUNT(*) FROM entity_observations WHERE label = ?", (label,)
            ).fetchone()
        else:
            row = self._get_conn().execute(
                "SELECT COUNT(*) FROM entity_observations"
            ).fetchone()
        return int(row[0]) if row else 0

    def purge_older_than(self, max_age_days: float = 7.0) -> int:
        """Delete observations older than max_age_days. Returns deleted count."""
        cutoff = time.time() - max_age_days * 86400.0
        conn = self._get_conn()
        cur = conn.execute(
            "DELETE FROM entity_observations WHERE ts < ?", (cutoff,)
        )
        conn.commit()
        return cur.rowcount
