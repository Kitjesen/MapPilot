"""SQLite-backed persistent storage for episodic and tagged memory data.

SQLite backend adapted for the LingTu navigation system.
No DimOS dimos.utils.data dependency. Paths can be absolute, relative, or ``:memory:``.
"""

from __future__ import annotations

# Copyright 2025-2026 穹沛科技 (Qiongpei Technology)
# Adapted from DimOS (dimensionalOS/dimos), Apache 2.0 License
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import pickle
import re
import sqlite3
from collections.abc import Iterator
from pathlib import Path
from typing import Any

from memory.storage.timeseries_store import TimeSeriesStore

# 合法 SQL 标识符：字母/下划线开头，仅含字母数字下划线
_VALID_IDENTIFIER = re.compile(r"^[a-zA-Z_][a-zA-Z0-9_]*$")


def _validate_identifier(name: str) -> str:
    """校验 SQL 标识符，防止注入。"""
    if not _VALID_IDENTIFIER.match(name):
        raise ValueError(
            f"非法标识符 '{name}'：只允许字母/数字/下划线，且不能以数字开头"
        )
    if len(name) > 128:
        raise ValueError(f"标识符过长: {len(name)} > 128")
    return name


class SqliteStore(TimeSeriesStore):
    """SQLite 时序数据后端。

    数据以 pickle BLOB 存储，timestamp 列建有索引，支持高效时间范围查询。

    用法::

        # 绝对路径
        store = SqliteStore("/data/nav/scene_graph.db")

        # 相对路径（相对于当前工作目录）
        store = SqliteStore("logs/scene_graph.db")

        # 内存数据库（测试用）
        store = SqliteStore(":memory:")

        # 单库多表
        store = SqliteStore("/data/nav/sensors.db", table="scene_graph")

        # 保存 / 加载
        store.save(time.time(), entry)
        entry = store.find_closest(target_ts, tolerance=0.5)
    """

    def __init__(self, path: str | Path, table: str = "sensor_data") -> None:
        """初始化 SqliteStore。

        Args:
            path:  数据库文件路径，绝对/相对路径或 ":memory:"。
            table: 表名（仅字母数字下划线，长度 ≤128）。
        """
        self._path = str(path)
        self._table = _validate_identifier(table)
        self._conn: sqlite3.Connection | None = None

    # ------------------------------------------------------------------
    # 连接管理
    # ------------------------------------------------------------------

    def _get_conn(self) -> sqlite3.Connection:
        """获取（或创建）数据库连接，首次连接时建表。"""
        if self._conn is None:
            if self._path != ":memory:":
                # 确保父目录存在
                db_file = Path(self._path)
                db_file.parent.mkdir(parents=True, exist_ok=True)
            self._conn = sqlite3.connect(self._path, check_same_thread=False)
            self._create_table()
        return self._conn

    def _create_table(self) -> None:
        """建表（不存在时）并在 timestamp 列创建索引。"""
        conn = self._conn
        assert conn is not None
        conn.execute(f"""
            CREATE TABLE IF NOT EXISTS {self._table} (
                timestamp REAL PRIMARY KEY,
                data BLOB NOT NULL
            )
        """)  # noqa: S608  # table name from class attr, not user input
        conn.execute(f"""
            CREATE INDEX IF NOT EXISTS idx_{self._table}_timestamp
            ON {self._table}(timestamp)
        """)  # noqa: S608  # table name from class attr, not user input
        conn.commit()

    def close(self) -> None:
        """显式关闭数据库连接。"""
        if self._conn is not None:
            self._conn.close()
            self._conn = None

    def __del__(self) -> None:
        self.close()

    # ------------------------------------------------------------------
    # TimeSeriesStore 抽象方法实现
    # ------------------------------------------------------------------

    def _save(self, timestamp: float, data: Any) -> None:
        conn = self._get_conn()
        blob = pickle.dumps(data)
        conn.execute(
            f"INSERT OR REPLACE INTO {self._table} (timestamp, data) VALUES (?, ?)",
            (timestamp, blob),
        )
        conn.commit()

    def _load(self, timestamp: float) -> Any | None:
        conn = self._get_conn()
        cursor = conn.execute(
            f"SELECT data FROM {self._table} WHERE timestamp = ?",
            (timestamp,),
        )
        row = cursor.fetchone()
        if row is None:
            return None
        return pickle.loads(row[0])  # noqa: S301  # trusted local SQLite store

    def _delete(self, timestamp: float) -> Any | None:
        data = self._load(timestamp)
        if data is not None:
            conn = self._get_conn()
            conn.execute(
                f"DELETE FROM {self._table} WHERE timestamp = ?",
                (timestamp,),
            )
            conn.commit()
        return data

    def _iter_items(
        self, start: float | None = None, end: float | None = None
    ) -> Iterator[tuple[float, Any]]:
        conn = self._get_conn()
        query = f"SELECT timestamp, data FROM {self._table}"
        params: list[float] = []
        conditions: list[str] = []

        if start is not None:
            conditions.append("timestamp >= ?")
            params.append(start)
        if end is not None:
            conditions.append("timestamp < ?")
            params.append(end)

        if conditions:
            query += " WHERE " + " AND ".join(conditions)
        query += " ORDER BY timestamp"

        cursor = conn.execute(query, params)
        for row in cursor:
            ts: float = row[0]
            data: Any = pickle.loads(row[1])  # noqa: S301  # trusted local SQLite store
            yield (ts, data)

    def _find_closest_timestamp(
        self, timestamp: float, tolerance: float | None = None
    ) -> float | None:
        conn = self._get_conn()

        # 最近的 timestamp <= 目标
        cursor = conn.execute(
            f"""
            SELECT timestamp FROM {self._table}
            WHERE timestamp <= ?
            ORDER BY timestamp DESC LIMIT 1
            """,
            (timestamp,),
        )
        before = cursor.fetchone()

        # 最近的 timestamp >= 目标
        cursor = conn.execute(
            f"""
            SELECT timestamp FROM {self._table}
            WHERE timestamp >= ?
            ORDER BY timestamp ASC LIMIT 1
            """,
            (timestamp,),
        )
        after = cursor.fetchone()

        candidates: list[float] = []
        if before:
            candidates.append(before[0])
        if after:
            candidates.append(after[0])

        if not candidates:
            return None

        closest = min(candidates, key=lambda ts: abs(ts - timestamp))

        if tolerance is not None and abs(closest - timestamp) > tolerance:
            return None

        return closest

    def _count(self) -> int:
        conn = self._get_conn()
        cursor = conn.execute(f"SELECT COUNT(*) FROM {self._table}")  # noqa: S608  # table name from class attr
        return cursor.fetchone()[0]  # type: ignore[no-any-return]

    def _last_timestamp(self) -> float | None:
        conn = self._get_conn()
        cursor = conn.execute(f"SELECT MAX(timestamp) FROM {self._table}")  # noqa: S608  # table name from class attr
        row = cursor.fetchone()
        if row is None or row[0] is None:
            return None
        return row[0]  # type: ignore[no-any-return]

    def _find_before(self, timestamp: float) -> tuple[float, Any] | None:
        conn = self._get_conn()
        cursor = conn.execute(
            f"""
            SELECT timestamp, data FROM {self._table}
            WHERE timestamp < ?
            ORDER BY timestamp DESC LIMIT 1
            """,
            (timestamp,),
        )
        row = cursor.fetchone()
        if row is None:
            return None
        return (row[0], pickle.loads(row[1]))  # noqa: S301  # trusted local SQLite store

    def _find_after(self, timestamp: float) -> tuple[float, Any] | None:
        conn = self._get_conn()
        cursor = conn.execute(
            f"""
            SELECT timestamp, data FROM {self._table}
            WHERE timestamp > ?
            ORDER BY timestamp ASC LIMIT 1
            """,
            (timestamp,),
        )
        row = cursor.fetchone()
        if row is None:
            return None
        return (row[0], pickle.loads(row[1]))  # noqa: S301  # trusted local SQLite store
