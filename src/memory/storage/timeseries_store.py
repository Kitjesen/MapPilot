"""Time-series storage for robot telemetry and sensor history.

Port from DimOS, adapted for the LingTu navigation system.
Removed RxPy/reactivex dependency (stream, pipe_save, consume_stream).
All other functionality retained, including seek/duration/loop iteration
and relative-time queries.
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
import time
from abc import ABC, abstractmethod
from collections.abc import Iterator
from dataclasses import dataclass, field
from typing import Any


@dataclass
class SceneGraphEntry:
    """场景图快照，用于时序存储。

    Attributes:
        timestamp: Unix 时间戳（秒，float）
        objects:   对象字典 {object_id: {label, position, features, ...}}
        rooms:     房间节点列表
        metadata:  附加元数据（version, robot_pose 等）
    """

    timestamp: float
    objects: dict = field(default_factory=dict)
    rooms: list = field(default_factory=list)
    metadata: dict = field(default_factory=dict)


class TimeSeriesStore(ABC):
    """时序存储抽象基类。

    后端（in-memory、sqlite 等）实现抽象方法，集合操作、遍历、
    时间切片逻辑由基类提供。

    接口约定：timestamp 为 float（Unix 秒），data 为任意可 pickle 对象。
    """

    # ------------------------------------------------------------------
    # 抽象方法（后端必须实现）
    # ------------------------------------------------------------------

    @abstractmethod
    def _save(self, timestamp: float, data: Any) -> None:
        """保存 (timestamp, data) 到后端。"""
        ...

    @abstractmethod
    def _load(self, timestamp: float) -> Any | None:
        """按精确时间戳加载数据，未找到返回 None。"""
        ...

    @abstractmethod
    def _delete(self, timestamp: float) -> Any | None:
        """删除精确时间戳对应条目，返回被删除的数据或 None。"""
        ...

    @abstractmethod
    def _iter_items(
        self, start: float | None = None, end: float | None = None
    ) -> Iterator[tuple[float, Any]]:
        """懒惰迭代 (timestamp, data)，可选时间范围 [start, end)。"""
        ...

    @abstractmethod
    def _find_closest_timestamp(
        self, timestamp: float, tolerance: float | None = None
    ) -> float | None:
        """查找最近时间戳；超出 tolerance 时返回 None。"""
        ...

    @abstractmethod
    def _count(self) -> int:
        """返回存储条目数。"""
        ...

    @abstractmethod
    def _last_timestamp(self) -> float | None:
        """返回最大时间戳，空时返回 None。"""
        ...

    @abstractmethod
    def _find_before(self, timestamp: float) -> tuple[float, Any] | None:
        """查找严格早于 timestamp 的最后一条 (ts, data)。"""
        ...

    @abstractmethod
    def _find_after(self, timestamp: float) -> tuple[float, Any] | None:
        """查找严格晚于 timestamp 的第一条 (ts, data)。"""
        ...

    # ------------------------------------------------------------------
    # 集合 API（基类提供，基于抽象方法）
    # ------------------------------------------------------------------

    def __len__(self) -> int:
        return self._count()

    def __iter__(self) -> Iterator[Any]:
        """按时间戳顺序迭代数据（不含时间戳）。"""
        for _, data in self._iter_items():
            yield data

    def last_timestamp(self) -> float | None:
        """返回最大时间戳。"""
        return self._last_timestamp()

    def last(self) -> Any | None:
        """返回最新条目的数据。"""
        ts = self._last_timestamp()
        if ts is None:
            return None
        return self._load(ts)

    @property
    def start_ts(self) -> float | None:
        return self.first_timestamp()

    @property
    def end_ts(self) -> float | None:
        return self._last_timestamp()

    def time_range(self) -> tuple[float, float] | None:
        """返回 (start, end) 时间范围；空时返回 None。"""
        s = self.first_timestamp()
        e = self._last_timestamp()
        if s is None or e is None:
            return None
        return (s, e)

    def duration(self) -> float:
        """返回存储跨度（秒）。"""
        r = self.time_range()
        return (r[1] - r[0]) if r else 0.0

    def first_timestamp(self) -> float | None:
        """返回最小时间戳。"""
        for ts, _ in self._iter_items():
            return ts
        return None

    def first(self) -> Any | None:
        """返回最旧条目的数据。"""
        for _, data in self._iter_items():
            return data
        return None

    # ------------------------------------------------------------------
    # 查询 API
    # ------------------------------------------------------------------

    def save(self, timestamp: float, data: Any) -> None:
        """保存 (timestamp, data)。"""
        self._save(timestamp, data)

    def load(self, timestamp: float) -> Any | None:
        """按精确时间戳加载数据。"""
        return self._load(timestamp)

    def delete(self, timestamp: float) -> Any | None:
        """删除精确时间戳条目，返回被删除的数据。"""
        return self._delete(timestamp)

    def find_closest(
        self,
        timestamp: float,
        tolerance: float | None = None,
    ) -> Any | None:
        """查找最接近给定时间戳的数据。"""
        closest_ts = self._find_closest_timestamp(timestamp, tolerance)
        if closest_ts is None:
            return None
        return self._load(closest_ts)

    def find_before(self, timestamp: float) -> Any | None:
        """查找严格早于 timestamp 的最新数据。"""
        result = self._find_before(timestamp)
        return result[1] if result else None

    def find_after(self, timestamp: float) -> Any | None:
        """查找严格晚于 timestamp 的最早数据。"""
        result = self._find_after(timestamp)
        return result[1] if result else None

    def slice_by_time(self, start: float, end: float) -> list[Any]:
        """返回 [start, end) 时间范围内的所有数据。"""
        return [data for _, data in self._iter_items(start=start, end=end)]

    def iter_items(
        self,
        start: float | None = None,
        end: float | None = None,
    ) -> Iterator[tuple[float, Any]]:
        """公开迭代 (timestamp, data)，可选时间范围 [start, end)。"""
        yield from self._iter_items(start=start, end=end)

    def prune_old(self, cutoff: float) -> None:
        """删除所有早于 cutoff 的条目。"""
        to_delete = [ts for ts, _ in self._iter_items(end=cutoff)]
        for ts in to_delete:
            self._delete(ts)

    def count(self) -> int:
        """返回存储条目数（_count 的公开别名）。"""
        return self._count()

    def find_closest_seek(
        self,
        relative_seconds: float,
        tolerance: float | None = None,
    ) -> Any | None:
        """查找距离起始时间 relative_seconds 秒处最近的数据。"""
        first = self.first_timestamp()
        if first is None:
            return None
        return self.find_closest(first + relative_seconds, tolerance)

    # ------------------------------------------------------------------
    # 高级迭代（seek / duration / loop）
    # ------------------------------------------------------------------

    def iterate_items(
        self,
        seek: float | None = None,
        duration: float | None = None,
        from_timestamp: float | None = None,
        loop: bool = False,
    ) -> Iterator[tuple[float, Any]]:
        """迭代 (timestamp, data)，支持 seek/duration/loop。

        Args:
            seek: 从起始时间偏移秒数开始迭代
            duration: 迭代持续秒数
            from_timestamp: 从绝对时间戳开始（优先于 seek）
            loop: 到达末尾后是否循环
        """
        first = self.first_timestamp()
        if first is None:
            return

        if from_timestamp is not None:
            start = from_timestamp
        elif seek is not None:
            start = first + seek
        else:
            start = None

        end = None
        if duration is not None:
            start_ts = start if start is not None else first
            end = start_ts + duration

        while True:
            yield from self._iter_items(start=start, end=end)
            if not loop:
                break

    def iterate(
        self,
        seek: float | None = None,
        duration: float | None = None,
        from_timestamp: float | None = None,
        loop: bool = False,
    ) -> Iterator[Any]:
        """迭代数据（不含时间戳），支持 seek/duration/loop。"""
        for _, data in self.iterate_items(
            seek=seek, duration=duration, from_timestamp=from_timestamp, loop=loop
        ):
            yield data

    # ------------------------------------------------------------------
    # 实时重放
    # ------------------------------------------------------------------

    def iterate_realtime(
        self,
        speed: float = 1.0,
        seek: float | None = None,
        duration: float | None = None,
        from_timestamp: float | None = None,
        loop: bool = False,
    ) -> Iterator[Any]:
        """实时速度迭代数据，按原始时间间隔 sleep。

        Args:
            speed: 播放速率（1.0=原速, 2.0=2倍速）
            seek: 从起始偏移秒数开始
            duration: 播放持续秒数
            from_timestamp: 从绝对时间戳开始
            loop: 到达末尾后是否循环
        """
        prev_ts: float | None = None
        for ts, data in self.iterate_items(
            seek=seek, duration=duration, from_timestamp=from_timestamp, loop=loop
        ):
            if prev_ts is not None:
                delay = (ts - prev_ts) / speed
                if delay > 0:
                    time.sleep(delay)
            prev_ts = ts
            yield data
