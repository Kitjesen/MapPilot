"""Scene graph parse cache with TTL and content-hash validation."""

from __future__ import annotations

import hashlib
import json
import time
from typing import Any, Callable

__all__ = ["SceneGraphCache"]


class SceneGraphCache:
    """Cache for parsed scene graph results.

    Avoids re-parsing identical scene graphs within a short time window.
    Uses content hashing plus a TTL to balance freshness and performance.
    """

    def __init__(self, ttl_sec: float = 2.0):
        self._ttl = ttl_sec
        self._cached_result: Any | None = None
        self._hash: str | None = None
        self._timestamp: float = 0.0

    @staticmethod
    def _compute_hash(sg_json: Any) -> str:
        """Compute a stable content hash for a scene graph payload."""
        try:
            serialized = json.dumps(sg_json, sort_keys=True, default=str)
        except (TypeError, ValueError):
            serialized = repr(sg_json)
        return hashlib.sha1(serialized.encode("utf-8")).hexdigest()

    def get_or_parse(
        self,
        sg_json: Any,
        parse_fn: Callable[[Any], Any],
        *,
        force_refresh: bool = False,
    ) -> Any:
        """Return cached parse result or invoke parse_fn on cache miss.

        Args:
            sg_json: The scene graph payload to parse.
            parse_fn: Callable that parses sg_json into a result object.
            force_refresh: When True, bypass the cache and re-parse.
        """
        now = time.monotonic()
        h = self._compute_hash(sg_json)
        if (
            not force_refresh
            and h == self._hash
            and self._cached_result is not None
            and (now - self._timestamp) < self._ttl
        ):
            return self._cached_result
        result = parse_fn(sg_json)
        self._cached_result = result
        self._hash = h
        self._timestamp = now
        return result

    def invalidate(self) -> None:
        """Clear the cached result."""
        self._cached_result = None
        self._hash = None
        self._timestamp = 0.0
