"""Compatibility exports for gateway runtime cache services.

Keep this thin module so older imports do not break while each service now
lives in its own file.
"""

from gateway.services.loc_cache import LocalizationRuntimeCacheService, LocCache
from gateway.services.session_cache import SessionCache, SessionRuntimeCacheService

__all__ = [
    "LocCache",
    "LocalizationRuntimeCacheService",
    "SessionCache",
    "SessionRuntimeCacheService",
]
