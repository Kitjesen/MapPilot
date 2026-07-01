"""Compatibility facade for :mod:`runtime.profiles.endpoints`.

Runtime endpoint/run-spec ownership lives under ``runtime.profiles``. This module
keeps older blueprint-layer imports stable while callers migrate.
"""

from __future__ import annotations

from runtime.profiles.endpoints import *  # noqa: F401,F403
