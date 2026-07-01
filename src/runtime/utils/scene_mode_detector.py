"""SceneModeDetector — runtime indoor / outdoor classification for SLAM gating.

Why this exists
---------------
LingTu deploys to **both indoor and outdoor** environments (~50/50). The two
have different optimal SLAM strategies:

* **Outdoor**: GNSS Factor in PGO is the dominant global anchor; map↔ENU
  Kabsch yaw alignment is necessary; speed limits more permissive.
* **Indoor**: GNSS unavailable, so Scan Context loop closure (N1) is the
  only global anchor; tighter speed limits because no absolute reference.

Hard-coding either strategy gives bad behaviour in the other regime. We
need a runtime mode switch with two channels:

1. **Manual** override via env / API (the user knows the deployment).
2. **Auto** detection from GNSS health (sustained RTK_FIXED → outdoor;
   sustained NO_FIX → indoor).

This module owns *only* the classification logic, not the wiring. The
SlamBridgeModule feeds GnssOdom samples and reads the current mode; other
modules subscribe through a port for state-change notifications.

Mode states
-----------
* ``outdoor`` — GNSS factor active, Kabsch yaw alignment running.
* ``indoor``  — GNSS factor disabled, Scan Context primary, tighter speed.
* ``unknown`` — initial state until enough samples accumulate (~5 s default).

Hysteresis
----------
Auto detection requires ``hold_seconds`` of consistent observations before
flipping. Otherwise a single GNSS dropout (shadow under a tree) would flap
the mode and unnecessarily reconfigure the SLAM stack.
"""

from __future__ import annotations

import logging
import os
import time
from dataclasses import dataclass

logger = logging.getLogger(__name__)

MODE_INDOOR = "indoor"
MODE_OUTDOOR = "outdoor"
MODE_UNKNOWN = "unknown"

VALID_MODES = (MODE_INDOOR, MODE_OUTDOOR, MODE_UNKNOWN)


@dataclass
class SceneModeConfig:
    # Auto-detect hold time before flipping mode (seconds).
    hold_seconds: float = 5.0
    # GNSS sample max age to consider for outdoor classification.
    gnss_max_age_s: float = 2.0
    # Only RTK_FIXED / RTK_FLOAT count toward "outdoor" evidence.
    require_rtk: bool = True


class SceneModeDetector:
    """Classify deployment as indoor / outdoor with hysteresis + manual override.

    Manual override sources, in priority order:
    1. ``set_manual_mode("indoor"|"outdoor"|None)`` — runtime API
    2. ``LINGTU_SCENE_MODE`` env var — set at process start
    3. Auto detection (default).

    A manual override of None falls back to auto.
    """

    def __init__(self, config: SceneModeConfig | None = None) -> None:
        self.config = config or SceneModeConfig()
        env_mode = os.environ.get("LINGTU_SCENE_MODE", "").strip().lower() or None
        if env_mode and env_mode not in VALID_MODES:
            logger.warning(
                "LINGTU_SCENE_MODE=%r is not one of %s — ignoring", env_mode, VALID_MODES)
            env_mode = None
        self._manual: str | None = env_mode if env_mode in (MODE_INDOOR, MODE_OUTDOOR) else None
        self._auto_mode: str = MODE_UNKNOWN
        # Wall-clock of the most recent observation that *agreed* with the
        # currently-pending candidate. Together with config.hold_seconds this
        # implements the flip hysteresis.
        self._candidate_mode: str = MODE_UNKNOWN
        self._candidate_since_ts: float = 0.0
        self._last_change_ts: float = 0.0

    # ── public API ────────────────────────────────────────────────────────

    @property
    def mode(self) -> str:
        """Currently effective mode (manual override wins, then auto)."""
        return self._manual if self._manual is not None else self._auto_mode

    @property
    def source(self) -> str:
        """How the current mode was decided — for logging / debugging."""
        if self._manual is not None:
            return "manual"
        if self._auto_mode == MODE_UNKNOWN:
            return "init"
        return "auto"

    def set_manual_mode(self, mode: str | None) -> None:
        """Set / clear manual override. Pass None to revert to auto detection."""
        if mode is not None:
            mode = mode.strip().lower()
            if mode not in (MODE_INDOOR, MODE_OUTDOOR):
                raise ValueError(f"Manual mode must be one of indoor/outdoor, got {mode!r}")
        prev = self.mode
        self._manual = mode
        new = self.mode
        if prev != new:
            logger.info("SceneMode: manual %r → effective %s", mode, new)
            self._last_change_ts = time.time()

    def observe_gnss(self, fix_type: str, age_s: float, now: float | None = None) -> bool:
        """Feed a GNSS observation; returns True if the *auto* mode flipped.

        ``fix_type`` should be the upper-case name (e.g. ``"RTK_FIXED"``).
        Stale samples (age > gnss_max_age_s) and non-RTK fixes (when
        require_rtk is on) are evidence for ``indoor``.
        """
        now = now if now is not None else time.time()
        candidate: str
        if age_s > self.config.gnss_max_age_s:
            candidate = MODE_INDOOR
        elif self.config.require_rtk and fix_type not in ("RTK_FIXED", "RTK_FLOAT"):
            candidate = MODE_INDOOR
        elif fix_type == "NO_FIX":
            candidate = MODE_INDOOR
        else:
            candidate = MODE_OUTDOOR
        return self._update_auto(candidate, now)

    def observe_no_gnss(self, now: float | None = None) -> bool:
        """Tick when no GNSS message has arrived recently (also indoor evidence)."""
        return self._update_auto(MODE_INDOOR, now if now is not None else time.time())

    # ── internal ──────────────────────────────────────────────────────────

    def _update_auto(self, candidate: str, now: float) -> bool:
        if candidate == self._candidate_mode:
            # Still building evidence for the same candidate.
            elapsed = now - self._candidate_since_ts
            if elapsed >= self.config.hold_seconds and self._auto_mode != candidate:
                # Threshold met — flip auto mode.
                old = self._auto_mode
                self._auto_mode = candidate
                self._last_change_ts = now
                # Only log if it would change the *effective* mode.
                if self._manual is None:
                    logger.info(
                        "SceneMode: auto %s → %s (consistent for %.1fs)",
                        old, candidate, elapsed)
                return True
        else:
            # Candidate flipped — restart the timer.
            self._candidate_mode = candidate
            self._candidate_since_ts = now
        return False
