"""Centralized DDS QoS profile loader and domain resolution.

This module loads the ROS2-style QoS profiles declared in
``config/qos_profiles.yaml`` and translates them into CycloneDDS
``Qos``/``Policy`` objects. It also provides a single place to resolve the
default DDS domain id from the ``LINGTU_DDS_DOMAIN_ID`` environment variable.

Design constraints (all new behavior is opt-in):
  * When CycloneDDS is not installed, the config file is missing, a profile
    is undefined, or a field fails to parse, the public helpers return
    ``None`` so callers keep their existing default behavior. Parse issues
    are logged as warnings, never raised, so dev/test environments stay
    healthy.
  * Results are cached per process; the cache can be cleared with
    :func:`reset_cache` (used by tests).

Topic -> profile mapping is derived from the ``topics`` list under each
profile in ``qos_profiles.yaml``.  Both the legacy nested layout
(``profile.qos.reliability``) and the current flat layout
(``profile.reliability``) are supported.  A future task (#2/#3) may instead
source this mapping from ``runtime.route_contract``; this module intentionally
does not depend on route_contract to stay within the transport-layer scope.
"""

from __future__ import annotations

import logging
import os
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)

# Repo root: transport -> runtime -> src -> <repo>
_REPO_ROOT = Path(__file__).resolve().parents[3]
_QOS_CONFIG_PATH = _REPO_ROOT / "config" / "qos_profiles.yaml"

# Environment variable that centralizes the default DDS domain id. This is the
# Keep the domain override aligned with the native navigation command client.
_DOMAIN_ENV_VAR = "LINGTU_DDS_DOMAIN_ID"

# Default max blocking time for RELIABLE writers/readers. Matches the value
# previously hardcoded in adapters/dds/reader.py to preserve behavior.
_DEFAULT_RELIABLE_BLOCKING_SECONDS = 1.0

try:
    from cyclonedds.qos import Policy, Qos
    from cyclonedds.util import duration

    _CYCLONE_AVAILABLE = True
except ImportError:  # pragma: no cover - exercised in dev without cyclonedds
    _CYCLONE_AVAILABLE = False
    Policy = Qos = None  # type: ignore[assignment]
    duration = None  # type: ignore[assignment]


# ── module-level caches ──────────────────────────────────────────────────
_raw_profiles_cache: dict[str, Any] | None = None
_raw_profiles_loaded = False
_topic_to_profile_cache: dict[str, str] | None = None
_qos_by_profile_cache: dict[str, Any] = {}


def reset_cache() -> None:
    """Clear all cached config and translated QoS objects (test helper)."""
    global _raw_profiles_cache, _raw_profiles_loaded
    global _topic_to_profile_cache
    _raw_profiles_cache = None
    _raw_profiles_loaded = False
    _topic_to_profile_cache = None
    _qos_by_profile_cache.clear()


def resolve_domain_id(domain_id: int | None = None) -> int:
    """Resolve the DDS domain id.

    When *domain_id* is explicitly provided it wins. Otherwise the value is
    read from the ``LINGTU_DDS_DOMAIN_ID`` environment variable (default 0).
    Invalid env values fall back to 0 with a warning.
    """
    if domain_id is not None:
        return int(domain_id)
    raw = os.environ.get(_DOMAIN_ENV_VAR, "0")
    try:
        return int(str(raw).strip() or "0")
    except (TypeError, ValueError):
        logger.warning("invalid %s=%r; falling back to domain 0", _DOMAIN_ENV_VAR, raw)
        return 0


def _load_raw_profiles() -> dict[str, Any]:
    """Load and cache the raw ``profiles`` mapping from qos_profiles.yaml."""
    global _raw_profiles_cache, _raw_profiles_loaded
    if _raw_profiles_loaded:
        return _raw_profiles_cache or {}
    _raw_profiles_loaded = True
    _raw_profiles_cache = {}
    try:
        import yaml
    except ImportError:  # pragma: no cover - pyyaml is a project dependency
        logger.warning("pyyaml unavailable; QoS profiles disabled")
        return {}
    try:
        text = _QOS_CONFIG_PATH.read_text(encoding="utf-8")
    except FileNotFoundError:
        logger.warning("QoS config not found at %s", _QOS_CONFIG_PATH)
        return {}
    except OSError as exc:
        logger.warning("failed reading QoS config %s: %s", _QOS_CONFIG_PATH, exc)
        return {}
    try:
        data = yaml.safe_load(text) or {}
    except Exception as exc:
        logger.warning("failed parsing QoS config %s: %s", _QOS_CONFIG_PATH, exc)
        return {}
    profiles = data.get("profiles") if isinstance(data, dict) else None
    if not isinstance(profiles, dict):
        logger.warning("QoS config has no 'profiles' mapping")
        return {}
    _raw_profiles_cache = profiles
    return profiles


def _topic_to_profile() -> dict[str, str]:
    """Build (and cache) the reverse topic -> profile-name mapping."""
    global _topic_to_profile_cache
    if _topic_to_profile_cache is not None:
        return _topic_to_profile_cache
    mapping: dict[str, str] = {}
    for name, spec in _load_raw_profiles().items():
        if not isinstance(spec, dict):
            continue
        topics = spec.get("topics") or []
        if not isinstance(topics, (list, tuple)):
            continue
        for topic in topics:
            if isinstance(topic, str):
                mapping[topic] = str(name)
    _topic_to_profile_cache = mapping
    return mapping


def _duration_ns(value: Any) -> int | None:
    """Parse a ROS2-style duration into integer nanoseconds.

    Accepts ``"200ms"``, ``"50us"``/``"50µs"``, ``"1s"``, ``"5ns"`` and bare
    numbers (interpreted as seconds). Returns ``None`` on failure.
    """
    if value is None:
        return None
    if isinstance(value, (int, float)):
        return int(float(value) * 1_000_000_000)
    text = str(value).strip().lower()
    if not text:
        return None
    units = (
        ("ms", 1_000_000),
        ("us", 1_000),
        ("\u00b5s", 1_000),  # micro sign µs
        ("ns", 1),
        ("s", 1_000_000_000),
    )
    for suffix, scale in units:
        if text.endswith(suffix):
            number = text[: -len(suffix)].strip()
            try:
                return int(float(number) * scale)
            except ValueError:
                return None
    try:
        return int(float(text) * 1_000_000_000)
    except ValueError:
        return None


def _build_qos(profile_name: str, qos_spec: dict[str, Any]) -> Any | None:
    """Translate one ROS2-style qos spec into a CycloneDDS ``Qos`` object.

    Returns ``None`` if no recognizable policy was produced (e.g. the ``tf``
    profile which only carries a ``note`` and must not be overridden).
    """
    policies: list[Any] = []

    reliability = qos_spec.get("reliability")
    if isinstance(reliability, str):
        token = reliability.strip().upper()
        if token == "RELIABLE":
            policies.append(Policy.Reliability.Reliable(duration(seconds=_DEFAULT_RELIABLE_BLOCKING_SECONDS)))
        elif token == "BEST_EFFORT":
            policies.append(Policy.Reliability.BestEffort)
        else:
            logger.warning("profile %s: unknown reliability %r", profile_name, reliability)

    history = qos_spec.get("history")
    depth = qos_spec.get("depth")
    if isinstance(history, str) and history.strip().upper() == "KEEP_ALL":
        policies.append(Policy.History.KeepAll)
    elif depth is not None:
        try:
            policies.append(Policy.History.KeepLast(int(depth)))
        except (TypeError, ValueError):
            logger.warning("profile %s: invalid depth %r", profile_name, depth)
    elif isinstance(history, str) and history.strip().upper() == "KEEP_LAST":
        policies.append(Policy.History.KeepLast(1))

    durability = qos_spec.get("durability")
    if isinstance(durability, str):
        token = durability.strip().upper()
        if token == "TRANSIENT_LOCAL":
            policies.append(Policy.Durability.TransientLocal)
        elif token == "VOLATILE":
            policies.append(Policy.Durability.Volatile)
        else:
            logger.warning("profile %s: unknown durability %r", profile_name, durability)

    deadline_ns = _duration_ns(qos_spec.get("deadline"))
    if deadline_ns is not None:
        policies.append(Policy.Deadline(duration(nanoseconds=deadline_ns)))

    lifespan_ns = _duration_ns(qos_spec.get("lifespan"))
    if lifespan_ns is not None:
        policies.append(Policy.Lifespan(duration(nanoseconds=lifespan_ns)))

    if not policies:
        return None
    return Qos(*policies)


def qos_for_profile(name: str) -> Any | None:
    """Return the CycloneDDS ``Qos`` for a named profile, or ``None``.

    ``None`` means "use the transport's existing default" and is returned when
    CycloneDDS is unavailable, the profile is undefined, or its spec produced
    no usable policy (e.g. the ``tf`` profile which only carries a ``note``).

    Both layouts are supported:
      * Nested: ``profile.qos.reliability`` (legacy)
      * Flat:    ``profile.reliability``     (current product format)
    """
    if not _CYCLONE_AVAILABLE or not name:
        return None
    if name in _qos_by_profile_cache:
        return _qos_by_profile_cache[name]
    spec = _load_raw_profiles().get(name)
    qos: Any | None = None
    if isinstance(spec, dict):
        # Support both nested ("qos:" sub-mapping) and flat layouts.
        # In the flat layout, QoS fields (reliability, durability, etc.) live
        # alongside "topics" and "note" on the profile dict itself.
        qos_spec = spec.get("qos") if isinstance(spec.get("qos"), dict) else spec
        try:
            qos = _build_qos(name, qos_spec)
        except Exception as exc:
            logger.warning("failed building QoS for profile %s: %s", name, exc)
            qos = None
    _qos_by_profile_cache[name] = qos
    return qos


def qos_for_topic(topic: str) -> Any | None:
    """Return the CycloneDDS ``Qos`` for a topic name, or ``None``.

    The topic -> profile mapping is derived from qos_profiles.yaml. Returns
    ``None`` when the topic is not mapped or when translation is unavailable.
    """
    if not _CYCLONE_AVAILABLE or not topic:
        return None
    profile = _topic_to_profile().get(topic)
    if profile is None:
        return None
    return qos_for_profile(profile)
