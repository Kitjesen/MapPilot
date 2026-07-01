"""Shared YAML/JSON persistence helpers for nav service modules."""

import json
import logging
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)

try:
    import yaml
except ImportError:
    yaml = None  # type: ignore[assignment]


def load_yaml(path: Path, default: Any = None) -> Any:
    """Load YAML (or JSON fallback) from *path*. Returns *default* on failure."""
    if default is None:
        default = {}
    if not path.exists():
        return default
    try:
        with open(path, encoding="utf-8") as f:
            if yaml is not None:
                data = yaml.safe_load(f)
            else:
                data = json.load(f)
            return data if data is not None else default
    except Exception:
        logger.warning("Failed to load %s, using default", path, exc_info=True)
        return default


def save_yaml(path: Path, data: Any) -> None:
    """Save *data* as YAML (or JSON fallback) to *path*."""
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            if yaml is not None:
                yaml.safe_dump(data, f, allow_unicode=True, default_flow_style=False)
            else:
                json.dump(data, f, ensure_ascii=False, indent=2)
    except Exception:
        logger.warning("Failed to save %s", path, exc_info=True)
