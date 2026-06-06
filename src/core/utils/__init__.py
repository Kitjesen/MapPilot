"""Cross-layer utility exports."""

from __future__ import annotations

import importlib
from typing import Any


_EXPORTS = {
    "IntrinsicsResult": ("core.utils.validation", "IntrinsicsResult"),
    "_try_empty_cuda_cache": ("core.utils.robustness", "_try_empty_cuda_cache"),
    "async_timeout": ("core.utils.robustness", "async_timeout"),
    "build_mid360_config_dict": ("core.utils.livox_config", "build_mid360_config_dict"),
    "ensure_mid360_config_file": ("core.utils.livox_config", "ensure_mid360_config_file"),
    "gpu_safe": ("core.utils.robustness", "gpu_safe"),
    "normalize_quaternion": ("core.utils.validation", "normalize_quaternion"),
    "retry": ("core.utils.robustness", "retry"),
    "retry_async": ("core.utils.robustness", "retry_async"),
    "safe_json_dump": ("core.utils.sanitize", "safe_json_dump"),
    "safe_json_dumps": ("core.utils.sanitize", "safe_json_dumps"),
    "safe_json_loads": ("core.utils.sanitize", "safe_json_loads"),
    "sanitize_dict": ("core.utils.sanitize", "sanitize_dict"),
    "sanitize_float": ("core.utils.sanitize", "sanitize_float"),
    "sanitize_position": ("core.utils.sanitize", "sanitize_position"),
    "validate_bgr": ("core.utils.validation", "validate_bgr"),
    "validate_depth": ("core.utils.validation", "validate_depth"),
    "validate_depth_pair": ("core.utils.validation", "validate_depth_pair"),
    "validate_intrinsics": ("core.utils.validation", "validate_intrinsics"),
}

__all__ = list(_EXPORTS)


def __getattr__(name: str) -> Any:
    if name not in _EXPORTS:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module_name, attr_name = _EXPORTS[name]
    module = importlib.import_module(module_name)
    value = getattr(module, attr_name)
    globals()[name] = value
    return value
