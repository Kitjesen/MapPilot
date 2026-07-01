"""Cross-layer utility exports."""

from __future__ import annotations

import importlib
from typing import Any


_EXPORTS = {
    "IntrinsicsResult": ("runtime.utils.validation", "IntrinsicsResult"),
    "_try_empty_cuda_cache": ("runtime.utils.robustness", "_try_empty_cuda_cache"),
    "async_timeout": ("runtime.utils.robustness", "async_timeout"),
    "build_mid360_config_dict": ("runtime.utils.livox_config", "build_mid360_config_dict"),
    "ensure_mid360_config_file": ("runtime.utils.livox_config", "ensure_mid360_config_file"),
    "gpu_safe": ("runtime.utils.robustness", "gpu_safe"),
    "normalize_quaternion": ("runtime.utils.validation", "normalize_quaternion"),
    "retry": ("runtime.utils.robustness", "retry"),
    "retry_async": ("runtime.utils.robustness", "retry_async"),
    "safe_json_dump": ("runtime.utils.sanitize", "safe_json_dump"),
    "safe_json_dumps": ("runtime.utils.sanitize", "safe_json_dumps"),
    "safe_json_loads": ("runtime.utils.sanitize", "safe_json_loads"),
    "sanitize_dict": ("runtime.utils.sanitize", "sanitize_dict"),
    "sanitize_float": ("runtime.utils.sanitize", "sanitize_float"),
    "sanitize_position": ("runtime.utils.sanitize", "sanitize_position"),
    "validate_bgr": ("runtime.utils.validation", "validate_bgr"),
    "validate_depth": ("runtime.utils.validation", "validate_depth"),
    "validate_depth_pair": ("runtime.utils.validation", "validate_depth_pair"),
    "validate_intrinsics": ("runtime.utils.validation", "validate_intrinsics"),
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
