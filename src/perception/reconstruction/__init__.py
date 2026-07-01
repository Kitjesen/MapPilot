"""Semantic 3D reconstruction package with lazy public exports."""

from __future__ import annotations

from importlib import import_module
from typing import Any

__all__ = [
    "ColorProjector",
    "DatasetRecorderModule",
    "ReconstructionModule",
    "ReconKeyframeExporterModule",
    "SemanticLabeler",
]

_EXPORTS = {
    "ColorProjector": ".color_projector",
    "DatasetRecorderModule": ".dataset_recorder_module",
    "ReconstructionModule": ".reconstruction_module",
    "ReconKeyframeExporterModule": ".keyframe_exporter_module",
    "SemanticLabeler": ".semantic_labeler",
}


def __getattr__(name: str) -> Any:
    module_name = _EXPORTS.get(name)
    if module_name is None:
        raise AttributeError(name)
    module = import_module(module_name, __name__)
    value = getattr(module, name)
    globals()[name] = value
    return value
