"""Deterministic local catalog for the simulation package contracts."""

from .composer import ComposedSession, SessionComposer, SessionIntent
from .diagnostics import CatalogDiagnostic, DiagnosticCode
from .management import SimCatalog
from .resolver import CatalogError, CatalogResolver, PackageRecord, ResolvedSession
from .visual_binding import RobotVisualManifest, VisualBindingError, compile_robot_visual_manifest
from .visual_projection import (
    RobotVisualProjection,
    VisualProjectionError,
    compile_robot_visual_projection,
    validate_entity_visual_projection,
    validate_robot_visual_projection,
)

__all__ = [
    "CatalogError",
    "CatalogDiagnostic",
    "CatalogResolver",
    "ComposedSession",
    "DiagnosticCode",
    "PackageRecord",
    "ResolvedSession",
    "SessionComposer",
    "SessionIntent",
    "SimCatalog",
    "RobotVisualManifest",
    "VisualBindingError",
    "compile_robot_visual_manifest",
    "RobotVisualProjection",
    "VisualProjectionError",
    "compile_robot_visual_projection",
    "validate_entity_visual_projection",
    "validate_robot_visual_projection",
]
