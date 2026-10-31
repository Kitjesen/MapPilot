"""Deterministic local catalog for the simulation package contracts."""

from .composer import ComposedSession, SessionComposer, SessionIntent
from .diagnostics import CatalogDiagnostic, DiagnosticCode
from .management import SimCatalog
from .resolver import CatalogError, CatalogResolver, PackageRecord, ResolvedSession, resolve_robot_default_policy
from .visual_binding import RobotVisualManifest, VisualBindingError, compile_robot_visual_manifest
from .visual_projection import (
    RobotVisualProjection,
    VisualProjectionError,
    compile_robot_visual_projection,
    validate_entity_visual_projection,
    validate_robot_visual_projection,
)

__all__ = [
    "CatalogDiagnostic",
    "CatalogError",
    "CatalogResolver",
    "ComposedSession",
    "DiagnosticCode",
    "PackageRecord",
    "ResolvedSession",
    "RobotVisualManifest",
    "RobotVisualProjection",
    "SessionComposer",
    "SessionIntent",
    "SimCatalog",
    "VisualBindingError",
    "VisualProjectionError",
    "compile_robot_visual_manifest",
    "compile_robot_visual_projection",
    "resolve_robot_default_policy",
    "validate_entity_visual_projection",
    "validate_robot_visual_projection",
]
