"""Shared algorithm gate definitions.

Keep product diagnostics and server-side closure presets on the same required
gate sequence so readiness claims cannot drift between surfaces.
"""

from __future__ import annotations

INSPECTION_MVP_REQUIRED_GATES: tuple[str, ...] = (
    "gateway_runtime_acceptance",
    "navigation_replay_deviation",
    "saved_map_relocalize",
)

DIMOS_BENCHMARK_REQUIRED_GATES: tuple[str, ...] = (
    "gateway_runtime_acceptance",
    "navigation_replay_deviation",
    "saved_map_relocalize",
    "bbs3d_kidnapped_relocalize",
)

G4_SERVER_FULL_SIM_REQUIRED_GATES: tuple[str, ...] = (
    "gateway_runtime_acceptance",
    "navigation_replay_deviation",
    "saved_map_relocalize",
    "bbs3d_kidnapped_relocalize",
)
