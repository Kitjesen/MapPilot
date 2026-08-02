"""Package-level migration plan for a Dart/Rust LingTu.

The catalog is metadata only. It turns the broad migration goal into a stable
package-by-package sequence that can be tested and refined without changing
runtime behavior.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

TargetForm = Literal[
    "rust_runtime",
    "rust_kernel",
    "rust_adapter",
    "rust_tooling",
    "dart_app",
    "dart_sdk",
    "native_engine_process",
    "schema_only",
    "python_compat_until_replaced",
]
RiskLevel = Literal["low", "medium", "high", "very_high"]


@dataclass(frozen=True)
class PackageMigrationTarget:
    """A source package or top-level area scheduled for Dart/Rust migration."""

    package: str
    current_role: str
    target_form: TargetForm
    phase: int
    risk: RiskLevel
    migration_action: str
    keep_running_strategy: str
    notes: str = ""


PACKAGE_MIGRATION_TARGETS: tuple[PackageMigrationTarget, ...] = (
    PackageMigrationTarget(
        package="src/kernels",
        current_role="Portable compute-kernel convergence point",
        target_form="rust_kernel",
        phase=1,
        risk="medium",
        migration_action="Add Rust candidates beside C++ portable kernels behind C ABI.",
        keep_running_strategy="Keep existing C++/Python adapters as reference backends.",
        notes="Materialized Rust kernels live here; nav C++ kernels stay in src/nav/kernel.",
    ),
    PackageMigrationTarget(
        package="src/nav",
        current_role="Navigation modules, maps, planning orchestration, safety gates",
        target_form="rust_kernel",
        phase=1,
        risk="high",
        migration_action=(
            "Extract planning/safety math into kernels before moving module logic; "
            "do not migrate SafetyRing, VelocityMux, or driver control first."
        ),
        keep_running_strategy="Python modules keep calling old backends until each kernel passes fixtures.",
    ),
    PackageMigrationTarget(
        package="src/nav/local",
        current_role="Terrain, local planner, path follower adapters and local autonomy hot paths",
        target_form="rust_kernel",
        phase=1,
        risk="high",
        migration_action="Move algorithm seams to src/kernels, then replace small kernels with Rust.",
        keep_running_strategy="Preserve nanobind/cmu_py/simple backends during migration.",
    ),
    PackageMigrationTarget(
        package="src/localization",
        current_role="SLAM, localization, GNSS, visual odometry, native LiDAR engines",
        target_form="native_engine_process",
        phase=1,
        risk="very_high",
        migration_action="Keep heavy SLAM engines native; expose portable/process ABI first.",
        keep_running_strategy="Use windows-fastlio2 and existing bridge/localizer profiles in parallel.",
    ),
    PackageMigrationTarget(
        package="src/*/adapters",
        current_role="Runtime adapters for LCM, ROS, portable localization, feeds",
        target_form="rust_adapter",
        phase=2,
        risk="medium",
        migration_action="Port stable adapters to Rust after their message contracts are frozen.",
        keep_running_strategy="Run Rust adapters as sidecars before replacing Python imports.",
    ),
    PackageMigrationTarget(
        package="src/*/vendor",
        current_role="Vendored algorithm/source trees kept beside their owning backend",
        target_form="native_engine_process",
        phase=2,
        risk="high",
        migration_action="Keep vendor algorithms isolated; wrap only stable inputs/outputs through LingTu services.",
        keep_running_strategy="Do not import external assets from normal Module code except through explicit service adapters.",
    ),
    PackageMigrationTarget(
        package="src/drivers",
        current_role="Robot, camera, LiDAR, GNSS, simulation, and ROS/hardware bridges",
        target_form="rust_adapter",
        phase=2,
        risk="very_high",
        migration_action="Port non-ROS endpoint drivers first; keep vendor/ROS drivers isolated.",
        keep_running_strategy="LCM/JSONL endpoint mode remains the fallback deployment path.",
    ),
    PackageMigrationTarget(
        package="src/lingtu/plugin_seed.py",
        current_role="Runtime plugin seeding and product runtime policy",
        target_form="rust_runtime",
        phase=3,
        risk="medium",
        migration_action="Create Rust runtime daemon that can load the same endpoint contracts.",
        keep_running_strategy="Run Rust daemon in shadow mode while Python runtime remains authoritative.",
    ),
    PackageMigrationTarget(
        package="src/runtime",
        current_role="Module framework, streams, registry, config, transports, lifecycle",
        target_form="rust_runtime",
        phase=3,
        risk="very_high",
        migration_action="Rebuild the Module/Blueprint runtime in Rust after adapters and kernels are stable.",
        keep_running_strategy="Keep Python core as compatibility runtime until graph snapshot parity exists.",
    ),
    PackageMigrationTarget(
        package="src/diagnostics",
        current_role="Runtime diagnostics, evidence gates, audits, and migration reports",
        target_form="rust_tooling",
        phase=3,
        risk="medium",
        migration_action="Move repeatable diagnostic gates to lingtuctl after runtime contracts stabilize.",
        keep_running_strategy="Keep Python diagnostics as the source of truth during runtime cleanup.",
    ),
    PackageMigrationTarget(
        package="src/message",
        current_role="Typed DDS topic schema, C++ topic constants, and Python conversion helpers",
        target_form="schema_only",
        phase=0,
        risk="medium",
        migration_action="Keep DDS schemas as the shared contract while C++ endpoints replace Python DDS adapters.",
        keep_running_strategy="Generate or mirror schemas for Python/C++ clients until the runtime owns typed transport end to end.",
    ),
    PackageMigrationTarget(
        package="cli",
        current_role="Command line entry points, profiles, REPL, daemon lifecycle",
        target_form="rust_tooling",
        phase=3,
        risk="medium",
        migration_action="Replace operational commands with a Rust lingtuctl binary one command at a time.",
        keep_running_strategy="Keep lingtu.py as compatibility entry until command parity is tested.",
    ),
    PackageMigrationTarget(
        package="src/gateway",
        current_role="FastAPI gateway, REST/SSE/WS, MCP and teleop endpoints",
        target_form="rust_runtime",
        phase=4,
        risk="high",
        migration_action="Move stable REST/WS endpoints to a Rust gateway after runtime contracts settle.",
        keep_running_strategy="Proxy or mirror Python FastAPI endpoints during rollout.",
    ),
    PackageMigrationTarget(
        package="src/memory",
        current_role="Semantic, episodic, tagged, vector, temporal, and KG memory",
        target_form="rust_runtime",
        phase=4,
        risk="high",
        migration_action="Port storage/query cores to Rust; keep embedding/model adapters external.",
        keep_running_strategy="SQLite and numpy/Chroma fallbacks stay until data migration is proven.",
    ),
    PackageMigrationTarget(
        package="src/lingtu/sdk",
        current_role="Local SDK surface for clients and product integrations",
        target_form="dart_sdk",
        phase=4,
        risk="medium",
        migration_action="Add Dart SDK over the stable gateway/runtime contracts.",
        keep_running_strategy="Keep Python SDK while Dart SDK reaches feature parity.",
    ),
    PackageMigrationTarget(
        package="src/lingtu",
        current_role="User-facing Python helper package",
        target_form="dart_sdk",
        phase=4,
        risk="medium",
        migration_action="Replace user-facing helper flows with Dart/Rust client packages.",
        keep_running_strategy="Keep Python helpers as examples and compatibility layer.",
    ),
    PackageMigrationTarget(
        package="web",
        current_role="React/Vite dashboard",
        target_form="dart_app",
        phase=4,
        risk="medium",
        migration_action="Rebuild operator UI as Dart/Flutter consuming stable gateway contracts.",
        keep_running_strategy="Keep current web dashboard until feature parity screenshots and smoke tests pass.",
    ),
    PackageMigrationTarget(
        package="src/perception",
        current_role="Perception, tracking, reconstruction, and model-backed scene understanding",
        target_form="python_compat_until_replaced",
        phase=5,
        risk="very_high",
        migration_action="Move only pure geometry/scoring kernels first; keep model backends as processes.",
        keep_running_strategy="Python perception stack stays behind runtime contracts until model parity exists.",
    ),
    PackageMigrationTarget(
        package="src/decision",
        current_role="LLM planner, visual servo, goal resolution, and semantic reasoning",
        target_form="python_compat_until_replaced",
        phase=5,
        risk="very_high",
        migration_action="Move only pure geometry/scoring kernels first; keep model backends as processes.",
        keep_running_strategy="Python decision stack stays behind runtime contracts until model parity exists.",
    ),
    PackageMigrationTarget(
        package="src/nav/services/plan",
        current_role="PCT, A*, OctoPlanner3D, tomogram and native global planning",
        target_form="native_engine_process",
        phase=5,
        risk="very_high",
        migration_action="Wrap heavy planners with process ABI before considering rewrites.",
        keep_running_strategy="Keep existing native planners selectable; Rust calls process ABI.",
    ),
    PackageMigrationTarget(
        package="src/nav/exploration",
        current_role="TARE and frontier exploration integrations",
        target_form="native_engine_process",
        phase=5,
        risk="high",
        migration_action="Port wavefront/frontier scoring after nav kernels; keep TARE as native process.",
        keep_running_strategy="Keep both internal explore routes operational until native parity.",
    ),
    PackageMigrationTarget(
        package="src/gateway/media",
        current_role="Optional aiortc/PyAV WebRTC video stream",
        target_form="native_engine_process",
        phase=5,
        risk="high",
        migration_action="Prefer external media sidecar or Rust gateway integration over Dart real-time media.",
        keep_running_strategy="Keep aiortc/go2rtc fallback labels until measured latency parity.",
    ),
    PackageMigrationTarget(
        package="calibration",
        current_role="Camera, IMU, LiDAR, and extrinsic calibration tools",
        target_form="rust_tooling",
        phase=6,
        risk="high",
        migration_action="Port repeatable validation/apply steps; keep OpenCV-heavy calibration tools native/Python.",
        keep_running_strategy="Calibration outputs and config schema stay stable.",
    ),
    PackageMigrationTarget(
        package="sim",
        current_role="Simulation, replay, smoke gates, portable runtime harnesses",
        target_form="rust_tooling",
        phase=6,
        risk="medium",
        migration_action="Move deterministic replay/gate runners to Rust after kernels stabilize.",
        keep_running_strategy="Keep Python sim scripts as fixture generators.",
    ),
    PackageMigrationTarget(
        package="scripts",
        current_role="Build, deploy, diagnostics, robot-side operations",
        target_form="rust_tooling",
        phase=6,
        risk="medium",
        migration_action="Replace cross-platform operations with lingtuctl; keep robot-only shell wrappers isolated.",
        keep_running_strategy="Migrate one command group at a time and preserve old command output.",
    ),
    PackageMigrationTarget(
        package="config",
        current_role="Robot, device, DDS, semantic, and runtime configuration",
        target_form="schema_only",
        phase=0,
        risk="low",
        migration_action="Freeze schemas so Python, Rust, and Dart read the same config.",
        keep_running_strategy="No runtime behavior change; add schema validation before consumers move.",
    ),
    PackageMigrationTarget(
        package="launch",
        current_role="ROS launch files and algorithm bridge launchers",
        target_form="schema_only",
        phase=0,
        risk="low",
        migration_action="Treat launch files as ROS compatibility, not core runtime.",
        keep_running_strategy="Keep launch unchanged while Rust runtime grows non-ROS endpoints.",
    ),
)


def targets_by_phase(phase: int) -> tuple[PackageMigrationTarget, ...]:
    """Return package targets scheduled for a migration phase."""

    return tuple(target for target in PACKAGE_MIGRATION_TARGETS if target.phase == phase)


def target_for_package(package: str) -> PackageMigrationTarget:
    """Return the migration target for a package path."""

    normalized = package.replace("\\", "/").rstrip("/")
    for target in PACKAGE_MIGRATION_TARGETS:
        if target.package == normalized:
            return target
    raise KeyError(package)


__all__ = [
    "PACKAGE_MIGRATION_TARGETS",
    "PackageMigrationTarget",
    "RiskLevel",
    "TargetForm",
    "target_for_package",
    "targets_by_phase",
]
