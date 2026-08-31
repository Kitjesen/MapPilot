"""Resolve native build scripts from Product process roles."""

from __future__ import annotations

from lingtu.products import product_name
from runtime.graph import load_runtime_graph

_NATIVE_BUILD_STAGES: tuple[tuple[frozenset[str], str], ...] = (
    (frozenset({"lidar", "imu"}), "scripts/build/build_livox_sdk2_stream.sh"),
    (frozenset({"slam"}), "scripts/build/build_slam_core.sh"),
    (frozenset({"maps"}), "scripts/build/build_mapd.sh"),
    (
        frozenset({"traversability", "nav", "explore"}),
        "scripts/build/build_nav_endpoint.sh",
    ),
    (frozenset({"camera"}), "scripts/build/build_orbbec_native.sh"),
    (frozenset({"camera"}), "scripts/build/build_camera_dds.sh"),
    (frozenset({"host"}), "scripts/build/build_dds_probe.sh"),
    (frozenset({"driver"}), "scripts/build/build_driver.sh"),
)
_REAL_CONTROL_SUPPORT_BUILDS = ("scripts/build/build_native_recording.sh",)


def product_native_build_scripts(product: str) -> tuple[str, ...]:
    """Return builds for the logical roles declared by one Product."""

    selected_product = product_name(product)
    process_names = load_runtime_graph().products[selected_product].get("processes") or ()
    return (
        *_native_build_scripts_for_processes(str(name) for name in process_names),
        *_REAL_CONTROL_SUPPORT_BUILDS,
    )


def _native_build_scripts_for_processes(process_names) -> tuple[str, ...]:
    names = set(process_names)
    return tuple(
        script
        for required_processes, script in _NATIVE_BUILD_STAGES
        if names & required_processes
    )
