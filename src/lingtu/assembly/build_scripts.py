"""Resolve native build scripts from Product process roles."""

from __future__ import annotations

from collections.abc import Iterable

from lingtu.assembly.graph import load_runtime_graph
from lingtu.assembly.graph.loader import resolve_product_variant_spec
from lingtu.products import product_name

_NATIVE_BUILD_STAGES: tuple[tuple[frozenset[str], str], ...] = (
    (frozenset({"lidar", "imu"}), "scripts/build/build_livox_sdk2_stream.sh"),
    (frozenset({"slam"}), "scripts/build/build_slam_core.sh"),
    (frozenset({"maps"}), "scripts/build/build_mapd.sh"),
    (
        frozenset({"traversability", "nav", "explore"}),
        "scripts/build/build_nav_endpoint.sh",
    ),
    (frozenset({"camera"}), "scripts/build/build_camera_dds.sh"),
    (frozenset({"video"}), "scripts/build/build_camera_video.sh"),
    (frozenset({"host"}), "scripts/build/build_dds_probe.sh"),
    (frozenset({"driver"}), "scripts/build/build_driver.sh"),
)
_REAL_CONTROL_SUPPORT_BUILDS = ("scripts/build/build_native_recording.sh",)
_CAMERA_CAPTURE_BUILDS = {
    "orbbec_native": "scripts/build/build_orbbec_native.sh",
    "realsense_native": "scripts/build/build_realsense_native.sh",
}


def product_native_build_scripts(
    product: str,
    *,
    product_variant: str | None = None,
    camera_driver: str = "orbbec_native",
) -> tuple[str, ...]:
    """Return builds for the logical roles declared by one Product."""

    selected_product = product_name(product)
    spec = resolve_product_variant_spec(
        selected_product,
        load_runtime_graph().products[selected_product],
        product_variant=product_variant,
    )
    process_names = spec.get("processes") or ()
    return (
        *_native_build_scripts_for_processes((str(name) for name in process_names), camera_driver=camera_driver),
        *_REAL_CONTROL_SUPPORT_BUILDS,
    )


def _native_build_scripts_for_processes(process_names: Iterable[str], *, camera_driver: str) -> tuple[str, ...]:
    names = set(process_names)
    scripts = []
    for required_processes, script in _NATIVE_BUILD_STAGES:
        if not names & required_processes:
            continue
        if "camera" in required_processes:
            try:
                scripts.append(_CAMERA_CAPTURE_BUILDS[camera_driver])
            except KeyError as exc:
                raise ValueError(f"unsupported camera capture driver: {camera_driver}") from exc
        scripts.append(script)
    return tuple(scripts)
