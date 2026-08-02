"""Local Profile pre-flight and map-selection helpers."""

from __future__ import annotations

import logging
import os
import subprocess
import sys

from maps.adapters.python.service import (
    MapsServiceNativeUnavailable,
    NativeMapsService,
)
from nav.kernel import nav_kernel_available, nav_kernel_build_hint
from runtime.profiles.binding_policy import nav_kernel_backend_required
from runtime.profiles.catalog.runtime_paths import _default_map_dir

from . import term as T


def _close_maps_service(service: object | None) -> None:
    close = getattr(service, "close", None)
    if callable(close):
        close()


def _navigation_map_entries(
    payload: dict,
    service: NativeMapsService,
) -> list[tuple[str, str, bool, bool]]:
    entries: list[tuple[str, str, bool, bool]] = []
    for item in payload.get("maps") or []:
        if not isinstance(item, dict):
            continue
        name = str(item.get("name") or "")
        if not name:
            continue
        octomap = service.get_bundle(name, "navigation_safety_3d")
        if octomap.get("success") is not True:
            continue
        pcd = service.get_bundle(name, "source_pointcloud")
        entries.append(
            (
                name,
                str((octomap.get("artifact") or {}).get("uri") or ""),
                True,
                pcd.get("success") is True,
            )
        )
    return entries


def _scan_maps(map_dir: str) -> list:
    """Return navigation-ready maps through the canonical maps service."""
    service = None
    try:
        service = NativeMapsService(map_dir)
        payload = service.list_maps()
        if payload.get("success") is not True:
            return []
        return sorted(
            _navigation_map_entries(payload, service),
            key=lambda entry: entry[0],
        )
    except (MapsServiceNativeUnavailable, OSError, RuntimeError) as exc:
        logging.getLogger(__name__).warning("maps service unavailable: %s", exc)
        return []
    finally:
        _close_maps_service(service)


def _select_map_interactive(cfg: dict, map_dir: str) -> None:
    """If slam=localizer and no active planner map exists, let the user pick one.

    Mutates the planner-map config in place if the user selects a map.
    Returns immediately (no-op) when not in an interactive TTY.
    """
    if not sys.stdin.isatty():
        return

    # Current planner map already valid.
    current = cfg.get("planner_map") or cfg.get("map_path") or cfg.get("octomap", "")
    if current and os.path.isfile(current):
        return

    service = None
    try:
        service = NativeMapsService(map_dir)
        catalog = service.list_maps()
    except (MapsServiceNativeUnavailable, OSError, RuntimeError) as exc:
        logging.getLogger(__name__).warning("maps service unavailable: %s", exc)
        catalog = {"success": False, "maps": [], "active": ""}
    maps = (
        sorted(_navigation_map_entries(catalog, service), key=lambda entry: entry[0])
        if catalog.get("success") is True and service is not None
        else []
    )
    active_name = str(catalog.get("active") or "")

    print()
    print(f"  {T.yellow('No active map found.')} Select how to proceed:\n")

    options = []

    if maps:
        print(f"  {T.bold('Saved maps:')}")
        for name, planner_map, has_octomap, has_pcd in maps:
            parts = []
            if has_octomap:
                parts.append("octomap")
            if has_pcd:
                parts.append("pcd")
            if planner_map.endswith("occupancy.npz"):
                parts.append("occupancy")
            marker = f"  {T.green('*')} (active)" if name == active_name else ""
            print(f"    [{len(options) + 1}] {T.green(name):30s} [{', '.join(parts)}]{marker}")
            options.append(("use", name, planner_map))
        print()

    print(f"  {T.bold('Other options:')}")
    idx_build = len(options) + 1
    print(f"    [{idx_build}] Switch to 'map' profile and build a new map first")
    options.append(("build", "", ""))

    idx_skip = len(options) + 1
    print(f"    [{idx_skip}] Continue without a map (navigation will fail to plan paths)")
    options.append(("skip", "", ""))

    print()
    while True:
        try:
            raw = input(f"  Choice [1-{len(options)}]: ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            sys.exit(0)
        if not raw:
            continue
        if raw.isdigit():
            idx = int(raw) - 1
            if 0 <= idx < len(options):
                action, name, planner_map = options[idx]
                break
        print(f"  {T.red('?')} Enter a number between 1 and {len(options)}")

    if action == "use":
        try:
            if service is None:
                raise RuntimeError("maps service is unavailable")
            activated = service.set_active_map(name, strict=True)
            if activated.get("success") is not True:
                raise RuntimeError(str(activated.get("message") or "map activation failed"))
            cfg["planner_map"] = planner_map
            cfg["map_path"] = planner_map
            cfg["octomap"] = planner_map
            print(f"  Active map set to: {T.green(name)}")
        except (OSError, RuntimeError) as exc:
            print(f"  {T.red('Error')}: Could not activate map: {exc}")

    elif action == "build":
        _close_maps_service(service)
        print()
        print(f"  Run:  {T.green('python lingtu.py map')}")
        print("  Then: drive the robot around to build the map.")
        print("  Then: map save <name> and map use <name>")
        print()
        sys.exit(0)

    # action == "skip": continue with current (possibly empty) map path
    _close_maps_service(service)
    print()


def _check_port_accessible(port: int) -> bool:
    """Return True if the port is reachable from localhost (i.e. not firewalled internally)."""
    import socket

    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.3)
        s.connect(("127.0.0.1", port))
        s.close()
        return True
    except OSError:
        return False


def _native_nav_kernel_available() -> bool:
    """Check if the LingTu native navigation kernel is importable."""
    return nav_kernel_available()


def _octoplanner3d_runtime_errors(cfg: dict) -> tuple[str, ...]:
    planner = str(cfg.get("planner") or cfg.get("planner_backend") or "").strip().lower()
    if planner not in {"octoplanner3d", "octplanner", "octo", "octomap"}:
        return ()

    from nav.services.plan.global_planner.algorithm.octoplanner3d_protocol import (
        SUPPORTED_MAP_EXTENSIONS,
    )
    from nav.services.plan.global_planner.algorithm.octoplanner3d_runtime import (
        OctoPlanner3DRuntime,
    )

    runtime = OctoPlanner3DRuntime(
        executable_path=cfg.get("octoplanner3d_executable") or None,
        timeout_s=cfg.get("octoplanner3d_timeout_s"),
    )
    map_path = str(cfg.get("planner_map") or cfg.get("map_path") or cfg.get("octomap") or "")
    return tuple(runtime.validate_map(map_path, SUPPORTED_MAP_EXTENSIONS))


def _uses_non_ros_localization_adapter(cfg: dict) -> bool:
    """Return True when localization is provided by an endpoint adapter."""

    adapter = str(cfg.get("localization_adapter") or cfg.get("_localization_adapter") or "").lower()
    if adapter in {"dds_endpoint", "cpp_slam_status"}:
        return True

    endpoint_transport = str(cfg.get("endpoint_transport") or cfg.get("_endpoint_transport") or "").lower()
    endpoint_contract = str(cfg.get("endpoint_contract") or cfg.get("_endpoint_contract") or "")
    return endpoint_transport == "dds" and bool(endpoint_contract)


def _ros_setup_path() -> str:
    return f"/opt/ros/{os.environ.get('ROS_DISTRO', 'humble')}/setup.bash"


def preflight(profile_name: str, cfg: dict) -> None:
    slam = cfg.get("slam_profile", "none")

    if (
        slam
        in (
            "fastlio2",
            "pointlio",
        )
        and os.name != "nt"
        and not _uses_non_ros_localization_adapter(cfg)
    ):
        import shutil

        if not shutil.which("ros2"):
            setup_path = _ros_setup_path()
            print(f"  {T.yellow('!')} ros2 not in PATH; this SLAM profile uses the ROS2 compatibility runtime")
            print(f"    Fix: {T.bold(f'source {setup_path}')}")
            _bashrc_cmd = f'echo "source {setup_path}" >> ~/.bashrc'
            print(f"    Permanent: {T.dim(_bashrc_cmd)}")
    elif (
        slam
        in (
            "fastlio2",
            "pointlio",
        )
        and os.name == "nt"
    ):
        print(
            f"  {T.yellow('!')} Windows local FastLIO2 has no supported portable "
            "runtime; the previous portable-lio endpoint was removed."
        )
        print("    Use the field DDS localization endpoint, or run ROS2 compatibility on Linux.")

    if (
        nav_kernel_backend_required(
            cfg,
            enable_native=bool(cfg.get("enable_native", True)),
        )
        and not _native_nav_kernel_available()
    ):
        print(f"  {T.red('Error')}: LingTu native navigation kernel is required by this profile")
        print(f"    {nav_kernel_build_hint()}")
        print("    The production chain will not silently fall back to Python.")
        sys.exit(2)

    octo_errors = _octoplanner3d_runtime_errors(cfg)
    if octo_errors:
        print(f"  {T.red('Error')}: OctoPlanner3D runtime is required by this profile")
        for error in octo_errors:
            print(f"    - {error}")
        print("    Run: bash scripts/build/build_octoplanner3d.sh")
        sys.exit(2)

    # Check if gateway port will be reachable from LAN (firewall check)
    if cfg.get("enable_gateway"):
        gw_port = cfg.get("gateway_port", 5050)
        import platform

        if platform.system() == "Linux":
            # Check if iptables is likely blocking the port
            try:
                result = subprocess.run(
                    ["iptables", "-L", "INPUT", "-n", "--line-numbers"],
                    capture_output=True,
                    text=True,
                    encoding="utf-8",
                    errors="replace",
                    timeout=2,
                )
                rules = result.stdout
                # If there's a DROP/REJECT default policy and no ACCEPT for our port, warn
                if "policy DROP" in rules or "policy REJECT" in rules:
                    port_open = any(str(gw_port) in line and "ACCEPT" in line for line in rules.splitlines())
                    if not port_open:
                        print(f"  {T.yellow('!')} Firewall may block port {gw_port} from LAN")
                        print(f"    Fix: {T.bold(f'sudo iptables -I INPUT -p tcp --dport {gw_port} -j ACCEPT')}")
                        print(f"    Also: {T.bold('sudo iptables -I INPUT -p tcp --dport 8090 -j ACCEPT')}")
            except (FileNotFoundError, subprocess.TimeoutExpired, PermissionError):
                pass

    # For saved-map localization profiles, offer interactive map selection.
    if slam == "localizer":
        map_dir = _default_map_dir()
        _select_map_interactive(cfg, map_dir)

        # Post-selection warning if still no valid planner map.
        planner_map = cfg.get("planner_map") or cfg.get("map_path") or cfg.get("octomap", "")
        if not planner_map or not os.path.isfile(planner_map):
            print(f"  {T.yellow('!')}: Planner map not found: {planner_map or '(none)'}")
            print("        Navigation will start but map-backed global planning may be unavailable.")
