"""Pre-flight checks, port cleanup, health, daemon."""

from __future__ import annotations

import logging
import os
import subprocess
import sys

from . import term as T
from .profiles_data import _default_map_dir

# Built-in sample tomogram (relative to project root, ships in repo)
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_SAMPLE_TOMOGRAM = os.path.join(
    _REPO_ROOT, "src", "global_planning", "pct_planner",
    "rsc", "tomogram", "building2_9.pickle",
)


def _scan_maps(map_dir: str) -> list:
    """Return sorted list of map names that have at least a tomogram or PCD."""
    if not os.path.isdir(map_dir):
        return []
    maps = []
    for d in sorted(os.listdir(map_dir)):
        if d == "active":
            continue
        full = os.path.join(map_dir, d)
        if not os.path.isdir(full):
            continue
        has_tomo = os.path.isfile(os.path.join(full, "tomogram.pickle"))
        has_pcd  = os.path.isfile(os.path.join(full, "map.pcd"))
        if has_tomo or has_pcd:
            maps.append((d, has_tomo, has_pcd))
    return maps


def _select_map_interactive(cfg: dict, map_dir: str) -> None:
    """If slam=localizer and no active tomogram, let the user pick a map
    or choose to build one / use the built-in sample.

    Mutates cfg['tomogram'] in place if the user selects a map.
    Returns immediately (no-op) when not in an interactive TTY.
    """
    if not sys.stdin.isatty():
        return

    # Current tomogram already valid 鈫?nothing to do
    current = cfg.get("tomogram", "")
    if current and os.path.isfile(current):
        return

    maps = _scan_maps(map_dir)
    active_link = os.path.join(map_dir, "active")
    active_name = (
        os.path.basename(os.readlink(active_link))
        if os.path.islink(active_link)
        else ""
    )

    print()
    print(f"  {T.yellow('No active map found.')} Select how to proceed:\n")

    options = []

    if maps:
        print(f"  {T.bold('Saved maps:')}")
        for name, has_tomo, has_pcd in maps:
            parts = []
            if has_tomo:
                parts.append("tomogram")
            if has_pcd:
                parts.append("pcd")
            marker = f"  {T.green('*')} (active)" if name == active_name else ""
            print(f"    [{len(options)+1}] {T.green(name):30s} [{', '.join(parts)}]{marker}")
            options.append(("use", name, os.path.join(map_dir, name, "tomogram.pickle")))
        print()

    # Built-in sample option
    sample_label = "Use built-in sample map (building2_9) 鈥?PCT test only, not your environment"
    print(f"  {T.bold('Other options:')}")
    idx_sample = len(options) + 1
    print(f"    [{idx_sample}] {sample_label}")
    options.append(("sample", "building2_9", _SAMPLE_TOMOGRAM))

    idx_build = len(options) + 1
    print(f"    [{idx_build}] Switch to 'map' profile 鈥?build a new map first")
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
                action, name, tomo_path = options[idx]
                break
        print(f"  {T.red('?')} Enter a number between 1 and {len(options)}")

    if action == "use":
        cfg["tomogram"] = tomo_path
        # Also update the active symlink so subsequent starts remember the choice
        try:
            map_path = os.path.join(map_dir, name)
            if os.path.islink(active_link):
                os.unlink(active_link)
            os.symlink(map_path, active_link)
            print(f"  Active map set to: {T.green(name)}")
        except OSError as e:
            print(f"  {T.yellow('WARN')}: Could not update active symlink: {e}")

    elif action == "sample":
        cfg["tomogram"] = _SAMPLE_TOMOGRAM
        print(f"  {T.yellow('Using sample map')} 鈥?results reflect demo environment, not yours.")
        print("  Run 'lingtu map' on your robot to build a real map.")

    elif action == "build":
        print()
        print(f"  Run:  {T.green('python lingtu.py map')}")
        print("  Then: drive the robot around to build the map.")
        print("  Then: map save <name>  鈫? map use <name>")
        print()
        sys.exit(0)

    # action == "skip": continue with current (possibly empty) tomogram
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


def _nav_core_available() -> bool:
    """Check if _nav_core nanobind extension is importable."""
    try:
        import importlib
        return importlib.util.find_spec("_nav_core") is not None
    except Exception:
        return False


def preflight(profile_name: str, cfg: dict) -> None:
    slam = cfg.get("slam_profile", "none")

    if slam in (
        "fastlio2",
        "pointlio",
        "super_lio",
        "super_lio_relocation",
    ) and os.name != "nt":
        import shutil
        if not shutil.which("ros2"):
            print(f"  {T.yellow('!')} ros2 not in PATH 鈥?SLAM won't start")
            print(f"    Fix: {T.bold('source /opt/ros/humble/setup.bash')}")
            _bashrc_cmd = 'echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc'
            print(f"    Permanent: {T.dim(_bashrc_cmd)}")

    # Warn if _nav_core C++ extension is missing (affects terrain/local_planner/path_follower)
    if not _nav_core_available() and cfg.get("enable_native", False):
        print(f"  {T.yellow('!')} C++ nav_core not compiled 鈥?terrain/local_planner will use Python fallbacks")
        print(f"    Fix: {T.bold('make build')}  (needs ROS2 + colcon on S100P)")

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
                    port_open = any(
                        str(gw_port) in line and "ACCEPT" in line
                        for line in rules.splitlines()
                    )
                    if not port_open:
                        print(f"  {T.yellow('!')} Firewall may block port {gw_port} from LAN")
                        print(f"    Fix: {T.bold(f'sudo iptables -I INPUT -p tcp --dport {gw_port} -j ACCEPT')}")
                        print(f"    Also: {T.bold('sudo iptables -I INPUT -p tcp --dport 8090 -j ACCEPT')}")
            except (FileNotFoundError, subprocess.TimeoutExpired, PermissionError):
                pass

    # For saved-map localization profiles, offer interactive map selection.
    if slam in ("localizer", "super_lio_relocation"):
        map_dir = _default_map_dir()
        _select_map_interactive(cfg, map_dir)

        # Post-selection warning if still no valid tomogram
        tomogram = cfg.get("tomogram", "")
        if not tomogram or not os.path.isfile(tomogram):
            print(f"  {T.yellow('!')}: Tomogram not found: {tomogram or '(none)'}")
            print("        Navigation will start but PCT planner will be unavailable.")


def kill_residual_ports(cfg: dict) -> None:
    import platform

    if platform.system() != "Linux":
        return
    ports = [cfg.get("gateway_port", 5050), 8090]
    for port in ports:
        try:
            result = subprocess.run(
                ["fuser", "-k", "%d/tcp" % port],
                capture_output=True,
                timeout=3,
            )
            if result.returncode == 0:
                logging.getLogger(__name__).info("Killed residual process on port %d", port)
        except (FileNotFoundError, subprocess.TimeoutExpired):
            pass


def health_check(system) -> bool:
    ok = True
    for name, mod in system.modules.items():
        if mod is None:
            logging.getLogger("lingtu").error("Health check: module %s is None", name)
            ok = False
            continue
        if not hasattr(mod, "ports_in") or not hasattr(mod, "ports_out"):
            logging.getLogger("lingtu").error("Health check: %s missing ports", name)
            ok = False
    return ok


def daemonize(log_file: str) -> bool:
    """Unix: fork and detach. Windows: warn and stay foreground."""
    if os.name == "nt":
        print(f"  {T.yellow('Daemon mode not supported on Windows. Running in foreground.')}")
        return False

    pid = os.fork()
    if pid > 0:
        print(f"  Daemon started (PID {pid})")
        print(f"  Logs: {log_file}")
        print("  Stop: lingtu stop")
        sys.exit(0)

    os.setsid()

    pid = os.fork()
    if pid > 0:
        sys.exit(0)

    sys.stdout.flush()
    sys.stderr.flush()
    devnull = open(os.devnull)
    os.dup2(devnull.fileno(), sys.stdin.fileno())
    log_f = open(log_file, "a")
    os.dup2(log_f.fileno(), sys.stdout.fileno())
    os.dup2(log_f.fileno(), sys.stderr.fileno())

    return True
