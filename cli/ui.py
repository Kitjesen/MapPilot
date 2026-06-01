"""Banner, profile listing, stop/status/log/show-config helpers."""

from __future__ import annotations

import json
import os
import re
import signal
import sys
import time
from collections import deque
from pathlib import Path

from . import term as T
from .profiles_data import PROFILES
from .run_state import clear_run_state, is_pid_alive, read_run_state, resolve_log_file
from .runtime_display import (
    format_frame_links,
    format_runtime_boundary,
    format_runtime_flow,
    format_runtime_flow_stages,
    format_runtime_frames,
    format_runtime_sources,
    format_runtime_topic_frames,
)


def _vlen(s: str) -> int:
    """Visible length of a string — strips ANSI escape codes."""
    return len(re.sub(r'\033\[[0-9;]*m', '', s))


# ── LOGO ──────────────────────────────────────────────────────────────────────
# figlet font: "ANSI Shadow"
_LOGO_LINES = [
    "  ██╗     ██╗███╗   ██╗ ██████╗     ████████╗██╗   ██╗",
    "  ██║     ██║████╗  ██║██╔════╝        ██╔══╝██║   ██║",
    "  ██║     ██║██╔██╗ ██║██║  ███╗       ██║   ██║   ██║",
    "  ██║     ██║██║╚██╗██║██║   ██║       ██║   ██║   ██║",
    "  ███████╗██║██║ ╚████║╚██████╔╝       ██║   ╚██████╔╝",
    "  ╚══════╝╚═╝╚═╝  ╚═══╝ ╚═════╝        ╚═╝    ╚═════╝ ",
]
_TAGLINE = "  Autonomous Navigation for Quadruped Robots"

# Profile icons and accent colors
_PROFILE_META = {
    "nav":     ("◉", "Navigate using a saved map",                  T.green),
    "explore": ("◎", "Explore unknown area",                         T.cyan),
    "map":     ("⊕", "Build a new map",                              T.yellow),
    "sim":     ("◈", "MuJoCo simulation",                            T.blue),
    "sim_gazebo": ("▣", "Gazebo/GZ ROS-native simulation",            T.blue),
    "sim_mujoco_live": ("M", "MuJoCo MID-360 + Fast-LIO live simulation", T.blue),
    "sim_industrial": ("#", "Gazebo industrial-yard delivery simulation", T.blue),
    "sim_cmu_tare": ("C", "CMU Unity + external TARE simulation",      T.blue),
    "dev":     ("◇", "Test perception & planning without a robot",   T.navy),
    "stub":    ("○", "Framework testing only",                       T.dim),
}

_PRODUCT_PROFILE_NAMES = ("map", "nav", "explore")
_ADVANCED_PROFILE_NAMES = ("tare_explore",)
_EXPERIMENTAL_PROFILE_NAMES = ("super_lio", "super_lio_relocation")


def _profile_tier(name: str) -> str:
    if name in _PRODUCT_PROFILE_NAMES:
        return "product"
    if name in _ADVANCED_PROFILE_NAMES:
        return "advanced"
    if name in _EXPERIMENTAL_PROFILE_NAMES:
        return "experimental"
    return "dev"


def _visible_profile_names(*, show_all: bool = False) -> list[str]:
    if show_all:
        return list(PROFILES.keys())
    return [name for name in _PRODUCT_PROFILE_NAMES if name in PROFILES]


# Which wizard questions are relevant per profile.
# Keys: "semantic", "gateway", "teleop"
_PROFILE_WIZARD: dict[str, tuple[bool, bool, bool]] = {
    #                  semantic  gateway  teleop
    "nav":     (True,  True,  True),   # full stack
    "explore": (True,  True,  False),  # exploring — no joystick needed
    "map":     (False, True,  False),  # mapping — no semantics, no joystick
    "sim":     (True,  True,  True),   # full stack in sim
    "sim_gazebo": (True, True, True),  # ROS-native Gazebo simulation
    "sim_mujoco_live": (False, True, False),  # external MuJoCo/Fast-LIO live gate
    "sim_industrial": (True, True, False),  # ROS-native delivery simulation
    "sim_cmu_tare": (False, True, False),  # external CMU Unity/TARE graph
    "dev":     (True,  True,  False),  # no robot → no teleop
    "stub":    (False, True,  False),  # bare framework
}


def _print_logo() -> None:
    for line in _LOGO_LINES:
        print(T.navy(line))
    print(T.dim(_TAGLINE))
    print()


def _panel(lines: list[str], *, color) -> None:
    """Print a compact config panel — ANSI-aware width."""
    if not lines:
        return
    width = max(_vlen(x) for x in lines)
    top = f"┌{'─' * (width + 2)}┐"
    bot = f"└{'─' * (width + 2)}┘"
    print(f"  {color(top)}")
    for line in lines:
        pad = width - _vlen(line)
        print(f"  {color('│')} {line}{' ' * pad} {color('│')}")
    print(f"  {color(bot)}")


def _local_ip() -> str:
    """Best-effort local LAN IP (not 127.0.0.1)."""
    import socket
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "localhost"


def print_banner(profile_name, cfg, system, log_dir: str) -> None:
    n   = len(system.modules)
    nc  = len(system.connections)
    desc = cfg.get("_desc", "custom")
    gw   = cfg.get("gateway_port", 5050)

    robot    = cfg.get("robot", "?")
    planner  = cfg.get("planner", "?")
    detector = cfg.get("detector", "?")
    llm      = cfg.get("llm", "?")

    # Wait briefly for TeleopModule to finish binding its port
    import time as _time
    tomogram  = cfg.get("tomogram", "")
    tomo_note = T.yellow(" ⚠ sample map") if (tomogram and "building2_9.pickle" in str(tomogram)) else ""

    icon, _, color = _PROFILE_META.get(profile_name, ("·", desc, T.dim))
    W = 58

    lan_ip = _local_ip()

    _print_logo()

    print(T.navy(f"  ┌{'─' * W}┐"))
    print(T.navy("  │") + T.bold(f"{'  System Ready':^{W}}") + T.navy("│"))
    print(T.navy(f"  ├{'─' * W}┤"))

    def _row(label: str, value: str) -> None:
        inner = f"  {T.dim(f'{label:<10}')} {value}"
        pad = max(0, W - _vlen(inner))
        print(T.navy("  │") + inner + " " * pad + T.navy("│"))

    _row("profile",  color(f"{icon} {profile_name}") + T.dim(f"  {desc}"))
    _row("robot",    f"{robot}  {T.dim('planner:')} {planner}")
    if cfg.get("enable_semantic"):
        _row("semantic", f"{T.dim('detector:')} {detector}  {T.dim('llm:')} {llm}")
    else:
        _row("semantic", T.dim("disabled"))
    if tomogram:
        tomo_short = os.path.basename(tomogram)
        _row("map",      T.dim(tomo_short) + tomo_note)
    if cfg.get("enable_gateway"):
        _row("gateway",  T.cyan(f"http://{lan_ip}:{gw}"))
        # Teleop is served by GatewayModule on the same port
        try:
            system.get_module("TeleopModule")
            _row("teleop",   T.cyan(f"ws://{lan_ip}:{gw}/ws/teleop"))
        except (KeyError, Exception):
            pass
    _row("health",   T.green(f"OK {n} modules") + T.dim(f"  {nc} connections"))
    _row("logs",     T.dim(log_dir))

    print(T.navy(f"  ├{'─' * W}┤"))
    hint_raw = "info · chat · agent · history · map list · help · Ctrl+C"
    hint_pad = max(0, W - len(hint_raw))
    lpad = hint_pad // 2
    rpad = hint_pad - lpad
    hint_line = T.dim(" " * lpad + hint_raw + " " * rpad)
    print(T.navy("  │") + hint_line + T.navy("│"))
    print(T.navy(f"  └{'─' * W}┘"))
    print()


def select_interactive() -> str:
    """Full-screen profile picker with LOGO and styled cards."""
    _print_logo()

    names = _visible_profile_names()
    W = 54  # card inner width

    print(T.navy(f"  ┌{'─' * W}┐"))
    print(T.navy("  │") + T.bold(f"{'  Select a profile to launch':^{W}}") + T.navy("│"))
    print(T.navy(f"  ├{'─' * W}┤"))

    for i, name in enumerate(names, 1):
        icon, desc, color = _PROFILE_META.get(name, ("·", PROFILES[name].get("_desc", ""), T.dim))
        num  = T.dim(f" {i} ")
        tag  = color(f" {icon} {name:<9}")
        body = T.dim(f" {desc}")
        inner = f"{num}{tag}{body}"
        pad = max(0, W - _vlen(inner))
        print(T.navy("  │") + inner + " " * pad + T.navy("│"))

    print(T.navy(f"  └{'─' * W}┘"))
    print()
    print(T.dim("  ↑↓ type number or name  ·  Ctrl+C to quit"))
    print()

    while True:
        try:
            choice = input(T.cyan("  › ")).strip()
        except (EOFError, KeyboardInterrupt):
            print()
            sys.exit(0)
        if not choice:
            continue
        if choice.isdigit():
            idx = int(choice) - 1
            if 0 <= idx < len(names):
                selected = names[idx]
                icon, _, color = _PROFILE_META.get(selected, ("·", "", T.dim))
                print(f"\n  {color(f'✓ {selected}')}  {T.dim(PROFILES[selected].get('_desc', ''))}\n")
                return selected
        if choice in PROFILES:
            icon, _, color = _PROFILE_META.get(choice, ("·", "", T.dim))
            print(f"\n  {color(f'✓ {choice}')}  {T.dim(PROFILES[choice].get('_desc', ''))}\n")
            return choice
        print(f"  {T.red('✗')} {T.dim(f'Unknown: {choice!r}  (try 1–{len(names)})')}")


def ask_bool(prompt: str, *, default: bool | None = None) -> bool:
    """Ask a yes/no question in TTY, returning a boolean.

    Inputs accepted: y/yes, n/no, empty (uses default if provided).
    """
    if not sys.stdin.isatty():
        return bool(default) if default is not None else False

    suffix = ""
    if default is True:
        suffix = " [Y/n]"
    elif default is False:
        suffix = " [y/N]"
    else:
        suffix = " [y/n]"

    while True:
        try:
            raw = input(f"  {prompt}{suffix}: ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            sys.exit(0)
        if raw == "" and default is not None:
            return default
        if raw in ("y", "yes"):
            return True
        if raw in ("n", "no"):
            return False
        print(f"  {T.red('?')} Please enter y or n")


def wizard_interactive(profile_name: str, cfg: dict) -> None:
    """TTY wizard: show only the toggles that make sense for this profile.

    Mutates cfg in place.
    """
    if not sys.stdin.isatty():
        return

    ask_sem, ask_gw, ask_tp = _PROFILE_WIZARD.get(
        profile_name, (True, True, True)
    )

    # Nothing to ask — skip the wizard entirely
    if not (ask_sem or ask_gw or ask_tp):
        return

    icon, _, color = _PROFILE_META.get(profile_name, ("·", "", T.dim))
    W = 54
    print(T.navy(f"  ┌{'─' * W}┐"))
    print(T.navy("  │") + T.bold(f"{'  Configure launch options':^{W}}") + T.navy("│"))
    print(T.navy(f"  ├{'─' * W}┤"))
    profile_val = color(f"{icon} {profile_name}")
    profile_row = f"  profile: {profile_val}"
    print(T.navy("  │") + profile_row + " " * max(0, W - _vlen(profile_row)) + T.navy("│"))
    print(T.navy(f"  ├{'─' * W}┤"))
    hint_row = T.dim("  Press Enter to accept defaults shown in [brackets]")
    print(T.navy("  │") + hint_row + " " * max(0, W - _vlen(hint_row)) + T.navy("│"))
    print(T.navy(f"  └{'─' * W}┘"))
    print()

    sem_def = bool(cfg.get("enable_semantic", True))
    gw_def  = bool(cfg.get("enable_gateway",  True))
    tp_def  = bool(cfg.get("enable_teleop",   True))

    if ask_sem:
        cfg["enable_semantic"] = ask_bool(
            f"  {T.cyan('◈')} Semantic  {T.dim('(object detection · memory · LLM reasoning)')}",
            default=sem_def,
        )

    if ask_gw:
        cfg["enable_gateway"] = ask_bool(
            f"  {T.cyan('◈')} Gateway   {T.dim('(HTTP API · MCP tools for external control)')}",
            default=gw_def,
        )

    if ask_tp and cfg.get("enable_gateway", gw_def):
        cfg["enable_teleop"] = ask_bool(
            f"  {T.cyan('◈')} Teleop    {T.dim('(joystick remote control · live camera)')}",
            default=tp_def,
        )
    else:
        cfg["enable_teleop"] = False

    print()


def list_profiles(*, show_all: bool = False):
    title = "Available product profiles" if not show_all else "Available profiles"
    print(f"\n  {T.bold(title + ':')}\n")
    for name in _visible_profile_names(show_all=show_all):
        p = PROFILES[name]
        tier = _profile_tier(name)
        tier_note = "" if tier == "product" else f" [{tier}]"
        print(f"  {T.green(f'{name:10s}')} {p['_desc']}{T.dim(tier_note)}")
        parts = []
        if p.get("robot"):
            parts.append(f"robot={p['robot']}")
        if p.get("detector"):
            parts.append(f"detector={p['detector']}")
        if p.get("llm"):
            parts.append(f"llm={p['llm']}")
        if not p.get("enable_native"):
            parts.append("no-native")
        if not p.get("enable_semantic"):
            parts.append("no-semantic")
        print(f"  {T.dim('           ' + ', '.join(parts))}")
    if not show_all:
        print(T.dim("  Use --list --all to show advanced, simulation, and dev profiles."))
    print("\n  Override any flag: lingtu nav --llm mock\n")


def _force_stop_signal() -> int:
    return getattr(signal, "SIGKILL", signal.SIGTERM)


def cmd_stop(force: bool = False) -> None:
    state = read_run_state()
    if state is None:
        print("  No running instance found (.lingtu/run.json missing)")
        sys.exit(1)

    pid = state.get("pid")
    if not pid or not is_pid_alive(pid):
        print(f"  Stale PID {pid} (process not alive). Cleaning up.")
        clear_run_state()
        sys.exit(0)

    print(f"  Stopping PID {pid} (profile: {state.get('profile', '?')})...")
    try:
        os.kill(pid, _force_stop_signal() if force else signal.SIGTERM)
    except OSError as e:
        print(f"  Failed to stop: {e}")
        sys.exit(1)

    if force:
        clear_run_state()
        print(f"  {T.green('Force killed.')}")
        return

    for _ in range(30):
        if not is_pid_alive(pid):
            print(f"  {T.green('Stopped.')}")
            clear_run_state()
            return
        time.sleep(0.5)

    print(f"  {T.yellow('Process still alive after 15s. Force kill?')}")
    try:
        os.kill(pid, _force_stop_signal())
        clear_run_state()
        print(f"  {T.green('Force killed.')}")
    except OSError:
        print(f"  Could not kill PID {pid}. Manual cleanup needed.")


def cmd_status_external(as_json: bool = False) -> None:
    state = read_run_state()
    if state is None:
        if as_json:
            print(json.dumps({"status": "not_running"}))
        else:
            print("  No running instance")
        return

    from .run_state import compute_uptime, format_uptime

    pid = state.get("pid")
    alive = is_pid_alive(pid) if pid else False
    uptime = compute_uptime(state) if alive else None

    if as_json:
        report = dict(state)
        report["alive"] = alive
        report["uptime_seconds"] = uptime
        report["runtime_status"] = (
            state.get("status", "running") if alive else "dead"
        )
        print(json.dumps(report, indent=2, default=str))
        return

    profile = state.get("profile", "?")
    started = state.get("started_at", "?")
    cwd = state.get("cwd", "?")
    log_dir = state.get("log_dir", "?")
    log_file = state.get("log_file", "?")
    daemon = state.get("daemon", False)
    argv = state.get("argv", [])
    host = state.get("host", "?")
    version = state.get("version", "?")
    modules = state.get("module_count")
    wires = state.get("wire_count")
    runtime_status = state.get("status", "running")
    runtime = state.get("runtime")
    runtime = runtime if isinstance(runtime, dict) else {}

    status_label = T.green(runtime_status) if alive else T.red("dead (stale PID)")
    print(f"\n  Status:    {status_label}")
    print(f"  PID:       {pid}")
    print(f"  Host:      {host}")
    print(f"  Version:   {version}")
    print(f"  Profile:   {profile}")
    if runtime:
        print(f"  Runtime:  {format_runtime_boundary(runtime)}")
        print(f"  SLAM:     {format_runtime_sources(runtime)}")
        print(f"  Frame ids: {format_runtime_frames(runtime)}")
        print(f"  Frames:   {format_frame_links(runtime)}")
        print(f"  Topic frames: {format_runtime_topic_frames(runtime)}")
        print(f"  Flow:     {format_runtime_flow(runtime)}")
        print(f"  Flow stages: {format_runtime_flow_stages(runtime)}")
    print(f"  Started:   {started}")
    if uptime is not None:
        print(f"  Uptime:    {format_uptime(uptime)}")
    print(f"  Mode:      {'daemon' if daemon else 'foreground'}")
    if modules is not None or wires is not None:
        mod_str = str(modules) if modules is not None else "?"
        wire_str = str(wires) if wires is not None else "?"
        print(f"  Blueprint: {mod_str} modules, {wire_str} wires")
    print(f"  CWD:       {cwd}")
    print(f"  Logs:      {log_dir}")
    print(f"  Log file:  {log_file}")
    if argv:
        print(f"  Args:      {' '.join(str(a) for a in argv)}")

    if not alive:
        print(f"\n  {T.yellow('Stale run state. Run `lingtu stop` to clean up.')}")
    print()


def cmd_show_config_external(profile_name: str, cfg: dict, as_json: bool = False) -> None:
    resolved = {k: v for k, v in cfg.items() if not k.startswith("_")}
    if as_json:
        print(json.dumps(resolved, indent=2, ensure_ascii=False, sort_keys=True, default=str))
        return
    print(f"\n  {T.bold('Resolved config')}  [{T.green(profile_name)}]\n")
    print(json.dumps(resolved, indent=2, ensure_ascii=False, sort_keys=True, default=str))
    print()


def cmd_restart() -> None:
    """Stop current instance, then restart it with the same argv + cwd."""
    state = read_run_state()
    if state is None:
        print("  No running instance to restart (.lingtu/run.json missing)")
        sys.exit(1)

    argv = list(state.get("argv") or [])
    cwd = state.get("cwd") or os.getcwd()

    cmd_stop(force=False)
    # read_run_state is cleared by cmd_stop on success; wait briefly for cleanup.
    time.sleep(0.5)

    python_exe = sys.executable or "python3"
    repo_root = Path(__file__).resolve().parent.parent
    entry = repo_root / "lingtu.py"
    new_cmd = [python_exe, str(entry), *argv]

    print(f"  Restarting: {' '.join(new_cmd)}")
    import subprocess as _sp

    try:
        _sp.Popen(new_cmd, cwd=cwd)
        print(f"  {T.green('Restart launched.')} Use `lingtu status` to verify.")
    except OSError as e:
        print(f"  {T.red('Restart failed')}: {e}")
        sys.exit(1)


def cmd_health_external(as_json: bool = False) -> None:
    """Query running instance's /api/v1/health endpoint and display sensor/module status."""
    state = read_run_state()
    if state is None:
        print("  无运行实例")
        return

    pid = state.get("pid")
    if not is_pid_alive(pid):
        print(f"  实例已停止 (PID {pid})")
        return

    port = state.get("config", {}).get("gateway_port", 5050)
    import urllib.error
    import urllib.request

    url = f"http://localhost:{port}/api/v1/health"
    try:
        req = urllib.request.Request(url, headers={"Accept": "application/json"})
        with urllib.request.urlopen(req, timeout=5) as resp:  # noqa: S310  # trusted localhost
            data = json.loads(resp.read())
    except (urllib.error.URLError, OSError) as e:
        print(f"  {T.red('无法连接 Gateway')}: {e}")
        return

    if as_json:
        print(json.dumps(data, indent=2, default=str))
        return

    status = data.get("status", "?")
    status_color = T.green(status) if status == "ok" else T.red(status)
    print(f"\n  系统状态:  {status_color}")
    print(f"  模块:      {T.green(str(data.get('modules_ok', '?')))} 正常"
          f"  {T.red(str(data.get('modules_fail', 0))) if data.get('modules_fail') else '0'} 异常")

    sensors = data.get("sensors", {})
    if sensors:
        print(f"\n  {'传感器':<12} {'状态':<12} {'详情'}")
        print(f"  {'─'*12} {'─'*12} {'─'*24}")
        for name, info in sensors.items():
            s = info.get("status", "?")
            s_colored = T.green(s) if s in ("streaming", "active", "connected") else T.yellow(s)
            details = []
            for k, v in info.items():
                if k != "status":
                    details.append(f"{k}={v}")
            print(f"  {name:<12} {s_colored:<20} {', '.join(details)}")

    teleop = data.get("teleop", {})
    gw = data.get("gateway", {})
    print(f"\n  Gateway:   端口 {gw.get('port', '?')}  SSE客户端 {gw.get('sse_clients', 0)}")
    print(f"  遥控:      {'活跃' if teleop.get('active') else '空闲'}  客户端 {teleop.get('clients', 0)}")
    print(f"  SLAM:      {data.get('slam_hz', 0)} Hz  点云帧 {data.get('map_points', 0)}")
    print(f"  里程计:    {'✓' if data.get('has_odom') else '✗'}")

    fails = [n for n, s in data.get("modules", {}).items() if s != "ok"]
    if fails:
        print(f"\n  {T.red('异常模块')}: {', '.join(fails)}")
    print()


def cmd_log_external(follow: bool = False, lines: int = 80) -> None:
    state = read_run_state()
    if state is None:
        print("  No running instance found (.lingtu/run.json missing)")
        sys.exit(1)

    log_file = resolve_log_file(state)
    if log_file is None:
        print("  No log file found for current run state")
        sys.exit(1)

    pid = state.get("pid")
    alive = is_pid_alive(pid) if pid else False
    print(f"  Reading: {log_file}")
    if follow:
        status = "running" if alive else "stale"
        print(f"  Follow:  on ({status})\n")
    else:
        print()

    try:
        with log_file.open("r", encoding="utf-8", errors="replace") as fh:
            if lines > 0:
                for line in deque(fh, maxlen=lines):
                    sys.stdout.write(line)
            else:
                sys.stdout.write(fh.read())
    except OSError as e:
        print(f"  Failed to read log file: {e}")
        sys.exit(1)

    if not follow:
        print()
        return

    try:
        with log_file.open("r", encoding="utf-8", errors="replace") as fh:
            fh.seek(0, os.SEEK_END)
            while True:
                line = fh.readline()
                if line:
                    sys.stdout.write(line)
                    sys.stdout.flush()
                    continue
                if pid and not is_pid_alive(pid):
                    print(f"\n  {T.yellow('Process exited; follow stopped.')}")
                    return
                time.sleep(0.5)
    except KeyboardInterrupt:
        print(f"\n  {T.dim('Log follow stopped')}")
