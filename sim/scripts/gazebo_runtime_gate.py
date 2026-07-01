#!/usr/bin/env python3
"""Start the ROS-native Gazebo runtime and verify the LingTu TF/topic contract."""

from __future__ import annotations

import argparse
import json
import os
import signal
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_GAZEBO_WORLD = (
    ROOT / "sim" / "worlds" / "gazebo" / "lingtu_gazebo_demo_room.sdf"
)


def _resolve_world_arg(value: str | os.PathLike[str]) -> str:
    raw = str(value)
    path = Path(raw)
    if path.exists():
        return str(path.resolve())
    candidate = ROOT / raw
    if candidate.exists():
        return str(candidate.resolve())
    return raw


def _phase_domain_id(base_domain: int, offset: int) -> int:
    if base_domain >= 100:
        base_domain = base_domain % 100
    return max(0, min(99, base_domain + offset))


def _terminate_process_tree(proc: subprocess.Popen[Any], timeout_s: float = 8.0) -> None:
    if os.name == "posix":
        try:
            os.killpg(proc.pid, signal.SIGINT)
        except ProcessLookupError:
            return
    else:
        if proc.poll() is not None:
            return
        proc.terminate()
    try:
        proc.wait(timeout=timeout_s)
        return
    except subprocess.TimeoutExpired:
        pass
    if os.name == "posix":
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
    else:
        proc.kill()
    try:
        proc.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        if os.name != "posix":
            proc.kill()


def _load_report(path: Path, schema_version: str = "lingtu.gazebo_runtime_smoke.v1") -> dict[str, Any]:
    if not path.exists():
        return {
            "schema_version": schema_version,
            "ok": False,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "errors": [f"smoke report not written: {path}"],
        }
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    if isinstance(data, dict):
        return data
    return {
        "schema_version": "lingtu.gazebo_runtime_smoke.v1",
        "ok": False,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "errors": ["smoke report was not a JSON object"],
    }


def _json_from_stdout(text: str) -> dict[str, Any]:
    try:
        data = json.loads(text)
        if isinstance(data, dict):
            return data
    except json.JSONDecodeError:
        pass
    return {
        "ok": False,
        "error": "stdout was not a JSON object",
        "stdout_tail": text[-2000:],
    }


def run_gate(args: argparse.Namespace) -> dict[str, Any]:
    out_path = Path(args.json_out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    launch_log = Path(args.launch_log)
    launch_log.parent.mkdir(parents=True, exist_ok=True)
    smoke_tmp = out_path.with_suffix(".smoke.tmp.json")
    nav_tmp = out_path.with_suffix(".nav.tmp.json")
    frontier_tmp = out_path.with_suffix(".frontier.tmp.json")

    launch_cmd = [
        "ros2",
        "launch",
        str(ROOT / "launch" / "gazebo_simulation.launch.py"),
        f"world:={_resolve_world_arg(args.world)}",
        f"headless:={'true' if args.headless else 'false'}",
        f"use_bridge:={'true' if args.use_bridge else 'false'}",
        f"spawn_robot:={'true' if args.spawn_robot else 'false'}",
        f"spawn_x:={args.spawn_x}",
        f"spawn_y:={args.spawn_y}",
        f"spawn_z:={args.spawn_z}",
        f"spawn_yaw:={args.spawn_yaw}",
    ]
    smoke_cmd = [
        sys.executable,
        str(ROOT / "sim" / "scripts" / "tf_contract_smoke.py"),
        "--timeout-sec",
        str(args.smoke_timeout_sec),
        "--min-samples",
        str(args.min_samples),
        "--require-sensors",
        "--json",
        "--json-out",
        str(smoke_tmp),
    ]
    if args.require_camera:
        smoke_cmd.append("--require-camera")
    nav_launch_cmd = [
        "ros2",
        "launch",
        str(ROOT / "sim" / "planning" / "sim_navigation.launch.py"),
        "use_sim_robot:=false",
        "use_terrain_passthrough:=false",
        "flatten_global_path_z:=true",
        "use_gazebo_line_planner:=true",
        f"goal_x:={args.nav_hold_goal_x}",
        f"goal_y:={args.nav_hold_goal_y}",
        f"goal_z:={args.nav_hold_goal_z}",
    ]
    frontier_nav_launch_cmd = [*nav_launch_cmd, "gazebo_line_require_grid:=true"]
    nav_smoke_cmd = [
        sys.executable,
        str(ROOT / "sim" / "scripts" / "gazebo_nav_loop_smoke.py"),
        "--timeout-sec",
        str(args.nav_timeout_sec),
        "--goal-delay-sec",
        str(args.nav_goal_delay_sec),
        "--goal-republish-sec",
        str(args.nav_goal_republish_sec),
        "--goal-x",
        str(args.nav_goal_x),
        "--goal-y",
        str(args.nav_goal_y),
        "--goal-z",
        str(args.nav_goal_z),
        "--json",
        "--json-out",
        str(nav_tmp),
    ]
    if args.require_forward_progress:
        nav_smoke_cmd.append("--require-forward-progress")
    frontier_smoke_cmd = [
        sys.executable,
        str(ROOT / "sim" / "scripts" / "gazebo_frontier_exploration_smoke.py"),
        "--timeout-sec",
        str(args.frontier_timeout_sec),
        "--json",
        "--json-out",
        str(frontier_tmp),
        "--coverage-size-m",
        str(args.frontier_coverage_size_m),
        "--room-min-x",
        str(args.frontier_room_min_x),
        "--room-max-x",
        str(args.frontier_room_max_x),
        "--room-min-y",
        str(args.frontier_room_min_y),
        "--room-max-y",
        str(args.frontier_room_max_y),
        "--max-trajectory-abs-y-m",
        str(args.frontier_max_trajectory_abs_y_m),
        "--static-roi-preset",
        str(args.frontier_static_roi_preset),
        "--goal-republish-sec",
        str(args.frontier_goal_republish_sec),
    ]
    if args.check_explored_map_pct or args.frontier_build_tomogram:
        frontier_smoke_cmd.extend(
            [
                "--pcd-out",
                str(Path(args.frontier_pcd_out)),
                "--tomogram-out",
                str(Path(args.frontier_tomogram_out)),
                "--build-tomogram",
            ]
        )
    if args.frontier_trace_out:
        frontier_smoke_cmd.extend(["--trace-out", str(Path(args.frontier_trace_out))])
    if args.frontier_continue_after_pass_sec > 0:
        frontier_smoke_cmd.extend(
            ["--continue-after-pass-sec", str(args.frontier_continue_after_pass_sec)]
        )
    frontier_smoke_cmd.append("--require-room-forward-exploration")
    frontier_smoke_cmd.append("--require-trajectory-quality")
    frontier_smoke_cmd.append("--require-terrain-map-topics")
    frontier_gate_requested = bool(
        args.check_frontier_exploration
        or args.check_cumulative_map
        or args.check_explored_map_pct
    )
    if frontier_gate_requested:
        frontier_smoke_cmd.append("--require-cumulative-map")

    base_env = os.environ.copy()
    base_env["PYTHONPATH"] = os.pathsep.join(
        [str(ROOT / "src"), str(ROOT), base_env.get("PYTHONPATH", "")]
    ).strip(os.pathsep)
    base_domain_id = int(
        args.ros_domain_id
        if args.ros_domain_id is not None
        else (30 + (os.getpid() % 30))
    )
    base_partition = str(
        args.gz_partition
        or base_env.get("GZ_PARTITION")
        or f"lingtu_gazebo_gate_{os.getpid()}_{int(time.time())}"
    )

    def phase_env(name: str, offset: int) -> dict[str, str]:
        env = base_env.copy()
        partition = f"{base_partition}_{name}" if name else base_partition
        env["ROS_DOMAIN_ID"] = str(_phase_domain_id(base_domain_id, offset))
        env["GZ_PARTITION"] = partition
        env["IGN_PARTITION"] = partition
        return env

    tf_env = phase_env("tf", 0)
    nav_env = phase_env("nav", 1) if args.isolate_nav_gate else tf_env
    frontier_env = (
        phase_env("frontier", 2)
        if args.isolate_frontier_gate and args.check_nav_loop
        else nav_env
    )

    with launch_log.open("wb") as log:
        def launch_gazebo(env: dict[str, str]) -> subprocess.Popen[Any]:
            return subprocess.Popen(
                launch_cmd,
                cwd=str(ROOT),
                env=env,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=(os.name == "posix"),
            )

        def launch_navigation(
            env: dict[str, str],
            *,
            require_grid: bool = False,
        ) -> subprocess.Popen[Any]:
            return subprocess.Popen(
                frontier_nav_launch_cmd if require_grid else nav_launch_cmd,
                cwd=str(ROOT),
                env=env,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=(os.name == "posix"),
            )

        active_env = tf_env
        proc = launch_gazebo(active_env)
        nav_proc: subprocess.Popen[Any] | None = None
        try:
            time.sleep(args.warmup_sec)
            smoke = subprocess.run(
                smoke_cmd,
                cwd=str(ROOT),
                env=active_env,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=args.smoke_timeout_sec + 10.0,
            )
            report = _load_report(smoke_tmp)
            errors = list(report.get("errors") or [])
            if proc.poll() is not None:
                errors.append(f"gazebo launch exited early with code {proc.returncode}")
            if smoke.returncode != 0 and report.get("ok") is not True:
                errors.append(f"tf_contract_smoke exited with code {smoke.returncode}")
            report["ok"] = bool(report.get("ok")) and not errors
            report["errors"] = errors
            report["gate"] = {
                "name": "gazebo_runtime",
                "profile": "product_frontier_mapping"
                if frontier_gate_requested
                else "tf_topic_smoke",
                "required_checks": [
                    name
                    for name, enabled in (
                        ("tf_contract", True),
                        ("nav_loop", args.check_nav_loop),
                        (
                            "frontier_exploration_from_gazebo_lidar_occupancy",
                            frontier_gate_requested,
                        ),
                        ("trajectory_quality", frontier_gate_requested),
                        ("cmu_style_terrain_topics", frontier_gate_requested),
                        ("cumulative_map", frontier_gate_requested),
                        ("explored_map_pct", args.check_explored_map_pct),
                        ("tare_contract", args.check_tare_contract),
                    )
                    if enabled
                ],
                "launch_cmd": launch_cmd,
                "world": _resolve_world_arg(args.world),
                "smoke_cmd": smoke_cmd,
                "launch_log": str(launch_log),
                "smoke_returncode": smoke.returncode,
                "launch_returncode_before_cleanup": proc.poll(),
                "simulation_only": True,
                "real_robot_motion": False,
                "cmd_vel_sent_to_hardware": False,
                "frontier_isolated_launch": False,
                "nav_isolated_launch": False,
                "ros_domain_id": active_env["ROS_DOMAIN_ID"],
                "gz_partition": active_env["GZ_PARTITION"],
                "phase_envs": {
                    "tf": {
                        "ros_domain_id": tf_env["ROS_DOMAIN_ID"],
                        "gz_partition": tf_env["GZ_PARTITION"],
                        "ign_partition": tf_env["IGN_PARTITION"],
                    },
                    "nav": {
                        "ros_domain_id": nav_env["ROS_DOMAIN_ID"],
                        "gz_partition": nav_env["GZ_PARTITION"],
                        "ign_partition": nav_env["IGN_PARTITION"],
                    },
                    "frontier": {
                        "ros_domain_id": frontier_env["ROS_DOMAIN_ID"],
                        "gz_partition": frontier_env["GZ_PARTITION"],
                        "ign_partition": frontier_env["IGN_PARTITION"],
                    },
                },
            }
            if smoke.stdout:
                report["gate"]["smoke_stdout_tail"] = smoke.stdout[-2000:]
            if smoke.stderr:
                report["gate"]["smoke_stderr_tail"] = smoke.stderr[-2000:]
            if (
                args.check_nav_loop
                or args.check_frontier_exploration
                or args.check_cumulative_map
                or args.check_explored_map_pct
            ) and report["ok"]:
                if args.isolate_nav_gate:
                    report["gate"]["launch_returncode_before_nav_relaunch"] = proc.poll()
                    _terminate_process_tree(proc)
                    time.sleep(1.0)
                    active_env = nav_env
                    proc = launch_gazebo(active_env)
                    report["gate"]["nav_isolated_launch"] = True
                    time.sleep(args.nav_gazebo_warmup_sec)
                frontier_requested = frontier_gate_requested
                nav_proc = launch_navigation(
                    active_env,
                    require_grid=frontier_requested and not args.check_nav_loop,
                )
                time.sleep(args.nav_warmup_sec)
                if args.check_nav_loop and report["ok"]:
                    nav_smoke = subprocess.run(
                        nav_smoke_cmd,
                        cwd=str(ROOT),
                        env=active_env,
                        text=True,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        timeout=args.nav_timeout_sec + 10.0,
                    )
                    nav_report = _load_report(nav_tmp, "lingtu.gazebo_nav_loop.v1")
                    nav_errors = list(nav_report.get("errors") or [])
                    if nav_proc.poll() is not None:
                        nav_errors.append(
                            f"navigation launch exited early with code {nav_proc.returncode}"
                        )
                    if nav_smoke.returncode != 0 and nav_report.get("ok") is not True:
                        nav_errors.append(
                            f"gazebo_nav_loop_smoke exited with code {nav_smoke.returncode}"
                        )
                    nav_report["ok"] = bool(nav_report.get("ok")) and not nav_errors
                    nav_report["errors"] = nav_errors
                    if nav_smoke.stdout:
                        nav_report["stdout_tail"] = nav_smoke.stdout[-2000:]
                    if nav_smoke.stderr:
                        nav_report["stderr_tail"] = nav_smoke.stderr[-2000:]
                    report["nav_loop"] = nav_report
                    report["ok"] = bool(report["ok"]) and bool(nav_report.get("ok"))
                    report["errors"] = list(report.get("errors") or []) + [
                        f"nav_loop: {err}" for err in nav_errors
                    ]
                    report["gate"]["nav_smoke_cmd"] = nav_smoke_cmd
                    report["gate"]["nav_smoke_returncode"] = nav_smoke.returncode
                    report["gate"]["nav_hold_goal"] = [
                        args.nav_hold_goal_x,
                        args.nav_hold_goal_y,
                        args.nav_hold_goal_z,
                    ]
                if frontier_gate_requested and report["ok"]:
                    if args.isolate_frontier_gate and args.check_nav_loop:
                        if nav_proc is not None:
                            _terminate_process_tree(nav_proc)
                            report["gate"]["nav_launch_returncode_before_frontier_relaunch"] = nav_proc.poll()
                            nav_proc = None
                        report["gate"]["launch_returncode_before_frontier_relaunch"] = proc.poll()
                        _terminate_process_tree(proc)
                        time.sleep(1.0)
                        active_env = frontier_env
                        proc = launch_gazebo(active_env)
                        report["gate"]["frontier_isolated_launch"] = True
                        time.sleep(args.frontier_gazebo_warmup_sec)
                        nav_proc = launch_navigation(active_env, require_grid=True)
                        time.sleep(args.nav_warmup_sec)
                    elif args.check_nav_loop:
                        if nav_proc is not None:
                            _terminate_process_tree(nav_proc)
                            report["gate"]["nav_launch_returncode_before_frontier_grid_relaunch"] = nav_proc.poll()
                        nav_proc = launch_navigation(active_env, require_grid=True)
                        report["gate"]["frontier_grid_nav_relaunch"] = True
                        time.sleep(args.nav_warmup_sec)
                    elif nav_proc is None:
                        nav_proc = launch_navigation(active_env, require_grid=True)
                        time.sleep(args.nav_warmup_sec)
                    frontier_smoke = subprocess.run(
                        frontier_smoke_cmd,
                        cwd=str(ROOT),
                        env=active_env,
                        text=True,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        timeout=(
                            args.frontier_timeout_sec
                            + max(10.0, args.frontier_continue_after_pass_sec)
                            + 60.0
                        ),
                    )
                    frontier_report = _load_report(
                        frontier_tmp,
                        "lingtu.gazebo_frontier_exploration.v1",
                    )
                    frontier_errors = list(frontier_report.get("errors") or [])
                    if nav_proc.poll() is not None:
                        frontier_errors.append(
                            f"navigation launch exited early with code {nav_proc.returncode}"
                        )
                    if frontier_smoke.returncode != 0 and frontier_report.get("ok") is not True:
                        frontier_errors.append(
                            "gazebo_frontier_exploration_smoke exited "
                            f"with code {frontier_smoke.returncode}"
                        )
                    frontier_report["ok"] = bool(frontier_report.get("ok")) and not frontier_errors
                    frontier_report["errors"] = frontier_errors
                    if frontier_smoke.stdout:
                        frontier_report["stdout_tail"] = frontier_smoke.stdout[-2000:]
                    if frontier_smoke.stderr:
                        frontier_report["stderr_tail"] = frontier_smoke.stderr[-2000:]
                    report["frontier_exploration"] = frontier_report
                    report["ok"] = bool(report["ok"]) and bool(frontier_report.get("ok"))
                    report["errors"] = list(report.get("errors") or []) + [
                        f"frontier_exploration: {err}" for err in frontier_errors
                    ]
                    report["gate"]["frontier_smoke_cmd"] = frontier_smoke_cmd
                    report["gate"]["frontier_smoke_returncode"] = frontier_smoke.returncode
                    if args.check_explored_map_pct and report["ok"]:
                        pct_preview_cmd = [
                            sys.executable,
                            str(ROOT / "scripts" / "planning" / "plan_preview.py"),
                            "--tomogram",
                            str(Path(args.frontier_tomogram_out)),
                            "--planner",
                            "pct",
                            "--internal-only",
                            "--strict",
                            "--compact",
                            "--timeout",
                            str(args.pct_preview_timeout_sec),
                        ]
                        pct_preview = subprocess.run(
                            pct_preview_cmd,
                            cwd=str(ROOT),
                            env=active_env,
                            text=True,
                            stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE,
                            timeout=args.pct_preview_timeout_sec + 10.0,
                        )
                        pct_report = _json_from_stdout(pct_preview.stdout)
                        pct_errors = list(pct_report.get("errors") or [])
                        if pct_report.get("error"):
                            pct_errors.append(str(pct_report["error"]))
                        for case in pct_report.get("cases") or []:
                            if not isinstance(case, dict):
                                continue
                            preview = case.get("preview") or {}
                            path_safety = preview.get("path_safety") or {}
                            if isinstance(path_safety, dict) and path_safety.get("ok") is not True:
                                name = case.get("name") or "pct_case"
                                blocked = path_safety.get("blocked_sample_count")
                                pct_errors.append(
                                    f"{name}:path_safety_failed:{blocked}"
                                    if blocked is not None
                                    else f"{name}:path_safety_failed"
                                )
                        if pct_preview.returncode != 0 and pct_report.get("ok") is not True:
                            pct_errors.append(
                                f"plan_preview pct exited with code {pct_preview.returncode}"
                            )
                        pct_ok = bool(pct_report.get("ok")) and not pct_errors
                        pct_report["ok"] = pct_ok
                        pct_report["errors"] = pct_errors
                        pct_report["simulation_only"] = True
                        pct_report["real_robot_motion"] = False
                        pct_report["cmd_vel_sent_to_hardware"] = False
                        if pct_preview.stderr:
                            pct_report["stderr_tail"] = pct_preview.stderr[-2000:]
                        report["explored_map_pct"] = pct_report
                        report["ok"] = bool(report["ok"]) and pct_ok
                        report["errors"] = list(report.get("errors") or []) + [
                            f"explored_map_pct: {err}" for err in pct_errors
                        ]
                        report["gate"]["pct_preview_cmd"] = pct_preview_cmd
                        report["gate"]["pct_preview_returncode"] = pct_preview.returncode
                report["gate"]["nav_launch_cmd"] = nav_launch_cmd
                report["gate"]["frontier_nav_launch_cmd"] = frontier_nav_launch_cmd
            if args.check_tare_contract and report["ok"]:
                tare_report = _tare_contract_report(require_runtime=args.require_tare_runtime)
                report["tare_exploration"] = tare_report
                report["ok"] = bool(report["ok"]) and bool(tare_report.get("ok"))
                report["errors"] = list(report.get("errors") or []) + [
                    f"tare_exploration: {err}"
                    for err in list(tare_report.get("errors") or [])
                ]
            if "gate" in report:
                report["gate"]["launch_returncode_before_cleanup"] = proc.poll()
                if nav_proc is not None:
                    report["gate"]["nav_launch_returncode_before_cleanup"] = nav_proc.poll()
            with out_path.open("w", encoding="utf-8") as f:
                json.dump(report, f, indent=2, ensure_ascii=False)
            return report
        finally:
            if nav_proc is not None:
                _terminate_process_tree(nav_proc)
            _terminate_process_tree(proc)
            if smoke_tmp.exists():
                try:
                    smoke_tmp.unlink()
                except OSError:
                    pass
            if nav_tmp.exists():
                try:
                    nav_tmp.unlink()
                except OSError:
                    pass
            if frontier_tmp.exists():
                try:
                    frontier_tmp.unlink()
                except OSError:
                    pass


def _tare_contract_report(*, require_runtime: bool) -> dict[str, Any]:
    source_root = ROOT / "src" / "exploration"
    tare_root = source_root / "tare_planner"
    installed_binary = ROOT / "install" / "tare_planner" / "lib" / "tare_planner" / "tare_planner_node"
    binary = shutil.which("tare_planner_node") or (
        str(installed_binary) if installed_binary.exists() else ""
    )
    checks = {
        "source_package": (tare_root / "package.xml").exists(),
        "launch_files": (tare_root / "launch" / "explore_forest.launch").exists(),
        "bridge_module": (source_root / "tare_explorer_module.py").exists(),
        "native_factory": (source_root / "native_factories.py").exists(),
        "supervisor_module": (source_root / "exploration_supervisor_module.py").exists(),
    }
    runtime_available = bool(binary)
    errors = [f"{name} missing" for name, ok in checks.items() if not ok]
    if require_runtime and not runtime_available:
        errors.append("tare_planner_node runtime binary missing")
    return {
        "schema_version": "lingtu.gazebo_tare_exploration_contract.v1",
        "ok": not errors,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "backend": "tare",
        "source_contract_ok": all(checks.values()),
        "runtime_required": bool(require_runtime),
        "runtime_available": runtime_available,
        "binary": binary,
        "checks": checks,
        "gazebo_runtime_verified": False,
        "errors": errors,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--json-out",
        default="artifacts/server_sim_closure/gazebo_runtime/report.json",
    )
    parser.add_argument(
        "--launch-log",
        default="artifacts/server_sim_closure/gazebo_runtime/launch.log",
    )
    parser.add_argument("--warmup-sec", type=float, default=55.0)
    parser.add_argument("--smoke-timeout-sec", type=float, default=45.0)
    parser.add_argument("--min-samples", type=int, default=3)
    parser.add_argument("--require-camera", action="store_true")
    parser.add_argument("--check-nav-loop", action="store_true")
    parser.add_argument("--check-frontier-exploration", action="store_true")
    parser.add_argument("--check-cumulative-map", action="store_true")
    parser.add_argument("--check-explored-map-pct", action="store_true")
    parser.add_argument("--check-tare-contract", action="store_true")
    parser.add_argument("--require-tare-runtime", action="store_true")
    parser.add_argument(
        "--world",
        default=str(DEFAULT_GAZEBO_WORLD),
        help=(
            "Gazebo/GZ world path or resource name passed through to "
            "launch/gazebo_simulation.launch.py."
        ),
    )
    parser.add_argument("--nav-warmup-sec", type=float, default=8.0)
    parser.add_argument("--nav-timeout-sec", type=float, default=30.0)
    parser.add_argument("--nav-goal-delay-sec", type=float, default=0.0)
    parser.add_argument("--nav-goal-republish-sec", type=float, default=0.5)
    parser.add_argument("--nav-goal-x", type=float, default=2.0)
    parser.add_argument("--nav-goal-y", type=float, default=0.0)
    parser.add_argument("--nav-goal-z", type=float, default=0.0)
    parser.add_argument("--nav-hold-goal-x", type=float, default=0.0)
    parser.add_argument("--nav-hold-goal-y", type=float, default=0.0)
    parser.add_argument("--nav-hold-goal-z", type=float, default=0.0)
    parser.add_argument("--spawn-x", type=float, default=0.0)
    parser.add_argument("--spawn-y", type=float, default=0.0)
    parser.add_argument("--spawn-z", type=float, default=0.0)
    parser.add_argument("--spawn-yaw", type=float, default=0.0)
    parser.add_argument(
        "--require-forward-progress",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--frontier-timeout-sec", type=float, default=45.0)
    parser.add_argument("--frontier-goal-republish-sec", type=float, default=0.5)
    parser.add_argument("--frontier-trace-out", default="")
    parser.add_argument("--frontier-continue-after-pass-sec", type=float, default=0.0)
    parser.add_argument("--frontier-coverage-size-m", type=float, default=12.0)
    parser.add_argument("--frontier-room-min-x", type=float, default=-0.95)
    parser.add_argument("--frontier-room-max-x", type=float, default=6.65)
    parser.add_argument("--frontier-room-min-y", type=float, default=-2.15)
    parser.add_argument("--frontier-room-max-y", type=float, default=2.15)
    parser.add_argument("--frontier-max-trajectory-abs-y-m", type=float, default=2.05)
    parser.add_argument(
        "--frontier-static-roi-preset",
        choices=("demo_room", "industrial_park", "none"),
        default="demo_room",
    )
    parser.add_argument(
        "--frontier-pcd-out",
        default="artifacts/server_sim_closure/gazebo_runtime/explored_map.pcd",
    )
    parser.add_argument(
        "--frontier-tomogram-out",
        default="artifacts/server_sim_closure/gazebo_runtime/tomogram.pickle",
    )
    parser.add_argument("--frontier-build-tomogram", action="store_true")
    parser.add_argument("--pct-preview-timeout-sec", type=float, default=8.0)
    parser.add_argument(
        "--ros-domain-id",
        type=int,
        default=None,
        help="ROS_DOMAIN_ID for the isolated gate run. Defaults to a process-local value.",
    )
    parser.add_argument(
        "--gz-partition",
        default="",
        help="GZ_PARTITION for the isolated gate run. Defaults to a unique process-local value.",
    )
    parser.add_argument(
        "--isolate-nav-gate",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--nav-gazebo-warmup-sec", type=float, default=8.0)
    parser.add_argument(
        "--isolate-frontier-gate",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--frontier-gazebo-warmup-sec", type=float, default=8.0)
    parser.add_argument("--headless", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--use-bridge", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--spawn-robot", action=argparse.BooleanOptionalAction, default=True)
    args = parser.parse_args()

    report = run_gate(args)
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if report.get("ok") is True else 1


if __name__ == "__main__":
    raise SystemExit(main())
