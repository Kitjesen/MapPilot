import importlib.util
import json
import os
import shlex
import shutil
import subprocess
import textwrap
import time
from contextlib import contextmanager
from pathlib import Path

import pytest

from runtime.runtime_interface import TOPICS, adapter_remappings

REPO_ROOT = Path(__file__).resolve().parents[3]


def _read(path: str) -> str:
    return (REPO_ROOT / path).read_text(encoding="utf-8")


def _load_soak_module():
    spec = importlib.util.spec_from_file_location(
        "lingtu_soak_under_test",
        REPO_ROOT / "scripts" / "diagnostics" / "soak.py",
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _healthy_soak_sample() -> dict:
    return {
        "endpoint_errors": [],
        "client_contract_violations": [],
        "has_odometry": True,
        "localization_state": "ready",
        "pose_fresh": True,
        "odom_age_ms": 90.0,
        "cloud_age_ms": 120.0,
        "diag_age_ms": 300.0,
        "localizer_health_topic_age_ms": 250.0,
        "slam_hz": 10.0,
        "map_points": 5000.0,
        "confidence": 0.95,
        "active_cmd_source": "none",
        "navigation_state": "IDLE",
        "navigation_blockers": ["navigation_session_inactive"],
        "non_motion_safe": True,
    }


def _soak_limits() -> dict:
    return {
        "min_slam_hz": 1.0,
        "min_map_points": 1.0,
        "max_odom_age_ms": 1500.0,
        "max_cloud_age_ms": 5000.0,
        "max_diag_age_ms": 3000.0,
        "max_localizer_health_age_ms": 3000.0,
        "min_localization_confidence": 0.5,
    }


def _wsl_path(path: Path) -> str:
    if os.name != "nt":
        return str(path)
    result = subprocess.run(
        ["bash", "-lc", f"wslpath -a {shlex.quote(str(path))}"],
        check=True,
        cwd=REPO_ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        encoding="utf-8",
        errors="ignore",
    )
    return result.stdout.strip()


def _write_executable(path: Path, text: str) -> None:
    path.write_text(textwrap.dedent(text).lstrip(), encoding="utf-8", newline="\n")
    path.chmod(0o755)


@contextmanager
def _gateway_fixture(tmp_path: Path, *, mutate_plan_state: bool = False):
    script = tmp_path / "gateway_fixture.py"
    log_path = tmp_path / "gateway_requests.jsonl"
    script.write_text(
        textwrap.dedent(
            r"""
            import json
            import sys
            from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

            log_path = sys.argv[1]
            mutate_plan_state = sys.argv[2] == "1"
            mission_state = "IDLE"


            def write_log(method, path, body=None):
                with open(log_path, "a", encoding="utf-8") as fh:
                    fh.write(json.dumps([method, path, body], separators=(",", ":")) + "\n")


            class Handler(BaseHTTPRequestHandler):
                def log_message(self, *_args):
                    return

                def send_json(self, payload, status=200):
                    body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
                    self.send_response(status)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)

                def do_GET(self):
                    write_log("GET", self.path)
                    if self.path == "/ready":
                        self.send_json({
                            "ready": True,
                            "data_ready": True,
                            "motion_ready": False,
                            "non_motion_safe": True,
                            "reasons": ["navigation_blocked:navigation_session_inactive"],
                            "failed_modules": [],
                            "runtime": {"summary": {"data_blockers": []}},
                        })
                    elif self.path == "/api/v1/readiness":
                        self.send_json({
                            "schema_version": 1,
                            "status": "degraded",
                            "ts": 123.0,
                            "ready": False,
                            "data_ready": True,
                            "motion_ready": False,
                            "non_motion_safe": True,
                            "reasons": ["navigation_blocked:navigation_session_inactive"],
                            "failed_modules": [],
                            "modules": {},
                        })
                    elif self.path == "/api/v1/health":
                        self.send_json({
                            "status": "ok",
                            "modules_ok": 8,
                            "modules_fail": 0,
                            "sensors": {"slam": {"status": "ok", "hz": 10.0}},
                            "map_points": 5000,
                            "has_odom": True,
                        })
                    elif self.path == "/api/v1/localization/status":
                        self.send_json({
                            "state": "TRACKING",
                            "ready": True,
                            "pose_fresh": True,
                            "has_odometry": True,
                            "odom_age_ms": 80.0,
                            "cloud_age_ms": 90.0,
                            "diag_age_ms": 80.0,
                            "localizer_health_topic_age_ms": 80.0,
                            "map_cloud_fresh": True,
                            "confidence": 0.96,
                            "backend": "localizer",
                        })
                    elif self.path == "/api/v1/navigation/status":
                        self.send_json({
                            "schema_version": 1,
                            "state": mission_state,
                            "can_accept_goal": True,
                            "readiness": {
                                "can_accept_goal": True,
                                "can_execute_autonomy": True,
                                "blockers": [],
                                "advisories": [],
                            },
                            "control": {"active_cmd_source": "none"},
                        })
                    elif self.path == "/api/v1/state":
                        self.send_json({
                            "schema_version": 1,
                            "odometry": {
                                "x": 1.0,
                                "y": 2.0,
                                "z": 0.0,
                                "yaw": 0.3,
                                "frame_id": "odom",
                            },
                            "localization": {
                                "state": "TRACKING",
                                "ready": True,
                                "map_odom_tf": {
                                    "valid": True,
                                    "frame_id": "map",
                                    "child_frame_id": "odom",
                                    "tx": 10.0,
                                    "ty": -5.0,
                                    "tz": 1.0,
                                    "qx": 0.0,
                                    "qy": 0.0,
                                    "qz": 0.0,
                                    "qw": 1.0,
                                },
                            },
                            "navigation": {"control": {"active_cmd_source": "none"}},
                        })
                    else:
                        self.send_json({}, status=404)

                def do_POST(self):
                    global mission_state
                    length = int(self.headers.get("Content-Length") or "0")
                    body = self.rfile.read(length).decode("utf-8", "replace")
                    write_log("POST", self.path, body)
                    if self.path == "/api/v1/navigation/plan":
                        if mutate_plan_state:
                            mission_state = "RUNNING"
                        self.send_json({
                            "schema_version": 1,
                            "ok": True,
                            "feasible": True,
                            "count": 3,
                            "planner": "pct",
                            "reasons": [],
                        })
                    else:
                        self.send_json({}, status=404)


            server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
            print(server.server_address[1], flush=True)
            server.serve_forever()
            """
        ).lstrip(),
        encoding="utf-8",
    )
    cmd = (
        f"python3 -u {shlex.quote(_wsl_path(script))} "
        f"{shlex.quote(_wsl_path(log_path))} {'1' if mutate_plan_state else '0'}"
    )
    proc = subprocess.Popen(
        ["bash", "-lc", cmd],
        cwd=REPO_ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        encoding="utf-8",
        errors="ignore",
    )
    try:
        assert proc.stdout is not None
        port = proc.stdout.readline().strip()
        if not port:
            stderr = proc.stderr.read() if proc.stderr else ""
            raise AssertionError(f"gateway fixture failed to start: {stderr}")
        time.sleep(0.1)
        yield f"http://127.0.0.1:{port}", {"requests_log": log_path}
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()


def _make_slamcheck_harness(tmp_path: Path) -> dict[str, Path | str]:
    fake_root = tmp_path / "fake"
    fake_bin = fake_root / "bin"
    fake_run = fake_root / "run"
    fake_home = fake_root / "home"
    fake_bin.mkdir(parents=True)
    fake_run.mkdir(parents=True)
    maps = fake_home / "data" / "nova" / "maps"
    (maps / "demo").mkdir(parents=True)
    (maps / "demo" / "map.pcd").write_text("pcd\n", encoding="utf-8")
    outside = fake_home / "data" / "nova" / "outside"
    outside.mkdir(parents=True)
    (outside / "map.pcd").write_text("outside\n", encoding="utf-8")

    _write_executable(
        fake_bin / "systemctl",
        r"""
        #!/usr/bin/env bash
        set -euo pipefail
        echo "systemctl:$*" >> "${FAKE_ROOT}/calls.log"
        cmd="${1:-}"
        shift || true
        case "$cmd" in
            is-active)
                quiet=0
                while [ "$#" -gt 0 ] && [[ "${1:-}" == --* ]]; do
                    [ "${1:-}" = "--quiet" ] && quiet=1
                    shift
                done
                unit="${1:-}"
                active=inactive
                case "$unit" in
                    lingtu-livox-dds.service|lingtu-slam-dds.service|lingtu-nav-dds.service|lingtu.service|robot-camera.service|robot-brainstem.service)
                        active=active ;;
                esac
                case " ${FAKE_LEGACY_ACTIVE:-} " in
                    *" $unit "*) active=active ;;
                esac
                if [ "$quiet" = "1" ]; then
                    [ "$active" = "active" ]
                    exit $?
                fi
                echo "$active"
                [ "$active" = "active" ] && exit 0 || exit 3
                ;;
            is-enabled)
                echo enabled
                exit 0
                ;;
            show)
            prop=""
            while [ "$#" -gt 0 ]; do
                if [ "${1:-}" = "-p" ]; then
                    prop="${2:-}"
                    shift 2
                    continue
                fi
                shift
            done
            case "$prop" in
                ActiveState) echo active ;;
                NRestarts) echo "${FAKE_NRESTARTS:-0}" ;;
                MainPID) echo 0 ;;
                *) echo "" ;;
            esac
                exit 0
                ;;
            start|stop|restart|reset-failed)
                exit 0
                ;;
        esac
        exit 0
        """,
    )
    _write_executable(
        fake_bin / "sudo",
        r"""
        #!/usr/bin/env bash
        set -euo pipefail
        echo "sudo:$*" >> "${FAKE_ROOT}/calls.log"
        cmd="${1:-}"
        shift || true
        case "$cmd" in
            mkdir)
                args=()
                for arg in "$@"; do
                    [ "$arg" = "/run/lingtu" ] && arg="${FAKE_RUN}/lingtu"
                    args+=("$arg")
                done
                exec mkdir "${args[@]}"
                ;;
            tee)
                path="${1:-}"
                shift || true
                case "$path" in
                    /run/lingtu/*) path="${FAKE_RUN}/lingtu/${path#/run/lingtu/}" ;;
                esac
                mkdir -p "$(dirname "$path")"
                exec tee "$path" "$@"
                ;;
            systemctl)
                exec systemctl "$@"
                ;;
            *)
                exec "$cmd" "$@"
                ;;
        esac
        """,
    )
    _write_executable(
        fake_bin / "curl",
        r"""
        #!/usr/bin/env bash
        set -euo pipefail
        url=""
        data=""
        while [ "$#" -gt 0 ]; do
            case "$1" in
                -d|--data|--data-raw|--data-binary)
                    data="${2:-}"
                    shift 2
                    ;;
                http://*|https://*)
                    url="$1"
                    shift
                    ;;
                *)
                    shift
                    ;;
            esac
        done
        state_file="${FAKE_ROOT}/current_profile"
        current="$(cat "$state_file" 2>/dev/null || echo localizer)"
        case "$url" in
            */api/v1/slam/switch)
                profile="$(printf '%s' "$data" | sed -n 's/.*"profile"[[:space:]]*:[[:space:]]*"\([^"]*\)".*/\1/p')"
                [ -n "$profile" ] || profile=unknown
                echo "$profile" > "$state_file"
                echo "switch:$profile" >> "${FAKE_ROOT}/calls.log"
                printf '{"schema_version":1,"ok":true,"success":true,"profile":"%s"}\n' "$profile"
                ;;
            */api/v1/slam/status)
                lidar=running
                slam=stopped
                localizer=stopped
                super_lio=stopped
                super_lio_relocation=stopped
                case "$current" in
                    localizer) slam=running; localizer=running ;;
                    super_lio) super_lio=running ;;
                    super_lio_relocation) super_lio_relocation=running ;;
                esac
                printf '{"mode":"%s","services":{"lidar":"%s","slam":"%s","localizer":"%s","super_lio":"%s","super_lio_relocation":"%s"}}\n' \
                    "$current" "$lidar" "$slam" "$localizer" "$super_lio" "$super_lio_relocation"
                ;;
            */api/v1/localization/status)
                signal="${FAKE_RECOVERY_SIGNAL:-RECOVERED}"
                action="${FAKE_RECOVERY_ACTION:-none}"
                if [ "$current" = "super_lio_relocation" ]; then
                    printf '{"state":"TRACKING","ready":true,"pose_fresh":true,"has_odometry":true,"odom_age_ms":100.0,"cloud_age_ms":120.0,"diag_age_ms":100.0,"localizer_health_topic_age_ms":100.0,"map_cloud_fresh":true,"confidence":0.91,"backend":"super_lio_relocation","health_source":"odom_map_cloud","map_save_source":"active_map","saved_map_relocalization_supported":false,"restart_recovery_supported":true,"recovery_method":"restart_super_lio_relocation","relocalization_state":"unsupported","recovery_signal":"%s","recovery_action":"%s"}\n' "$signal" "$action"
                elif [ "$current" = "super_lio" ]; then
                    printf '{"state":"TRACKING","ready":true,"pose_fresh":true,"has_odometry":true,"odom_age_ms":100.0,"cloud_age_ms":120.0,"diag_age_ms":100.0,"localizer_health_topic_age_ms":100.0,"map_cloud_fresh":true,"confidence":0.91,"backend":"super_lio","health_source":"odom_map_cloud","map_save_source":"live_map_cloud_snapshot","saved_map_relocalization_supported":false,"restart_recovery_supported":true,"recovery_method":"restart_super_lio","relocalization_state":"unsupported","recovery_signal":"%s","recovery_action":"%s"}\n' "$signal" "$action"
                else
                    printf '{"state":"TRACKING","ready":true,"pose_fresh":true,"has_odometry":true,"odom_age_ms":80.0,"cloud_age_ms":90.0,"diag_age_ms":80.0,"localizer_health_topic_age_ms":80.0,"map_cloud_fresh":true,"confidence":0.96,"backend":"localizer","health_source":"localizer_health_topic","map_save_source":"active_map","saved_map_relocalization_supported":true,"restart_recovery_supported":true,"recovery_method":"relocalize_service","relocalization_state":"idle","recovery_signal":"RECOVERED","recovery_action":"none"}\n'
                fi
                ;;
            */api/v1/state)
                signal="${FAKE_RECOVERY_SIGNAL:-RECOVERED}"
                action="${FAKE_RECOVERY_ACTION:-none}"
                if [ "$current" = "super_lio_relocation" ]; then
                    x="${FAKE_CANDIDATE_X:-1.0}"
                    y="${FAKE_CANDIDATE_Y:-2.0}"
                    yaw="${FAKE_CANDIDATE_YAW:-0.3}"
                    ready="${FAKE_CANDIDATE_READY:-true}"
                    if [ -n "${FAKE_CANDIDATE_READY_ON_STATE_CALL:-}" ]; then
                        counter="${FAKE_ROOT}/candidate_state_calls"
                        count="$(cat "$counter" 2>/dev/null || echo 0)"
                        count="$((count + 1))"
                        echo "$count" > "$counter"
                        if [ "$count" -lt "$FAKE_CANDIDATE_READY_ON_STATE_CALL" ]; then
                            ready=false
                        else
                            ready=true
                        fi
                    fi
                    printf '{"schema_version":1,"odometry":{"x":%s,"y":%s,"z":0.0,"yaw":%s},"localization":{"backend":"super_lio_relocation","health_source":"odom_map_cloud","state":"TRACKING","ready":%s,"confidence":0.91,"icp_quality":0.02,"odom_age_ms":100.0,"cloud_age_ms":120.0,"map_save_source":"active_map","recovery_signal":"%s","recovery_action":"%s"},"session":{"localization_backend":"super_lio_relocation","health_source":"odom_map_cloud","map_save_source":"active_map","icp_quality":0.02,"recovery_signal":"%s","recovery_action":"%s"},"navigation":{"control":{"active_cmd_source":"none"}},"map":{"live_points":5000,"live_cloud_frames":1}}\n' \
                        "$x" "$y" "$yaw" "$ready" "$signal" "$action" "$signal" "$action"
                else
                    x="${FAKE_LOCALIZER_X:-1.0}"
                    y="${FAKE_LOCALIZER_Y:-2.0}"
                    yaw="${FAKE_LOCALIZER_YAW:-0.3}"
                    ready="${FAKE_LOCALIZER_READY:-true}"
                    printf '{"schema_version":1,"odometry":{"x":%s,"y":%s,"z":0.0,"yaw":%s},"localization":{"backend":"localizer","health_source":"localizer_health_topic","state":"TRACKING","ready":%s,"confidence":0.96,"icp_quality":0.02,"odom_age_ms":80.0,"cloud_age_ms":90.0,"map_save_source":"active_map","recovery_signal":"RECOVERED","recovery_action":"none"},"session":{"localization_backend":"localizer","health_source":"localizer_health_topic","map_save_source":"active_map","icp_quality":0.02,"recovery_signal":"RECOVERED","recovery_action":"none"},"navigation":{"control":{"active_cmd_source":"none"}},"map":{"live_points":5000,"live_cloud_frames":1}}\n' \
                        "$x" "$y" "$yaw" "$ready"
                fi
                ;;
            */api/v1/navigation/status)
                blocker="${FAKE_NAV_BLOCKER:-}"
                mission_state="$(cat "${FAKE_ROOT}/mission_state" 2>/dev/null || echo IDLE)"
                if [ -n "$blocker" ]; then
                    printf '{"schema_version":1,"state":"IDLE","can_accept_goal":false,"readiness":{"can_accept_goal":false,"can_execute_autonomy":false,"blockers":["%s"],"advisories":[]},"control":{"active_cmd_source":"none"}}\n' "$blocker"
                else
                    printf '{"schema_version":1,"state":"%s","can_accept_goal":true,"readiness":{"can_accept_goal":true,"can_execute_autonomy":true,"blockers":[],"advisories":[]},"control":{"active_cmd_source":"none"},"failure_reason":""}\n' "$mission_state"
                fi
                ;;
            */api/v1/navigation/plan)
                echo "plan:$data" >> "${FAKE_ROOT}/calls.log"
                if [ "${FAKE_PLAN_FEASIBLE:-1}" = "0" ]; then
                    printf '{"schema_version":1,"ok":true,"feasible":false,"count":0,"planner":"pct","selected_planner":"pct","plan_safety_policy":"reject","path_safety":{"ok":false,"blocked_sample_count":2},"fallback_reason":"pct path_safety failed","rejected_plans":[{"planner":"pct","reason":"unsafe"}],"reasons":["blocked_by_costmap"]}\n'
                else
                    printf '{"schema_version":1,"ok":true,"feasible":true,"count":3,"planner":"pct","selected_planner":"pct","plan_safety_policy":"fallback_astar","path_safety":{"ok":true},"fallback_reason":"","rejected_plans":[],"reasons":[]}\n'
                fi
                ;;
            */api/v1/goal)
                echo "goal:$data" >> "${FAKE_ROOT}/calls.log"
                fail_profile="${FAKE_GOAL_FAIL_PROFILE:-}"
                if [ -n "$fail_profile" ] && [ "$fail_profile" = "$current" ]; then
                    echo FAILED > "${FAKE_ROOT}/mission_state"
                    printf '{"schema_version":1,"ok":false,"accepted":false,"error":"fake_goal_rejected"}\n'
                elif [ -n "${FAKE_GOAL_NONTERMINAL_PROFILE:-}" ] && [ "$FAKE_GOAL_NONTERMINAL_PROFILE" = "$current" ]; then
                    echo RUNNING > "${FAKE_ROOT}/mission_state"
                    printf '{"schema_version":1,"ok":true,"accepted":true,"command":{"accepted":true}}\n'
                else
                    echo SUCCESS > "${FAKE_ROOT}/mission_state"
                    printf '{"schema_version":1,"ok":true,"accepted":true,"command":{"accepted":true}}\n'
                fi
                ;;
            */api/v1/slam/relocalize)
                echo "relocalize:$data" >> "${FAKE_ROOT}/calls.log"
                printf '{"schema_version":1,"ok":true,"success":true,"message":"relocalized"}\n'
                ;;
            */api/v1/stop)
                echo "stop" >> "${FAKE_ROOT}/calls.log"
                echo CANCELLED > "${FAKE_ROOT}/mission_state"
                printf '{"schema_version":1,"ok":true,"accepted":true,"status":"stopped"}\n'
                ;;
            */api/v1/session/start)
                echo "session_start:$data" >> "${FAKE_ROOT}/calls.log"
                echo IDLE > "${FAKE_ROOT}/mission_state"
                fail_profile="${FAKE_SESSION_START_FAIL_PROFILE:-}"
                if [ -n "$fail_profile" ] && [ "$fail_profile" = "$current" ]; then
                    printf '{"schema_version":1,"ok":false,"success":false,"error":"fake_session_start_failed"}\n'
                else
                    printf '{"schema_version":1,"ok":true,"success":true,"session":{"mode":"navigating"}}\n'
                fi
                ;;
            */api/v1/session/end)
                echo "session_end" >> "${FAKE_ROOT}/calls.log"
                echo IDLE > "${FAKE_ROOT}/mission_state"
                printf '{"schema_version":1,"ok":true,"success":true,"session":{"mode":"idle"}}\n'
                ;;
            */api/v1/session)
                echo '{}'
                ;;
            */api/v1/map/activate)
                name="$(printf '%s' "$data" | sed -n 's/.*"name"[[:space:]]*:[[:space:]]*"\([^"]*\)".*/\1/p')"
                [ -n "$name" ] || name=demo
                echo "activate:$name" >> "${FAKE_ROOT}/calls.log"
                ln -sfn "$name" "$HOME/data/nova/maps/active"
                printf '{"schema_version":1,"ok":true,"success":true,"active":"%s"}\n' "$name"
                ;;
            */ready)
                echo '{"ready":true,"data_ready":true,"motion_ready":false,"non_motion_safe":true,"reasons":["navigation_blocked:navigation_session_inactive"],"failed_modules":[],"runtime":{"summary":{"data_blockers":[]}}}'
                ;;
            */api/v1/readiness)
                echo '{"schema_version":1,"status":"degraded","ts":123.0,"ready":false,"data_ready":true,"motion_ready":false,"non_motion_safe":true,"reasons":["navigation_blocked:navigation_session_inactive"],"failed_modules":[],"modules":{}}'
                ;;
            */api/v1/health)
                echo '{"status":"ok","modules_ok":8,"modules_fail":0,"sensors":{"slam":{"status":"ok","hz":10.0}},"map_points":5000,"has_odom":true}'
                ;;
            */api/v1/maps)
                echo "maps:$data" >> "${FAKE_ROOT}/calls.log"
                action="$(printf '%s' "$data" | sed -n 's/.*"action"[[:space:]]*:[[:space:]]*"\([^"]*\)".*/\1/p')"
                name="$(printf '%s' "$data" | sed -n 's/.*"name"[[:space:]]*:[[:space:]]*"\([^"]*\)".*/\1/p')"
                active="$(readlink "$HOME/data/nova/maps/active" 2>/dev/null || true)"
                [ -n "$action" ] || action=list
                [ -n "$name" ] || name=demo
                printf '{"schema_version":1,"ok":true,"success":true,"action":"%s","name":"%s","active":"%s","map_dir":"%s/data/nova/maps/%s","octomap":"octomap.ot","occupancy":"occupancy.npz","occupancy_ok":true}\n' \
                    "$action" "$name" "$active" "$HOME" "$name"
                ;;
            */api/v1/map/restore_predufo)
                echo "restore:$data" >> "${FAKE_ROOT}/calls.log"
                printf '{"schema_version":1,"ok":true,"success":true,"name":"demo","restored_size":123}\n'
                ;;
            */api/v1/map/save)
                echo "save:$data" >> "${FAKE_ROOT}/calls.log"
                echo '{"success":true,"map_save_source":"live_map_cloud_snapshot","saved_map_relocalization_supported":false}'
                ;;
            *)
                echo '{}'
                ;;
        esac
        """,
    )
    _write_executable(
        fake_bin / "ros2",
        r"""
        #!/usr/bin/env bash
        set -euo pipefail
        case "${1:-} ${2:-}" in
            "node list")
                printf '/livox_driver_node\n/lio_node\n/localizer_node\n/camera/camera\n/camera/camera_container\n'
                ;;
            "topic info")
                topic="${3:-}"
                case "$topic" in
                    /nav/lidar_scan|/nav/imu|/nav/odometry|/nav/localization_health|/nav/map_cloud|/nav/localization_quality|/camera/color/image_raw|/camera/depth/image_raw|/camera/color/camera_info|/camera/depth/camera_info|/camera/device_status)
                        pubs=1 ;;
                    *)
                        pubs=0 ;;
                esac
                printf 'Type: std_msgs/msg/String\nPublisher count: %s\nSubscription count: 1\n' "$pubs"
                ;;
            "topic echo")
                printf 'device_online: true\nconnection_type: usb\ncolor_frame_rate_cur: 30.0\ndepth_frame_rate_cur: 30.0\n'
                ;;
            *)
                ;;
        esac
        """,
    )
    _write_executable(
        fake_bin / "journalctl",
        r"""
        #!/usr/bin/env bash
        set -euo pipefail
        echo "May 06 12:00:00 sunrise livox_driver[1]: Init lds lidar success"
        """,
    )
    _write_executable(
        fake_bin / "ip",
        r"""
        #!/usr/bin/env bash
        set -euo pipefail
        if [ "${1:-}" = "-br" ]; then
            iface="${4:-eth0}"
            printf '%s             UP             192.168.1.5/24\n' "$iface"
        fi
        """,
    )

    fake_root_posix = _wsl_path(fake_root)
    fake_home_posix = _wsl_path(fake_home)
    fake_bin_posix = _wsl_path(fake_bin)
    fake_run_posix = _wsl_path(fake_run)
    subprocess.run(
        [
            "bash",
            "-lc",
            (f"ln -sfn demo {shlex.quote(fake_home_posix + '/data/nova/maps/active')}"),
        ],
        check=True,
        cwd=REPO_ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        encoding="utf-8",
        errors="ignore",
    )
    return {
        "root": fake_root,
        "run": fake_run,
        "home_posix": fake_home_posix,
        "bin_posix": fake_bin_posix,
        "root_posix": fake_root_posix,
        "run_posix": fake_run_posix,
    }


def _run_lingtu_command(
    tmp_path: Path,
    command_args: str,
    extra_env: dict[str, str] | None = None,
    active_target: str | None = None,
    seed_relocation_env: str | None = None,
):
    if shutil.which("bash") is None:
        pytest.skip("bash is required for scripts/lingtu shell behavior tests")
    harness = _make_slamcheck_harness(tmp_path)
    if seed_relocation_env is not None:
        env_file = harness["run"] / "lingtu" / "super_lio_relocation.env"
        env_file.parent.mkdir(parents=True, exist_ok=True)
        env_file.write_text(seed_relocation_env, encoding="utf-8")
    if active_target is not None:
        subprocess.run(
            [
                "bash",
                "-lc",
                (
                    f"ln -sfn {shlex.quote(active_target)} "
                    f"{shlex.quote(str(harness['home_posix']) + '/data/nova/maps/active')}"
                ),
            ],
            check=True,
            cwd=REPO_ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            encoding="utf-8",
            errors="ignore",
        )
    exports = {
        "HOME": harness["home_posix"],
        "FAKE_ROOT": harness["root_posix"],
        "FAKE_RUN": harness["run_posix"],
        "GW": "http://fake-gateway:5050",
        "LINGTU_ROS2_BIN": f"{harness['bin_posix']}/ros2",
        "LINGTU_SYSTEMCTL_BIN": f"{harness['bin_posix']}/systemctl",
        "LINGTU_JOURNALCTL_BIN": f"{harness['bin_posix']}/journalctl",
        "LINGTU_IP_BIN": f"{harness['bin_posix']}/ip",
    }
    if extra_env:
        exports.update(extra_env)
    export_cmd = "; ".join(f"export {name}={shlex.quote(str(value))}" for name, value in exports.items())
    command = (
        f"{export_cmd}; "
        f'export PATH={shlex.quote(str(harness["bin_posix"]))}:"$PATH"; '
        "hash -r; "
        f"bash scripts/lingtu {command_args}"
    )
    result = subprocess.run(
        ["bash", "-lc", command],
        cwd=REPO_ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        encoding="utf-8",
        errors="ignore",
    )
    return result, harness


def _run_slamcheck(
    tmp_path: Path,
    args: str,
    extra_env: dict[str, str] | None = None,
    active_target: str | None = None,
    seed_relocation_env: str | None = None,
):
    return _run_lingtu_command(
        tmp_path,
        f"slamcheck {args}",
        extra_env=extra_env,
        active_target=active_target,
        seed_relocation_env=seed_relocation_env,
    )


def test_robot_localizer_service_matches_slam_bridge_topics():
    text = _read("docs/04-deployment/services/robot-localizer.service")

    assert "MAP_PATH=$${NAV_MAP_PATH:-/home/sunrise/data/nova/maps/active/map}" in text
    for source, target in adapter_remappings("localizer").items():
        assert f"-r {source}:={target}" in text
    assert f"-r /localization_quality:={TOPICS.localization_quality}" in text
    assert f"-r global_relocalize:={TOPICS.global_relocalize_service}" in text
    assert f"-r global_relocalize_status:={TOPICS.global_relocalize_status_service}" in text
    assert "-r map_cloud:=/nav/map_cloud" not in text


def test_robot_fastlio2_service_matches_adapter_alias_contract():
    text = _read("docs/04-deployment/services/robot-fastlio2.service")

    for source, target in adapter_remappings("fastlio2").items():
        assert f"-r {source}:={target}" in text
    assert "-r /path:=/lio_path" in text


def test_s100p_lidar_service_normalizes_livox_topics_at_adapter_boundary():
    text = _read("scripts/deploy/s100p/lidar.service")

    assert f"-r livox/lidar:={TOPICS.lidar_scan}" in text
    assert f"-r livox/imu:={TOPICS.imu}" in text
    assert "-r /nav/lidar_scan:=livox/lidar" not in text
    assert "-r /nav/imu:=livox/imu" not in text


def test_s100p_slam_service_matches_fastlio2_adapter_contract():
    text = _read("scripts/deploy/s100p/slam.service")

    for source, target in adapter_remappings("fastlio2").items():
        assert f"-r {source}:={target}" in text
    assert "-r /path:=/lio_path" in text


def test_s100p_slam_pgo_service_uses_canonical_fastlio_topics():
    text = _read("scripts/deploy/s100p/slam_pgo.service")

    for source, target in adapter_remappings("pgo").items():
        assert f"-r {source}:={target}" in text


def test_s100p_localizer_template_matches_relocalize_api():
    text = _read("scripts/deploy/s100p/localizer.service")

    assert "MAP_PATH=$${NAV_MAP_PATH:-/home/sunrise/data/nova/maps/active/map}" in text
    assert "/home/sunrise/data/inovxio/data/maps/active/map" not in text
    for source, target in adapter_remappings("localizer").items():
        assert f"-r {source}:={target}" in text
    assert f"-r /localization_quality:={TOPICS.localization_quality}" in text
    assert f"-r global_relocalize:={TOPICS.global_relocalize_service}" in text
    assert f"-r global_relocalize_status:={TOPICS.global_relocalize_status_service}" in text
    assert "-r map_cloud:=/nav/map_cloud" not in text


@pytest.mark.parametrize(
    "service_path",
    (
        "scripts/deploy/s100p/super_lio.service",
        "scripts/deploy/s100p/super_lio_relocation.service",
    ),
)
def test_s100p_super_lio_services_publish_canonical_nav_topics(service_path):
    text = _read(service_path)

    assert f"Environment=SUPER_LIO_LIDAR_TOPIC={TOPICS.lidar_scan}" in text
    assert f"Environment=SUPER_LIO_IMU_TOPIC={TOPICS.imu}" in text
    assert '-p "lio.ros.lidar_topic:=$${SUPER_LIO_LIDAR_TOPIC}"' in text
    assert '-p "lio.ros.imu_topic:=$${SUPER_LIO_IMU_TOPIC}"' in text
    assert f'-r "/lio/odom:={TOPICS.odometry}"' in text
    assert f'-r "/lio/cloud_world:={TOPICS.map_cloud}"' in text
    assert '-r "/lio/imu/odom:=/nav/imu_odom"' in text
    assert '-r "/lio/robo/odom:=/nav/robot_odom"' in text


def test_s100p_genz_icp_service_publishes_canonical_slam_topics():
    text = _read("scripts/deploy/s100p/genz_icp.service")

    assert f"-r pointcloud_topic:={TOPICS.lidar_scan}" in text
    assert f"-r /genz/odometry:={TOPICS.odometry}" in text
    assert f"-r /genz/local_map:={TOPICS.map_cloud}" in text
    assert "-p odom_frame:=odom" in text
    assert "-p base_frame:=body" in text


def test_s100p_hba_service_exposes_offline_map_optimization_services():
    text = _read("scripts/deploy/s100p/hba.service")

    assert "exec ros2 run hba hba_node" in text
    assert "-r refine_map:=/slam/hba/refine_map" in text
    assert "-r save_poses:=/slam/hba/save_poses" in text
    assert "-r map_points:=/slam/hba/map_points" in text


def test_topic_contract_names_static_map_and_global_relocalize():
    text = _read("config/topic_contract.yaml")

    assert f"saved_map_cloud: {TOPICS.saved_map_cloud}" in text
    assert f"quality: {TOPICS.localization_quality}" in text
    assert f"health: {TOPICS.localization_health}" in text
    assert f"global_relocalize_service: {TOPICS.global_relocalize_service}" in text
    assert f"global_relocalize_status_service: {TOPICS.global_relocalize_status_service}" in text


def test_qos_profiles_use_canonical_localization_topics():
    text = _read("config/qos_profiles.yaml")

    assert f"- {TOPICS.localization_quality}" in text
    assert f"- {TOPICS.localization_health}" in text
    assert "- /localization_quality" not in text


def test_record_bag_captures_canonical_localization_topics():
    text = _read("scripts/hardware/record_bag.sh")
    topic_lines = {line.strip() for line in text.splitlines() if line.strip().startswith("/")}

    assert TOPICS.localization_quality in topic_lines
    assert TOPICS.localization_health in topic_lines
    assert "/localization_quality" not in topic_lines


def test_record_bag_captures_published_camera_intrinsics_topics():
    text = _read("scripts/hardware/record_bag.sh")
    topic_lines = {line.strip() for line in text.splitlines() if line.strip().startswith("/")}

    assert "/camera/color/camera_info" in topic_lines
    assert "/camera/depth/camera_info" in topic_lines
    assert "/camera/camera_info" not in topic_lines


def test_localizer_health_topic_has_steady_heartbeat():
    text = _read("src/localization/localizer/src/localizer_node.cpp")

    assert "m_health_heartbeat_interval" in text
    assert "std::chrono::milliseconds(500)" in text
    assert "heartbeat_due" in text
    assert "publishable && (state_changed || heartbeat_due)" in text
    assert "Localization health heartbeat" in text
    assert "RCLCPP_DEBUG" in text
    assert 'state_changed && desired == "LOST"' in text


def test_lingtu_doctor_checks_camera_intrinsics_topic_family():
    text = _read("scripts/lingtu")

    assert "camera.camera_info" in text
    assert '"/camera/color/camera_info"' in text
    assert '"/camera/depth/camera_info"' in text
    assert '"/camera/camera_info"' in text
    assert "camera intrinsics topic has publishers" in text


def test_lingtu_doctor_checks_camera_device_online_status():
    text = _read("scripts/lingtu")

    assert "camera.device_status" in text
    assert "--require-camera" in text
    assert 'require_camera = sys.argv[5] == "1"' in text
    assert "camera_check_status" in text
    assert "Mode: camera required for full App/Web visual readiness" in text
    assert '"/camera/device_status"' in text
    assert "parse_camera_device_status" in text
    assert "device_online" in text
    assert "connection_type" in text
    assert "color_frame_rate_cur" in text
    assert "depth_frame_rate_cur" in text
    assert '"required": require_camera' in text
    assert "camera driver is alive but /camera/device_status reports device_online=false" in text
    assert "check USB/power/cable before restarting software" in text


def test_lingtu_doctor_reports_standard_quality_topic_with_legacy_hint():
    text = _read("scripts/lingtu")

    assert "/slam/localization_quality" in text
    assert "/localization_quality" in text
    assert "legacy; remap to /slam/localization_quality" in text


def test_lingtu_doctor_json_gates_runtime_readiness_freshness():
    text = _read("scripts/lingtu")

    assert '"lingtu-camera-dds.service"' in text
    assert 'elif svc == "lingtu-camera-dds.service"' in text
    assert "livox.sdk_init" in text
    assert "latest_livox_sdk_event" in text
    assert "Init lds lidar success" in text
    assert "livox.netdev_carrier" in text
    assert "LINGTU_LIDAR_NETDEV" in text
    assert "LINGTU_LIVOX_NET_IFACE" in text
    assert 'systemd_environment("lingtu-livox-dds.service")' in text
    assert "/sys/class/net/{name}" in text
    assert "bind failed" in text
    assert "gateway.slam_stream" in text
    assert "gateway.client_readiness" in text
    assert "/api/v1/readiness exposes client-readable readiness" in text
    assert 'client_ready_code, client_ready, client_ready_err = http_json("/api/v1/readiness"' in text
    assert "gateway.localization_status" in text
    assert "add_dataflow_pressure_check" in text
    assert '"VoxelGridModule", "map_cloud", "p1"' in text
    assert "LINGTU_DOCTOR_MAX_DATAFLOW_P95_MS" in text
    assert "LINGTU_DOCTOR_MAX_DATAFLOW_DROP_RATIO" in text
    assert "callback pressure detected" in text
    assert "LINGTU_DOCTOR_MIN_SLAM_HZ" in text
    assert "LINGTU_DOCTOR_MAX_ODOM_AGE_MS" in text
    assert "LINGTU_DOCTOR_MAX_LOC_DIAG_AGE_MS" in text
    assert "LINGTU_DOCTOR_MAX_LOCALIZER_HEALTH_AGE_MS" in text
    assert "odom_age_ms" in text
    assert "diag_age_ms" in text
    assert "localizer_health_topic_age_ms" in text
    assert "map_cloud_fresh=false" in text
    assert "gateway.navigation_plan_preview" in text
    assert "/api/v1/navigation/plan" in text
    assert "LINGTU_DOCTOR_PLAN_PREVIEW_OFFSET_M" in text
    assert "navigation plan preview produced a path without taking control" in text
    assert "active_cmd_source_after" in text
    assert "state_after" in text
    assert "Navigation plan preview (non-motion)" in text
    assert "preview did not take control" in text


def test_lingtu_doctor_realtime_smoke_is_explicit_non_motion_gate():
    text = _read("scripts/lingtu")

    assert "Usage: lingtu doctor [--non-motion] [--json] [--strict] [--realtime] [--require-camera]" in text
    assert "--realtime) realtime=1" in text
    assert "gateway.realtime_sse" in text
    assert "gateway.websocket_camera" in text
    assert "gateway.websocket_cloud" in text
    assert 'ws_smoke("/ws/camera", b"\\xff\\xd8\\xff"' in text
    assert 'ws_smoke("/ws/cloud", b"PCL"' in text


def test_lingtu_doctor_normalizes_structured_active_command_source():
    text = _read("scripts/lingtu")

    assert "def command_source_name(control):" in text
    assert 'source.get("name") or source.get("source") or source.get("owner") or "none"' in text
    assert "isinstance(s,dict)" in text


def test_lingtu_doctor_non_motion_json_runs_without_motion_side_effects(tmp_path):
    with _gateway_fixture(tmp_path) as (gateway_url, gateway):
        result, harness = _run_lingtu_command(
            tmp_path,
            "doctor --non-motion --json --strict",
            extra_env={"GW": gateway_url, "LINGTU_LIDAR_NETDEV": "lo"},
        )

    assert result.returncode == 0, result.stdout + result.stderr
    payload = json.loads(result.stdout)
    assert payload["ok"] is True
    preview = next(check for check in payload["checks"] if check["id"] == "gateway.navigation_plan_preview")
    client_readiness = next(check for check in payload["checks"] if check["id"] == "gateway.client_readiness")
    assert client_readiness["status"] == "pass"
    assert client_readiness["evidence"]["status"] == "degraded"
    assert preview["status"] == "pass"
    assert preview["evidence"]["active_cmd_source_after"] == "none"
    assert preview["evidence"]["state_after"] == "IDLE"
    calls = (harness["root"] / "calls.log").read_text(encoding="utf-8")
    requests = [
        tuple(json.loads(line)[:2]) for line in gateway["requests_log"].read_text(encoding="utf-8").splitlines()
    ]
    assert ("POST", "/api/v1/navigation/plan") in requests
    assert ("POST", "/api/v1/goal") not in requests
    assert ("POST", "/api/v1/session/start") not in requests
    assert ("POST", "/api/v1/session/end") not in requests
    assert "sudo:" not in calls
    assert "goal:" not in calls
    assert "session_start:" not in calls
    assert "session_end" not in calls
    assert "cmd_vel" not in calls
    assert "systemctl:restart" not in calls
    assert "systemctl:stop" not in calls
    preview_request = next(
        json.loads(json.loads(line)[2])
        for line in gateway["requests_log"].read_text(encoding="utf-8").splitlines()
        if tuple(json.loads(line)[:2]) == ("POST", "/api/v1/navigation/plan")
    )
    assert preview_request == {"x": 10.8, "y": -3.0, "z": 1.0}


def test_lingtu_doctor_non_motion_preview_fails_if_plan_changes_state(tmp_path):
    with _gateway_fixture(tmp_path, mutate_plan_state=True) as (gateway_url, gateway):
        result, harness = _run_lingtu_command(
            tmp_path,
            "doctor --non-motion --json --strict",
            extra_env={"GW": gateway_url, "LINGTU_LIDAR_NETDEV": "lo"},
        )

    assert result.returncode != 0
    payload = json.loads(result.stdout)
    preview = next(check for check in payload["checks"] if check["id"] == "gateway.navigation_plan_preview")
    assert preview["status"] == "fail"
    assert "mission state" in preview["message"]
    assert preview["evidence"]["active_cmd_source_after"] == "none"
    assert preview["evidence"]["state_after"] == "RUNNING"
    calls = (harness["root"] / "calls.log").read_text(encoding="utf-8")
    requests = [
        tuple(json.loads(line)[:2]) for line in gateway["requests_log"].read_text(encoding="utf-8").splitlines()
    ]
    assert ("POST", "/api/v1/navigation/plan") in requests
    assert ("POST", "/api/v1/goal") not in requests
    assert ("POST", "/api/v1/session/start") not in requests
    assert "goal:" not in calls
    assert "session_start:" not in calls
    assert "cmd_vel" not in calls


def test_lingtu_soak_is_read_only_non_motion_field_verification():
    text = _read("scripts/lingtu")
    impl = _read("scripts/diagnostics/soak.py")
    start = text.index("cmd_soak()")
    end = text.index("# -- Subcommand: slamcheck --")
    soak = text[start:end]

    assert "Usage: lingtu soak [--duration SEC] [--interval SEC] [--json] [--strict]" in text
    assert "soak|field-soak|readiness-soak) shift; cmd_soak" in text
    assert 'python3 "$SCRIPT_DIR/diagnostics/soak.py"' in soak
    assert "--duration)" in soak
    assert "--interval)" in soak
    for endpoint in (
        '"/ready"',
        '"/api/v1/readiness"',
        '"/api/v1/app/bootstrap"',
        '"/api/v1/app/capabilities"',
        '"/api/v1/localization/status"',
        '"/api/v1/navigation/status"',
        '"/api/v1/health"',
        '"/api/v1/state"',
        '"/api/v1/path"',
        '"/api/v1/scene_graph"',
        '"/api/v1/locations"',
        '"/api/v1/devices"',
    ):
        assert endpoint in impl
    assert '"/api/v1/app/traffic"' in impl
    assert "READ_ONLY_ENDPOINT_NAMES" in impl
    assert "SCHEMA_VERSIONED_ENDPOINTS" in impl
    assert "advertised_read_only_endpoints" in impl
    assert "client_contract_violations" in impl
    assert '("state", "events", "scene_graph", "locations", "path", "readiness")' in impl
    assert "capabilities.endpoints" in impl
    assert "readiness.reasons" in impl
    assert "readiness.modules" in impl
    assert "readiness.status" in impl
    assert "data_ready:false" in impl
    assert '"non_motion_safe": as_bool' in impl
    assert "NON_MOTION_NAVIGATION_BLOCKERS" in impl
    assert "NON_MOTION_READY_REASONS" in impl
    assert "ready_status_is_non_motion_safe" in impl
    assert "blockers_are_non_motion_safe" in impl
    assert '"navigation_session_inactive"' in impl
    assert '"app_web": {' in impl
    assert '"client_readiness_status": client_readiness.get("status")' in impl
    assert '"readiness_statuses": sorted' in impl
    assert '"readiness_reason_count": stat' in impl
    assert "readiness_reasons={}" in impl
    for forbidden in (
        "-X POST",
        "/api/v1/goal",
        "/api/v1/cmd_vel",
        "/api/v1/session/start",
        "/api/v1/session/end",
        "systemctl restart",
        "systemctl stop",
        "cmd_nav",
        "cmd_map",
    ):
        assert forbidden not in soak
        assert forbidden not in impl
    assert "LINGTU_SOAK_MAX_XY_DRIFT_M" in impl
    assert "LINGTU_SOAK_MAX_ODOM_AGE_MS" in impl
    assert "LINGTU_SOAK_MAX_CLOUD_AGE_MS" in impl
    assert "LINGTU_SOAK_MAX_MAP_POINTS_DROP_RATIO" in impl
    assert '"mode": "non_motion_soak"' in impl
    assert '"max_xy_drift_m": max_xy_drift_m' in impl
    assert '"active_cmd_source_values":' in impl
    assert 'return 1 if args.strict and report["violations"] else 0' in impl


def test_lingtu_soak_accepts_ready_503_when_only_navigation_session_is_inactive():
    soak = _load_soak_module()
    payload = {
        "data_ready": True,
        "non_motion_safe": True,
        "reasons": ["navigation_blocked:navigation_session_inactive"],
        "failed_modules": [],
    }
    sample = _healthy_soak_sample()

    assert soak.ready_status_is_non_motion_safe("ready", 503, payload) is True
    violations, warnings = soak.sample_violations(sample, _soak_limits())

    assert violations == []
    assert warnings == []


def test_lingtu_soak_checks_client_readiness_contract_shape():
    soak = _load_soak_module()
    payloads = {
        "bootstrap": {
            "schema_version": 1,
            "links": {
                "state": "/api/v1/state",
                "events": "/api/v1/events",
                "scene_graph": "/api/v1/scene_graph",
                "locations": "/api/v1/locations",
                "path": "/api/v1/path",
                "readiness": "/api/v1/readiness",
            },
        },
        "capabilities": {"schema_version": 1, "endpoints": {"state": {}}},
        "readiness": {
            "schema_version": 1,
            "status": "degraded",
            "reasons": ["navigation_blocked:navigation_session_inactive"],
            "modules": {},
        },
        "localization": {"schema_version": 1},
        "navigation": {"schema_version": 1},
        "state": {"schema_version": 1},
        "path": {"schema_version": 1},
        "scene_graph": {"schema_version": 1},
        "locations": {"schema_version": 1},
    }

    assert soak.client_contract_violations(payloads) == []

    payloads["readiness"] = {
        "schema_version": 1,
        "status": "blocked",
        "reasons": "navigation_blocked:navigation_session_inactive",
        "modules": [],
    }

    assert soak.client_contract_violations(payloads) == [
        "readiness.reasons",
        "readiness.modules",
        "readiness.status",
    ]


def test_lingtu_soak_rejects_ready_503_when_data_is_not_ready():
    soak = _load_soak_module()
    payload = {
        "data_ready": False,
        "non_motion_safe": True,
        "reasons": ["navigation_blocked:navigation_session_inactive"],
        "failed_modules": [],
    }

    assert soak.ready_status_is_non_motion_safe("ready", 503, payload) is False


def test_lingtu_soak_rejects_ready_503_when_command_source_is_active():
    soak = _load_soak_module()
    sample = _healthy_soak_sample()
    sample["active_cmd_source"] = "teleop"

    violations, _warnings = soak.sample_violations(sample, _soak_limits())

    assert "active_cmd_source=teleop" in violations
    assert "navigation_blockers=navigation_session_inactive" in violations


def test_lingtu_soak_rejects_ready_503_when_additional_navigation_blocker_exists():
    soak = _load_soak_module()
    payload = {
        "data_ready": True,
        "non_motion_safe": True,
        "reasons": [
            "navigation_blocked:navigation_session_inactive",
            "navigation_blocked:safety_stop",
        ],
        "failed_modules": [],
    }
    sample = _healthy_soak_sample()
    sample["navigation_blockers"] = ["navigation_session_inactive", "safety_stop"]

    violations, _warnings = soak.sample_violations(sample, _soak_limits())

    assert soak.ready_status_is_non_motion_safe("ready", 503, payload) is False
    assert "navigation_blockers=navigation_session_inactive,safety_stop" in violations


def test_lingtu_slamcheck_writes_float_relocation_initial_pose():
    text = _read("scripts/lingtu")

    assert "format_relocation_initial_pose()" in text
    assert "initial_pose=$(format_relocation_initial_pose" in text
    assert "SUPER_LIO_RELOCATION_INIT_POSE=${initial_pose:-[0.0,0.0,0.0,0.0,0.0,0.0]}" in text
    assert "SUPER_LIO_RELOCATION_INIT_POSE=[0,0,0.0,0.0,0.0,0]" not in text
    assert "initial pose must be numeric X Y YAW" in text


def test_lingtu_map_save_uses_map_manager_lifecycle():
    text = _read("scripts/lingtu")
    cmd_map = text.split("cmd_map() {", 1)[1].split("cmd_nav() {", 1)[0]

    assert "Saving MapManager artifacts" in cmd_map
    assert '\\"action\\":\\"save\\"' in cmd_map
    assert '"$GW/api/v1/maps"' in cmd_map
    assert '"$GW/api/v1/map/save"' not in cmd_map
    assert "restore_ok=" in cmd_map
    assert "for rebuild_action in build_octomap build_occupancy" in cmd_map
    assert '\\"action\\":\\"$rebuild_action\\"' in cmd_map
    assert "rebuild_ok=" in cmd_map


def test_lingtu_map_start_delegates_to_full_product_mode_switch():
    text = _read("scripts/lingtu")
    cmd_map = text.split("cmd_map() {", 1)[1].split("cmd_nav() {", 1)[0]
    map_start = cmd_map.split("start)", 1)[1].split("save)", 1)[0]

    assert "cmd_mode switch map" in map_start
    assert "slam_dds_set_mode" not in map_start
    assert "/api/v1/session/start" not in map_start


def test_lingtu_map_save_executes_map_manager_lifecycle(tmp_path):
    result, harness = _run_lingtu_command(tmp_path, "map save shell_map")

    assert result.returncode == 0, result.stdout + result.stderr
    assert "Saving MapManager artifacts" in result.stdout
    calls = (harness["root"] / "calls.log").read_text(encoding="utf-8")
    assert 'maps:{"action":"save","name":"shell_map"}' in calls
    assert "save:" not in calls


def test_lingtu_map_restore_rebuilds_derived_artifacts(tmp_path):
    result, harness = _run_lingtu_command(tmp_path, "map restore demo")

    assert result.returncode == 0, result.stdout + result.stderr
    assert "Rebuilding octomap for demo" in result.stdout
    assert "Rebuilding occupancy for demo" in result.stdout
    calls = (harness["root"] / "calls.log").read_text(encoding="utf-8")
    restore = 'restore:{"name":"demo"}'
    octomap = 'maps:{"action":"build_octomap","name":"demo"}'
    occupancy = 'maps:{"action":"build_occupancy","name":"demo"}'
    assert restore in calls
    assert octomap in calls
    assert occupancy in calls
    assert calls.index(restore) < calls.index(octomap) < calls.index(occupancy)


def test_lingtu_slamcheck_activates_map_through_gateway_and_sets_runtime_env():
    text = _read("scripts/lingtu")

    assert 'active_maps_json=$(curl -s --max-time 4 "$GW/api/v1/maps"' in text
    assert "resolve_relocation_map_name()" in text
    assert "lingtu_maps_root()" in text
    assert 'MAP_DIR="${MAP_DIR:-${NAV_MAP_DIR:-$HOME/data/nova/maps}}"' in text
    assert "$HOME/data/lingtu/maps" in text
    assert 'maps_root="$(lingtu_maps_root)"' in text
    assert '[ ! -f "$map_dir/map.pcd" ]' in text
    assert '"$GW/api/v1/map/activate"' in text
    assert 'ln -sfn "$map_name" "$maps_root/active"' not in text
    assert "SUPER_LIO_RELOCATION_MAP_DIR=$maps_root/active" in text
    assert "SUPER_LIO_RELOCATION_MAP_NAME=map.pcd" in text
    assert "SUPER_LIO_RELOCATION_UPDATE_MAP=false" in text


def test_lingtu_slamcheck_validates_recovery_signal_and_action_contract():
    text = _read("scripts/lingtu")

    assert 'recovery_signal=$(pjson "$LOCJ"' in text
    assert 'recovery_action=$(pjson "$LOCJ"' in text
    assert '[ "$recovery_signal" = "NONE" ] || [ "$recovery_signal" = "RECOVERED" ]' in text
    assert '[ "$recovery_action" = "none" ] || [ "$recovery_action" = "-" ]' in text
    assert '[ "$recovery_action" = "$expected_recovery" ]' in text
    assert '[ "$recovery_signal_ok" = "1" ]' in text
    assert '[ "$recovery_action_ok" = "1" ]' in text
    assert "LOC_DIVERGED" in _read("src/localization/adapters/ros2/slam_bridge.py")


def test_lingtu_slamcompare_is_non_motion_static_gate():
    text = _read("scripts/lingtu")
    start = text.index("slamcompare_usage()")
    end = text.index("# -- Subcommand: routecheck --")
    impl = text[start:end]

    assert "Usage: lingtu slamcompare" in text
    assert "slamcompare|slam-compare|super-lio-compare" in text
    assert 'python3 "$SCRIPT_DIR/diagnostics/static_localization_probe.py"' in impl
    assert "cmd_slamcheck super_lio_relocation" in impl
    assert 'slamcompare_wait_ready "localizer"' in impl
    assert 'slamcompare_wait_ready "super_lio_relocation"' in impl
    assert "Rollback SLAM mode: localizer" in impl
    assert "No motion commands are sent." in impl
    assert "/api/v1/goal" not in impl
    assert "/api/v1/cmd_vel" not in impl
    assert "cmd_nav" not in impl


def test_lingtu_routecheck_is_non_motion_route_preflight():
    text = _read("scripts/lingtu")
    start = text.index("routecheck_usage()")
    end = text.index("# -- Subcommand: routecompare --", start)
    impl = text[start:end]

    assert "Usage: lingtu routecheck" in text
    assert "routecheck|route-check|route-preflight" in text
    assert "/api/v1/navigation/plan" in impl
    assert "cmd_slamcheck super_lio_relocation" in impl
    assert 'slamcompare_wait_ready "localizer"' in impl
    assert 'slamcompare_wait_ready "super_lio_relocation"' in impl
    assert "Rollback SLAM mode: localizer" in impl
    assert "No motion commands are sent" in impl
    assert "routecheck_require_no_active_command_source" in impl
    assert "active_cmd_source" in impl
    assert "routecheck_write_summary" in impl
    assert "plan_summary.json" in impl
    assert "selected_planner" in impl
    assert "fallback_reason" in impl
    assert "rejected_plan_count" in impl
    assert "/api/v1/goal" not in impl
    assert "/api/v1/cmd_vel" not in impl
    assert "cmd_nav" not in impl


def test_lingtu_plan_preview_is_offline_non_motion_planner_gate():
    text = _read("scripts/lingtu")
    start = text.index("plan_preview_usage()")
    end = text.index("# -- Subcommand: routecheck --", start)
    impl = text[start:end]

    assert "Usage: lingtu plan-preview" in text
    assert "plan-preview|planpreview|preview-plan" in text
    assert 'python3 "$SCRIPT_DIR/planning/plan_preview.py"' in impl
    assert '--map-root "$map_root"' in impl
    assert "Gateway calls, goals, or cmd_vel" in impl
    assert "/api/v1/goal" not in impl
    assert "/api/v1/navigation/plan" not in impl
    assert "/api/v1/cmd_vel" not in impl
    assert "cmd_nav" not in impl


def test_lingtu_routecompare_is_allow_motion_gated():
    text = _read("scripts/lingtu")
    start = text.index("routecompare_usage()")
    end = text.index("# -- Subcommand: log --", start)
    impl = text[start:end]

    assert "Usage: lingtu routecompare" in text
    assert "routecompare|route-compare" in text
    assert "--allow-motion" in impl
    assert "routecompare requires --allow-motion" in impl
    assert "routecheck_require_no_active_command_source" in impl
    assert "/api/v1/session/start" in impl
    assert "/api/v1/goal" in impl
    assert "/api/v1/cmd_vel" not in impl
    cmd_impl = impl[impl.index("cmd_routecompare()") :]
    assert cmd_impl.index('if [ "$allow_motion" != "1" ]') < cmd_impl.index("routecompare_run_backend")


def test_lingtu_exposes_product_acceptance_gateway_commands():
    text = _read("scripts/lingtu")
    start = text.index("cmd_dataflow_usage()")
    end = text.index("# -- Subcommand: evidence --", start)
    impl = text[start:end]

    assert "Usage: lingtu dataflow" in impl
    assert "Usage: lingtu gateway-runtime-acceptance" in impl
    assert "Usage: lingtu field-check" in impl
    assert "Usage: lingtu saved-map-artifact-gate" in impl
    assert "Gateway + ModulePorts" in impl
    assert "ROS2 topic inspection is not required" in impl
    assert 'cmd+=(--gateway-url "$GW")' in impl
    assert "dataflow|flow" in text
    assert "gateway-runtime-acceptance|gateway-acceptance|gw-accept" in text
    assert "field-check|field-acceptance|accept" in text
    assert "saved-map-artifact-gate|map-gate" in text
    assert "/api/v1/goal" not in impl
    assert "/api/v1/cmd_vel" not in impl
    assert "cmd_nav" not in impl


def test_lingtu_product_acceptance_wrappers_delegate_to_lingtu_py(tmp_path):
    fake_python = tmp_path / "fake_python.sh"
    calls = tmp_path / "python_calls.txt"
    _write_executable(
        fake_python,
        f"""
        #!/usr/bin/env bash
        printf '%s\n' "$*" >> {shlex.quote(_wsl_path(calls))}
        case "$*" in
            *"dataflow"*) echo "Runtime dataflow: PASS" ;;
            *"gateway-runtime-acceptance"*) echo "Gateway runtime acceptance: PASS" ;;
            *"field-check"*) echo "Product field check: PASS" ;;
            *"saved-map-artifact-gate"*) echo "Saved-map artifact gate: PASS" ;;
        esac
        """,
    )

    commands = [
        "dataflow odometry",
        "gw-accept --acceptance-mode field",
        "accept /tmp/map --require-tomogram",
        "map-gate /tmp/map --require-occupancy",
    ]
    for index, command in enumerate(commands):
        run_dir = tmp_path / f"run_{index}"
        run_dir.mkdir()
        result, _harness = _run_lingtu_command(
            run_dir,
            command,
            extra_env={"LINGTU_PYTHON": _wsl_path(fake_python)},
        )
        assert result.returncode == 0, result.stdout + result.stderr

    log = calls.read_text(encoding="utf-8")
    assert "lingtu.py dataflow --gateway-url http://fake-gateway:5050 odometry" in log
    assert (
        "lingtu.py gateway-runtime-acceptance --gateway-url http://fake-gateway:5050 --acceptance-mode field"
    ) in log
    assert ("lingtu.py field-check --gateway-url http://fake-gateway:5050 /tmp/map --require-tomogram") in log
    assert "lingtu.py saved-map-artifact-gate /tmp/map --require-occupancy" in log


def test_lingtu_nav_goal_prechecks_readiness_and_plan_preview(tmp_path):
    result, harness = _run_lingtu_command(tmp_path, "nav goal 1 2")

    assert result.returncode == 0, result.stdout + result.stderr
    assert "plan preview feasible: points=3 planner=pct" in result.stdout
    assert "policy=fallback_astar safety=true" in result.stdout
    calls = (harness["root"] / "calls.log").read_text(encoding="utf-8")
    assert 'plan:{"x":1.0,"y":2.0,"z":0.0,"yaw":0.0}' in calls
    assert 'goal:{"x":1.0,"y":2.0,"z":0.0,"yaw":0.0}' in calls
    assert calls.index("plan:") < calls.index("goal:")


def test_lingtu_nav_relocalize_defaults_to_zero_initial_pose(tmp_path):
    result, harness = _run_lingtu_command(tmp_path, "nav relocalize demo")

    assert result.returncode == 0, result.stdout + result.stderr
    calls = (harness["root"] / "calls.log").read_text(encoding="utf-8")
    assert 'relocalize:{"map_name":"demo","x":0.0,"y":0.0,"yaw":0.0}' in calls


def test_lingtu_nav_goal_rejects_non_numeric_payload_before_curl(tmp_path):
    result, harness = _run_lingtu_command(tmp_path, "nav goal bad 2")

    assert result.returncode != 0
    assert "goal must be numeric X Y [YAW]" in result.stdout
    calls = harness["root"] / "calls.log"
    if calls.exists():
        text = calls.read_text(encoding="utf-8")
        assert "plan:" not in text
        assert "goal:" not in text


def test_lingtu_nav_goal_rejects_readiness_blocker_before_plan(tmp_path):
    result, harness = _run_lingtu_command(
        tmp_path,
        "nav goal 1 2",
        extra_env={"FAKE_NAV_BLOCKER": "localization_recovery_active"},
    )

    assert result.returncode != 0
    assert "navigation is not ready for goals" in result.stdout
    assert "localization_recovery_active" in result.stdout
    calls = harness["root"] / "calls.log"
    if calls.exists():
        text = calls.read_text(encoding="utf-8")
        assert "plan:" not in text
        assert "goal:" not in text


def test_lingtu_nav_goal_rejects_infeasible_plan_preview_before_goal(tmp_path):
    result, harness = _run_lingtu_command(
        tmp_path,
        "nav goal 1 2",
        extra_env={"FAKE_PLAN_FEASIBLE": "0"},
    )

    assert result.returncode != 0
    assert "navigation plan preview is not feasible" in result.stdout
    assert "blocked_by_costmap" in result.stdout
    assert "policy=reject safety=false" in result.stdout
    assert "fallback=pct path_safety failed rejected=1" in result.stdout
    calls = (harness["root"] / "calls.log").read_text(encoding="utf-8")
    assert "plan:" in calls
    assert "goal:" not in calls


def test_lingtu_svc_rejects_unknown_robot_prefixed_unit(tmp_path):
    result, harness = _run_lingtu_command(tmp_path, "svc stop robot-unrelated")

    assert result.returncode != 0
    assert "Usage: lingtu svc stop" in result.stdout
    calls = harness["root"] / "calls.log"
    if calls.exists():
        text = calls.read_text(encoding="utf-8")
        assert "sudo:systemctl stop robot-unrelated.service" not in text
        assert "systemctl:stop robot-unrelated.service" not in text


def test_lingtu_slamcompare_waits_for_candidate_ready_before_probe(tmp_path):
    result, harness = _run_lingtu_command(
        tmp_path,
        "slamcompare --map demo --duration 0 --warmup 0",
        extra_env={
            "FAKE_CANDIDATE_READY_ON_STATE_CALL": "2",
            "LINGTU_SLAMCOMPARE_READY_POLL": "0",
            "LINGTU_SLAMCOMPARE_READY_TIMEOUT": "2",
        },
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert "PASS: static localization compare met non-motion thresholds" in result.stdout
    calls = (harness["root"] / "calls.log").read_text(encoding="utf-8")
    assert "activate:demo" in calls
    assert "switch:super_lio_relocation" in calls
    assert "switch:localizer" in calls
    assert (harness["root"] / "candidate_state_calls").read_text(encoding="utf-8").strip() == "3"


def test_lingtu_slamcheck_executes_super_lio_snapshot_contract(tmp_path):
    result, harness = _run_slamcheck(
        tmp_path,
        "super_lio --duration 0 --save-map smoke_shell --rollback previous",
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert "PASS: super_lio Gateway contract stayed ready" in result.stdout
    assert "PASS: map-save response uses live_map_cloud_snapshot" in result.stdout
    calls = (harness["root"] / "calls.log").read_text(encoding="utf-8")
    assert "switch:super_lio" in calls
    assert 'save:{"name":"smoke_shell"}' in calls
    assert "switch:localizer" in calls


def test_lingtu_slamcheck_executes_relocation_active_map_env_and_rollback(tmp_path):
    result, harness = _run_slamcheck(
        tmp_path,
        "super_lio_relocation --duration 0 --initial-pose 1 2 3 --rollback previous",
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert "Relocation map: demo" in result.stdout
    assert "PASS: super_lio_relocation Gateway contract stayed ready" in result.stdout
    env_file = harness["run"] / "lingtu" / "super_lio_relocation.env"
    env_text = env_file.read_text(encoding="utf-8")
    assert "SUPER_LIO_RELOCATION_MAP_NAME=map.pcd" in env_text
    assert "SUPER_LIO_RELOCATION_UPDATE_MAP=false" in env_text
    assert "SUPER_LIO_RELOCATION_INIT_POSE=[1.000000,2.000000,0.0,0.0,0.0,3.000000]" in env_text
    calls = (harness["root"] / "calls.log").read_text(encoding="utf-8")
    assert "activate:demo" in calls
    assert "switch:super_lio_relocation" in calls
    assert "switch:localizer" in calls


def test_lingtu_slamcheck_overwrites_stale_relocation_runtime_env(tmp_path):
    result, harness = _run_slamcheck(
        tmp_path,
        "super_lio_relocation --duration 0 --initial-pose 4 5 6 --rollback none",
        seed_relocation_env=(
            "SUPER_LIO_RELOCATION_MAP_DIR=/stale\nSUPER_LIO_RELOCATION_INIT_POSE=[9,9,9]\nSTALE_KEY=must_disappear\n"
        ),
    )

    assert result.returncode == 0, result.stdout + result.stderr
    env_file = harness["run"] / "lingtu" / "super_lio_relocation.env"
    env_text = env_file.read_text(encoding="utf-8")
    assert "STALE_KEY" not in env_text
    assert "/stale" not in env_text
    assert "SUPER_LIO_RELOCATION_MAP_NAME=map.pcd" in env_text
    assert "SUPER_LIO_RELOCATION_UPDATE_MAP=false" in env_text
    assert "SUPER_LIO_RELOCATION_INIT_POSE=[4.000000,5.000000,0.0,0.0,0.0,6.000000]" in env_text


def test_lingtu_slamcheck_rejects_unsafe_explicit_relocation_map(tmp_path):
    result, harness = _run_slamcheck(
        tmp_path,
        "super_lio_relocation --duration 0 --map ../outside --rollback none",
    )

    assert result.returncode != 0
    assert "unsafe relocation map name or active map target: ../outside" in result.stdout
    env_file = harness["run"] / "lingtu" / "super_lio_relocation.env"
    assert not env_file.exists()
    calls = harness["root"] / "calls.log"
    if calls.exists():
        assert "switch:super_lio_relocation" not in calls.read_text(encoding="utf-8")


def test_lingtu_slamcheck_rejects_unsafe_active_map_symlink(tmp_path):
    result, harness = _run_slamcheck(
        tmp_path,
        "super_lio_relocation --duration 0 --rollback none",
        active_target="../outside",
    )

    assert result.returncode != 0
    assert "unsafe relocation map name or active map target: ../outside" in result.stdout
    env_file = harness["run"] / "lingtu" / "super_lio_relocation.env"
    assert not env_file.exists()
    calls = harness["root"] / "calls.log"
    if calls.exists():
        assert "switch:super_lio_relocation" not in calls.read_text(encoding="utf-8")


def test_lingtu_slamcheck_rejects_non_numeric_initial_pose(tmp_path):
    result, harness = _run_slamcheck(
        tmp_path,
        "super_lio_relocation --duration 0 --initial-pose bad 2 3 --rollback none",
    )

    assert result.returncode != 0
    assert "initial pose must be numeric X Y YAW" in result.stdout
    env_file = harness["run"] / "lingtu" / "super_lio_relocation.env"
    assert not env_file.exists()
    calls = harness["root"] / "calls.log"
    if calls.exists():
        assert "switch:super_lio_relocation" not in calls.read_text(encoding="utf-8")


def test_lingtu_slamcheck_rejects_missing_initial_pose_values(tmp_path):
    result, harness = _run_slamcheck(
        tmp_path,
        "super_lio_relocation --duration 0 --initial-pose 1 2",
    )

    assert result.returncode != 0
    assert "Usage: lingtu slamcheck" in result.stdout
    env_file = harness["run"] / "lingtu" / "super_lio_relocation.env"
    assert not env_file.exists()


def test_lingtu_slamcheck_fails_on_diverged_recovery_signal_and_rolls_back(tmp_path):
    result, harness = _run_slamcheck(
        tmp_path,
        "super_lio --duration 0 --rollback previous",
        extra_env={"FAKE_RECOVERY_SIGNAL": "LOC_DIVERGED"},
    )

    assert result.returncode != 0
    assert "signal=LOC_DIVERGED" in result.stdout
    assert "FAIL: super_lio Gateway contract did not stay ready" in result.stdout
    calls = (harness["root"] / "calls.log").read_text(encoding="utf-8")
    assert "switch:super_lio" in calls
    assert "switch:localizer" in calls


def test_lingtu_slamcheck_fails_on_unexpected_recovery_action_and_rolls_back(tmp_path):
    result, harness = _run_slamcheck(
        tmp_path,
        "super_lio_relocation --duration 0 --rollback previous",
        extra_env={"FAKE_RECOVERY_ACTION": "restart_wrong_backend"},
    )

    assert result.returncode != 0
    assert "action=restart_wrong_backend" in result.stdout
    assert "FAIL: super_lio_relocation Gateway contract did not stay ready" in result.stdout
    calls = (harness["root"] / "calls.log").read_text(encoding="utf-8")
    assert "switch:super_lio_relocation" in calls
    assert "switch:localizer" in calls


def test_lingtu_slamcompare_passes_static_baseline_and_candidate(tmp_path):
    result, harness = _run_lingtu_command(
        tmp_path,
        "slamcompare --map demo --duration 0 --warmup 0",
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert "PASS: static localization compare met non-motion thresholds" in result.stdout
    assert "No motion commands are sent." in result.stdout
    calls = (harness["root"] / "calls.log").read_text(encoding="utf-8")
    assert "switch:localizer" in calls
    assert "switch:super_lio_relocation" in calls
    assert calls.rstrip().endswith("switch:localizer")
    assert "goal:" not in calls


def test_lingtu_routecheck_previews_baseline_candidate_and_rolls_back(tmp_path):
    artifact = tmp_path / "route-artifacts"
    result, harness = _run_lingtu_command(
        tmp_path,
        (f"routecheck --map demo --goal 1 2 0 --candidate-warmup 0 --artifact-dir {shlex.quote(_wsl_path(artifact))}"),
        extra_env={
            "LINGTU_SLAMCOMPARE_READY_POLL": "0",
            "LINGTU_SLAMCOMPARE_READY_TIMEOUT": "2",
        },
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert "PASS: route validation preflight completed without motion" in result.stdout
    assert (
        "baseline route plan preview feasible "
        "(points=3 planner=pct policy=fallback_astar safety=true fallback=- rejected=0)"
    ) in result.stdout
    assert (
        "candidate route plan preview feasible "
        "(points=3 planner=pct policy=fallback_astar safety=true fallback=- rejected=0)"
    ) in result.stdout
    calls = (harness["root"] / "calls.log").read_text(encoding="utf-8")
    assert "switch:localizer" in calls
    assert "switch:super_lio_relocation" in calls
    switch_calls = [line for line in calls.splitlines() if line.startswith("switch:")]
    assert switch_calls[-1] == "switch:localizer"
    assert calls.count("plan:") == 2
    assert "goal:" not in calls
    assert (artifact / "baseline" / "plan.json").exists()
    assert (artifact / "candidate" / "plan.json").exists()
    baseline = json.loads((artifact / "baseline" / "plan_summary.json").read_text(encoding="utf-8"))
    candidate = json.loads((artifact / "candidate" / "plan_summary.json").read_text(encoding="utf-8"))
    summary = json.loads((artifact / "summary.json").read_text(encoding="utf-8"))
    assert baseline["selected_planner"] == "pct"
    assert baseline["plan_safety_policy"] == "fallback_astar"
    assert baseline["path_safety_ok"] is True
    assert baseline["active_cmd_source_before"] == "none"
    assert baseline["rejected_plan_count"] == 0
    assert candidate["selected_planner"] == "pct"
    assert candidate["plan_safety_policy"] == "fallback_astar"
    assert summary["mode"] == "routecheck_non_motion"
    assert summary["non_motion"] is True
    assert summary["outcome"] == "pass"
    assert summary["phases"]["baseline"]["selected_planner"] == "pct"
    assert (artifact / "after_rollback" / "localization.json").exists()


def test_lingtu_routecheck_blocks_plan_and_captures_failed_rollback(tmp_path):
    artifact = tmp_path / "route-artifacts"
    result, harness = _run_lingtu_command(
        tmp_path,
        (f"routecheck --map demo --goal 1 2 0 --candidate-warmup 0 --artifact-dir {shlex.quote(_wsl_path(artifact))}"),
        extra_env={
            "FAKE_NAV_BLOCKER": "safety_stop",
            "LINGTU_SLAMCOMPARE_READY_POLL": "0",
            "LINGTU_SLAMCOMPARE_READY_TIMEOUT": "2",
        },
    )

    assert result.returncode != 0
    assert "baseline route preflight blocked before plan" in result.stdout
    assert "safety_stop" in result.stdout
    calls = (harness["root"] / "calls.log").read_text(encoding="utf-8")
    switch_calls = [line for line in calls.splitlines() if line.startswith("switch:")]
    assert switch_calls[-1] == "switch:localizer"
    assert "plan:" not in calls
    assert "goal:" not in calls
    assert (artifact / "failed_rollback" / "localization.json").exists()
    summary = json.loads((artifact / "summary.json").read_text(encoding="utf-8"))
    assert summary["mode"] == "routecheck_non_motion"
    assert summary["outcome"] == "fail"
    assert summary["exit_status"] != 0
    assert summary["non_motion"] is True
    assert summary["map"] == "demo"
    assert summary["goal"] == {"x": 1.0, "y": 2.0, "yaw": 0.0}
    assert summary["phases"]["baseline"] == {}
    assert summary["artifacts"]["failed_rollback"].endswith("failed_rollback")


def test_lingtu_routecompare_requires_allow_motion_before_gateway_calls(tmp_path):
    artifact = tmp_path / "route-compare"
    result, harness = _run_lingtu_command(
        tmp_path,
        (
            "routecompare --map demo --goal 1 2 0 --candidate-warmup 0 "
            f"--artifact-dir {shlex.quote(_wsl_path(artifact))}"
        ),
    )

    assert result.returncode != 0
    assert "routecompare requires --allow-motion" in result.stdout
    calls = harness["root"] / "calls.log"
    if calls.exists():
        text = calls.read_text(encoding="utf-8")
        assert "session_start:" not in text
        assert "session_end" not in text
        assert "goal:" not in text
        assert "switch:" not in text
    assert not (artifact / "baseline" / "goal.json").exists()


def test_lingtu_routecompare_allow_motion_runs_baseline_candidate_and_rolls_back(tmp_path):
    artifact = tmp_path / "route-compare"
    result, harness = _run_lingtu_command(
        tmp_path,
        (
            "routecompare --map demo --goal 1 2 0 --candidate-warmup 0 "
            "--timeout 2 --poll 0 --allow-motion "
            f"--artifact-dir {shlex.quote(_wsl_path(artifact))}"
        ),
        extra_env={
            "LINGTU_SLAMCOMPARE_READY_POLL": "0",
            "LINGTU_SLAMCOMPARE_READY_TIMEOUT": "2",
        },
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert "MOTION ENABLED" in result.stdout
    assert "PASS: route A/B motion compare completed" in result.stdout
    calls = (harness["root"] / "calls.log").read_text(encoding="utf-8")
    assert calls.count("goal:") == 2
    assert calls.count("session_start:") == 2
    assert calls.count("session_end") == 2
    assert "switch:localizer" in calls
    assert "switch:super_lio_relocation" in calls
    assert calls.index("session_start:") < calls.index("goal:")
    switch_calls = [line for line in calls.splitlines() if line.startswith("switch:")]
    assert switch_calls[-1] == "switch:localizer"
    assert (artifact / "before" / "localization.json").exists()
    assert (artifact / "baseline" / "goal.json").exists()
    assert (artifact / "baseline" / "result.txt").exists()
    assert (artifact / "candidate" / "goal.json").exists()
    assert (artifact / "candidate" / "result.txt").exists()
    assert (artifact / "after_rollback" / "localization.json").exists()
    summary = json.loads((artifact / "routecompare_summary.json").read_text())
    assert summary["outcome"] == "pass"
    assert summary["experimental_profile_required"] is True
    assert summary["baseline"]["terminal_state"] == "SUCCESS"
    assert summary["candidate"]["terminal_state"] == "SUCCESS"
    assert "route_delta" in summary["baseline"]
    assert summary["rollback"]["localization"]["backend"] == "localizer"


def test_lingtu_routecompare_failure_stops_session_rolls_back_and_captures(tmp_path):
    artifact = tmp_path / "route-compare"
    result, harness = _run_lingtu_command(
        tmp_path,
        (
            "routecompare --map demo --goal 1 2 0 --candidate-warmup 0 "
            "--timeout 2 --poll 0 --allow-motion "
            f"--artifact-dir {shlex.quote(_wsl_path(artifact))}"
        ),
        extra_env={
            "FAKE_GOAL_FAIL_PROFILE": "super_lio_relocation",
            "LINGTU_SLAMCOMPARE_READY_POLL": "0",
            "LINGTU_SLAMCOMPARE_READY_TIMEOUT": "2",
        },
    )

    assert result.returncode != 0
    assert "candidate route goal was rejected" in result.stdout
    calls = (harness["root"] / "calls.log").read_text(encoding="utf-8")
    assert calls.count("goal:") == 2
    assert "stop" in calls
    assert "session_end" in calls
    switch_calls = [line for line in calls.splitlines() if line.startswith("switch:")]
    assert switch_calls[-1] == "switch:localizer"
    assert (artifact / "failed_rollback" / "localization.json").exists()
    assert (artifact / "failed_rollback" / "navigation.json").exists()
    assert (artifact / "failed_rollback" / "services.txt").exists()
    summary = json.loads((artifact / "routecompare_summary.json").read_text())
    assert summary["outcome"] == "failed"
    assert summary["rollback"]["failed_rollback_present"] is True
    assert summary["rollback"]["localization"]["backend"] == "localizer"


def test_lingtu_routecompare_start_failure_stops_session_rolls_back_and_captures(tmp_path):
    artifact = tmp_path / "route-compare"
    result, harness = _run_lingtu_command(
        tmp_path,
        (
            "routecompare --map demo --goal 1 2 0 --candidate-warmup 0 "
            "--timeout 2 --poll 0 --allow-motion "
            f"--artifact-dir {shlex.quote(_wsl_path(artifact))}"
        ),
        extra_env={
            "FAKE_SESSION_START_FAIL_PROFILE": "localizer",
            "LINGTU_SLAMCOMPARE_READY_POLL": "0",
            "LINGTU_SLAMCOMPARE_READY_TIMEOUT": "2",
        },
    )

    assert result.returncode != 0
    assert "baseline navigation session did not start" in result.stdout
    calls = (harness["root"] / "calls.log").read_text(encoding="utf-8")
    assert "session_start:" in calls
    assert "goal:" not in calls
    assert "stop" in calls
    assert "session_end" in calls
    switch_calls = [line for line in calls.splitlines() if line.startswith("switch:")]
    assert switch_calls[-1] == "switch:localizer"
    assert (artifact / "failed_rollback" / "localization.json").exists()
    summary = json.loads((artifact / "routecompare_summary.json").read_text())
    assert summary["outcome"] == "failed"
    assert summary["rollback"]["localization"]["backend"] == "localizer"


def test_lingtu_routecompare_timeout_stops_session_rolls_back_and_captures(tmp_path):
    artifact = tmp_path / "route-compare"
    result, harness = _run_lingtu_command(
        tmp_path,
        (
            "routecompare --map demo --goal 1 2 0 --candidate-warmup 0 "
            "--timeout 0 --poll 0 --allow-motion "
            f"--artifact-dir {shlex.quote(_wsl_path(artifact))}"
        ),
        extra_env={
            "FAKE_GOAL_NONTERMINAL_PROFILE": "localizer",
            "LINGTU_SLAMCOMPARE_READY_POLL": "0",
            "LINGTU_SLAMCOMPARE_READY_TIMEOUT": "2",
        },
    )

    assert result.returncode != 0
    assert "baseline route did not reach a terminal state" in result.stdout
    calls = (harness["root"] / "calls.log").read_text(encoding="utf-8")
    assert "goal:" in calls
    assert "stop" in calls
    assert "session_end" in calls
    switch_calls = [line for line in calls.splitlines() if line.startswith("switch:")]
    assert switch_calls[-1] == "switch:localizer"
    assert (artifact / "failed_rollback" / "navigation.json").exists()
    summary = json.loads((artifact / "routecompare_summary.json").read_text())
    assert summary["outcome"] == "failed"
    assert summary["rollback"]["failed_rollback_present"] is True


def test_lingtu_slamcompare_fails_pose_jump_and_rolls_back(tmp_path):
    result, harness = _run_lingtu_command(
        tmp_path,
        "slamcompare --map demo --duration 0 --warmup 0",
        extra_env={"FAKE_CANDIDATE_X": "2.2"},
    )

    assert result.returncode != 0
    assert "FAIL: static localization compare blocked motion promotion" in result.stdout
    assert "pose jump" in result.stdout
    calls = (harness["root"] / "calls.log").read_text(encoding="utf-8")
    assert "switch:super_lio_relocation" in calls
    assert calls.rstrip().endswith("switch:localizer")
    assert "goal:" not in calls


def test_lingtu_slamcompare_fails_candidate_not_ready_and_rolls_back(tmp_path):
    result, harness = _run_lingtu_command(
        tmp_path,
        "slamcompare --map demo --duration 0 --warmup 0",
        extra_env={"FAKE_CANDIDATE_READY": "false"},
    )

    assert result.returncode != 0
    assert "did not become ready before static compare sampling" in result.stdout
    assert "ready=false" in result.stdout
    calls = (harness["root"] / "calls.log").read_text(encoding="utf-8")
    assert calls.rstrip().endswith("switch:localizer")


def test_super_lio_relocation_service_uses_active_map_and_float_pose():
    text = _read("scripts/deploy/s100p/super_lio_relocation.service")

    assert "Environment=SUPER_LIO_RELOCATION_MAP_DIR=/home/sunrise/data/nova/maps/active" in text
    assert "Environment=SUPER_LIO_RELOCATION_INIT_POSE=[0.0,0.0,0.0,0.0,0.0,0.0]" in text
    assert '-p "lio.relocation.init_pose:=$${SUPER_LIO_RELOCATION_INIT_POSE}"' in text
    assert '-p "lio.map.save_map_dir:=$${EFFECTIVE_MAP_DIR}"' in text
    assert 'ln -sfn "$${SOURCE_MAP_DIR}" "$${SUPER_LIO_ROOT}/map/lingtu_active"' in text
    assert 'EFFECTIVE_MAP_DIR="map/lingtu_active"' in text
    assert "Super-LIO relocation map: $${MAP_CHECK_DIR}/$${SUPER_LIO_RELOCATION_MAP_NAME} bytes=$${MAP_BYTES}" in text


def test_camera_deployment_retires_legacy_units():
    install = _read("docs/04-deployment/services/install.sh")
    readme = _read("docs/04-deployment/README.md")
    camera_service = _read("docs/04-deployment/services/robot-camera.service")

    assert "robot-camera.service" in install
    assert "camera.service orbbec-camera.service" in install
    assert 'systemctl mask "$legacy"' in install
    assert "duplicate Orbbec node instances or image" in install
    assert "One /camera/camera plus one /camera/camera_container is normal" in install
    assert "Only `robot-camera.service` should own the Orbbec ROS driver" in readme
    assert "Seeing one `/camera/camera` node" in readme
    assert "KillMode=control-group" in camera_service


def test_lingtu_target_is_installed_enabled_and_starts_full_stack():
    install = _read("docs/04-deployment/services/install.sh")
    target = _read("docs/04-deployment/services/lingtu.target")

    assert 'sudo cp "$SERVICES_DIR/lingtu.target" "$SYSTEMD_DIR/lingtu.target"' in install
    assert (
        "sudo systemctl enable robot-lidar robot-camera robot-brainstem "
        "robot-fastlio2 robot-localizer lingtu lingtu.target"
    ) in install
    assert "sudo systemctl start lingtu.target" in install

    for unit in (
        "robot-brainstem.service",
        "robot-camera.service",
        "robot-lidar.service",
        "robot-fastlio2.service",
        "robot-localizer.service",
        "lingtu.service",
    ):
        assert unit in target


def test_robot_lidar_uses_installed_ros2_env_helper():
    install = _read("docs/04-deployment/services/install.sh")
    lidar_service = _read("docs/04-deployment/services/robot-lidar.service")

    assert 'ros2-env.sh" "$LINGTU_CONFIG/ros2-env.sh"' in install
    assert "source /opt/lingtu/config/ros2-env.sh" in lidar_service
    assert "ExecStartPost=/bin/bash -c" in lidar_service
    assert "robot-lidar readiness failed" in lidar_service
    assert "TimeoutStopSec=10s" in lidar_service
    assert "KillMode=control-group" in lidar_service
    assert 'sed -n "s/^Publisher count: //p"' in lidar_service
    assert "ros2-env.conf" not in lidar_service


def test_thunder_legacy_service_installer_installs_ros2_env_helper():
    install = _read("scripts/deploy/s100p/install_services.sh")
    thunder_install = _read("scripts/deploy/thunder/install_services.sh")

    assert 'CONFIG_DIR="${LINGTU_CONFIG_DIR:-/opt/lingtu/config}"' in install
    assert 'cp "${ROS2_ENV_SRC}" "${CONFIG_DIR}/ros2-env.sh"' in install
    assert "LINGTU_ENABLE_LEGACY_ROS2_SERVICES" in install
    assert "Legacy ROS2 compat services installed but not enabled" in install
    assert "Native default: scripts/deploy/thunder/install_services.sh field-cpp" in install
    assert 'cp "${SCRIPT_DIR}/ros2-env.sh" "${CONFIG_DIR}/ros2-env.sh"' in thunder_install


def test_gateway_service_contract_is_in_process_lingtu_service():
    readme = _read("docs/04-deployment/README.md")
    lingtu_service = _read("docs/04-deployment/services/lingtu.service")

    assert "no standalone `lingtu-gateway.service`" in readme
    assert "Gateway runs inside" in readme
    assert "lingtu-gateway.service" not in lingtu_service


def test_lingtu_cli_has_lightweight_localization_recovery():
    script = _read("scripts/lingtu")
    docs = _read("docs/04-deployment/lingtu_cli.md")

    assert "svc_restart_localization_chain()" in script
    assert "svc_start_unit lingtu-livox-dds.service" in script
    assert "svc_start_unit lingtu-slam-dds.service" in script
    assert 'slam_dds_wait_status "" 0 35' in script
    assert "svc_force_stop_unit robot-localizer.service" in script
    assert "svc_force_stop_unit robot-fastlio2.service" in script
    assert "native SLAM DDS service is not installed: lingtu-slam-dds.service" in script
    assert "Use legacy_fastlio2 / legacy_localizer explicitly" in script
    assert "svc_wait_gateway_ready 35" in script
    assert "localization|localization_chain|slam_chain|loc)" in script
    assert "lingtu svc restart localization" in docs
    assert "lingtu-slam-dds.service" in docs
    assert "legacy_fastlio2" in docs


def test_lingtu_nav_start_auto_relocalizes_when_tracking_not_reusable():
    script = _read("scripts/lingtu")
    nav_start = script.split("cmd_nav() {", 1)[1].split("\n        stop|end)", 1)[0]
    seeded_relocalize = script.split("nav_relocalize_saved_map() {", 1)[1].split(
        "\n}\n\nnav_global_relocalize_saved_map() {", 1
    )[0]
    reuse_check = script.split("nav_relocalization_ready_without_request() {", 1)[1].split("\nPY\n}", 1)[0]

    assert 'elif nav_relocalization_ready_without_request "$map"; then' in nav_start
    assert 'nav_global_relocalize_saved_map "$map" || exit 1' in nav_start
    assert 'nav_relocalize_saved_map "$map" "$initial_x" "$initial_y" "$initial_yaw" || exit 1' in nav_start
    assert 'raw_map="$map"' in nav_start
    assert 'map=$(resolve_relocation_map_name "$maps_root" "$raw_map")' in nav_start
    assert 'tf.get("valid") is True' in reuse_check
    assert "reported_map == map_name" in reuse_check
    assert "pose_fresh_ok" in reuse_check
    assert "*timeout*)" in seeded_relocalize


def test_lingtu_svc_status_defaults_to_native_services():
    script = _read("scripts/lingtu")
    docs = _read("docs/04-deployment/lingtu_cli.md")
    status_case = script.split("cmd_svc() {", 1)[1].split("\n        restart)", 1)[0]
    default_status = status_case.split("status)", 1)[1].split("status-legacy|legacy-status", 1)[0]
    legacy_status = status_case.split("status-legacy|legacy-status", 1)[1]

    assert "lingtu-livox-dds.service" in default_status
    assert "lingtu-slam-dds.service" in default_status
    assert "lingtu-nav-dds.service" in default_status
    assert "lingtu.service" in default_status
    assert "robot-lidar.service" not in default_status
    assert "robot-fastlio2.service" not in default_status
    assert "robot-localizer.service" not in default_status
    assert "robot-lidar.service" in legacy_status
    assert "robot-fastlio2.service" in legacy_status
    assert "robot-localizer.service" in legacy_status
    assert "legacy/experimental services are active" in script
    assert "status-legacy|restart" in script
    assert 'if [ "$ros2" = "1" ]; then' in script
    assert "cmd_svc status-legacy" in script
    assert "lingtu svc status-legacy" in docs


def test_lingtu_svc_status_output_hides_legacy_rows_by_default(tmp_path):
    result, _ = _run_lingtu_command(tmp_path, "svc status")

    assert result.returncode == 0, result.stderr
    assert "lingtu-livox-dds.service" in result.stdout
    assert "lingtu-slam-dds.service" in result.stdout
    assert "lingtu-nav-dds.service" in result.stdout
    assert "legacy_lidar" not in result.stdout
    assert "robot-lidar.service" not in result.stdout
    assert "legacy_fastlio2" not in result.stdout
    assert "robot-fastlio2.service" not in result.stdout
    assert "legacy/experimental services are active" not in result.stdout
    assert "lingtu svc status-legacy" not in result.stdout


def test_lingtu_svc_status_legacy_output_lists_compat_rows(tmp_path):
    result, _ = _run_lingtu_command(tmp_path, "svc status-legacy")

    assert result.returncode == 0, result.stderr
    assert "lingtu-livox-dds.service" in result.stdout
    assert "lingtu-slam-dds.service" in result.stdout
    assert "legacy_lidar" in result.stdout
    assert "robot-lidar.service" in result.stdout
    assert "legacy_fastlio2" in result.stdout
    assert "robot-fastlio2.service" in result.stdout
    assert "legacy_localizer" in result.stdout
    assert "robot-localizer.service" in result.stdout
    assert "super_lio" in result.stdout
