import importlib.util
import json
import os
import shlex
import shutil
import subprocess
import sys
import textwrap
import time
from contextlib import contextmanager
from pathlib import Path

import pytest

from lingtu.run_plan import CURRENT_RUN_SCHEMA
from runtime.runtime_interface import TOPICS

REPO_ROOT = Path(__file__).resolve().parents[3]
MAP_CLIENT_API_KEY = "m" * 64


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


def _make_lingtu_harness(tmp_path: Path) -> dict[str, Path | str]:
    fake_root = tmp_path / "fake"
    fake_bin = fake_root / "bin"
    fake_run = fake_root / "run"
    fake_home = fake_root / "home"
    map_client_env = fake_root / "map-client.env"
    fake_bin.mkdir(parents=True)
    fake_run.mkdir(parents=True)
    map_client_env.write_text(
        f"LINGTU_MAP_API_KEY={MAP_CLIENT_API_KEY}\n",
        encoding="utf-8",
        newline="\n",
    )
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
                    lingtu-livox-dds.service|lingtu-slam-dds.service|lingtu-nav-dds.service|lingtu-driver.service|lingtu.service|robot-camera.service|robot-brainstem.service)
                        active=active ;;
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
        original_args=("$@")
        url=""
        data=""
        method="GET"
        map_api_key=""
        product_session_id=""
        map_api_key_in_argv=0
        x_api_key_headers=0
        product_session_headers=0
        while [ "$#" -gt 0 ]; do
            case "$1" in
                -X|--request)
                    method="${2:-}"
                    shift 2
                    ;;
                -d|--data|--data-raw|--data-binary)
                    data="${2:-}"
                    shift 2
                    ;;
                -H|--header)
                    header="${2:-}"
                    if [[ "$header" =~ ^[Xx]-[Aa][Pp][Ii]-[Kk][Ee][Yy]:[[:space:]]*(.*)$ ]]; then
                        x_api_key_headers=$((x_api_key_headers + 1))
                        map_api_key="${BASH_REMATCH[1]}"
                    elif [[ "$header" =~ ^[Xx]-[Ll][Ii][Nn][Gg][Tt][Uu]-[Pp][Rr][Oo][Dd][Uu][Cc][Tt]-[Ss][Ee][Ss][Ss][Ii][Oo][Nn]:[[:space:]]*(.*)$ ]]; then
                        product_session_headers=$((product_session_headers + 1))
                        product_session_id="${BASH_REMATCH[1]}"
                    fi
                    shift 2
                    ;;
                -K|--config)
                    config="${2:-}"
                    if [ "$config" = "-" ]; then
                        while IFS= read -r config_line; do
                            if [[ "$config_line" =~ ^header[[:space:]]*=[[:space:]]*\"[Xx]-[Aa][Pp][Ii]-[Kk][Ee][Yy]:[[:space:]]*([A-Za-z0-9_-]+)\"$ ]]; then
                                x_api_key_headers=$((x_api_key_headers + 1))
                                map_api_key="${BASH_REMATCH[1]}"
                            elif [[ "$config_line" =~ ^header[[:space:]]*=[[:space:]]*\"[Xx]-[Ll][Ii][Nn][Gg][Tt][Uu]-[Pp][Rr][Oo][Dd][Uu][Cc][Tt]-[Ss][Ee][Ss][Ss][Ii][Oo][Nn]:[[:space:]]*([A-Za-z0-9_.-]+)\"$ ]]; then
                                product_session_headers=$((product_session_headers + 1))
                                product_session_id="${BASH_REMATCH[1]}"
                            fi
                        done
                    fi
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
        if [ -n "$map_api_key" ]; then
            for arg in "${original_args[@]}"; do
                if [[ "$arg" == *"$map_api_key"* ]]; then
                    map_api_key_in_argv=1
                fi
            done
        fi
        if [ -n "$product_session_id" ]; then
            for arg in "${original_args[@]}"; do
                if [[ "$arg" == *"$product_session_id"* ]]; then
                    map_api_key_in_argv=1
                fi
            done
        fi
        echo "curl:$method:$url:x_api_key_headers=$x_api_key_headers:product_session_headers=$product_session_headers:key_in_argv=$map_api_key_in_argv" \
            >> "${FAKE_ROOT}/calls.log"
        current=localizer
        case "$url" in
            */api/v1/slam/status)
                echo '{"mode":"localizer","services":{"lidar":"running","slam":"running","localizer":"running"}}'
                ;;
            */api/v1/localization/status)
                echo '{"state":"TRACKING","ready":true,"pose_fresh":true,"has_odometry":true,"odom_age_ms":80.0,"cloud_age_ms":90.0,"diag_age_ms":80.0,"localizer_health_topic_age_ms":80.0,"map_cloud_fresh":true,"confidence":0.96,"backend":"localizer","health_source":"localizer_health_topic","map_save_source":"active_map","saved_map_relocalization_supported":true,"restart_recovery_supported":true,"recovery_method":"relocalize_service","relocalization_state":"idle","recovery_signal":"RECOVERED","recovery_action":"none"}'
                ;;
            */api/v1/state)
                x="${FAKE_LOCALIZER_X:-1.0}"
                y="${FAKE_LOCALIZER_Y:-2.0}"
                yaw="${FAKE_LOCALIZER_YAW:-0.3}"
                ready="${FAKE_LOCALIZER_READY:-true}"
                printf '{"schema_version":1,"odometry":{"x":%s,"y":%s,"z":0.0,"yaw":%s},"localization":{"backend":"localizer","health_source":"localizer_health_topic","state":"TRACKING","ready":%s,"confidence":0.96,"icp_quality":0.02,"odom_age_ms":80.0,"cloud_age_ms":90.0,"map_save_source":"active_map","recovery_signal":"RECOVERED","recovery_action":"none"},"session":{"localization_backend":"localizer","health_source":"localizer_health_topic","map_save_source":"active_map","icp_quality":0.02,"recovery_signal":"RECOVERED","recovery_action":"none"},"navigation":{"control":{"active_cmd_source":"none"}},"map":{"live_points":5000,"live_cloud_frames":1}}\n' \
                    "$x" "$y" "$yaw" "$ready"
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
            */api/v1/explore/start)
                echo "explore_task_start" >> "${FAKE_ROOT}/calls.log"
                echo '{"schema_version":"lingtu.explore.run.v1","ok":true,"accepted":true,"request_id":"01K1M9S4FX27T8XMY6QJNBAV3V","exploration_run_id":"01K1M9S4FX27T8XMY6QJNBAV3W","state":"submitted","reason":"exploration_started","terminal":false}'
                ;;
            */api/v1/explore/status)
                echo "explore_status" >> "${FAKE_ROOT}/calls.log"
                echo '{"schema_version":1,"available":true,"active":false,"state":"IDLE"}'
                ;;
            */api/v1/maps/*/build_octomap)
                name="${url%/build_octomap}"
                name="${name##*/}"
                echo "build_octomap:$method:$name:$data" >> "${FAKE_ROOT}/calls.log"
                printf '{"schema_version":1,"ok":true,"success":true,"action":"build_octomap","name":"%s","octomap":"octomap.ot"}\n' "$name"
                ;;
            */api/v1/maps/*/build_occupancy)
                name="${url%/build_occupancy}"
                name="${name##*/}"
                echo "build_occupancy:$method:$name:$data" >> "${FAKE_ROOT}/calls.log"
                printf '{"schema_version":1,"ok":true,"success":true,"action":"build_occupancy","name":"%s","occupancy":"occupancy.npz"}\n' "$name"
                ;;
            */api/v1/slam/maps)
                echo "map_list:$method:$data" >> "${FAKE_ROOT}/calls.log"
                if [ "${FAKE_MAP_LIST_HTTP_ERROR:-0}" = "1" ]; then
                    echo '{"schema_version":1,"ok":false,"message":"unavailable"}' >&2
                    exit 22
                fi
                printf '{"schema_version":1,"maps":[{"name":"demo","has_pcd":true,"navigation_ready":true}],"count":1,"active":"demo"}\n'
                ;;
            */api/v1/map/restore_predufo)
                echo "restore:$data" >> "${FAKE_ROOT}/calls.log"
                printf '{"schema_version":1,"ok":true,"success":true,"name":"demo","restored_size":123}\n'
                ;;
            */api/v1/maps/operations/*)
                echo "save_status:$url" >> "${FAKE_ROOT}/calls.log"
                request_id="${url##*/}"
                mode="${FAKE_MAP_SAVE_MODE:-success}"
                case "$mode" in
                    async_success|replay_missing_identity|replay_identity_mismatch|replay_running_ok_false|succeeded_not_ready)
                        printf '{"schema_version":1,"ok":true,"success":true,"action":"save_map_status","operation":{"operation_id":"%s","request_id":"%s","state":"SUCCEEDED","compatibility_ready":true}}\n' \
                            "$request_id" "$request_id"
                        ;;
                    status_missing_identity)
                        echo '{"schema_version":1,"ok":true,"success":true,"action":"save_map_status","operation":{"state":"RUNNING","compatibility_ready":false}}'
                        ;;
                    status_identity_mismatch)
                        printf '{"schema_version":1,"ok":true,"success":true,"action":"save_map_status","operation":{"operation_id":"%s","request_id":"different_request_id","state":"RUNNING","compatibility_ready":false}}\n' \
                            "$request_id"
                        ;;
                    unknown_state)
                        printf '{"schema_version":1,"ok":true,"success":true,"action":"save_map_status","operation":{"operation_id":"%s","request_id":"%s","state":"PAUSED","compatibility_ready":false}}\n' \
                            "$request_id" "$request_id"
                        ;;
                    query_failed)
                        printf '{"schema_version":1,"ok":false,"success":false,"action":"save_map_status","operation":{"operation_id":"%s","request_id":"%s","state":"RUNNING","compatibility_ready":false}}\n' \
                            "$request_id" "$request_id"
                        ;;
                    cancelled)
                        printf '{"schema_version":1,"ok":true,"success":true,"action":"save_map_status","operation":{"operation_id":"%s","request_id":"%s","state":"CANCELLED","compatibility_ready":false,"reason_code":"cancelled"}}\n' \
                            "$request_id" "$request_id"
                        ;;
                    timeout)
                        printf '{"schema_version":1,"ok":true,"success":true,"action":"save_map_status","operation":{"operation_id":"%s","request_id":"%s","state":"RUNNING","compatibility_ready":false}}\n' \
                            "$request_id" "$request_id"
                        ;;
                    *)
                        echo '{"schema_version":1,"ok":false,"success":false,"message":"operation not found"}'
                        exit 22
                        ;;
                esac
                ;;
            */api/v1/map/save)
                echo "save:$data" >> "${FAKE_ROOT}/calls.log"
                request_id="$(printf '%s' "$data" | python3 -c 'import json,sys; print(json.load(sys.stdin).get("request_id", ""))')"
                mode="${FAKE_MAP_SAVE_MODE:-success}"
                counter="${FAKE_ROOT}/map_save_submit_calls"
                count="$(cat "$counter" 2>/dev/null || echo 0)"
                count="$((count + 1))"
                echo "$count" > "$counter"
                case "$mode" in
                    network_error)
                        exit 7
                        ;;
                    http_error)
                        echo '{"schema_version":1,"ok":false,"success":false,"message":"unavailable"}' >&2
                        exit 22
                        ;;
                    bad_json)
                        echo 'not-json'
                        ;;
                    missing_identity)
                        echo '{"schema_version":1,"ok":true,"success":true,"status":"ready","navigation_ready":true,"compatibility_ready":true,"operation":{"state":"SUCCEEDED","compatibility_ready":true}}'
                        ;;
                    identity_mismatch)
                        printf '{"schema_version":1,"ok":true,"success":true,"status":"ready","operation_id":"%s","navigation_ready":true,"compatibility_ready":true,"operation":{"operation_id":"%s","request_id":"different_request_id","state":"SUCCEEDED","compatibility_ready":true}}\n' \
                            "$request_id" "$request_id"
                        ;;
                    submit_running_ok_false)
                        printf '{"schema_version":1,"ok":false,"success":false,"accepted":true,"status":"running","operation_id":"%s","operation":{"operation_id":"%s","request_id":"%s","state":"RUNNING","compatibility_ready":false}}\n' \
                            "$request_id" "$request_id" "$request_id"
                        ;;
                    failed)
                        printf '{"schema_version":1,"ok":false,"success":false,"status":"failed","operation_id":"%s","operation":{"operation_id":"%s","request_id":"%s","state":"FAILED","reason_code":"artifact_failed"}}\n' \
                            "$request_id" "$request_id" "$request_id"
                        ;;
                    cancelled|timeout|status_missing_identity|status_identity_mismatch|unknown_state|query_failed)
                        printf '{"schema_version":1,"ok":true,"success":false,"accepted":true,"status":"running","operation_id":"%s","operation":{"operation_id":"%s","request_id":"%s","state":"RUNNING","compatibility_ready":false}}\n' \
                            "$request_id" "$request_id" "$request_id"
                        ;;
                    async_success|replay_missing_identity|replay_identity_mismatch|replay_running_ok_false|succeeded_not_ready)
                        if [ "$count" -eq 1 ]; then
                            printf '{"schema_version":1,"ok":true,"success":false,"accepted":true,"status":"running","operation_id":"%s","operation":{"operation_id":"%s","request_id":"%s","state":"RUNNING","compatibility_ready":false}}\n' \
                                "$request_id" "$request_id" "$request_id"
                        else
                            case "$mode" in
                                async_success)
                                    printf '{"schema_version":1,"ok":true,"success":true,"status":"ready","operation_id":"%s","navigation_ready":true,"compatibility_ready":true,"operation":{"operation_id":"%s","request_id":"%s","state":"SUCCEEDED","compatibility_ready":true}}\n' \
                                        "$request_id" "$request_id" "$request_id"
                                    ;;
                                replay_missing_identity)
                                    echo '{"schema_version":1,"ok":true,"success":true,"status":"ready","navigation_ready":true,"compatibility_ready":true,"operation":{"state":"SUCCEEDED","compatibility_ready":true}}'
                                    ;;
                                replay_identity_mismatch)
                                    printf '{"schema_version":1,"ok":true,"success":true,"status":"ready","operation_id":"%s","navigation_ready":true,"compatibility_ready":true,"operation":{"operation_id":"%s","request_id":"different_request_id","state":"SUCCEEDED","compatibility_ready":true}}\n' \
                                        "$request_id" "$request_id"
                                    ;;
                                succeeded_not_ready)
                                    printf '{"schema_version":1,"ok":true,"success":true,"status":"not_ready","operation_id":"%s","navigation_ready":true,"compatibility_ready":true,"operation":{"operation_id":"%s","request_id":"%s","state":"SUCCEEDED","compatibility_ready":true}}\n' \
                                        "$request_id" "$request_id" "$request_id"
                                    ;;
                                replay_running_ok_false)
                                    printf '{"schema_version":1,"ok":false,"success":false,"accepted":true,"status":"running","operation_id":"%s","operation":{"operation_id":"%s","request_id":"%s","state":"RUNNING","compatibility_ready":false}}\n' \
                                        "$request_id" "$request_id" "$request_id"
                                    ;;
                            esac
                        fi
                        ;;
                    partial)
                        printf '{"schema_version":1,"ok":true,"success":true,"status":"partial","operation_id":"%s","navigation_ready":false,"compatibility_ready":true,"operation":{"operation_id":"%s","request_id":"%s","state":"SUCCEEDED","compatibility_ready":true}}\n' \
                            "$request_id" "$request_id" "$request_id"
                        ;;
                    compatibility_degraded)
                        printf '{"schema_version":1,"ok":true,"success":true,"status":"ready","operation_id":"%s","navigation_ready":true,"compatibility_ready":false,"operation":{"operation_id":"%s","request_id":"%s","state":"SUCCEEDED","compatibility_ready":false}}\n' \
                            "$request_id" "$request_id" "$request_id"
                        ;;
                    *)
                        printf '{"schema_version":1,"ok":true,"success":true,"status":"ready","operation_id":"%s","navigation_ready":true,"compatibility_ready":true,"operation":{"operation_id":"%s","request_id":"%s","state":"SUCCEEDED","compatibility_ready":true}}\n' \
                            "$request_id" "$request_id" "$request_id"
                        ;;
                esac
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
        "map_client_env_posix": _wsl_path(map_client_env),
        "root_posix": fake_root_posix,
        "run_posix": fake_run_posix,
    }


def _seed_current_run(harness: dict[str, Path | str], product: str) -> Path:
    from lingtu.assembly.products import resolve_product_host_config
    from lingtu.assembly.profile_builder import compile_run_plan

    config = resolve_product_host_config(product, "real", run_startup_checks=False)
    plan = compile_run_plan(product, "real", config)
    state_root = Path(harness["root"]) / "runtime-state"
    plan_path = state_root / "plans" / f"{plan.fingerprint}.json"
    plan.write(plan_path)
    current_path = state_root / "current.json"
    current_path.write_text(
        json.dumps(
            {
                "schema_version": CURRENT_RUN_SCHEMA,
                "product": plan.product,
                "product_variant": plan.product_variant,
                "env": plan.env,
                "fingerprint": plan.fingerprint,
                "run_plan_path": _wsl_path(plan_path),
            }
        ),
        encoding="utf-8",
    )
    return current_path


def _run_lingtu_command(
    tmp_path: Path,
    command_args: str,
    extra_env: dict[str, str] | None = None,
    active_product: str | None = None,
    active_fingerprint: str | None = None,
):
    if shutil.which("bash") is None:
        pytest.skip("bash is required for scripts/lingtu shell behavior tests")
    harness = _make_lingtu_harness(tmp_path)
    exports = {
        "HOME": harness["home_posix"],
        "FAKE_ROOT": harness["root_posix"],
        "FAKE_RUN": harness["run_posix"],
        "GW": "http://fake-gateway:5050",
        "LINGTU_ROS2_BIN": f"{harness['bin_posix']}/ros2",
        "LINGTU_SYSTEMCTL_BIN": f"{harness['bin_posix']}/systemctl",
        "LINGTU_JOURNALCTL_BIN": f"{harness['bin_posix']}/journalctl",
        "LINGTU_IP_BIN": f"{harness['bin_posix']}/ip",
        "LINGTU_MAP_CLIENT_ENV_FILE": harness["map_client_env_posix"],
        "LINGTU_SESSION_ROOT": f"{harness['run_posix']}/lingtu",
    }
    if active_product is not None:
        current_run_path = _seed_current_run(harness, active_product)
        if active_fingerprint is not None:
            payload = json.loads(current_run_path.read_text(encoding="utf-8"))
            payload["fingerprint"] = active_fingerprint
            current_run_path.write_text(json.dumps(payload), encoding="utf-8")
        exports["LINGTU_CURRENT_FILE"] = _wsl_path(current_run_path)
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


def test_legacy_docs_service_graph_stays_removed():
    service_dir = REPO_ROOT / "docs" / "04-deployment" / "services"

    assert not service_dir.exists()
    assert not (service_dir / "install.sh").exists()
    assert not (service_dir / "lingtu.target").exists()


@pytest.mark.parametrize(
    "filename",
    (
        "lidar.service",
        "slam.service",
        "slam_pgo.service",
        "localizer.service",
        "genz_icp.service",
        "hba.service",
    ),
)
def test_s100p_ros2_systemd_service_templates_stay_removed(filename: str):
    assert not (REPO_ROOT / "scripts" / "deploy" / "s100p" / filename).exists()


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
    text = _read("scripts/compat/ros2/hardware/record_bag.sh")
    topic_lines = {line.strip() for line in text.splitlines() if line.strip().startswith("/")}

    assert TOPICS.localization_quality in topic_lines
    assert TOPICS.localization_health in topic_lines
    assert "/localization_quality" not in topic_lines


def test_record_bag_captures_published_camera_intrinsics_topics():
    text = _read("scripts/compat/ros2/hardware/record_bag.sh")
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
    text = _read("src/diagnostics/field/doctor.py")

    assert "camera.camera_info" in text
    assert '"/camera/color/camera_info"' in text
    assert '"/camera/depth/camera_info"' in text
    assert '"/camera/camera_info"' in text
    assert "camera intrinsics topic has publishers" in text


def test_lingtu_doctor_checks_camera_device_online_status():
    text = _read("src/diagnostics/field/doctor.py")

    assert "camera.device_status" in text
    assert "--require-camera" in text
    assert "require_camera = options.require_camera" in text
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
    text = _read("src/diagnostics/field/doctor.py")

    assert "/slam/localization_quality" in text
    assert "/localization_quality" in text
    assert "legacy; remap to /slam/localization_quality" in text


def test_lingtu_doctor_json_gates_runtime_readiness_freshness():
    text = _read("src/diagnostics/field/doctor.py")

    assert "RunPlan.load" in text
    assert "plan.assert_compatible" in text
    assert 'process.manager == "systemd"' in text
    assert "current Product does not match RunPlan" in text
    assert "current Env does not match RunPlan" in text
    assert "canonical_services" not in text
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
    assert "client_ready_code, client_ready, client_ready_err = http_json(" in text
    assert '"/api/v1/readiness",' in text
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
    wrapper = _read("scripts/lingtu")
    text = _read("src/diagnostics/field/doctor.py")

    assert "Usage: lingtu doctor [--non-motion] [--json] [--strict] [--realtime] [--require-camera]" in wrapper
    assert "--realtime" in wrapper
    assert 'parser.add_argument("--realtime", action="store_true")' in text
    assert "gateway.realtime_sse" in text
    assert "gateway.websocket_camera" in text
    assert "gateway.websocket_cloud" in text
    assert 'ws_smoke("/ws/camera", b"\\xff\\xd8\\xff"' in text
    assert 'ws_smoke("/ws/cloud", b"PCL"' in text


def test_lingtu_doctor_normalizes_structured_active_command_source():
    text = _read("src/diagnostics/field/doctor.py")

    assert "def command_source_name(control):" in text
    assert 'source.get("name") or source.get("source") or source.get("owner") or "none"' in text
    assert "isinstance(source, dict)" in text


def test_lingtu_doctor_non_motion_json_runs_without_motion_side_effects(tmp_path):
    with _gateway_fixture(tmp_path) as (gateway_url, gateway):
        result, harness = _run_lingtu_command(
            tmp_path,
            "doctor --non-motion --json --strict",
            extra_env={"GW": gateway_url, "LINGTU_LIDAR_NETDEV": "lo"},
            active_product="teleop",
        )

    assert result.returncode == 0, result.stdout + result.stderr
    payload = json.loads(result.stdout)
    assert payload["ok"] is True
    preview = next(check for check in payload["checks"] if check["id"] == "gateway.navigation_plan_preview")
    runtime = next(check for check in payload["checks"] if check["id"] == "run.current_plan")
    client_readiness = next(check for check in payload["checks"] if check["id"] == "gateway.client_readiness")
    assert client_readiness["status"] == "pass"
    assert client_readiness["evidence"]["status"] == "degraded"
    assert preview["status"] == "pass"
    assert runtime["status"] == "pass"
    assert runtime["evidence"]["product"] == "teleop"
    required_services = {
        check["id"]
        for check in payload["checks"]
        if check["id"].startswith("service.")
        and check["id"] != "service.unmanaged_known"
    }
    assert required_services == {
        "service.lingtu-driver.service",
        "service.lingtu-nav-dds.service",
        "service.lingtu.service",
    }
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
            active_product="teleop",
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


def test_lingtu_doctor_strict_fails_closed_without_current_run_plan(tmp_path):
    with _gateway_fixture(tmp_path) as (gateway_url, _gateway):
        result, _harness = _run_lingtu_command(
            tmp_path,
            "doctor --non-motion --json --strict",
            extra_env={"GW": gateway_url, "LINGTU_LIDAR_NETDEV": "lo"},
        )

    assert result.returncode != 0
    payload = json.loads(result.stdout)
    runtime = next(check for check in payload["checks"] if check["id"] == "run.current_plan")
    assert runtime["status"] == "fail"
    assert "cannot read current run record" in runtime["evidence"]["error"]


def test_lingtu_doctor_strict_rejects_current_run_plan_fingerprint_mismatch(tmp_path):
    with _gateway_fixture(tmp_path) as (gateway_url, _gateway):
        result, _harness = _run_lingtu_command(
            tmp_path,
            "doctor --non-motion --json --strict",
            extra_env={"GW": gateway_url, "LINGTU_LIDAR_NETDEV": "lo"},
            active_product="teleop",
            active_fingerprint="0" * 64,
        )

    assert result.returncode != 0
    payload = json.loads(result.stdout)
    runtime = next(check for check in payload["checks"] if check["id"] == "run.current_plan")
    assert runtime["status"] == "fail"
    assert "fingerprint does not match" in runtime["evidence"]["error"]


def test_lingtu_soak_is_read_only_non_motion_field_verification():
    text = _read("scripts/lingtu")
    impl = _read("scripts/diagnostics/soak.py")
    start = text.index("cmd_soak()")
    end = text.index("# -- Subcommand: log --")
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


def test_lingtu_removed_experiment_tombstones_are_physically_absent():
    text = _read("scripts/lingtu")

    for removed in (
        "cmd_removed_slam_experiment()",
        "cmd_slamcheck()",
        "cmd_slamcompare()",
        "cmd_removed_route_experiment()",
        "cmd_routecheck()",
        "cmd_routecompare()",
        "slamcheck|slam-check",
        "slamcompare|slam-compare",
        "routecheck|route-check",
        "routecompare|route-compare",
        "super-lio-smoke",
        "super_lio_smoke",
    ):
        assert removed not in text


def test_static_localization_probe_ignores_optional_super_lio_processes():
    text = _read("scripts/diagnostics/static_localization_probe.py").lower()

    assert "super_lio" not in text
    assert "super-lio" not in text
    assert "relocation_node" not in text


def test_lingtu_gateway_map_request_uses_only_the_scoped_client_credential():
    text = _read("scripts/lingtu")
    helper = text.split("gateway_map_request() {", 1)[1].split(
        "\n}\n\nmap_save_post()", 1
    )[0]
    cmd_map = text.split("cmd_map() {", 1)[1].split("cmd_nav() {", 1)[0]
    map_check = cmd_map.split("check)", 1)[1].split("restore)", 1)[0]
    map_restore = cmd_map.split("restore)", 1)[1].split("*) echo", 1)[0]

    assert text.count("gateway_map_request() {") == 1
    assert text.count("gateway_map_request") == 4
    assert "${LINGTU_MAP_CLIENT_ENV_FILE:-/etc/lingtu/map-client.env}" in helper
    assert "mapfile -t credential_lines" in helper
    assert "^LINGTU_MAP_API_KEY=([A-Za-z0-9_-]{64,})$" in helper
    assert '[ -L "$credential_file" ]' in helper
    assert '[ ! -f "$credential_file" ]' in helper
    assert "stat -c '%h'" in helper
    assert "source " not in helper
    assert "LINGTU_API_KEY" not in helper
    assert "gateway.env" not in helper
    assert 'curl --disable --config - "$@"' in helper
    assert '-H "X-API-Key:' not in helper
    assert "gateway_map_request" not in map_check
    assert "gateway_map_request" not in map_restore


def test_lingtu_map_save_uses_dedicated_operation_contract():
    text = _read("scripts/lingtu")
    cmd_map = text.split("cmd_map() {", 1)[1].split("cmd_nav() {", 1)[0]
    map_list = cmd_map.split("list|ls)", 1)[1].split("check)", 1)[0]
    map_restore = cmd_map.split("restore)", 1)[1].split("*) echo", 1)[0]

    assert "Saving durable map artifacts" in cmd_map
    assert 'cmd_map_save "$name"' in cmd_map
    assert '"$GW/api/v1/map/save"' in text
    assert '"$GW/api/v1/maps/operations/' in text
    assert "request_id" in text
    assert 'gateway_map_request -fsS "$GW/api/v1/slam/maps"' in map_list
    assert "maps_json=$(" in map_list
    assert "formatted_maps=$(" in map_list
    assert "map list returned invalid JSON" in map_list
    assert "-X POST" not in map_list
    assert "-d " not in map_list
    assert "restore_ok=" in map_restore
    assert "for rebuild_action in build_octomap build_occupancy" in map_restore
    assert '"$GW/api/v1/maps/$name/$rebuild_action"' in map_restore
    assert '\\"action\\"' not in map_restore
    assert "rebuild_ok=" in map_restore


def test_lingtu_map_list_uses_read_only_map_collection(tmp_path):
    result, harness = _run_lingtu_command(tmp_path, "map list")

    assert result.returncode == 0, result.stdout + result.stderr
    calls_text = (harness["root"] / "calls.log").read_text(encoding="utf-8")
    calls = calls_text.splitlines()
    assert "map_list:GET:" in calls
    assert (
        "curl:GET:http://fake-gateway:5050/api/v1/slam/maps:"
        "x_api_key_headers=1:key_in_argv=0"
    ) in calls
    assert MAP_CLIENT_API_KEY not in result.stdout + result.stderr + calls_text


@pytest.mark.parametrize("command", ["map list", "map save missing_key_map"])
def test_lingtu_real_map_requests_require_a_scoped_key_before_curl(tmp_path, command):
    missing_env = tmp_path / "missing-map-client.env"

    result, harness = _run_lingtu_command(
        tmp_path,
        command,
        extra_env={"LINGTU_MAP_CLIENT_ENV_FILE": _wsl_path(missing_env)},
    )

    assert result.returncode != 0
    assert "credential file is required in env=real" in result.stderr
    calls_path = harness["root"] / "calls.log"
    assert not calls_path.exists() or "curl:" not in calls_path.read_text(encoding="utf-8")


@pytest.mark.parametrize(
    "contents",
    [
        f"LINGTU_MAP_API_KEY={MAP_CLIENT_API_KEY}\nLINGTU_MAP_API_KEY={MAP_CLIENT_API_KEY}\n",
        f"LINGTU_MAP_API_KEY={MAP_CLIENT_API_KEY}\nEXTRA_ASSIGNMENT=forbidden\n",
    ],
)
def test_lingtu_real_map_requests_reject_extra_credential_lines_before_curl(
    tmp_path, contents
):
    client_env = tmp_path / "malformed-map-client.env"
    client_env.write_text(contents, encoding="utf-8", newline="\n")

    result, harness = _run_lingtu_command(
        tmp_path,
        "map list",
        extra_env={"LINGTU_MAP_CLIENT_ENV_FILE": _wsl_path(client_env)},
    )

    assert result.returncode != 0
    assert "malformed map client credential file" in result.stderr
    calls_path = harness["root"] / "calls.log"
    assert not calls_path.exists() or "curl:" not in calls_path.read_text(encoding="utf-8")


def test_lingtu_map_client_credential_file_is_parsed_without_sourcing(tmp_path):
    client_env = tmp_path / "shell-syntax-map-client.env"
    sentinel = tmp_path / "credential-was-sourced"
    client_env.write_text(
        f"LINGTU_MAP_API_KEY=$(touch {shlex.quote(_wsl_path(sentinel))})\n",
        encoding="utf-8",
        newline="\n",
    )

    result, harness = _run_lingtu_command(
        tmp_path,
        "map list",
        extra_env={"LINGTU_MAP_CLIENT_ENV_FILE": _wsl_path(client_env)},
    )

    assert result.returncode != 0
    assert not sentinel.exists()
    calls_path = harness["root"] / "calls.log"
    assert not calls_path.exists() or "curl:" not in calls_path.read_text(encoding="utf-8")


def test_lingtu_real_map_requests_reject_non_regular_credential_path_before_curl(tmp_path):
    client_env = tmp_path / "map-client-directory"
    client_env.mkdir()

    result, harness = _run_lingtu_command(
        tmp_path,
        "map list",
        extra_env={"LINGTU_MAP_CLIENT_ENV_FILE": _wsl_path(client_env)},
    )

    assert result.returncode != 0
    assert "unsafe map client credential file" in result.stderr
    calls_path = harness["root"] / "calls.log"
    assert not calls_path.exists() or "curl:" not in calls_path.read_text(encoding="utf-8")


def test_lingtu_sim_map_list_allows_a_missing_scoped_key_without_a_header(tmp_path):
    missing_env = tmp_path / "missing-sim-map-client.env"

    result, harness = _run_lingtu_command(
        tmp_path,
        "--env sim map list",
        extra_env={"LINGTU_MAP_CLIENT_ENV_FILE": _wsl_path(missing_env)},
    )

    assert result.returncode == 0, result.stdout + result.stderr
    calls = (harness["root"] / "calls.log").read_text(encoding="utf-8")
    assert (
        "curl:GET:http://fake-gateway:5050/api/v1/slam/maps:"
        "x_api_key_headers=0:key_in_argv=0"
    ) in calls
    assert MAP_CLIENT_API_KEY not in result.stdout + result.stderr + calls


def test_lingtu_sim_rejects_an_existing_malformed_scoped_credential(tmp_path):
    client_env = tmp_path / "malformed-sim-map-client.env"
    client_env.write_text(
        f"LINGTU_MAP_API_KEY={MAP_CLIENT_API_KEY}\nEXTRA_ASSIGNMENT=forbidden\n",
        encoding="utf-8",
        newline="\n",
    )

    result, harness = _run_lingtu_command(
        tmp_path,
        "--env sim map list",
        extra_env={"LINGTU_MAP_CLIENT_ENV_FILE": _wsl_path(client_env)},
    )

    assert result.returncode != 0
    assert "malformed map client credential file" in result.stderr
    calls_path = harness["root"] / "calls.log"
    assert not calls_path.exists() or "curl:" not in calls_path.read_text(encoding="utf-8")


def test_lingtu_map_list_propagates_http_failure(tmp_path):
    missing_env = tmp_path / "missing-sim-map-client.env"

    result, harness = _run_lingtu_command(
        tmp_path,
        "--env sim map list",
        extra_env={
            "FAKE_MAP_LIST_HTTP_ERROR": "1",
            "LINGTU_MAP_CLIENT_ENV_FILE": _wsl_path(missing_env),
        },
    )

    assert result.returncode != 0
    assert "map list request failed" in result.stderr
    calls = (harness["root"] / "calls.log").read_text(encoding="utf-8")
    assert "map_list:GET:" in calls


@pytest.mark.parametrize(
    "command",
    ["map check demo --goal 1 2 0", "map restore demo"],
)
def test_lingtu_real_map_admin_flows_fail_before_curl(tmp_path, command):
    result, harness = _run_lingtu_command(tmp_path, command)

    assert result.returncode != 0
    assert "not migrated to typed native map control" in result.stderr
    calls_path = harness["root"] / "calls.log"
    assert not calls_path.exists() or "curl:" not in calls_path.read_text(encoding="utf-8")


def test_lingtu_map_start_delegates_to_full_product_mode_switch():
    text = _read("scripts/lingtu")
    cmd_map = text.split("cmd_map() {", 1)[1].split("cmd_nav() {", 1)[0]
    map_start = cmd_map.split("start)", 1)[1].split("save)", 1)[0]

    assert "cmd_mode switch map" in map_start
    assert "slam_dds_set_mode" not in map_start
    assert "/api/v1/session/start" not in map_start


def test_lingtu_explore_start_activates_product_without_starting_task(tmp_path):
    fake_python = tmp_path / "fake_control_python.sh"
    control_calls = tmp_path / "control_calls.txt"
    _write_executable(
        fake_python,
        f"""
        #!/usr/bin/env bash
        printf '%s\n' "$*" >> {shlex.quote(_wsl_path(control_calls))}
        printf '{{"ok":true,"status":"active"}}\n'
        """,
    )

    result, harness = _run_lingtu_command(
        tmp_path,
        "explore start",
        extra_env={"LINGTU_PYTHON": _wsl_path(fake_python)},
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert (
        "-m lingtu.control switch explore --env real --json"
        in control_calls.read_text(encoding="utf-8")
    )
    calls_path = harness["root"] / "calls.log"
    if calls_path.exists():
        assert "/api/v1/explore/start" not in calls_path.read_text(encoding="utf-8")


def test_lingtu_explore_start_forwards_saved_map_to_product_control(tmp_path):
    fake_python = tmp_path / "fake_control_python.sh"
    control_calls = tmp_path / "control_calls.txt"
    _write_executable(
        fake_python,
        f"""
        #!/usr/bin/env bash
        printf '%s\n' "$*" >> {shlex.quote(_wsl_path(control_calls))}
        printf '{{"ok":true,"status":"active"}}\n'
        """,
    )

    result, harness = _run_lingtu_command(
        tmp_path,
        "explore start --map demo",
        extra_env={"LINGTU_PYTHON": _wsl_path(fake_python)},
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert (
        "-m lingtu.control switch explore --env real --map demo --json"
        in control_calls.read_text(encoding="utf-8")
    )
    calls_path = harness["root"] / "calls.log"
    if calls_path.exists():
        assert "/api/v1/explore/start" not in calls_path.read_text(encoding="utf-8")


def test_lingtu_explore_task_start_is_the_explicit_gateway_motion_command(tmp_path):
    operator_key = "o" * 64

    result, harness = _run_lingtu_command(
        tmp_path,
        "explore task start",
        extra_env={"LINGTU_API_KEY": operator_key},
    )

    assert result.returncode == 0, result.stdout + result.stderr
    payload = json.loads(result.stdout)
    assert payload["accepted"] is True
    assert payload["exploration_run_id"] == "01K1M9S4FX27T8XMY6QJNBAV3W"
    calls = (harness["root"] / "calls.log").read_text(encoding="utf-8")
    assert "explore_task_start" in calls
    assert (
        "curl:POST:http://fake-gateway:5050/api/v1/explore/start:"
        "x_api_key_headers=1:product_session_headers=0:key_in_argv=0"
    ) in calls
    assert "lingtu.control" not in calls


def test_lingtu_explore_task_start_uses_local_product_session_in_real_env(tmp_path):
    session_root = tmp_path / "session-root"
    session_root.mkdir()
    (session_root / "session.env").write_text(
        'LINGTU_PRODUCT_SESSION_ID="session-token-1234"\n',
        encoding="utf-8",
    )
    result, harness = _run_lingtu_command(
        tmp_path,
        "explore task start",
        extra_env={
            "GW": "http://127.0.0.1:5050",
            "LINGTU_SESSION_ROOT": _wsl_path(session_root),
        },
    )

    assert result.returncode == 0, result.stdout + result.stderr
    calls = (harness["root"] / "calls.log").read_text(encoding="utf-8")
    assert (
        "curl:POST:http://127.0.0.1:5050/api/v1/explore/start:"
        "x_api_key_headers=0:product_session_headers=1:key_in_argv=0"
    ) in calls


def test_lingtu_explore_task_start_requires_admin_key_for_remote_gateway(tmp_path):
    result, harness = _run_lingtu_command(
        tmp_path,
        "explore task start",
        extra_env={"LINGTU_PRODUCT_SESSION_ID": "session-token-1234"},
    )

    assert result.returncode != 0
    assert "LINGTU_API_KEY is required for remote" in result.stderr
    calls_path = harness["root"] / "calls.log"
    if calls_path.exists():
        assert "/api/v1/explore/start" not in calls_path.read_text(encoding="utf-8")


def test_lingtu_explore_stop_uses_canonical_product_control_stop(tmp_path):
    fake_python = tmp_path / "fake_control_python.sh"
    control_calls = tmp_path / "control_calls.txt"
    _write_executable(
        fake_python,
        f"""
        #!/usr/bin/env bash
        printf '%s\n' "$*" >> {shlex.quote(_wsl_path(control_calls))}
        if [ "$*" = "-m lingtu.control stop --expected-product explore --json" ]; then
            printf '{{"ok":true,"status":"stopped"}}\n'
            exit 0
        fi
        exit 2
        """,
    )

    result, harness = _run_lingtu_command(
        tmp_path,
        "explore stop",
        extra_env={
            "LINGTU_PYTHON": _wsl_path(fake_python),
            "FAKE_CURRENT_PRODUCT": "explore",
        },
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert control_calls.read_text(encoding="utf-8").splitlines() == [
        "-m lingtu.control stop --expected-product explore --json",
    ]
    calls_path = harness["root"] / "calls.log"
    if calls_path.exists():
        assert "/api/v1/explore/stop" not in calls_path.read_text(encoding="utf-8")


def test_lingtu_explore_stop_refuses_to_stop_navigation_product(tmp_path):
    fake_python = tmp_path / "fake_control_python.sh"
    control_calls = tmp_path / "control_calls.txt"
    _write_executable(
        fake_python,
        f"""
        #!/usr/bin/env bash
        printf '%s\n' "$*" >> {shlex.quote(_wsl_path(control_calls))}
        if [ "$*" = "-m lingtu.control stop --expected-product explore --json" ]; then
            printf '{{"ok":false,"reason":"current_product_mismatch","error":"expected current Product explore, found nav"}}\n' >&2
            exit 3
            exit 0
        fi
        exit 2
        """,
    )

    result, _ = _run_lingtu_command(
        tmp_path,
        "explore stop",
        extra_env={
            "LINGTU_PYTHON": _wsl_path(fake_python),
            "FAKE_CURRENT_PRODUCT": "nav",
        },
    )

    assert result.returncode != 0
    assert "current_product_mismatch" in result.stderr
    assert control_calls.read_text(encoding="utf-8").splitlines() == [
        "-m lingtu.control stop --expected-product explore --json"
    ]


def test_lingtu_explore_status_reads_existing_gateway_state(tmp_path):
    operator_key = "o" * 64

    result, harness = _run_lingtu_command(
        tmp_path,
        "explore status",
        extra_env={"LINGTU_API_KEY": operator_key},
    )

    assert result.returncode == 0, result.stdout + result.stderr
    payload = json.loads(result.stdout)
    assert payload == {
        "schema_version": 1,
        "available": True,
        "active": False,
        "state": "IDLE",
    }
    calls = (harness["root"] / "calls.log").read_text(encoding="utf-8")
    assert "explore_status" in calls
    assert (
        "curl:GET:http://fake-gateway:5050/api/v1/explore/status:"
        "x_api_key_headers=1:product_session_headers=0:key_in_argv=0"
    ) in calls


def test_lingtu_map_and_nav_stop_use_product_control_session_stop():
    text = _read("scripts/lingtu")
    cmd_map = text.split("cmd_map() {", 1)[1].split("cmd_nav() {", 1)[0]
    cmd_nav = text.split("cmd_nav() {", 1)[1].split("\ncmd_loc() {", 1)[0]
    map_stop = cmd_map.split("end|stop)", 1)[1].split("list|ls)", 1)[0]
    nav_stop = cmd_nav.split("stop|end)", 1)[1].split(
        "relocalize|init-pose|initial-pose)", 1
    )[0]

    assert "cmd_mode stop-session" in map_stop
    assert "cmd_mode stop-session" in nav_stop
    assert "/api/v1/session/end" not in map_stop
    assert "/api/v1/session/end" not in nav_stop


def test_lingtu_map_save_executes_dedicated_operation_contract(tmp_path):
    request_id = "cli_save_contract_001"
    result, harness = _run_lingtu_command(
        tmp_path,
        "map save shell_map",
        extra_env={"LINGTU_MAP_SAVE_REQUEST_ID": request_id},
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert "Saving durable map artifacts" in result.stdout
    calls = (harness["root"] / "calls.log").read_text(encoding="utf-8")
    request_log = (
        "curl:POST:http://fake-gateway:5050/api/v1/map/save:"
        "x_api_key_headers=1:key_in_argv=0"
    )
    assert request_log in calls
    assert MAP_CLIENT_API_KEY not in result.stdout + result.stderr + calls
    save_call = next(line.removeprefix("save:") for line in calls.splitlines() if line.startswith("save:"))
    assert json.loads(save_call) == {"name": "shell_map", "request_id": request_id}
    assert "maps:" not in calls

def test_lingtu_map_save_generates_canonical_ulid_request_id():
    script = _read("scripts/lingtu")
    generator = script.split("request_id=$(python3 - <<'PY'\n", 1)[1].split(
        "\nPY\n", 1
    )[0]
    result = subprocess.run(
        [sys.executable, "-c", generator],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stdout + result.stderr
    request_id = result.stdout.strip()
    alphabet = set("0123456789ABCDEFGHJKMNPQRSTVWXYZ")

    assert len(request_id) == 26
    assert request_id[0] in "01234567"
    assert set(request_id) <= alphabet


def test_lingtu_map_save_polls_202_to_terminal_and_replays_same_request(tmp_path):
    request_id = "cli_save_async_001"
    result, harness = _run_lingtu_command(
        tmp_path,
        "map save async_map",
        extra_env={
            "FAKE_MAP_SAVE_MODE": "async_success",
            "LINGTU_MAP_SAVE_REQUEST_ID": request_id,
            "LINGTU_MAP_SAVE_POLL_INTERVAL_SEC": "0.01",
        },
    )

    assert result.returncode == 0, result.stdout + result.stderr
    calls = (harness["root"] / "calls.log").read_text(encoding="utf-8").splitlines()
    save_payloads = [json.loads(line.removeprefix("save:")) for line in calls if line.startswith("save:")]
    assert save_payloads == [
        {"name": "async_map", "request_id": request_id},
        {"name": "async_map", "request_id": request_id},
    ]
    assert f"save_status:http://fake-gateway:5050/api/v1/maps/operations/{request_id}" in calls
    assert (
        f"curl:GET:http://fake-gateway:5050/api/v1/maps/operations/{request_id}:"
        "x_api_key_headers=1:key_in_argv=0"
    ) in calls
    map_curls = [
        line
        for line in calls
        if line.startswith("curl:")
        and ("/api/v1/map/save:" in line or "/api/v1/maps/operations/" in line)
    ]
    assert len(map_curls) == 3
    assert all("x_api_key_headers=1:key_in_argv=0" in line for line in map_curls)
    assert MAP_CLIENT_API_KEY not in result.stdout + result.stderr + "\n".join(calls)


@pytest.mark.parametrize(
    ("mode", "failure_text"),
    [
        pytest.param("network_error", "map save request failed", id="network-error"),
        pytest.param("http_error", "map save request failed", id="http-error"),
        pytest.param("bad_json", "invalid JSON", id="bad-json"),
        pytest.param(
            "missing_identity",
            "did not return one stable operation_id",
            id="missing-identity",
        ),
        pytest.param("identity_mismatch", "(IDENTITY_MISMATCH)", id="identity-mismatch"),
        pytest.param("status_missing_identity", "(MISSING_IDENTITY)", id="status-missing-identity"),
        pytest.param("status_identity_mismatch", "(IDENTITY_MISMATCH)", id="status-identity-mismatch"),
        pytest.param("replay_missing_identity", "(MISSING_IDENTITY)", id="replay-missing-identity"),
        pytest.param("replay_identity_mismatch", "(IDENTITY_MISMATCH)", id="replay-identity-mismatch"),
        pytest.param("unknown_state", "(UNKNOWN_STATE)", id="unknown-state"),
        pytest.param("query_failed", "(QUERY_FAILED)", id="query-failed"),
        pytest.param("succeeded_not_ready", "(NOT_READY)", id="succeeded-not-ready"),
        pytest.param("submit_running_ok_false", "(FAILED)", id="submit-running-ok-false"),
        pytest.param("replay_running_ok_false", "(FAILED)", id="replay-running-ok-false"),
        pytest.param("failed", "(FAILED)", id="failed"),
        pytest.param("cancelled", "(CANCELLED)", id="cancelled"),
        pytest.param("partial", "(PARTIAL)", id="partial"),
        pytest.param(
            "compatibility_degraded",
            "(COMPATIBILITY_DEGRADED)",
            id="compatibility-degraded",
        ),
    ],
)
def test_lingtu_map_save_fails_closed_for_non_ready_outcomes(
    tmp_path,
    mode,
    failure_text,
):
    result, _harness = _run_lingtu_command(
        tmp_path,
        "map save rejected_map",
        extra_env={
            "FAKE_MAP_SAVE_MODE": mode,
            "LINGTU_MAP_SAVE_REQUEST_ID": f"cli_save_{mode}",
            "LINGTU_MAP_SAVE_POLL_INTERVAL_SEC": "0.01",
            "LINGTU_MAP_SAVE_TIMEOUT_SEC": "1",
        },
    )

    assert result.returncode != 0
    assert failure_text in result.stdout + result.stderr


def test_lingtu_map_save_times_out_while_durable_operation_is_running(tmp_path):
    request_id = "cli_save_timeout_001"
    result, harness = _run_lingtu_command(
        tmp_path,
        "map save slow_map",
        extra_env={
            "FAKE_MAP_SAVE_MODE": "timeout",
            "LINGTU_MAP_SAVE_REQUEST_ID": request_id,
            "LINGTU_MAP_SAVE_TIMEOUT_SEC": "1",
            "LINGTU_MAP_SAVE_POLL_INTERVAL_SEC": "0.05",
        },
    )

    assert result.returncode != 0
    assert "timed out waiting for durable map save" in result.stderr
    calls = (harness["root"] / "calls.log").read_text(encoding="utf-8")
    assert f"save_status:http://fake-gateway:5050/api/v1/maps/operations/{request_id}" in calls


def test_lingtu_map_restore_rebuilds_derived_artifacts(tmp_path):
    result, harness = _run_lingtu_command(tmp_path, "--env sim map restore demo")

    assert result.returncode == 0, result.stdout + result.stderr
    assert "Rebuilding octomap for demo" in result.stdout
    assert "Rebuilding occupancy for demo" in result.stdout
    calls = (harness["root"] / "calls.log").read_text(encoding="utf-8")
    restore = 'restore:{"name":"demo"}'
    octomap = "build_octomap:POST:demo:"
    occupancy = "build_occupancy:POST:demo:"
    assert restore in calls
    assert octomap in calls
    assert occupancy in calls
    assert calls.index(restore) < calls.index(octomap) < calls.index(occupancy)
    map_curls = [line for line in calls.splitlines() if line.startswith("curl:")]
    assert len(map_curls) == 3
    assert all("x_api_key_headers=0:key_in_argv=0" in line for line in map_curls)
    assert MAP_CLIENT_API_KEY not in result.stdout + result.stderr + calls


def test_p0_mapping_uses_product_control_nav_transition():
    text = _read("docs/07-testing/p0_mapping.sh")

    assert 'scripts/lingtu" map save "$MAP_NAME"' in text
    assert 'scripts/lingtu" nav start "$MAP_NAME"' in text
    assert "ProductControl transaction boundary" in text
    assert "metadata.json" in text
    assert "octomap.ot" in text
    assert "tomogram.pickle" not in text
    assert "http://localhost:5050/api/v1/map/activate" not in text
    assert "/api/v1/session/start" not in text
    assert "LINGTU_SLAM_MAP" not in text
    assert "json_payload action=set_active" not in text


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


def test_lingtu_nav_relocalize_defaults_to_zero_initial_pose_via_product_control():
    text = _read("scripts/lingtu")
    nav = text.split("cmd_nav() {", 1)[1].split("\ncmd_loc() {", 1)[0]

    assert 'local map="${2:-}"; local x="${3:-0}"; local y="${4:-0}"; local yaw="${5:-0}"' in text
    assert 'cmd_mode switch nav --map "$map" --initial-pose "$x" "$y" "$yaw"' in nav
    assert "/api/v1/slam/relocalize" not in nav


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
    assert "Process-level stop is not supported" in result.stderr
    calls = harness["root"] / "calls.log"
    if calls.exists():
        text = calls.read_text(encoding="utf-8")
        assert "sudo:systemctl stop robot-unrelated.service" not in text
        assert "systemctl:stop robot-unrelated.service" not in text


@pytest.mark.parametrize(
    "command",
    [
        "slamcheck super_lio",
        "slamcompare --map demo",
        "routecheck --map demo --goal 1 2 0",
        "routecompare --map demo --goal 1 2 0 --allow-motion",
        "super-lio-smoke",
        "super-lio-routecheck --map demo --goal 1 2 0",
    ],
)
def test_lingtu_removed_experiment_names_are_plain_unknown_commands(tmp_path, command):
    result, harness = _run_lingtu_command(tmp_path, command)

    assert result.returncode == 1
    assert f"Unknown command: {command.split()[0]}" in result.stdout
    assert "was removed:" not in result.stdout + result.stderr
    calls = harness["root"] / "calls.log"
    if calls.exists():
        text = calls.read_text(encoding="utf-8")
        assert "switch:" not in text
        assert "session_start:" not in text
        assert "goal:" not in text


def test_thunder_service_installer_has_no_s100p_ros2_dispatch():
    thunder_install = _read("scripts/deploy/thunder/install_services.sh")

    assert "../s100p/install_services.sh" not in thunder_install
    assert "ros-compat|legacy)" not in thunder_install
    assert "LINGTU_ENABLE_LEGACY_ROS2_SERVICES" not in thunder_install
    assert 'cp "${SCRIPT_DIR}/ros2-env.sh"' not in thunder_install


def test_lingtu_cli_service_mutations_use_active_product_control():
    script = _read("scripts/lingtu")

    assert "lingtu_control reapply" in script
    assert 'lingtu_control restart --process "$target"' in script
    assert "lingtu_control stop" in script
    assert "svc_process() {" not in script
    assert "svc_restart_localization_chain()" not in script
    assert "svc_force_stop_unit" not in script
    assert "svc_wait_gateway_ready" not in script


def test_explore_endpoint_exit_always_requests_native_motion_stop():
    unit = _read("scripts/deploy/thunder/lingtu-explore-dds.service")

    assert (
        "ExecStopPost=/opt/lingtu/current/build/nav_endpoint/"
        "lingtu_nav_control stop explore_endpoint_exit"
    ) in unit
    assert "TimeoutStopSec=30" in unit


def test_field_host_uses_a_durable_explore_run_journal():
    unit = _read("scripts/deploy/thunder/lingtu.service")

    assert (
        "Environment=LINGTU_EXPLORE_RUN_JOURNAL="
        "/home/sunrise/data/lingtu/task_journal/explore_runs.json"
    ) in unit


@pytest.mark.parametrize(
    ("target", "product", "unit"),
    (
        ("nav", "nav", "lingtu-nav-dds.service"),
        ("explore", "explore", "lingtu-explore-dds.service"),
    ),
)
def test_lingtu_svc_restart_accepts_run_plan_logical_processes(
    tmp_path, target, product, unit
):
    result, _ = _run_lingtu_command(
        tmp_path,
        f"svc restart {target} --dry-run",
        active_product=product,
    )

    assert result.returncode == 0, result.stdout + result.stderr
    payload = json.loads(result.stdout)
    assert payload["action"] == "restart"
    assert payload["planned"] == [unit]


@pytest.mark.parametrize(
    "target",
    (
        "lingtu-nav-dds.service",
        "fastlio2",
        "localization",
        "nav_dds",
    ),
)
def test_lingtu_svc_restart_rejects_non_logical_process_names(tmp_path, target):
    result, _ = _run_lingtu_command(
        tmp_path,
        f"svc restart {target} --dry-run",
        active_product="nav",
    )

    assert result.returncode == 2
    assert (
        "lingtu svc restart "
        "[lidar|slam|maps|traversability|nav|driver|camera|explore|host|all]"
        in result.stderr
    )


@pytest.mark.parametrize("action", ("start", "apply"))
def test_lingtu_svc_rejects_removed_lifecycle_aliases(tmp_path, action):
    result, _ = _run_lingtu_command(tmp_path, f"svc {action}")

    assert result.returncode == 1
    assert (
        "Usage: lingtu svc "
        "[status|restart <logical-process>|reapply|stop all]"
        in result.stdout
    )


def test_lingtu_rejects_removed_service_command_alias(tmp_path):
    result, _ = _run_lingtu_command(tmp_path, "service status")

    assert result.returncode == 1
    assert "Unknown command: service" in result.stdout


def test_lingtu_nav_start_delegates_to_full_product_mode_switch():
    script = _read("scripts/lingtu")
    nav_start = script.split("cmd_nav() {", 1)[1].split("\n        stop|end)", 1)[0]

    assert 'cmd_mode switch nav --map "$map"' in nav_start
    assert "slam_dds_set_mode" not in nav_start
    assert "/api/v1/session/start" not in nav_start
    assert "resolve_relocation_map_name" not in nav_start


def test_lingtu_svc_status_defaults_to_native_services():
    script = _read("scripts/lingtu")
    docs = _read("docs/04-deployment/lingtu_cli.md")
    status_case = script.split("cmd_svc() {", 1)[1].split("\n        restart)", 1)[0]
    default_status = status_case.split("status)", 1)[1]

    assert "lingtu-livox-dds.service" in default_status
    assert "lingtu-slam-dds.service" in default_status
    assert "lingtu-nav-dds.service" in default_status
    assert "lingtu.service" in default_status
    assert "robot-lidar.service" not in default_status
    assert "robot-fastlio2.service" not in default_status
    assert "robot-localizer.service" not in default_status
    assert "lingtu_control restart --process" in script
    assert "status-legacy" not in script
    assert "legacy-status" not in script
    assert "warn_legacy_services" not in script
    assert "lingtu svc status-legacy" not in docs


def test_lingtu_svc_status_output_lists_only_current_rows(tmp_path):
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


def test_lingtu_svc_rejects_removed_legacy_status_subcommand(tmp_path):
    result, _ = _run_lingtu_command(tmp_path, "svc status-legacy")

    assert result.returncode == 1
    assert "Usage: lingtu svc [status|restart <logical-process>|reapply|stop all]" in result.stdout
    assert "robot-lidar.service" not in result.stdout
    assert "super_lio" not in result.stdout
