from __future__ import annotations

import json
import os
import shlex
import shutil
import subprocess
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]


def _bash_binary():
    bash = shutil.which("bash")
    if os.name == "nt":
        git = shutil.which("git")
        bash = str(Path(git).resolve().parents[1] / "bin" / "bash.exe") if git else None
    if not bash or not Path(bash).is_file():
        pytest.skip("The field scripts require Bash")
    return bash


@pytest.mark.parametrize(
    ("api_key", "require_auth", "change_axes", "expected_code"),
    [
        ("route-preview-test-key", True, False, 0),
        (None, False, False, 0),
        (None, True, False, 22),
        ("route-preview-test-key", True, True, 6),
    ],
)
def test_p0_route_preview_auth_and_motion_boundary(
    tmp_path, api_key, require_auth, change_axes, expected_code
):
    bash = _bash_binary()
    calls = []

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args):
            pass

        def do_GET(self):
            self.reply()

        def do_POST(self):
            self.reply()

        def reply(self):
            body = self.rfile.read(int(self.headers.get("Content-Length", "0")))
            calls.append((self.command, self.path, body))
            status = 200
            if require_auth and self.headers.get("X-API-Key") != "route-preview-test-key":
                status, payload = 401, {"error": "unauthorized"}
            elif self.command == "GET" and self.path == "/api/v1/navigation/status":
                after_preview = len(calls) == 3
                payload = {
                    "task": {"state": "IDLE"},
                    "goal_admission": {"state": "ACCEPTING"},
                    "control": {
                        "authority": "OPERATOR" if change_axes and after_preview else "NONE",
                        "resume_required": False,
                    },
                    "motion": {
                        "permission": "CLEAR", "observation": "QUIET",
                        "stop_confirmation": "NOT_REQUESTED",
                    },
                }
            elif self.command == "POST" and self.path == "/api/v1/navigation/plan":
                assert json.loads(body) == {"x": 2.0, "y": 0.0, "z": 0.0, "frame_id": "map"}
                payload = {"feasible": True, "count": 2, "planner": "mock-native-planner"}
            else:
                status, payload = 404, {"error": "unexpected endpoint"}
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps(payload).encode())

    with ThreadingHTTPServer(("127.0.0.1", 0), Handler) as server:
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        log_dir = tmp_path / "logs"
        text = (REPO_ROOT / "scripts/gates/field/p0_route_safety.sh").read_text(encoding="utf-8")
        text = text.replace("http://localhost:5050", f"http://127.0.0.1:{server.server_port}")
        text = text.replace('LOG_DIR="${HOME}/data/nav_logs"', f"LOG_DIR={shlex.quote(log_dir.as_posix())}")
        script = tmp_path / "preview.sh"
        script.write_text(text, encoding="utf-8", newline="\n")
        environment = {**os.environ, "TEST_PYTHON": Path(sys.executable).as_posix(), "NO_PROXY": "127.0.0.1"}
        if api_key is None:
            environment.pop("LINGTU_API_KEY", None)
        else:
            environment["LINGTU_API_KEY"] = api_key
        try:
            result = subprocess.run(
                [bash, "-c", 'python3() { "$TEST_PYTHON" "$@"; }; source "$1" 2.0 0.0',
                 "route-preview-test", script.as_posix()],
                env=environment, capture_output=True, text=True, timeout=20, check=False,
            )
        finally:
            server.shutdown()
            thread.join(timeout=2)

    assert result.returncode == expected_code, result.stdout + result.stderr
    if expected_code == 22:
        assert [(method, path) for method, path, _ in calls] == [("GET", "/api/v1/navigation/status")]
    else:
        assert [(method, path) for method, path, _ in calls] == [
            ("GET", "/api/v1/navigation/status"),
            ("POST", "/api/v1/navigation/plan"),
            ("GET", "/api/v1/navigation/status"),
        ]
    if expected_code == 0:
        assert "PASS - native no-motion route preview is feasible" in result.stdout
    if change_axes:
        assert "navigation state changed during preview" in result.stdout
    logged = "".join(path.read_text(encoding="utf-8") for path in log_dir.glob("*.log"))
    assert "route-preview-test-key" not in result.stdout + result.stderr + logged


@pytest.mark.parametrize(
    ("script_name", "outcome", "confirmation", "expected_code"),
    [
        ("p0_goto.sh", "SUCCESS", "RUN", 0),
        ("p0_goto.sh", "FAILED", "RUN", 3),
        ("p0_goto.sh", "CANCELLED", "RUN", 4),
        ("p0_goto.sh", "SUCCESS", "NO", 3),
        ("p0_explore.sh", "ready", "", 0),
        ("p0_explore.sh", "not-ready", "", 3),
    ],
)
def test_p0_supervised_scripts_authenticate_without_real_motion(
    tmp_path, script_name, outcome, confirmation, expected_code
):
    bash = _bash_binary()
    field_dir = tmp_path / "scripts" / "gates" / "field"
    field_dir.mkdir(parents=True)
    log_dir = tmp_path / "logs"
    for name in (script_name, "p0_route_safety.sh"):
        source = (REPO_ROOT / "scripts" / "gates" / "field" / name).read_text(encoding="utf-8")
        source = source.replace('LOG_DIR="${HOME}/data/nav_logs"', f"LOG_DIR={shlex.quote(log_dir.as_posix())}")
        (field_dir / name).write_text(source, encoding="utf-8", newline="\n")

    # Replace both side-effect boundaries before executing any copied script.
    mock_curl = tmp_path / "mock_curl.py"
    mock_curl.write_text('''import json, os, sys
from pathlib import Path
from urllib.parse import urlparse

args = iter(sys.argv[1:])
method, path, body, headers = "GET", "", None, []
for arg in args:
    if arg in ("-H", "--header"):
        headers.append(next(args))
    elif arg == "-X":
        method = next(args)
    elif arg in ("-d", "--data-binary"):
        body = json.loads(next(args))
    elif arg.startswith("http"):
        path = urlparse(arg).path
auth = [h for h in headers if h.startswith(("X-API-Key:", "Authorization:"))]
key = os.environ["LINGTU_API_KEY"]
if len(auth) != 1 or auth[0] not in ("X-API-Key: " + key, "Authorization: Bearer " + key):
    raise SystemExit(22)
log = Path(os.environ["MOCK_REQUESTS"])
prior = [json.loads(line) for line in log.read_text().splitlines()] if log.exists() else []
with log.open("a") as handle:
    handle.write(json.dumps({"method": method, "path": path, "body": body}) + "\\n")
goal_sent = any(row.get("path") == "/api/v1/goal" for row in prior)
if (method, path) == ("GET", "/api/v1/health"):
    payload = {"has_odom": True}
elif (method, path) == ("GET", "/api/v1/navigation/status"):
    payload = {
        "schema_version": 3,
        "task": {"state": os.environ["MOCK_OUTCOME"] if goal_sent else "IDLE"},
        "goal_admission": {"state": "ACCEPTING"},
        "control": {"authority": "NONE", "resume_required": False},
        "motion": {"permission": "CLEAR", "observation": "QUIET", "stop_confirmation": "NOT_REQUESTED"},
    }
elif (method, path) == ("POST", "/api/v1/navigation/plan"):
    assert body == {"x": 1.25, "y": 0.5, "z": 0.0, "frame_id": "map"}
    payload = {"feasible": True, "count": 2, "planner": "mock-native-planner"}
elif (method, path) == ("POST", "/api/v1/goal"):
    assert body["x"] == 1.25 and body["y"] == 0.5 and body["z"] == 0.0
    assert body["frame_id"] == "map" and body["client_id"] == "p0_goto"
    assert body["request_id"].startswith("p0-goto-")
    payload = {"accepted": True}
elif (method, path) == ("GET", "/api/v1/explore/status"):
    started = any(row.get("path") == "/api/v1/explore/start" for row in prior)
    payload = {"available": True, "can_start": os.environ["MOCK_OUTCOME"] == "ready",
               "backend": "mock-native-explorer", "exploring": started, "frontier_count": 1, "blockers": []}
elif (method, path) == ("POST", "/api/v1/explore/start"):
    assert body == {}
    payload = {"accepted": True}
else:
    raise SystemExit(22)
print(json.dumps(payload))
''', encoding="utf-8")
    (tmp_path / "scripts" / "lingtu").write_text('''#!/usr/bin/env bash
python3 - "$@" <<'PY'
import json, os, sys
with open(os.environ["MOCK_REQUESTS"], "a") as handle:
    handle.write(json.dumps({"method": "ProductControl", "args": sys.argv[1:]}) + "\\n")
PY
''', encoding="utf-8", newline="\n")
    request_log = tmp_path / "requests.jsonl"
    environment = {
        **os.environ,
        "LINGTU_API_KEY": "supervised-script-test-key",
        "TEST_PYTHON": Path(sys.executable).as_posix(),
        "TEST_BASH": Path(bash).as_posix(),
        "MOCK_CURL": mock_curl.as_posix(),
        "MOCK_REQUESTS": request_log.as_posix(),
        "MOCK_OUTCOME": outcome,
        "MOCK_CONFIRMATION": confirmation,
    }
    for name in ("LINGTU_P0_GOAL_X", "LINGTU_P0_GOAL_Y", "LINGTU_P0_GOTO_TIMEOUT"):
        environment.pop(name, None)
    args = ["1.25", "0.5", "10"] if script_name == "p0_goto.sh" else ["1"]
    if script_name == "p0_explore.sh" and outcome == "ready":
        args += ["--map", "fixture-map"]
    result = subprocess.run(
        [bash, "-c", 'python3() { "$TEST_PYTHON" "$@"; }; '
         'curl() { "$TEST_PYTHON" "$MOCK_CURL" "$@"; }; '
         'export -f python3 curl; printf "%s\\n" "$MOCK_CONFIRMATION" | "$TEST_BASH" "$@"',
         "supervised-script-test", (field_dir / script_name).as_posix(), *args],
        env=environment, capture_output=True, text=True,
        timeout=30, check=False,
    )
    assert result.returncode == expected_code, result.stdout + result.stderr
    calls = [json.loads(line) for line in request_log.read_text().splitlines()]
    requests = [(row["method"], row.get("path")) for row in calls]
    if script_name == "p0_goto.sh":
        expected = [
            ("GET", "/api/v1/health"), ("GET", "/api/v1/navigation/status"),
            ("POST", "/api/v1/navigation/plan"), ("GET", "/api/v1/navigation/status"),
        ]
        if confirmation == "RUN":
            expected += [("POST", "/api/v1/goal"), ("GET", "/api/v1/navigation/status")]
            assert f"nav state: ? -> {outcome}" in result.stdout
        else:
            assert "operator did not confirm motion goal" in result.stdout
        assert requests == expected
        assert "Type RUN" in result.stdout
        assert "FAIL - timeout" not in result.stdout
    else:
        assert requests[:3] == [
            ("GET", "/api/v1/health"), ("ProductControl", None),
            ("GET", "/api/v1/explore/status"),
        ]
        assert calls[1]["args"] == (
            ["switch", "explore", "--map", "fixture-map"] if outcome == "ready" else ["switch", "explore"]
        )
        assert calls[-1] == {"method": "ProductControl", "args": ["stop"]}
        if outcome == "ready":
            assert requests[3] == ("POST", "/api/v1/explore/start")
            assert requests[4:-1] and set(requests[4:-1]) == {("GET", "/api/v1/explore/status")}
        else:
            assert len(calls) == 4
            assert "exploration is not ready" in result.stdout
    logs = "".join(path.read_text(encoding="utf-8") for path in log_dir.glob("*.log"))
    assert "supervised-script-test-key" not in result.stdout + result.stderr + logs + request_log.read_text()


def test_p0_scripts_use_current_gateway_contracts():
    goto = (REPO_ROOT / "scripts/gates/field/p0_goto.sh").read_text(
        encoding="utf-8"
    )
    estop = (REPO_ROOT / "scripts/gates/field/p0_estop.sh").read_text(
        encoding="utf-8"
    )
    mapping = (REPO_ROOT / "scripts/gates/field/p0_mapping.sh").read_text(
        encoding="utf-8"
    )
    route_safety = (
        REPO_ROOT / "scripts/gates/field/p0_route_safety.sh"
    ).read_text(encoding="utf-8")
    explore = (REPO_ROOT / "scripts/gates/field/p0_explore.sh").read_text(
        encoding="utf-8"
    )

    assert "/api/v1/navigation/status" in goto
    assert "/api/v1/nav/status" not in goto
    assert 'GOAL_X="${LINGTU_P0_GOAL_X:-${1:-}}"' in goto
    assert 'GOAL_Y="${LINGTU_P0_GOAL_Y:-${2:-}}"' in goto
    assert 'GOAL_X="${1:-2.0}"' not in goto
    assert 'GOAL_Y="${2:-0.0}"' not in goto
    assert "p0_route_safety.sh" in goto
    assert "Type RUN" in goto
    assert '\\"frame_id\\":\\"map\\"' in goto
    assert '\\"client_id\\":\\"p0_goto\\"' in goto
    assert "P0-04 Goto" in goto

    goto_preview_index = goto.index('p0_route_safety.sh" "$GOAL_X" "$GOAL_Y"')
    goto_confirm_index = goto.index('if [[ "$answer" != "RUN" ]]')
    goto_post_index = goto.index("curl -sf -X POST http://localhost:5050/api/v1/goal")
    assert goto_preview_index < goto_confirm_index < goto_post_index

    assert "POST /api/v1/stop" in estop
    assert "GET /api/v1/state" in estop
    assert "PRE_STOP_SPEED" in estop
    assert "cleanup_stop" in estop
    assert "STOP_ON_EXIT=1" in estop
    assert "p0-estop-cleanup" in estop
    assert "current_speed_mps" in estop
    assert "P0-05 E-stop" in estop
    assert "/api/v1/safety/state" not in estop
    assert "curl -sf http://localhost:5050/api/v1/cmd_vel" not in estop

    assert "/api/v1/map/save" in mapping
    assert "ProductControl transaction boundary" in mapping
    assert "metadata.json" in mapping
    assert "occupancy.npz" in mapping
    assert "octomap.ot" in mapping
    assert "/api/v1/map/activate" not in mapping
    assert "/api/v1/session/start" not in mapping
    assert "action=save" not in mapping
    assert "action=set_active" not in mapping
    assert "SAVE_PATH=" not in mapping
    assert "SAVED_MAP_DIR=" in mapping
    assert "resolve_map_root" in mapping
    assert "LINGTU_SLAM_MAP" not in mapping
    assert "NAV_MAP_DIR" in mapping
    assert "/var/lib/lingtu/maps" in mapping
    assert "${MAP_DIR" not in mapping
    assert "~/data/nova/maps" not in mapping
    assert "data/inovxio/data/maps" not in mapping
    assert "/api/v1/navigation/plan" in route_safety
    assert "p0_route_safety" in route_safety
    assert "planner" in route_safety
    assert "path_safety" not in route_safety
    assert 'task.get("state")' in route_safety
    assert 'admission.get("state")' in route_safety
    assert 'control.get("authority")' in route_safety
    assert 'motion.get("permission")' in route_safety
    assert 'motion.get("observation")' in route_safety
    assert 'motion.get("stop_confirmation")' in route_safety
    assert 'or "UNKNOWN"' in route_safety
    assert "active_cmd_source" not in route_safety
    assert "/api/v1/goal" not in route_safety
    assert "/api/v1/cmd_vel" not in route_safety

    assert "curl -sf -X POST http://localhost:5050/api/v1/explore/start" not in explore
    assert "curl -sf -X POST http://localhost:5050/api/v1/explore/stop" not in explore
    assert "explore Product" in explore
    assert "tare_explore" not in explore
    assert "exploring=true" in explore


def test_p0_field_runbook_matches_script_contracts():
    p0_all = (REPO_ROOT / "scripts/gates/field/p0_all.sh").read_text(
        encoding="utf-8"
    )
    p0_explore = (REPO_ROOT / "scripts/gates/field/p0_explore.sh").read_text(
        encoding="utf-8"
    )
    p0_scripts = [
        REPO_ROOT / "scripts/gates/field/p0_all.sh",
        REPO_ROOT / "scripts/gates/field/p0_cold_boot.sh",
        REPO_ROOT / "scripts/gates/field/p0_estop.sh",
        REPO_ROOT / "scripts/gates/field/p0_explore.sh",
        REPO_ROOT / "scripts/gates/field/p0_goto.sh",
        REPO_ROOT / "scripts/gates/field/p0_mapping.sh",
        REPO_ROOT / "scripts/gates/field/p0_route_safety.sh",
    ]

    assert "confirm_after_preview" not in p0_all
    assert 'run_one "P0-03 route safety"' not in p0_all
    assert 'run_one "P0-03/P0-04 route preview + goto"' in p0_all
    assert 'return "$code"' in p0_all
    assert "LINGTU_P0_GOAL_X" in p0_all
    assert "LINGTU_P0_EXPLORE_MAP" in p0_all
    assert "tare_explore" not in p0_all
    assert "mode switch explore" not in p0_all
    assert "mode switch explore" not in p0_explore
    assert "/api/v1/session/start" not in p0_all
    assert "/api/v1/session/end" not in p0_all
    assert "session_start" not in p0_all
    assert "session_end" not in p0_all
    assert "profile switch" not in p0_all
    assert "fastlio2" not in p0_all
    assert "localizer" not in p0_all

    fail_return_index = p0_all.index('return "$code"')
    goto_index = p0_all.index('run_one "P0-03/P0-04 route preview + goto"')
    assert fail_return_index < goto_index

    for script in p0_scripts:
        script_text = script.read_text(encoding="utf-8")
        script_text.encode("ascii")
        if script.name.startswith("p0_"):
            assert "/api/v1/session/start" not in script_text
            assert "/api/v1/session/end" not in script_text
            assert "/api/v1/map/activate" not in script_text
            assert "sudo systemctl" not in script_text
            assert "systemctl stop" not in script_text
            assert "systemctl start" not in script_text
