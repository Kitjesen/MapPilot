import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "sim" / "scripts" / "run_dimos_linux_closure.sh"


def _script_text() -> str:
    return SCRIPT.read_text(encoding="utf-8")


def _usable_bash() -> str | None:
    bash = shutil.which("bash")
    if not bash:
        return None
    try:
        result = subprocess.run(
            [bash, "-lc", "exit 0"],
            text=True,
            encoding="utf-8",
            errors="replace",
            capture_output=True,
            timeout=5,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    return bash if result.returncode == 0 else None


def _require_bash() -> str:
    bash = _usable_bash()
    if bash is None:
        pytest.skip("a usable bash is required for shell contract tests")
    return bash


def _green_host_preflight_payload(*, generated_at: float | None = None) -> dict[str, object]:
    return {
        "schema_version": "lingtu.server_sim_host_preflight.v1",
        "execution_mode": "host_preflight_only",
        "generated_at": time.time() if generated_at is None else generated_at,
        "ok": True,
        "blocked_gates": [],
        "required_gate_sequence": [
            "gateway_runtime_acceptance",
            "routecheck_preflight",
            "blocked_route_replan_preflight",
            "navigation_replay_deviation",
            "large_terrain",
            "native_pct_mujoco",
            "dynamic_obstacle_local_planner",
            "fastlio2_dynamic_inspection",
            "moving_obstacle_sweep",
            "large_loop_closure",
            "gazebo_runtime",
            "saved_map_relocalize",
            "pct_saved_map_navigation",
        ],
    }


def test_dimos_linux_closure_script_is_safe_by_default():
    text = _script_text()

    assert text.startswith("#!/usr/bin/env bash")
    assert "EXECUTE=0" in text
    assert "--execute" in text
    assert "--dry-run" in text
    assert "--allow-host-blocked-run-missing" not in text
    assert 'if [[ "$(uname -s)" != "Linux" ]]' in text
    assert "ROS_DOMAIN_ID must be in 1..231" in text
    assert "source /opt/ros/humble/setup.bash" in text
    assert "set +u" in text
    assert "PYTHONPATH=src:." not in text
    assert "export PYTHONPATH=" in text
    assert (
        "source \"$ROOT/install/setup.bash\"\n"
        "    set -u\n"
        "  fi\n"
        "  # Some setup files clear ROS_DOMAIN_ID; restore the isolated domain"
    ) in text
    assert "run_preflight()" in text
    assert "eval " not in text
    assert "require_preflight_green()" in text
    assert "dimos_host_preflight_guard.py" in text
    assert 'python3 sim/scripts/dimos_host_preflight_guard.py "$HOST_PREFLIGHT_OUT"' in text


def test_dimos_host_preflight_guard_rejects_red_report_with_setup_plan(
    tmp_path: Path,
):
    report = tmp_path / "host_preflight.json"
    report.write_text(
        """
{
  "ok": false,
  "blocked_gates": ["native_pct_mujoco"],
  "host_setup_plan": {
    "failed_checks": [
      {
        "check": "pct_native",
        "gates": ["native_pct_mujoco"],
        "blockers": ["PCT native runtime unavailable"],
        "diagnostic_commands": ["python sim/scripts/pct_runtime_preflight.py"]
      }
    ]
  }
}
""".strip(),
        encoding="utf-8",
    )

    result = subprocess.run(
        [
            sys.executable,
            "sim/scripts/dimos_host_preflight_guard.py",
            str(report),
        ],
        cwd=ROOT,
        text=True,
        encoding="utf-8",
        errors="replace",
        capture_output=True,
        check=False,
    )

    assert result.returncode == 3
    assert "Host preflight is not green" in result.stderr
    assert "Blocked gates: native_pct_mujoco" in result.stderr
    assert "Failed host checks:" in result.stderr
    assert "- pct_native: gates=native_pct_mujoco" in result.stderr
    assert "blocker: PCT native runtime unavailable" in result.stderr
    assert "diagnostic: python sim/scripts/pct_runtime_preflight.py" in result.stderr


def test_dimos_host_preflight_guard_accepts_green_report(tmp_path: Path):
    report = tmp_path / "host_preflight.json"
    report.write_text(json.dumps(_green_host_preflight_payload()), encoding="utf-8")

    result = subprocess.run(
        [
            sys.executable,
            "sim/scripts/dimos_host_preflight_guard.py",
            str(report),
        ],
        cwd=ROOT,
        text=True,
        encoding="utf-8",
        errors="replace",
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0
    assert result.stderr == ""


def test_dimos_host_preflight_guard_rejects_stale_manual_green_report(tmp_path: Path):
    report = tmp_path / "host_preflight.json"
    report.write_text(
        json.dumps(_green_host_preflight_payload(generated_at=0.0)),
        encoding="utf-8",
    )
    old_mtime = time.time() - 90_000
    os.utime(report, (old_mtime, old_mtime))

    result = subprocess.run(
        [
            sys.executable,
            "sim/scripts/dimos_host_preflight_guard.py",
            str(report),
        ],
        cwd=ROOT,
        text=True,
        encoding="utf-8",
        errors="replace",
        capture_output=True,
        check=False,
    )

    assert result.returncode == 3
    assert "host_preflight_file_age_s" in result.stderr
    assert "host_preflight_generated_age_s" in result.stderr


def test_dimos_linux_closure_script_orders_preflight_before_runtime_gates():
    text = _script_text()

    host_preflight = text.index('run_preflight "$HOST_PREFLIGHT_OUT"')
    guard = text.rindex("\nrequire_preflight_green")
    run_missing = text.index("run_runtime_closure python3")
    gap_report = text.index("run_gap_report python3")

    assert host_preflight < guard < run_missing < gap_report
    assert "--max-report-age-s 86400" in text
    assert "--preset dimos_benchmark --required-only" in text


def test_dimos_linux_closure_readme_documents_execution_contract():
    readme = (ROOT / "sim" / "scripts" / "README.md").read_text(encoding="utf-8")

    assert "run_dimos_linux_closure.sh" in readme
    assert "Default is `--dry-run`" in readme
    assert "refuses non-Linux `--execute`" in readme
    assert "always stops on red preflight" in readme
    assert "--allow-host-blocked-run-missing" not in readme


def test_dimos_linux_closure_execute_rejects_non_linux_before_runtime():
    bash = _require_bash()

    bash_env = ROOT / "artifacts" / "test_bash_env_nonlinux.sh"
    bash_env.parent.mkdir(parents=True, exist_ok=True)
    bash_env.write_bytes(
        b'uname() {\n'
        b'  if [[ "${1:-}" == "-s" ]]; then\n'
        b"    echo Darwin\n"
        b"  else\n"
        b'    command uname "$@"\n'
        b"  fi\n"
        b"}\n",
    )
    try:
        result = subprocess.run(
            [
                bash,
                "-lc",
                (
                    "BASH_ENV=artifacts/test_bash_env_nonlinux.sh "
                    '"$BASH" sim/scripts/run_dimos_linux_closure.sh '
                    "--execute --ros-domain-id 77"
                ),
            ],
            cwd=ROOT,
            text=True,
            encoding="utf-8",
            errors="replace",
            capture_output=True,
            check=False,
        )
    finally:
        bash_env.unlink(missing_ok=True)

    assert result.returncode == 2
    assert "require a Linux simulation host" in result.stderr
    assert "/opt/ros/humble/setup.bash" not in result.stderr
    assert "--host-preflight" not in result.stdout
    assert "--run-missing" not in result.stdout


def test_dimos_linux_closure_execute_stops_after_red_host_preflight():
    bash = _require_bash()

    bash_env = ROOT / "artifacts" / "test_bash_env_red_preflight.sh"
    call_log = ROOT / "artifacts" / "test_red_preflight_python_calls.log"
    run_missing_log = ROOT / "artifacts" / "test_red_preflight_run_missing.log"
    host_report = (
        ROOT / "artifacts" / "server_sim_closure" / "test_red_host_preflight.json"
    )
    for path in (call_log, run_missing_log, host_report):
        path.unlink(missing_ok=True)
    bash_env.parent.mkdir(parents=True, exist_ok=True)
    bash_env.write_bytes(
        b'python3() {\n'
        b'  printf "%s\\n" "$*" >> artifacts/test_red_preflight_python_calls.log\n'
        b'  args=" $* "\n'
        b'  if [[ "$args" == *"sim/scripts/server_sim_closure.py"* && "$args" == *" --host-preflight "* ]]; then\n'
        b'      out=""\n'
        b'      while [[ $# -gt 0 ]]; do\n'
        b'        if [[ "$1" == "--json-out" ]]; then\n'
        b'          shift\n'
        b'          out="$1"\n'
        b'        fi\n'
        b'        shift || true\n'
        b'      done\n'
        b'      mkdir -p "$(dirname "$out")"\n'
        b'      printf \'{"schema_version": "lingtu.server_sim_host_preflight.v1", "execution_mode": "host_preflight_only", "generated_at": 1, "ok": false, "blocked_gates": ["native_pct_mujoco"], "required_gate_sequence": ["gateway_runtime_acceptance"], "host_setup_plan": {"failed_checks": [{"check": "pct_native"}]}}\\n\' > "$out"\n'
        b'      return 0\n'
        b'  fi\n'
        b'  if [[ "$args" == *"sim/scripts/dimos_host_preflight_guard.py"* ]]; then\n'
        b'      return 3\n'
        b'  fi\n'
        b'  if [[ "$args" == *" --run-missing "* ]]; then\n'
        b'      printf "run_missing\\n" >> artifacts/test_red_preflight_run_missing.log\n'
        b'      return 0\n'
        b'  fi\n'
        b'  return 0\n'
        b'}\n',
    )
    try:
        result = subprocess.run(
            [
                bash,
                "-lc",
                (
                    "BASH_ENV=artifacts/test_bash_env_red_preflight.sh "
                    '"$BASH" sim/scripts/run_dimos_linux_closure.sh '
                    "--execute --ros-domain-id 77 "
                    "--host-preflight-out artifacts/server_sim_closure/test_red_host_preflight.json "
                    "--summary-out artifacts/server_sim_closure/test_red_summary.json "
                    "--gap-out artifacts/server_sim_closure/test_red_gap.json"
                ),
            ],
            cwd=ROOT,
            text=True,
            encoding="utf-8",
            errors="replace",
            capture_output=True,
            check=False,
        )
    finally:
        bash_env.unlink(missing_ok=True)
        call_log.unlink(missing_ok=True)
        run_missing_log.unlink(missing_ok=True)
        host_report.unlink(missing_ok=True)

    assert result.returncode == 3
    assert "--host-preflight --json-out artifacts/server_sim_closure/test_red_host_preflight.json.tmp." in result.stdout
    assert "--run-missing" not in result.stdout
    assert "--include-dataflow" not in result.stdout
    assert not run_missing_log.exists()


def test_dimos_linux_closure_execute_does_not_reuse_old_preflight_after_command_failure():
    bash = _require_bash()

    bash_env = ROOT / "artifacts" / "test_bash_env_failed_preflight.sh"
    run_missing_log = ROOT / "artifacts" / "test_failed_preflight_run_missing.log"
    host_report = (
        ROOT / "artifacts" / "server_sim_closure" / "test_stale_green_host_preflight.json"
    )
    host_report.parent.mkdir(parents=True, exist_ok=True)
    host_report.write_text(json.dumps(_green_host_preflight_payload()), encoding="utf-8")
    for path in (run_missing_log,):
        path.unlink(missing_ok=True)
    bash_env.parent.mkdir(parents=True, exist_ok=True)
    bash_env.write_bytes(
        b'python3() {\n'
        b'  args=" $* "\n'
        b'  if [[ "$args" == *"sim/scripts/server_sim_closure.py"* && "$args" == *" --host-preflight "* ]]; then\n'
        b'      return 9\n'
        b'  fi\n'
        b'  if [[ "$args" == *" --run-missing "* ]]; then\n'
        b'      printf "run_missing\\n" >> artifacts/test_failed_preflight_run_missing.log\n'
        b'      return 0\n'
        b'  fi\n'
        b'  return 0\n'
        b'}\n',
    )
    try:
        result = subprocess.run(
            [
                bash,
                "-lc",
                (
                    "BASH_ENV=artifacts/test_bash_env_failed_preflight.sh "
                    '"$BASH" sim/scripts/run_dimos_linux_closure.sh '
                    "--execute --ros-domain-id 77 "
                    "--host-preflight-out artifacts/server_sim_closure/test_stale_green_host_preflight.json "
                    "--summary-out artifacts/server_sim_closure/test_failed_summary.json "
                    "--gap-out artifacts/server_sim_closure/test_failed_gap.json"
                ),
            ],
            cwd=ROOT,
            text=True,
            encoding="utf-8",
            errors="replace",
            capture_output=True,
            check=False,
        )
    finally:
        bash_env.unlink(missing_ok=True)
        run_missing_log.unlink(missing_ok=True)
        host_report.unlink(missing_ok=True)

    assert result.returncode == 9
    assert "refusing to reuse stale" in result.stderr
    assert "--run-missing" not in result.stdout
    assert not run_missing_log.exists()


def test_dimos_linux_closure_execute_writes_gap_after_runtime_red():
    bash = _require_bash()

    bash_env = ROOT / "artifacts" / "test_bash_env_runtime_red.sh"
    call_log = ROOT / "artifacts" / "test_runtime_red_python_calls.log"
    host_report = (
        ROOT / "artifacts" / "server_sim_closure" / "test_runtime_red_host.json"
    )
    summary_report = (
        ROOT / "artifacts" / "server_sim_closure" / "test_runtime_red_summary.json"
    )
    gap_report = ROOT / "artifacts" / "server_sim_closure" / "test_runtime_red_gap.json"
    for path in (call_log, host_report, summary_report, gap_report):
        path.unlink(missing_ok=True)
    bash_env.parent.mkdir(parents=True, exist_ok=True)
    bash_env.write_bytes(
        b'python3() {\n'
        b'  printf "%s\\n" "$*" >> artifacts/test_runtime_red_python_calls.log\n'
        b'  args=" $* "\n'
        b'  out=""\n'
        b'  while [[ $# -gt 0 ]]; do\n'
        b'    if [[ "$1" == "--json-out" ]]; then\n'
        b'      shift\n'
        b'      out="$1"\n'
        b'    fi\n'
        b'    shift || true\n'
        b'  done\n'
        b'  if [[ "$args" == *"sim/scripts/server_sim_closure.py"* && "$args" == *" --host-preflight "* ]]; then\n'
        b'      mkdir -p "$(dirname "$out")"\n'
        b'      printf \'{"schema_version": "lingtu.server_sim_host_preflight.v1", "execution_mode": "host_preflight_only", "generated_at": 1, "ok": true, "blocked_gates": [], "required_gate_sequence": ["large_loop_closure"]}\\n\' > "$out"\n'
        b'      return 0\n'
        b'  fi\n'
        b'  if [[ "$args" == *"sim/scripts/dimos_host_preflight_guard.py"* ]]; then\n'
        b'      return 0\n'
        b'  fi\n'
        b'  if [[ "$args" == *"sim/scripts/server_sim_closure.py"* && "$args" == *" --run-missing "* ]]; then\n'
        b'      mkdir -p "$(dirname "$out")"\n'
        b'      printf \'{"ok": false, "failed_required_gates": ["large_loop_closure"]}\\n\' > "$out"\n'
        b'      return 7\n'
        b'  fi\n'
        b'  if [[ "$args" == *"sim/scripts/dimos_gap_report.py"* ]]; then\n'
        b'      mkdir -p "$(dirname "$out")"\n'
        b'      printf \'{"lingtu_readiness": {"ok": false}, "claim_allowed": false}\\n\' > "$out"\n'
        b'      return 1\n'
        b'  fi\n'
        b'  return 0\n'
        b'}\n',
    )
    try:
        result = subprocess.run(
            [
                bash,
                "-lc",
                (
                    "BASH_ENV=artifacts/test_bash_env_runtime_red.sh "
                    '"$BASH" sim/scripts/run_dimos_linux_closure.sh '
                    "--execute --ros-domain-id 77 "
                    "--host-preflight-out artifacts/server_sim_closure/test_runtime_red_host.json "
                    "--summary-out artifacts/server_sim_closure/test_runtime_red_summary.json "
                    "--gap-out artifacts/server_sim_closure/test_runtime_red_gap.json"
                ),
            ],
            cwd=ROOT,
            text=True,
            encoding="utf-8",
            errors="replace",
            capture_output=True,
            check=False,
        )
    finally:
        bash_env.unlink(missing_ok=True)
        call_log.unlink(missing_ok=True)

    assert result.returncode == 7
    assert summary_report.exists()
    assert gap_report.exists()
    assert "--run-missing --strict" in result.stdout
    assert "--include-dataflow --host-preflight-report" in result.stdout
    run_missing = result.stdout.index("--run-missing --strict")
    gap_report_cmd = result.stdout.index("dimos_gap_report.py")
    assert run_missing < gap_report_cmd

    host_report.unlink(missing_ok=True)
    summary_report.unlink(missing_ok=True)
    gap_report.unlink(missing_ok=True)


def test_dimos_linux_closure_execute_rejects_missing_gap_output_after_runtime_red():
    bash = _require_bash()

    bash_env = ROOT / "artifacts" / "test_bash_env_missing_gap.sh"
    host_report = (
        ROOT / "artifacts" / "server_sim_closure" / "test_missing_gap_host.json"
    )
    summary_report = (
        ROOT / "artifacts" / "server_sim_closure" / "test_missing_gap_summary.json"
    )
    gap_report = ROOT / "artifacts" / "server_sim_closure" / "test_missing_gap.json"
    for path in (host_report, summary_report, gap_report):
        path.unlink(missing_ok=True)
    bash_env.parent.mkdir(parents=True, exist_ok=True)
    bash_env.write_bytes(
        b'python3() {\n'
        b'  args=" $* "\n'
        b'  out=""\n'
        b'  while [[ $# -gt 0 ]]; do\n'
        b'    if [[ "$1" == "--json-out" ]]; then\n'
        b'      shift\n'
        b'      out="$1"\n'
        b'    fi\n'
        b'    shift || true\n'
        b'  done\n'
        b'  if [[ "$args" == *"sim/scripts/server_sim_closure.py"* && "$args" == *" --host-preflight "* ]]; then\n'
        b'      mkdir -p "$(dirname "$out")"\n'
        b'      printf \'{"schema_version": "lingtu.server_sim_host_preflight.v1", "execution_mode": "host_preflight_only", "generated_at": 1, "ok": true, "blocked_gates": [], "required_gate_sequence": ["large_loop_closure"]}\\n\' > "$out"\n'
        b'      return 0\n'
        b'  fi\n'
        b'  if [[ "$args" == *"sim/scripts/dimos_host_preflight_guard.py"* ]]; then\n'
        b'      return 0\n'
        b'  fi\n'
        b'  if [[ "$args" == *"sim/scripts/server_sim_closure.py"* && "$args" == *" --run-missing "* ]]; then\n'
        b'      mkdir -p "$(dirname "$out")"\n'
        b'      printf \'{"ok": false, "failed_required_gates": ["large_loop_closure"]}\\n\' > "$out"\n'
        b'      return 7\n'
        b'  fi\n'
        b'  if [[ "$args" == *"sim/scripts/dimos_gap_report.py"* ]]; then\n'
        b'      return 9\n'
        b'  fi\n'
        b'  return 0\n'
        b'}\n',
    )
    try:
        result = subprocess.run(
            [
                bash,
                "-lc",
                (
                    "BASH_ENV=artifacts/test_bash_env_missing_gap.sh "
                    '"$BASH" sim/scripts/run_dimos_linux_closure.sh '
                    "--execute --ros-domain-id 77 "
                    "--host-preflight-out artifacts/server_sim_closure/test_missing_gap_host.json "
                    "--summary-out artifacts/server_sim_closure/test_missing_gap_summary.json "
                    "--gap-out artifacts/server_sim_closure/test_missing_gap.json"
                ),
            ],
            cwd=ROOT,
            text=True,
            encoding="utf-8",
            errors="replace",
            capture_output=True,
            check=False,
        )
    finally:
        bash_env.unlink(missing_ok=True)

    assert result.returncode == 9
    assert summary_report.exists()
    assert not gap_report.exists()
    assert "refusing to reuse stale gap report" in result.stderr

    host_report.unlink(missing_ok=True)
    summary_report.unlink(missing_ok=True)


def test_dimos_linux_closure_dry_run_prints_target_commands():
    bash = _require_bash()

    result = subprocess.run(
        [
            bash,
            "-lc",
            (
                '"$BASH" sim/scripts/run_dimos_linux_closure.sh --dry-run '
                "--ros-domain-id 77 "
                "--host-preflight-out artifacts/server_sim_closure/test_host.json "
                "--summary-out artifacts/server_sim_closure/test_summary.json "
                "--gap-out artifacts/server_sim_closure/test_gap.json"
            ),
        ],
        cwd=ROOT,
        text=True,
        encoding="utf-8",
        errors="replace",
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "+ export ROS_DOMAIN_ID=77" in result.stdout
    assert "+ source /opt/ros/humble/setup.bash" in result.stdout
    assert "--host-preflight --json-out artifacts/server_sim_closure/test_host.json.tmp." in result.stdout
    assert "--run-missing --strict" in result.stdout
    assert "--skip-host-blocked" not in result.stdout
    assert "--summary artifacts/server_sim_closure/test_summary.json" in result.stdout
    assert "--host-preflight-report artifacts/server_sim_closure/test_host.json" in result.stdout

    host_preflight = result.stdout.index("--host-preflight")
    run_missing = result.stdout.index("--run-missing")
    gap_report = result.stdout.index("dimos_gap_report.py")
    assert host_preflight < run_missing < gap_report
