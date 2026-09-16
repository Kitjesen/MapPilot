from __future__ import annotations

import os
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

from lingtu.assembly.compiler import compile_run_plan
from lingtu.real.backend import FieldBackend

ROOT = Path(__file__).resolve().parents[3]
DEPLOY = ROOT / "scripts" / "deploy" / "thunder"


@pytest.mark.parametrize(
    ("product", "local_planner"),
    [("teleop_avoid", None), ("teleop_avoid", "cmu"), ("map", None), ("teleop", None)],
    ids=["default-scan", "cmu", "map-without-path-library", "teleop-without-path-library"],
)
@pytest.mark.parametrize("missing_geometry", [False, True], ids=["complete", "missing-geometry"])
def test_nav_runner_consumes_compiled_product_session(
    tmp_path: Path, product: str, local_planner: str | None, missing_geometry: bool
) -> None:
    bash = shutil.which("bash")
    if os.name == "nt":
        git = shutil.which("git")
        bash = str(Path(git).resolve().parents[1] / "bin" / "bash.exe") if git else None
    if not bash or not Path(bash).is_file():
        pytest.skip("The navigation wrapper requires Bash")

    plan = compile_run_plan(product, "real", robot="unitree/go2", local_planner=local_planner)
    session_id = "nav-runner-test"
    plan_path = tmp_path / f"plan-{session_id}.json"
    plan.write(plan_path)
    session = FieldBackend(environment={})._session_environment(
        run_plan_path=plan_path,
        plan=plan,
        native_environment=plan.native_process_environment,
        slam_mode="mapping",
        map_identity=None,
        product_session_id=session_id,
    )
    assert "LINGTU_TELEOP_SLOW_DISTANCE_M" not in session
    assert "LINGTU_TELEOP_STOP_DISTANCE_M" not in session

    # Relocate deployment files while retaining the real session and role guard.
    runtime_env = DEPLOY / "runtime-env.sh"
    runner = (DEPLOY / "run_nav_dds.sh").read_text(encoding="utf-8")
    for deployed, local in (
        ("/opt/lingtu/config/thunder-runtime-env.sh", runtime_env),
        (
            "/opt/lingtu/current/scripts/deploy/thunder/require_product_session.sh",
            DEPLOY / "require_product_session.sh",
        ),
    ):
        runner = runner.replace(deployed, shlex.quote(local.as_posix()))
    script = tmp_path / "run_nav_dds.sh"
    script.write_text(runner, encoding="utf-8", newline="\n")
    endpoint = tmp_path / "navd"
    endpoint.write_text(
        '#!/usr/bin/env bash\nprintf "nav-started:%s\\n" "$LINGTU_NAV_LOCAL_PLANNER_BACKEND"\n',
        encoding="utf-8",
        newline="\n",
    )
    endpoint.chmod(0o755)
    session.update(
        {
            "LINGTU_RUN_PLAN": plan_path.as_posix(),
            "LINGTU_SESSION_ROOT": tmp_path.as_posix(),
            "LINGTU_REPO": ROOT.as_posix(),
            "LINGTU_CONFIG_DIR": tmp_path.as_posix(),
            "LINGTU_RUNTIME_ENV_FILE": runtime_env.as_posix(),
            "LINGTU_PYTHON": Path(sys.executable).as_posix(),
            "LINGTU_NAV_DDS_BIN": endpoint.as_posix(),
        }
    )
    if local_planner == "cmu":
        paths = tmp_path / Path(session["LINGTU_LOCAL_PLANNER_PATHS"]).name
        paths.mkdir()
        for asset in ("startPaths.ply", "pathList.ply", "paths.ply", "correspondences.txt", "search_radius.txt"):
            (paths / asset).write_text("test asset\n", encoding="utf-8")
        session["LINGTU_LOCAL_PLANNER_PATHS"] = paths.as_posix()
    if missing_geometry:
        del session["LINGTU_NAV_SENSOR_OFFSET_Z_M"]
    environment = {key: value for key, value in os.environ.items() if not key.startswith("LINGTU_")}
    completed = subprocess.run(
        [bash, script.as_posix()],
        env={**environment, **session},
        capture_output=True,
        text=True,
        timeout=15,
        check=False,
    )

    if missing_geometry:
        assert completed.returncode != 0
        assert "Product session is missing LINGTU_NAV_SENSOR_OFFSET_Z_M" in completed.stderr
        assert "nav-started" not in completed.stdout
    else:
        assert completed.returncode == 0, completed.stderr
        assert completed.stdout.strip() == f"nav-started:{plan.native_nav['local_planner']}"
