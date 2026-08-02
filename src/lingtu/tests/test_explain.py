from __future__ import annotations

import json
from pathlib import Path

from lingtu.explain import explain_product, explain_status
from lingtu.run_plan import CURRENT_RUN_SCHEMA
from lingtu.product_switch import session_explanation


def test_inspect_explains_parameter_source_priority() -> None:
    payload = explain_product(
        "explore",
        env="real",
        session_overrides={"segment.max_distance_m": 3.0},
        process_environment={},
    )

    parameter = payload["parameters"]["parameters"]["segment.max_distance_m"]
    assert parameter == {
        "value": 3.0,
        "source": "session_override",
        "env_key": "LINGTU_NAV_SEGMENT_MAX_DISTANCE_M",
    }


def test_status_reports_standby_without_current_run(tmp_path: Path) -> None:
    payload = explain_status(
        state_dir=tmp_path,
        process_environment={},
        systemd_show=lambda _unit: {},
    )

    assert payload["state"] == "standby"
    assert payload["reason"] == "no_current_run"


def test_status_reports_actual_ephemeral_session(
    tmp_path: Path,
) -> None:
    from lingtu.control import ProductControl

    plan = ProductControl(env="real", process_env={}).resolve("explore")
    run_plan_path = plan.write(tmp_path / "plan.json")
    (tmp_path / "current.json").write_text(
        json.dumps(
            {
                "schema_version": CURRENT_RUN_SCHEMA,
                "product": plan.product,
                "env": plan.env,
                "run_plan_path": str(run_plan_path),
                "fingerprint": plan.fingerprint,
                "committed_at": 1.0,
            }
        ),
        encoding="utf-8",
    )
    runtime_root = tmp_path / "run" / "lingtu"
    session = session_explanation(plan, runtime_root=runtime_root)
    path = Path(session["session_file"])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        'LINGTU_PRODUCT_SESSION_ID="session-a"\n'
        f'LINGTU_RUN_PLAN_FINGERPRINT="{plan.fingerprint}"\n',
        encoding="utf-8",
    )

    payload = explain_status(
        state_dir=tmp_path,
        process_environment={"LINGTU_SESSION_ROOT": str(runtime_root)},
        systemd_show=lambda _unit: {
            "ActiveState": "active",
            "SubState": "running",
            "MainPID": "42",
        },
    )

    assert payload["state"] == "active"
    assert payload["session_id"] == "session-a"
    assert payload["session_consistent"] is True
    assert all(
        row["session_file_present"]
        for row in payload["processes"]
        if row["lifecycle"] == "mode"
    )
