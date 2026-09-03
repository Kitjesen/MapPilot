from __future__ import annotations

import json
import os
import stat

import pytest

from lingtu.control import ProductControl
from lingtu.run_plan import RunPlan
from runtime.graph import ProcessArtifact, ProcessCommand, ProcessReadiness

pytestmark = pytest.mark.usefixtures("allow_unbuilt_process_artifacts")


def test_run_plan_round_trips_without_re_resolving_runtime_graph(tmp_path) -> None:
    plan = ProductControl(robot="unitree/go2", env="real", process_env={})._resolve("nav")

    path = plan.write(tmp_path / "run-plan.json")

    loaded = RunPlan.load(path)

    assert loaded == plan
    assert loaded.robot == "unitree/go2"
    assert "_robot_model" not in loaded.host_config
    assert set(plan.as_dict()) == {"identity", "launch", "host", "checks"}
    assert plan.as_dict()["launch"]["parameters"] == plan.parameters
    assert "parameter_profile" not in plan.as_dict()["launch"]
    assert "parameter_overrides" not in plan.as_dict()["launch"]


@pytest.mark.parametrize(
    ("env", "robot", "env_config"),
    (
        ("real", "unitree/go2", None),
        ("sim", "doso/thunder_v4", {"backend": "mujoco"}),
    ),
)
def test_run_plan_records_final_parameter_overrides(
    env: str,
    robot: str,
    env_config: dict[str, str] | None,
) -> None:
    plan = ProductControl(
        robot=robot,
        env=env,
        env_config=env_config,
        process_env={},
    )._resolve(
        "explore",
        parameter_overrides={"segment.max_distance_m": 2.5},
    )

    assert plan.parameters["segment.max_distance_m"] == 2.5
    assert plan.native_process_environment["LINGTU_NAV_SEGMENT_MAX_DISTANCE_M"] == "2.5"


def test_run_plan_rejects_the_previous_schema() -> None:
    payload = ProductControl(robot="unitree/go2", env="real", process_env={})._resolve("nav").as_dict()
    payload["identity"]["schema"] = "lingtu.run_plan.v7"

    with pytest.raises(ValueError, match="unsupported RunPlan schema"):
        RunPlan.from_dict(payload)


@pytest.mark.skipif(os.name == "nt", reason="POSIX file modes are not represented by NTFS")
def test_run_plan_is_readable_by_managed_service_users(tmp_path) -> None:
    plan = ProductControl(robot="unitree/go2", env="real", process_env={})._resolve("teleop_avoid")

    path = plan.write(tmp_path / "run-plan.json")

    assert stat.S_IMODE(path.stat().st_mode) == 0o644


def test_run_plan_output_is_deterministic(tmp_path) -> None:
    left = ProductControl(robot="unitree/go2", env="real", process_env={})._resolve("nav")
    right = ProductControl(robot="unitree/go2", env="real", process_env={})._resolve("nav")

    assert left == right
    assert left.write(tmp_path / "left.json").read_bytes() == right.write(
        tmp_path / "right.json"
    ).read_bytes()


def test_run_plan_properties_return_independent_data() -> None:
    plan = ProductControl(robot="unitree/go2", env="real", process_env={})._resolve("nav")
    host_config = plan.host_config
    lifecycle = plan.lifecycle

    host_config["changed"] = True
    lifecycle["slam_mode"] = "changed"

    assert "changed" not in plan.host_config
    assert plan.lifecycle["slam_mode"] == "localization"


def test_run_plan_rejects_unknown_and_legacy_fields() -> None:
    payload = ProductControl(robot="unitree/go2", env="real", process_env={})._resolve("nav").as_dict()
    payload["future_interpreter"] = {}
    with pytest.raises(ValueError, match="unsupported fields"):
        RunPlan.from_dict(payload)

    with pytest.raises(ValueError, match="top level"):
        RunPlan.from_dict(
            {
                "schema_version": "lingtu.run_plan.legacy",
                "product": "nav",
                "env": "real",
            }
        )


def test_run_plan_rejects_legacy_stop_plan_field() -> None:
    payload = ProductControl(robot="unitree/go2", env="real", process_env={})._resolve("nav").as_dict()
    payload["launch"]["stop_plan"] = payload["launch"].pop("stop_before_start")

    with pytest.raises(ValueError, match=r"stop_before_start.*stop_plan"):
        RunPlan.from_dict(payload)


def test_run_plan_rejects_non_string_persisted_values() -> None:
    plan = ProductControl(robot="unitree/go2", env="real", process_env={})._resolve("nav")
    payload = plan.as_dict()
    payload["identity"]["product"] = 7

    with pytest.raises(ValueError):
        RunPlan.from_dict(payload)


def test_run_plan_summary_stays_compact() -> None:
    plan = ProductControl(robot="unitree/go2", env="real", process_env={})._resolve("nav")

    summary = plan.summary()

    assert summary["product"] == "nav"
    assert summary["env"] == "real"
    assert summary["processes"] == [process.name for process in plan.processes]
    assert "host_config" not in summary
    assert "native_process_environment" not in summary
    assert "required_topics" not in json.dumps(plan.as_dict(), sort_keys=True)


def test_simulation_run_plan_does_not_embed_acceptance_tooling() -> None:
    plan = ProductControl(
        robot="doso/thunder_v4",
        env="sim",
        env_config={"backend": "mujoco"},
        process_env={},
    )._resolve("teleop")

    assert "acceptance" not in plan.as_dict()["launch"]
    assert RunPlan.from_dict(plan.as_dict()) == plan


def test_direct_process_contract_round_trips_with_the_run_plan() -> None:
    plan = ProductControl(
        robot="doso/thunder_v4",
        env="sim",
        env_config={"backend": "mujoco"},
        process_env={},
    )._resolve("teleop")

    direct_processes = [process for process in plan.processes if process.manager == "direct"]

    assert direct_processes
    assert all(process.command is not None for process in direct_processes)
    assert RunPlan.from_dict(plan.as_dict()) == plan


def test_direct_command_requires_a_spawn_entry() -> None:
    with pytest.raises(ValueError, match="argv"):
        ProcessCommand(
            argv=(),
            cwd=".",
            env=(),
            artifact=ProcessArtifact("build/nav-runtime"),
            readiness=ProcessReadiness("process"),
        )


def test_direct_command_artifact_must_be_the_spawn_entry() -> None:
    with pytest.raises(ValueError, match="spawn entry"):
        ProcessCommand(
            argv=("wrapper", "build/nav-runtime"),
            cwd=".",
            env=(),
            artifact=ProcessArtifact("build/nav-runtime"),
            readiness=ProcessReadiness("process"),
        )


def test_run_plan_rechecks_a_persisted_direct_command() -> None:
    plan = ProductControl(
        robot="doso/thunder_v4",
        env="sim",
        env_config={"backend": "mujoco"},
        process_env={},
    )._resolve("teleop")
    payload = plan.as_dict()
    catalog = payload["launch"]["process_catalog"]
    direct_name = next(
        process["name"]
        for process in catalog["selected"]
        if process["manager"] == "direct"
    )
    for section in ("selected", "available"):
        process = next(item for item in catalog[section] if item["name"] == direct_name)
        process["command"]["argv"] = ["wrapper", process["command"]["artifact"]["path"]]

    with pytest.raises(ValueError, match="spawn entry"):
        RunPlan.from_dict(payload)
