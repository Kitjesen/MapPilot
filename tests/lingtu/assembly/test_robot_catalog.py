from pathlib import Path

import yaml

from lingtu.assembly.compiler import compile_run_plan
from lingtu.assembly.products import resolve_product_host_runtime
from runtime.config import load_config

REPO_ROOT = Path(__file__).resolve().parents[3]
ROBOT_ROOT = REPO_ROOT / "config" / "robots"


def _load(relative: str) -> dict:
    return yaml.safe_load((ROBOT_ROOT / relative).read_text(encoding="utf-8"))


def test_robot_catalog_is_company_then_model() -> None:
    expected = {
        "doso/thunder_v4": {
            "sensors": {
                "mid360": {
                    "fastlio2_config": "sensors/mid360_fastlio2.yaml",
                    "extrinsic_profile": "thunder_v4_mid360",
                }
            },
        },
        "unitree/go2": {
            "sensors": {
                "mid360": {
                    "fastlio2_config": "sensors/mid360_fastlio2.yaml",
                    "extrinsic_profile": "go2_mid360",
                }
            },
        },
    }

    for model_id, runtime in expected.items():
        model_dir = ROBOT_ROOT / model_id
        model = _load(f"{model_id}/model.yaml")

        assert model == runtime
        assert not {"real", "sim"}.intersection(model)
        mid360_config = model_dir / model["sensors"]["mid360"]["fastlio2_config"]
        assert mid360_config.is_file()


def test_thunder_v4_has_real_and_sim_inputs() -> None:
    model_dir = ROBOT_ROOT / "doso" / "thunder_v4"
    config = load_config(str(model_dir / "robot.yaml"))

    assert config.driver.backend == "doso"
    assert config.driver.target == "192.168.66.12:13145"
    assert config.driver.network_interface == ""
    assert config.lidar.lidar_ip == "192.168.1.178"
    assert config.lidar.network_interface == "eth1"
    assert (REPO_ROOT / "sim" / "sessions" / "products" / "doso" / "thunder_v4" / "default.yaml").is_file()

    plans = []
    for env, env_config in (("real", None), ("sim", {"backend": "mujoco"})):
        resolved = resolve_product_host_runtime(
            "nav",
            env,
            robot="doso/thunder_v4",
            env_config=env_config,
        )
        plans.append(
            compile_run_plan(
                resolved.product,
                resolved.env,
                robot="doso/thunder_v4",
                env_config=env_config,
            )
        )

    assert [(plan.env, plan.robot, plan.process_control) for plan in plans] == [
        ("real", "doso/thunder_v4", "systemd"),
        ("sim", "doso/thunder_v4", "subprocess"),
    ]
