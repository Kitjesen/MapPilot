"""Policy selection must keep existing baselines separate from CTS exports."""

from pathlib import Path

import pytest
from sim.catalog import CatalogResolver
from sim.scripts.mujoco.stair_policy_probe import POLICIES, ROOT, resolve_policy


def test_packaged_aliases_preserve_baselines_and_select_cts_separately():
    assert resolve_policy("4998") == POLICIES / "policy_4998.onnx"
    assert resolve_policy("1119") == POLICIES / "policy_1119.onnx"
    assert resolve_policy("cts_moe") == (
        ROOT / "sim/packages/controllers/doso/thunder_v4/cts_moe/policy/policy_cts_moe_v39_61128.onnx"
    )


def test_explicit_model_path_is_not_reinterpreted_as_a_packaged_number(tmp_path):
    model = tmp_path / "custom_model.onnx"
    assert resolve_policy(str(model)) == model
    assert resolve_policy("./custom_model.onnx") == Path("custom_model.onnx").resolve()


@pytest.mark.parametrize("session", ["session.yaml", "descent.yaml"])
def test_stair_sessions_use_v39_without_switching_models(session):
    resolved = CatalogResolver.from_repository(ROOT).resolve(
        ROOT / "sim/sessions/examples/thunderv4_stairs" / session
    )
    controller = resolved.control_plan["controllers"][0]
    assert controller["adapter"]["plugin"] == "thunderv4_cts_moe"
    policy = controller["policy"]
    assert policy["artifact"].endswith("policy_cts_moe_v39_61128.onnx")
    for direction in ("stairs_up", "stairs_down"):
        assert policy["profiles"][direction] == {
            "artifact": policy["artifact"], "manifest": policy["manifest"],
        }
