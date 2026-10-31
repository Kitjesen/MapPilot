"""Development defaults and formal plans must agree on controller selection."""

from pathlib import Path
from types import SimpleNamespace

import pytest
from sim.catalog import resolve_robot_default_policy
from sim.compat.engine.core.robot import RobotConfig

ROOT = Path(__file__).resolve().parents[2]


def test_development_entrypoints_follow_robot_default_controller():
    from sim.scripts.mujoco import continuous_walk, native_dds_sensors, stair_policy_probe

    from drivers.sim.mujoco.driver import MujocoDriverModule

    policy = resolve_robot_default_policy(ROOT)
    assert policy["artifact"].name == "policy_cts_moe_v39_61128.onnx"
    assert policy["manifest"].name == "policy_v39_manifest.json"
    assert Path(RobotConfig.default_thunder_v4().policy_onnx) == policy["artifact"]
    assert Path(MujocoDriverModule(drive_mode="policy")._policy_path) == policy["artifact"]
    assert native_dds_sensors._resolve_policy_path_for_drive("policy", "") == policy["artifact"]
    assert continuous_walk.POLICY == policy["artifact"]
    assert stair_policy_probe.resolve_policy("cts_moe") == policy["artifact"]


def test_explicit_development_policy_does_not_resolve_default(monkeypatch, tmp_path):
    from sim.scripts.mujoco import native_dds_sensors

    from drivers.sim.mujoco import driver

    def unexpected_default(_root):
        raise AssertionError("an explicit policy must not resolve authoring defaults")

    monkeypatch.setattr(driver, "resolve_robot_default_policy", unexpected_default)
    monkeypatch.setattr(native_dds_sensors, "resolve_robot_default_policy", unexpected_default)
    explicit = tmp_path / "comparison.onnx"
    explicit.write_bytes(b"explicit comparison model")
    assert Path(driver.MujocoDriverModule(policy_path=str(explicit), drive_mode="policy")._policy_path) == explicit
    assert native_dds_sensors._resolve_policy_path_for_drive("policy", str(explicit)) == explicit
    assert native_dds_sensors._resolve_policy_path_for_drive("kinematic", "") is None


def test_formal_acceptance_consumes_runplan_policy_and_rejects_override():
    from sim.scripts.mujoco.native_navigation_acceptance import _bind_manifest_policy_to_run_plan

    artifact = "sim/resolved-controller.onnx"
    plan = SimpleNamespace(simulation={"control_plan": {"controllers": [{"policy": {"artifact": artifact}}]}})
    manifest = {"paths": {"sensor_runner": "sensor.py"}}
    _bind_manifest_policy_to_run_plan(manifest, plan)
    assert manifest["paths"] == {"sensor_runner": "sensor.py", "policy": str(ROOT / artifact)}
    with pytest.raises(ValueError, match="must come from RunPlan"):
        _bind_manifest_policy_to_run_plan({"paths": {"policy": "old-controller.onnx"}}, plan)


def test_explicit_cts_engine_keeps_embedded_timing_without_resolving_defaults(monkeypatch):
    pytest.importorskip("mujoco")
    pytest.importorskip("onnxruntime")
    from drivers.sim.mujoco.runtime import build_engine

    policy = resolve_robot_default_policy(ROOT)["artifact"]

    def unexpected_default(cls):
        raise AssertionError("runtime must use its resolved policy")

    monkeypatch.setattr(RobotConfig, "default_thunder_v4", classmethod(unexpected_default))
    engine = build_engine(
        world=ROOT / "sim/packages/worlds/open_field/physics/open_field.xml",
        drive_mode="policy", start=[0.0, 0.0, 0.0], mujoco_memory="",
        policy_path=policy, initial_keyframe="v4_nominal_stand",
        mid360_pattern=None, mid360_samples_per_frame=16, require_product_lidar_backend=False,
    )
    try:
        assert engine.active_policy_path == str(policy)
        assert engine.dt == pytest.approx(0.005)
        assert engine.control_dt == pytest.approx(0.02)
    finally:
        engine.close()
