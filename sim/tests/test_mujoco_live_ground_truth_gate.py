from __future__ import annotations

from pathlib import Path

import pytest


def _gate_args(tmp_path: Path, **overrides):
    args = {
        "world": tmp_path / "world.xml",
        "duration": 0.1,
        "drive_vx": 0.0,
        "drive_vy": 0.0,
        "drive_wz": 0.0,
        "n_rays": 16,
        "start": None,
        "mujoco_memory": "",
        "mid360_pattern": None,
        "mid360_samples_per_frame": 16,
        "lidar_backend": "mujoco_lidar",
        "mujoco_lidar_backend": "cpu",
        "allow_legacy_lidar_fallback": False,
        "startup_sleep": 0.0,
        "settle_sleep": 0.0,
        "work_dir": tmp_path,
        "backend_profile": "test",
        "drive_mode": "kinematic",
    }
    args.update(overrides)
    return args


def test_ground_truth_nav_source_bypasses_removed_portable_lio(tmp_path, monkeypatch):
    from sim.scripts.mujoco import live_gate as mujoco_live_gate

    seen = {}

    def fake_ground_truth_gate(**cfg):
        seen.update(cfg)
        return {"ok": True, "nav_data_source": "mujoco_ground_truth"}

    monkeypatch.setattr(
        mujoco_live_gate,
        "_run_mujoco_ground_truth_exploration_gate",
        fake_ground_truth_gate,
    )

    report = mujoco_live_gate.run_gate(
        **_gate_args(
            tmp_path,
            nav_data_source="mujoco_ground_truth",
            run_lingtu_frontier=True,
            require_goal_arrival=True,
            goal_arrival_threshold=0.3,
        )
    )

    assert report["ok"] is True
    assert report["nav_data_source"] == "mujoco_ground_truth"
    assert seen["nav_data_source"] == "mujoco_ground_truth"
    assert seen["run_lingtu_frontier"] is True
    assert seen["require_goal_arrival"] is True
    assert seen["goal_arrival_threshold"] == 0.3


def test_fastlio_nav_source_still_requires_real_localization_endpoint(tmp_path):
    from sim.scripts.mujoco import live_gate as mujoco_live_gate

    with pytest.raises(ValueError, match="portable_lio was removed"):
        mujoco_live_gate.run_gate(**_gate_args(tmp_path, nav_data_source="fastlio2"))


def test_ground_truth_engine_init_failure_is_explicit(tmp_path, monkeypatch):
    from sim.scripts.mujoco import live_gate as mujoco_live_gate

    def fail_build_engine(**_cfg):
        raise RuntimeError("missing mujoco_lidar")

    monkeypatch.setattr(mujoco_live_gate, "_build_engine", fail_build_engine)

    report = mujoco_live_gate.run_gate(
        **_gate_args(tmp_path, nav_data_source="mujoco_ground_truth")
    )

    assert report["ok"] is False
    assert report["schema_version"] == "lingtu.mujoco_ground_truth_live_gate.v1"
    assert "mujoco_engine_init_failed" in report["remaining_gaps"]
    assert "missing mujoco_lidar" in report["lidar_backend"]["error"]
