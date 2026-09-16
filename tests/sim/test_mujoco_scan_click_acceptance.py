from __future__ import annotations

import json
import time
from pathlib import Path
from types import SimpleNamespace

import pytest
from sim.scripts.mujoco import native_navigation_acceptance as nav
from sim.scripts.mujoco import product_acceptance as product

from lingtu.assembly.compiler import compile_run_plan

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "config/acceptance/mujoco/scan_click.json"
SESSION = "a" * 32


@pytest.fixture(scope="module")
def plan():
    return compile_run_plan("nav", "sim", robot="doso/thunder_v4", local_planner="scan",
                            env_config={"backend": "mujoco", "localization": "fastlio2"})


@pytest.fixture
def attached(tmp_path, monkeypatch, plan):
    manifest = nav._load_manifest(MANIFEST)
    manifest["goal"] = manifest["goal_world"] = [5.0, 4.0, 0.0]
    stamp = time.time()
    elapsed = [0.0]
    snapshots = {
        "nav.status.json": {
            "native_product": {"product_session_id": SESSION},
            "stamp_s": stamp,
            "input_gate": {"ready": True},
            "control_loop_health": {"ready": True, "healthy": True},
            "last_local": {"tracking": {"trajectory_id": 1}},
        },
        "slam.status.json": {
            "native_product": {"product_session_id": SESSION},
            "snapshot_written_at_s": stamp, "stamp_s": stamp, "has_odom": True, "state": "TRACKING",
            "odom_prior_enabled": False,
            "map_odom_tf": {"valid": True},
        },
        "mujoco_feeder.live.json": {"product_session_id": SESSION, "position_m": [5, 4, .5]},
    }
    requests = []
    response = {"found": True, "task_id": "gateway-task", "status": {
        "task_id": "gateway-task", "native_state": 4, "phase": "REACHED", "state": 5,
    }}

    class Client:
        def __init__(self, *_args):
            pass

        def send_map_goal(self, target):
            requests.append(target)
            return {"accepted": True, "task_id": "gateway-task", "native_request_id": "request"}

        def _request(self, path):
            assert path == "/api/v1/navigation/tasks/gateway-task"
            return response

        def close(self):
            requests.append("closed")

    def read_snapshot(path):
        snapshot = dict(snapshots.get(Path(path).name, {}))
        for key in ("stamp_s", "snapshot_written_at_s"):
            if key in snapshot:
                snapshot[key] += elapsed[0]
        return snapshot

    monkeypatch.setattr(nav, "ViewerGoal", Client)
    monkeypatch.setattr(nav, "_load_json", read_snapshot)
    monkeypatch.setattr(nav.time, "time", lambda: stamp + elapsed[0])
    monkeypatch.setattr(nav.time, "monotonic", lambda: elapsed[0])
    monkeypatch.setattr(nav.time, "sleep", lambda delay: elapsed.__setitem__(0, elapsed[0] + delay))
    return SimpleNamespace(plan=plan, manifest=manifest, snapshots=snapshots, requests=requests,
                           response=response, out=tmp_path, path=tmp_path / "plan.json")


def run_attached(case):
    return nav.run_attached_goal(case.plan, case.path, SESSION, case.manifest, case.out / "goal")


def test_attached_click_uses_gateway_task_and_native_phase(attached):
    report = run_attached(attached)
    assert report["ok"] is True
    assert report["goal_status"]["native_state"] == 4
    assert report["goal_status"]["state"] == 5
    assert report["goal_ack"]["task_id"] == "gateway-task"
    assert report["input_scope"] == "gateway_map_click"
    assert attached.requests == [[5.0, 4.0, 0.0], "closed"]
    samples = json.loads((attached.out / "goal/samples.json").read_text())
    assert samples[0]["slam"]["has_odom"]


@pytest.mark.parametrize("failure", ["no_odom", "old_slam", "old_nav", "lost_tracking", "pose_prior", "other_session", "wrong_task", "ack_only"])
def test_attached_click_cannot_pass_without_current_localization_and_completion(attached, monkeypatch, failure):
    if failure == "no_odom":
        attached.snapshots["slam.status.json"]["has_odom"] = False
    elif failure == "old_slam":
        attached.snapshots["slam.status.json"]["stamp_s"] -= 5
    elif failure == "old_nav":
        attached.snapshots["nav.status.json"]["stamp_s"] -= 5
    elif failure == "lost_tracking":
        attached.snapshots["slam.status.json"]["state"] = "LOST"
    elif failure == "pose_prior":
        attached.snapshots["slam.status.json"]["odom_prior_enabled"] = True
    elif failure == "other_session":
        attached.snapshots["nav.status.json"]["native_product"]["product_session_id"] = "b" * 32
    elif failure == "wrong_task":
        attached.response["status"]["task_id"] = "another-task"
    else:
        attached.response.update(found=False, status=None)
        attached.manifest["motion_duration_s"] = 0.6
    report = run_attached(attached)
    assert report["ok"] is False
    assert report["blockers"]
    assert attached.requests[-1] == "closed"


def test_actual_slam_case_rejects_truth_fixture_before_http(tmp_path):
    truth = compile_run_plan("nav", "sim", robot="doso/thunder_v4", local_planner="scan",
                             env_config={"backend": "mujoco", "localization": "truth"})
    with pytest.raises(ValueError, match="Fast-LIO2"):
        nav.run_attached_goal(truth, tmp_path / "plan.json", SESSION,
                              nav._load_manifest(MANIFEST), tmp_path / "goal")


def test_goal_error_uses_world_reference_separately_from_map_goal():
    manifest = nav._load_manifest(MANIFEST)
    manifest["goal_world"] = [5.0, 4.0, 0.0]
    manifest["goal"] = [105, 204, 0]
    motion = {"end_position_m": [5, 4, .5], "path_length_xy_m": 2, "net_displacement_xy_m": 2}
    blockers, error = nav._product_goal_blockers(manifest, motion,
        {"complete": True, "entity_contact_steps": 0}, {"state": 4},
        {"terminal_ack": True, "outcome": "zero_applied"}, 1.0)
    assert blockers == [] and error == 0


def test_nav_dispatcher_selects_attach_only_scenario(plan, tmp_path, monkeypatch):
    target = product.AcceptanceTarget("nav", Path(nav.__file__), MANIFEST)
    seen = []
    monkeypatch.setattr(nav, "run_attached_goal", lambda *args: seen.append(args) or {"ok": True})
    result = product._nav_case(target)(plan, tmp_path / "plan.json", SESSION)
    assert result["ok"] is True
    assert seen[0][:3] == (plan, tmp_path / "plan.json", SESSION)
    assert "nav" in product._LIFECYCLE_PRODUCTS


@pytest.mark.parametrize("failure", [None, "contacts_session", "stop_session", "collision", "no_stop", "not_reached", "wrong_position"])
def test_post_stop_check_requires_current_physical_arrival_and_zero(plan, tmp_path, monkeypatch, failure):
    motion = {"end_position_m": [5, 4, .5], "path_length_xy_m": 2, "net_displacement_xy_m": 2}
    contacts = {"product_session_id": SESSION, "complete": True, "entity_contact_steps": 0}
    stopped = {"product_session_id": SESSION, "terminal_ack": True, "outcome": "zero_applied"}
    scenario = {"criteria": nav._load_manifest(MANIFEST), "goal_status": {"state": 4}, "ready_fraction": 1.0}
    scenario["criteria"]["goal_world"] = [5.0, 4.0, 0.0]
    if failure == "contacts_session":
        contacts["product_session_id"] = "b" * 32
    elif failure == "stop_session":
        stopped["product_session_id"] = "b" * 32
    elif failure == "collision":
        contacts["entity_contact_steps"] = 1
    elif failure == "no_stop":
        stopped["terminal_ack"] = False
    elif failure == "not_reached":
        scenario["goal_status"] = {"state": 2}
    elif failure == "wrong_position":
        motion["end_position_m"] = [3, 4, .5]
    (tmp_path / "mujoco_feeder.contacts.json").write_text(json.dumps(contacts))
    (tmp_path / "mujoco_feeder.stop.json").write_text(json.dumps(stopped))
    monkeypatch.setattr(product, "_check_motion", lambda *_args: motion)
    if failure:
        with pytest.raises(RuntimeError):
            product._check_nav_goal(tmp_path, plan, (), scenario, SESSION)
    else:
        assert product._check_nav_goal(tmp_path, plan, (), scenario, SESSION)["goal_error_m"] == 0


def test_product_map_selection_is_retained_for_rollback(plan, tmp_path, monkeypatch):
    calls = []
    target = product.AcceptanceTarget("nav", Path(nav.__file__), MANIFEST)
    control = SimpleNamespace(switch=lambda *args, **kwargs: calls.append((args, kwargs)) or {"ok": False})
    rollback = []
    monkeypatch.setattr(product, "check_rollback", lambda *args, **kwargs: rollback.append((args, kwargs)) or {"ok": True})
    report = product.run(control, target, tmp_path / "control", lambda *_args: {"ok": True},
                         expected_plan=plan, map_name="industrial_park_tracking",
                         rollback_control=object(), rollback_root=tmp_path / "rollback")
    assert report["ok"] is False
    assert calls[0][1]["map_name"] == rollback[0][1]["map_name"] == "industrial_park_tracking"
    assert calls[0][1]["local_planner"] == rollback[0][1]["local_planner"] == "scan"
    assert calls[0][1]["initial_pose"] == rollback[0][1]["initial_pose"] == tuple(nav._load_manifest(MANIFEST)["initial_pose"])


def test_product_map_cli_defaults_to_actual_slam_click_manifest(tmp_path, monkeypatch):
    seen = []
    monkeypatch.setattr(nav, "run_product_goal", lambda args: seen.append(args) or {"ok": True})
    assert nav.main(["--product-map", str(tmp_path)]) == 0
    assert Path(seen[0].manifest) == MANIFEST


def test_saved_map_entry_uses_shared_lifecycle_and_preserves_failure(tmp_path, monkeypatch):
    controls = []
    calls = []

    def control(**kwargs):
        controls.append(kwargs)
        return SimpleNamespace(status=lambda: {"status": "stopped"})

    def supervisor(root, *_args, **_kwargs):
        assert root.is_dir()
        return object()

    monkeypatch.setattr(product, "ProductControl", control)
    monkeypatch.setattr(product, "ensure_sim_supervisor", supervisor)
    monkeypatch.setattr(product, "run", lambda *args, **kwargs: calls.append((args, kwargs)) or {"ok": False})
    map_dir = tmp_path / "maps/factory_workshop_v2"
    map_dir.mkdir(parents=True)
    (map_dir / "metadata.json").write_text(json.dumps({"source_profile": "factory_workshop_v2"}))
    assert nav.main(["--product-map", str(map_dir),
                     "--out-dir", str(tmp_path / "evidence")]) == 1
    assert all(item["env_config"]["localization"] == "fastlio2" for item in controls)
    assert calls[0][1]["map_name"] == "factory_workshop_v2"
    assert calls[0][1]["check"] is product._check_nav_goal
    assert calls[0][1]["rollback_control"] is not None


def test_click_fixture_matches_current_product_world_and_spawn(plan):
    import math

    manifest = nav._load_manifest(MANIFEST)
    session = plan.simulation["session"]
    assert manifest["scene_contract"]["world"] == session["world"]
    spawn = session["robots"][0]["spawn"]
    assert manifest["initial_pose"][:2] == spawn["position_m"][:2]
    w, x, y, z = spawn["quaternion_wxyz"]
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    assert manifest["initial_pose"][3] == pytest.approx(yaw)


def test_wrong_scene_map_is_rejected_before_product_start(tmp_path, monkeypatch):
    (tmp_path / "metadata.json").write_text(json.dumps({"source_profile": "industrial_park"}))
    monkeypatch.setattr(product, "ProductControl", lambda **_kwargs: pytest.fail("Wrong map started Product"))
    with pytest.raises(ValueError, match="map source mismatch"):
        nav.main(["--product-map", str(tmp_path), "--out-dir", str(tmp_path / "out")])


def test_wrong_world_is_rejected_before_goal_submission(attached):
    attached.manifest["scene_contract"]["world"] = "industrial_park@1.0.0"
    with pytest.raises(ValueError, match="world does not match"):
        run_attached(attached)
    assert attached.requests == []


def test_product_scope_nav_dispatcher_uses_lifecycle_and_propagates_failure(plan, tmp_path, monkeypatch):
    path = plan.write(tmp_path / f"plan-{SESSION}.json")
    calls = []
    monkeypatch.setattr(product, "ensure_sim_supervisor", lambda *_args, **_kwargs: object())
    monkeypatch.setattr(product, "run", lambda *args, **kwargs: calls.append((args, kwargs)) or {"ok": False})
    assert product.main(["--run-plan", str(path), "--runner", str(Path(nav.__file__)),
                         "--manifest", str(MANIFEST), "--state-root", str(tmp_path / "control"),
                         "--rollback-state-root", str(tmp_path / "rollback"),
                         "--map", "industrial_park_tracking"]) == 1
    assert calls[0][1]["expected_plan"] == plan
    assert calls[0][1]["map_name"] == "industrial_park_tracking"
    assert calls[0][1]["check"] is product._check_nav_goal
