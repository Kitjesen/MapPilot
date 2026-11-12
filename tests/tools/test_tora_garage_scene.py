import json
import xml.etree.ElementTree as ET

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

from tools.bench.tora_garage_scene import build_scene, render_replay, replay_timeline


@pytest.fixture(scope="module")
def garage(tmp_path_factory):
    output = tmp_path_factory.mktemp("tora-garage")
    scene = build_scene(output)
    return output, scene


def test_render_and_planner_obstacles_are_identical(garage):
    output, scene = garage
    model = mujoco.MjModel.from_xml_path(str(output / "world.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert json.loads((output / "scene.json").read_text()) == scene
    physical = {model.geom(index).name for index in range(model.ngeom) if model.geom_contype[index]}
    assert physical == {"floor", *(box["name"] for box in scene["obstacles"])}
    for box in scene["obstacles"]:
        geom = model.geom(box["name"])
        assert model.geom_type[geom.id] == mujoco.mjtGeom.mjGEOM_BOX
        assert data.geom_xpos[geom.id] == pytest.approx(box["center"])
        assert model.geom_size[geom.id] == pytest.approx(box["half_size"])
    for pose in (scene["start"], scene["goal"]):
        assert all(np.linalg.norm(np.maximum(np.abs(np.asarray(pose[:2]) - box["center"][:2]) - box["half_size"][:2], 0)) > .52
                   for box in scene["obstacles"])


def test_robot_has_complete_solid_mesh_entity_and_scene_is_not_overwritten(garage):
    output, _ = garage
    document = ET.parse(output / "render_scene.xml").getroot()
    robot = document.find("worldbody/body[@name='tora']")
    assert robot.get("mocap") == "true"
    assert len(robot.findall("geom[@type='mesh']")) == 86
    assert len(document.findall("asset/mesh")) == 86
    with pytest.raises(FileExistsError):
        build_scene(output)


def test_replay_holds_recorded_state_without_skipping_stopped_intervals():
    rows = [{"time_s": value} for value in (200, 200.1, 200.5)]
    times, indices = replay_timeline(rows, 10)
    assert times == pytest.approx([200, 200.1, 200.2, 200.3, 200.4, 200.5])
    assert indices.tolist() == [0, 1, 1, 1, 1, 2]
    with pytest.raises(ValueError):
        replay_timeline([{"time_s": 1}, {"time_s": 1}], 25)


def test_failed_global_plan_produces_snapshot_report_without_video(tmp_path, monkeypatch):
    monkeypatch.setattr("tools.bench.tora_garage_scene.render_snapshot", lambda _: None)
    report = render_replay(tmp_path, tmp_path)
    assert report["status"] == "no_feedback"
    assert report["frames"] == 0
    assert not (tmp_path / "navigation.mp4").exists()
