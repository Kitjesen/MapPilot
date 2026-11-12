import json
from itertools import pairwise

import numpy as np
import pytest
from tools.bench.tora_garage_complex import build_complex_scene

mujoco = pytest.importorskip("mujoco")


@pytest.fixture(scope="module")
def garage(tmp_path_factory):
    output = tmp_path_factory.mktemp("complex-garage")
    scene = build_complex_scene(output)
    return output, scene


def test_complex_garage_physical_geometry_matches_planning_boxes(garage):
    output, scene = garage
    model = mujoco.MjModel.from_xml_path(str(output / "world.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert scene == json.loads((output / "scene.json").read_text())
    physical = {model.geom(index).name for index in range(model.ngeom) if model.geom_contype[index]}
    assert physical == {"floor", *(box["name"] for box in scene["obstacles"])}
    for box in scene["obstacles"]:
        geom = model.geom(box["name"])
        assert data.geom_xpos[geom.id] == pytest.approx(box["center"])
        assert model.geom_size[geom.id] == pytest.approx(box["half_size"])
    assert scene["parked_cars"] == 12
    assert scene["columns"] == 8


def test_alternating_parking_rows_leave_connected_multi_turn_aisles(garage):
    _, scene = garage
    waypoints = [scene["start"][:2], [7, -6.7], [7, 0], [-7, 0], [-7, 6.7], scene["goal"][:2]]
    # This checks authored connectivity only. The native planner receives start and goal.
    for first, second in pairwise(waypoints):
        points = np.linspace(first, second, 300)
        for box in scene["obstacles"]:
            gap = np.maximum(np.abs(points - np.asarray(box["center"][:2])) - box["half_size"][:2], 0)
            assert np.min(np.linalg.norm(gap, axis=1)) > .52
    assert scene["centre_aisle_width_m"] >= 2.5
    with pytest.raises(FileExistsError):
        build_complex_scene(garage[0])
