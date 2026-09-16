"""The installed campus uses the real viewer and LiDAR group conventions."""

import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import pytest
from sim.catalog import CatalogResolver
from sim.compat.engine.core.sensor import LidarConfig
from sim.compat.engine.mujoco.lidar import MuJoCoLidar

from lingtu.assembly.compiler import compile_run_plan

ROOT = Path(__file__).resolve().parents[2]


def test_navigation_consumers_allow_the_campus_feeder_to_finish_cold_start():
    plan = compile_run_plan("nav", "sim", robot="doso/thunder_v4", env_config={"backend": "mujoco"})
    feeder = next(process for process in plan.processes if process.name == "mujoco_feeder")
    assert plan.process("traversability").timeout_s >= feeder.timeout_s


@pytest.mark.parametrize("product", ["default", "nav"])
def test_product_preset_selects_installed_campus_with_formal_robot(product):
    spec = ROOT / f"sim/sessions/products/doso/thunder_v4/{product}.yaml"
    resolved = CatalogResolver.from_repository(ROOT).resolve(spec)
    assert resolved.session["world"] == "factory_workshop@2.0.0"
    robot = resolved.physics_plan["robots"][0]
    assert robot["spawn"]["position_m"] == [61.0, 16.5, .02]
    assert robot["model"]["initial_keyframe"] == "v4_nominal_stand"
    assert resolved.session["robots"][0]["controller"] == "thunderv4_locomotion@1.0.0"


def test_hidden_campus_contact_is_visible_to_formal_lidar():
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <geom name="wall" type="box" pos="3 0 1" size="1 2 1" group="4"/>
      <geom name="paint" type="box" pos="1 0 1" size=".1 .1 .1" group="2" contype="0" conaffinity="0"/>
    </worldbody></mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    mask = MuJoCoLidar._geomgroup_from_config(LidarConfig(geom_group=0))
    distance = mujoco.mj_ray(model, data, np.array([0., 0., 1.]), np.array([1., 0., 0.]),
                             mask, 1, -1, np.zeros(1, dtype=np.int32))
    assert distance == pytest.approx(2)
    assert mask[2] == 0 and mask[3] == 0 and mask[5] == 0


def test_world_roof_is_not_in_the_hidden_robot_collision_group():
    root = ET.parse(ROOT / "sim/packages/worlds/factory_workshop/2.0.0/physics/campus.xml")
    roof = root.find('./worldbody/geom[@name="visual_roof"]')
    assert roof.get("group") == "2"
    assert roof.get("contype") == roof.get("conaffinity") == "0"
    assert not root.findall('./worldbody/geom[@group="3"]')
