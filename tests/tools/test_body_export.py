"""Check actual URDF preservation and conservative exported body bounds."""

import json
import math
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("mujoco")

from tools.bench.body_export import DEFAULT_URDF, export_body
from tools.bench.body_model import BodyModel


def primitive_points(geom):
    size, kind = geom["size"], geom["type"]
    if kind == "box":
        return np.array([[x, y, z] for x in (-size[0], size[0])
                         for y in (-size[1], size[1]) for z in (-size[2], size[2])])
    angles = np.linspace(0, 2 * math.pi, 40, endpoint=False)
    if kind == "cylinder":
        return np.array([[size[0] * math.cos(a), size[0] * math.sin(a), z]
                         for a in angles for z in (-size[1], size[1])])
    return np.array([[size[0]*math.sin(p)*math.cos(a), size[0]*math.sin(p)*math.sin(a),
                      size[0]*math.cos(p)] for p in np.linspace(0, math.pi, 13) for a in angles])


def test_exports_all_parts_and_native_text_with_pose_metadata(tmp_path):
    output = tmp_path / "body.txt"
    report = export_body(DEFAULT_URDF, output)
    lines = output.read_text().splitlines()
    assert lines[0] == f"BODY2 {report['primitive_count']}"
    assert report["primitive_count"] == 34
    assert len(lines) == 35
    assert all(len(line.split()) == 21 for line in lines[1:])
    assert len(report["joint_angles"]) == 12
    assert report["known_missing_geometry"] == ["external D435i and its bracket"]
    assert report["installation_qualified"] is False
    assert "fixed joint-pose snapshot" in report["pose_scope"]
    assert json.loads(output.with_suffix(".json").read_text()) == report
    links = {part["link"] for part in report["parts"]}
    assert {"base", "livox_frame", "mid360_lid"} <= links
    base = next(part for part in report["parts"] if part["link"] == "base")
    assert base["half_size"] == pytest.approx([.1881, .04675, .057])
    assert base["center"] == pytest.approx([0, 0, 0])
    assert np.asarray(base["rotation"]) == pytest.approx(np.eye(3))
    for line, part in zip(lines[1:], report["parts"]):
        columns = line.split()
        expected = [*part["center"], *np.asarray(part["rotation"]).ravel(), *part["half_size"]]
        assert columns[0] == part["link"]
        assert [float(value) for value in columns[1:16]] == expected
        assert columns[16] == part["source_type"]
        assert [float(value) for value in columns[17:19]] == [part["radius_m"], part["half_length_m"]]
        assert columns[19:] == ["0", "0"]


def test_each_exported_obb_contains_its_original_primitive(tmp_path):
    report = export_body(DEFAULT_URDF, tmp_path / "body.txt")
    geometry = BodyModel(DEFAULT_URDF, []).geometry([0, 0, 0, 0])
    assert {part["source_type"] for part in report["parts"]} == {"box", "sphere", "cylinder"}
    for geom, part in zip(geometry, report["parts"]):
        source_rotation = np.asarray(geom["rotation"])
        world_points = primitive_points(geom) @ source_rotation.T + geom["center"]
        exported_rotation = np.asarray(part["rotation"])
        exported_local = (world_points - part["center"]) @ exported_rotation
        assert np.all(np.abs(exported_local) <= np.asarray(part["half_size"]) + 1e-12), part["link"]


def test_joint_override_changes_geometry_without_losing_rigid_payload(tmp_path):
    nominal = export_body(DEFAULT_URDF, tmp_path / "nominal.txt")
    overrides = {f"{leg}_hip_joint": sign*.25 for leg, sign in
                 (("FL", 1), ("RL", 1), ("FR", -1), ("RR", -1))}
    spread = export_body(DEFAULT_URDF, tmp_path / "spread.txt", overrides)

    def lateral_width(parts):
        low, high = math.inf, -math.inf
        for part in parts:
            extent = np.abs(np.asarray(part["rotation"])[1]) @ np.asarray(part["half_size"])
            low = min(low, part["center"][1]-extent)
            high = max(high, part["center"][1]+extent)
        return high-low

    assert lateral_width(spread["parts"]) > lateral_width(nominal["parts"]) + .10
    assert len(spread["joint_angles"]) == 12
    assert spread["joint_angles"]["FL_hip_joint"] == .25
    assert spread["joint_angles"]["FL_thigh_joint"] == .8
    for link in ("base", "livox_frame"):
        before = next(part for part in nominal["parts"] if part["link"] == link)
        after = next(part for part in spread["parts"] if part["link"] == link)
        assert before == after


def test_unsupported_collision_mesh_still_fails(tmp_path):
    source = tmp_path / "mesh.urdf"
    source.write_text('<robot name="mesh"><link name="base"><collision>'
                      '<geometry><mesh filename="unknown.stl" /></geometry>'
                      '</collision></link></robot>')
    with pytest.raises(ValueError, match="not discarded"):
        export_body(source, tmp_path / "body.txt")
    assert not (tmp_path / "body.txt").exists()


def test_sphere_bounds_do_not_depend_on_collision_rotation(tmp_path):
    exports = []
    for index, rpy in enumerate(("0 0 0", ".8 -.7 .2")):
        source = tmp_path / f"sphere_{index}.urdf"
        source.write_text('<robot name="sphere"><link name="base"><collision>'
                          f'<origin xyz=".1 -.2 -.3" rpy="{rpy}" />'
                          '<geometry><sphere radius=".022" /></geometry>'
                          '</collision></link></robot>')
        exports.append(export_body(source, tmp_path / f"sphere_{index}.txt")["parts"][0])
    assert np.asarray(exports[0]["source_rotation"]) != pytest.approx(np.asarray(exports[1]["source_rotation"]))
    for part in exports:
        assert part["center"] == pytest.approx([.1, -.2, -.3])
        assert part["half_size"] == pytest.approx([.022] * 3)
        assert np.asarray(part["rotation"]) == pytest.approx(np.eye(3))
        assert part["axes_policy"] == "URDF-root identity axes; sphere orientation is immaterial"


def test_foot_sphere_margin_does_not_gain_rotation_padding(tmp_path):
    report = export_body(DEFAULT_URDF, tmp_path / "body.txt")
    original = BodyModel(DEFAULT_URDF, []).geometry([0, 0, 0, 0])
    margin = .02
    foot_parts = [part for part in report["parts"]
                  if part["link"].endswith("_foot") and part["source_type"] == "sphere"]
    assert len(foot_parts) == 4
    for part in foot_parts:
        source = next(geom for geom in original if geom["link"] == part["link"])
        extent_z = np.abs(np.asarray(part["rotation"])[2]) @ (np.asarray(part["half_size"]) + margin)
        expected = source["center"][2] - source["size"][0] - margin
        assert part["center"][2] - extent_z == pytest.approx(expected, abs=1e-12)
        assert .35 + part["center"][2] - extent_z > 0


def test_support_roles_require_explicit_geometry_links(tmp_path):
    feet = tuple(f"{leg}_foot" for leg in ("FL", "FR", "RL", "RR"))
    report = export_body(DEFAULT_URDF, tmp_path / "supported.txt", support_contact_links=feet)
    contacts = [part for part in report["parts"] if part["support_contact"]]
    assert {part["link"] for part in contacts} == set(feet)
    assert all(part["source_type"] == "sphere" for part in contacts)
    assert all(part["radius_m"] == pytest.approx(.022) for part in contacts)
    assert report["support_contact_links"] == list(feet)
    with pytest.raises(ValueError, match="no collision geometry"):
        export_body(DEFAULT_URDF, tmp_path / "missing.txt", support_contact_links=("nonexistent",))


def test_non_go2_urdf_does_not_inherit_go2_joint_pose(tmp_path):
    source = tmp_path / "another_robot.urdf"
    source.write_text('''<robot name="another_robot"><link name="base"/>
      <joint name="FR_thigh_joint" type="revolute"><parent link="base"/>
        <child link="leg"/><axis xyz="0 1 0"/><limit lower="-2" upper="2" effort="1" velocity="1"/>
      </joint><link name="leg"><inertial><mass value="1"/>
        <inertia ixx=".01" iyy=".01" izz=".01" ixy="0" ixz="0" iyz="0"/></inertial>
        <collision><origin xyz="0 0 -.1"/><geometry><box size=".03 .03 .2"/></geometry></collision>
      </link></robot>''')
    report = export_body(source, tmp_path / "body.txt")
    assert report["joint_angles"] == {}
    assert report["model_report"]["joint_state_source"] == "URDF default joint positions"
    assert report["parts"][0]["center"] == pytest.approx([0, 0, -.1])


def test_raised_go2_export_uses_installation_urdf_and_reports_missing_camera(tmp_path):
    raised = DEFAULT_URDF.with_name("go2_mid360_raised_frame.urdf")
    report = export_body(raised, tmp_path / "raised.body2")
    assert report["urdf"] == "web/public/assets/robots/go2/urdf/go2_mid360_raised_frame.urdf"
    assert report["model_report"]["urdf"] == report["urdf"]
    assert {"mid360_box", "mid360_lid", "livox_frame"} <= {part["link"] for part in report["parts"]}
    assert "mid360_base" not in {part["link"] for part in report["parts"]}
    assert report["known_missing_geometry"] == ["external D435i and its bracket"]


def test_selected_go2_model_regenerates_from_raised_installation(tmp_path):
    import yaml
    from tools.robot.export_navigation_bodies import GO2_BODY, ROOT, export_go2

    generated = tmp_path / "go2.body2"
    report = export_go2(generated)
    selected = ROOT / GO2_BODY
    assert generated.read_bytes() == selected.read_bytes()
    assert report == json.loads(selected.with_suffix(".json").read_text(encoding="utf-8"))
    geometry = yaml.safe_load((selected.parent.parent / "robot.yaml").read_text(encoding="utf-8"))["geometry"]
    assert geometry["navigation_body_file"] == "body/go2_raised_nominal.body2"
    assert report["primitive_count"] == 17
    assert report["installation_qualified"] is False
    assert report["geometry_margin_m"] == 0
    assert report["known_missing_geometry"] == ["external D435i and its bracket"]
    assert all("support_required" not in part for part in report["parts"])
    assert not any(part["support_contact"] for part in report["parts"])
    assert report["support_contact_links"] == []
    assert not any("_calf" in part["link"] or "_foot" in part["link"] for part in report["parts"])
    assert {f"{leg}_foot" for leg in ("FL", "FR", "RL", "RR")} <= set(report["excluded_navigation_links"])
    assert {"base", "FL_thigh", "RR_thigh", "mid360_box", "livox_frame"} <= {part["link"] for part in report["parts"]}
    assert report["nominal_fit"]["foot_bottom_body_z_m"] == pytest.approx(-.35)
    cover = next(part for part in report["parts"] if part["link"] == "unmeasured_payload_cover")
    assert cover["center"][2] - cover["half_size"][2] == pytest.approx(.057)
    assert cover["center"][2] + cover["half_size"][2] == pytest.approx(.24)
    assert cover["half_size"][:2] == pytest.approx([.38, .155])


def test_thunder_model_regenerates_identically_for_real_and_sim(tmp_path):
    import yaml
    from tools.robot.export_navigation_bodies import (
        ROOT,
        THUNDER_BODY,
        THUNDER_SIM_BODY,
        export_thunder,
    )

    generated, sim_generated = tmp_path / "thunder.body2", tmp_path / "sim" / "thunder.body2"
    report = export_thunder(generated, sim_generated)
    assert generated.read_bytes() == sim_generated.read_bytes()
    assert generated.read_bytes() == (ROOT / THUNDER_BODY).read_bytes()
    assert generated.read_bytes() == (ROOT / THUNDER_SIM_BODY).read_bytes()
    assert report == json.loads((ROOT / THUNDER_BODY).with_suffix(".json").read_text(encoding="utf-8"))
    assert report == json.loads((ROOT / THUNDER_SIM_BODY).with_suffix(".json").read_text(encoding="utf-8"))
    geometry = yaml.safe_load((ROOT / THUNDER_BODY).parent.parent.joinpath("robot.yaml").read_text(encoding="utf-8"))["geometry"]
    assert geometry["navigation_body_file"] == "body/thunderv4_navigation.body2"
    assert report["source_keyframe"] == "v4_nominal_stand"
    assert len(report["joint_angles"]) == 16
    assert report["primitive_count"] == 8
    assert report["source_primitive_count"] == 38
    assert report["geometry_margin_m"] == 0
    assert report["installation_qualified"] is False
    assert report["support_contact_links"] == []
    assert {part["link"] for part in report["parts"]} == {
        "chassis", "nose", "tail", "sensor",
        "FR_clearance", "FL_clearance", "RR_clearance", "RL_clearance",
    }
    sources = {part["link"]: part for part in report["source_bounds"]}
    covered = []
    for part in report["parts"]:
        assert part["source_type"] == "box"
        assert part["support_contact"] is False
        assert "support_required" not in part
        assert np.asarray(part["rotation"]) == pytest.approx(np.eye(3))
        minimum = np.array(part["center"]) - part["half_size"]
        maximum = np.array(part["center"]) + part["half_size"]
        covered.extend(part["footprint_source_parts"])
        for link in part["footprint_source_parts"]:
            source = sources[link]
            assert np.all(minimum[:2] <= np.array(source["minimum"])[:2] + 1e-12)
            assert np.all(maximum[:2] >= np.array(source["maximum"])[:2] - 1e-12)
        for link in part["height_source_parts"]:
            source = sources[link]
            assert minimum[2] <= source["minimum"][2] + 1e-12
            assert maximum[2] >= source["maximum"][2] - 1e-12
        if part["link"].endswith("_clearance"):
            assert len(part["footprint_source_parts"]) == 8
            assert len(part["height_source_parts"]) == 4
            assert all(sources[link]["body"].endswith(("_hip", "_thigh"))
                       for link in part["height_source_parts"])
    assert sorted(covered) == sorted(sources)
    assert report["envelope_bounds"]["size"] == pytest.approx([.835, .5983312173, .4115492487])
    assert report["envelope_bounds"]["minimum"][2] + .435 > .2
    assert set(report["excluded_height_bodies"]) == {
        f"{leg}_{link}" for leg in ("FL", "FR", "RL", "RR") for link in ("calf", "foot")}
