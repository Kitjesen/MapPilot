import numpy as np
import pytest
import yaml
from tools.robot.tora_geometry import MODEL_DIR, envelope_report, load_collision_meshes


@pytest.fixture(scope="module")
def geometry():
    return load_collision_meshes()


def test_locked_pose_meshes_fit_cylinder(geometry):
    meshes, metadata = geometry
    report = envelope_report(meshes, metadata)
    profile = yaml.safe_load((MODEL_DIR / "navigation.yaml").read_text(encoding="utf-8"))
    cylinder = profile["geometry"]

    assert len(meshes) == metadata["collision_mesh_count"] == 86
    assert metadata["triangle_count"] == 1128377
    assert report["raw_radius_m"] == pytest.approx(0.464487680898422)
    assert report["cylinder_radius_m"] == 0.52
    assert cylinder["collision_cylinder_radius"] == report["cylinder_radius_m"]
    assert cylinder["collision_cylinder_offset"] == 0
    assert profile["reference_frame"] == metadata["reference_link"]
    assert report["bounds_m"]["minimum"] == pytest.approx([-0.3725910765, -0.2509998134, -0.1779854431])
    assert report["bounds_m"]["maximum"] == pytest.approx([0.4068886071, 0.2509998618, 1.5138600202])
    for name, points in meshes.items():
        assert np.all(np.linalg.norm(points[:, :2], axis=1) <= cylinder["collision_cylinder_radius"] - 0.05)
        if "wheel" not in name:
            assert np.all(points[:, 2] >= -cylinder["collision_clearance_below"])
        assert np.all(points[:, 2] <= cylinder["collision_clearance_above"])
    assert report["nonwheel_minimum_z_m"] > -cylinder["collision_clearance_below"]
    assert report["full_mesh_clearance_below_m"] == .18


def test_reference_transform_matches_base_imu(geometry):
    meshes, metadata = geometry
    root_meshes, root_metadata = load_collision_meshes(reference="base_link")
    transform = np.asarray(metadata["root_from_reference"])

    assert metadata["root_link"] == "base_link"
    assert metadata["reference_link"] == "base_imu_link_0"
    assert transform[:3, :3] == pytest.approx(np.eye(3))
    assert transform[:3, 3] == pytest.approx([0.0195, 0, 0.178000853265161])
    assert root_metadata["root_from_reference"] == np.eye(4).tolist()
    for name, points in meshes.items():
        assert np.allclose(points + transform[:3, 3], root_meshes[name])


def test_missing_mesh_is_reported(tmp_path):
    package = tmp_path / "description"
    (package / "urdf").mkdir(parents=True)
    (package / "package.xml").write_text("<package><name>tora</name></package>", encoding="utf-8")
    urdf = package / "urdf/missing.urdf"
    urdf.write_text('<robot name="tora"><link name="base"><collision><geometry>'
                    '<mesh filename="package://tora/meshes/missing.stl"/>'
                    '</geometry></collision></link></robot>', encoding="utf-8")

    with pytest.raises(FileNotFoundError, match=r"missing\.stl"):
        load_collision_meshes(urdf, reference="base")
