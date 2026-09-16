from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("mujoco")

from sim.tools.worlds.factory_workshop.export_map import export_map, sample_solids


def test_export_map_uses_manifest_source_and_samples_contact_geometry_only(tmp_path):
    package = tmp_path / "micro_campus" / "2.0.0"
    physics = package / "physics" / "campus.xml"
    physics.parent.mkdir(parents=True)
    physics.write_text(
        """\
<mujoco model="micro_campus">
  <worldbody>
    <geom name="contact_box" type="box" pos="3 -2 0.5" size="0.5 0.25 0.5" contype="1" conaffinity="1" />
    <geom name="thin_post" type="cylinder" pos="3 -2 0.5" size="0.02 0.1" />
    <geom name="visual_only" type="box" pos="-9 8 0.5" size="0.5 0.5 0.5" group="1" contype="0" conaffinity="0" />
  </worldbody>
</mujoco>
""",
        encoding="utf-8",
    )
    manifest = package / "world.package.yaml"
    manifest.write_text(
        """\
schema: lingtu.sim.world-package.v1
id: micro_campus
version: 2.0.0
kind: world
physics:
  mjcf: physics/campus.xml
visual:
  binding: WorldVisual:MicroCampus
""",
        encoding="utf-8",
    )

    output = tmp_path / "map"
    provenance = export_map(output, spacing=0.5, world_package=manifest)

    assert provenance["world"] == "micro_campus@2.0.0"
    assert provenance["source_scene"] == str(physics.resolve())
    assert provenance["data_source"] == "synthetic_mujoco_collision_geometry"
    assert provenance["slam"] is False

    payload = (output / "map.pcd").read_bytes().split(b"DATA binary\n", 1)[1]
    points = np.frombuffer(payload, dtype="<f4").reshape(-1, 3)
    assert len(points) == provenance["point_count"]
    np.testing.assert_allclose(points.min(axis=0), [2.5, -2.25, 0.0])
    np.testing.assert_allclose(points.max(axis=0), [3.5, -1.75, 1.0])
    assert np.all(points[:, 0] > 0.0)
    header = (output / "map.pcd").read_bytes().split(b"DATA binary\n", 1)[0].decode("ascii")
    assert f"POINTS {len(points):20d}" in header

    import mujoco

    model = mujoco.MjModel.from_xml_path(str(physics))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    original = np.concatenate(list(sample_solids(model, data, spacing=0.5)))
    def occupied_voxels(samples):
        return np.unique(np.floor(samples.astype(np.float64) / 0.1).astype(np.int64), axis=0)
    np.testing.assert_array_equal(occupied_voxels(points), occupied_voxels(original))
    assert len(points) < len(original)


def test_dense_floor_is_sampled_in_bounded_chunks():
    import mujoco

    model = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><geom name="fw_floor" type="box" size="10 10 0.1"/></worldbody></mujoco>'
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    count = 0
    for points in sample_solids(model, data, spacing=0.05):
        assert len(points) <= 65536
        count += len(points)
        np.testing.assert_allclose(points[:, 2], 0.1)
    assert count == 401 * 401
