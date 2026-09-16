"""Promoted campus identity, relative assets, and contact preservation."""

import xml.etree.ElementTree as ET

import pytest
import yaml
from sim.tools.worlds.factory_workshop.install_campus import install_campus


def test_promotion_relinks_assets_without_changing_contacts(tmp_path):
    source = tmp_path / "review"
    native = source / "native"
    native.mkdir(parents=True)
    (native / "mesh.obj").write_text("v 0 0 0\n", encoding="utf-8")
    (native / "light.png").write_bytes(b"test-image-content")
    (native / "campus.xml").write_text('''<mujoco>
      <compiler angle="radian" strippath="false"/>
      <option timestep="0.005" gravity="0 0 -9.81" integrator="Euler" solver="Newton" iterations="100"/>
      <asset><mesh name="part" file="mesh.obj"/><texture name="light" file="light.png"/></asset>
      <worldbody><geom name="wall" type="box" size="1 2 3" pos="4 5 6" group="4" contype="1"/>
      <geom name="visual" type="mesh" mesh="part" group="2" contype="0" conaffinity="0"/></worldbody>
    </mujoco>''', encoding="utf-8")
    destination = tmp_path / "factory_workshop/2.0.0"
    report = install_campus(source, destination)
    manifest = yaml.safe_load((destination / "world.package.yaml").read_text(encoding="utf-8"))
    assert (manifest["id"], manifest["version"]) == ("factory_workshop", "2.0.0")
    assert manifest["physics"]["global_policy"]["timestep_s"] == .005
    output = ET.parse(destination / manifest["physics"]["mjcf"])
    for asset in output.getroot().find("asset"):
        assert (destination / "physics" / asset.get("file")).is_file()
    before = ET.parse(native / "campus.xml").find("worldbody")
    assert [g.attrib for g in output.find("worldbody")] == [g.attrib for g in before]
    assert report["asset_count"] == 2
    assert (destination / "visual/native/light.png").read_bytes() == b"test-image-content"
    with pytest.raises(FileExistsError):
        install_campus(source, destination)


def test_missing_asset_does_not_replace_or_publish_world(tmp_path):
    native = tmp_path / "source/native"
    native.mkdir(parents=True)
    (native / "campus.xml").write_text('<mujoco><asset><mesh name="part" file="missing.obj"/></asset></mujoco>', encoding="utf-8")
    destination = tmp_path / "factory_workshop/2.0.0"
    with pytest.raises(FileNotFoundError):
        install_campus(native.parent, destination)
    assert not destination.exists()
