"""Thermal/semantic images from actual nearest MuJoCo geometry, not color."""

from dataclasses import replace

import numpy as np
import pytest
from sim.runtime.sensors.camera_geometry import CameraPose, PinholeCalibration, calibrated_rays
from sim.runtime.sensors.samples import SensorSampleStamp
from sim.runtime.sensors.thermal import ThermalConfig, ThermalSensor, ThermalSurface


def test_physical_raster_blocks_hot_surface_with_unknown_foreground():
    from sim.runtime.sensors.measurement_scene import MujocoRasterizer, thermal_calibration

    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_string("""<mujoco><worldbody>
      <geom name="hot_wall" type="box" pos="0 0 4" size="4 4 .1"/>
      <geom name="unknown_cover" type="box" pos="0 0 2" size=".3 .3 .1"/>
    </worldbody></mujoco>""")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    stamp = SensorSampleStamp("fixture", "robot", "robot.thermal", "robot/camera", 0, 0, 0, 0)
    calibration = PinholeCalibration(5, 3, 3, 3, 2.5, 1.5)
    rays = calibrated_rays(calibration, stamp, base_pose=CameraPose())
    hit = MujocoRasterizer(model, data, stamp).cast(rays, max_distance_m=10)
    hot = int(model.geom("hot_wall").id)
    cover = int(model.geom("unknown_cover").id)
    assert hit.geom_ids.shape == (3, 5)
    assert hit.geom_ids[1, 2] == cover
    assert np.count_nonzero(hit.geom_ids == hot) > 0
    sensor = ThermalSensor(ThermalConfig(thermal_calibration(rays), 293.15), {hot: ThermalSurface(400, 1)})
    frame = sensor.measure(
        stamp, source_stamp=hit.stamp, geom_ids=hit.geom_ids, distances_m=hit.distances_m, valid_rays=hit.valid_mask
    )
    assert np.isnan(frame.temperature_k[1, 2]) and not frame.valid_mask[1, 2]
    assert np.all(frame.temperature_k[frame.valid_mask] == 400)
    assert data.time == 0


def test_poststep_moving_geom_uses_current_position_without_advancing_physics():
    from sim.runtime.sensors.measurement_scene import MujocoRasterizer

    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_string("""<mujoco><option gravity="0 0 0" timestep=".01"/>
      <worldbody><body pos="0 0 2"><freejoint/>
      <geom type="sphere" size=".2"/></body></worldbody></mujoco>""")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    data.qvel[0] = 100
    mujoco.mj_step(model, data)
    stamp = SensorSampleStamp("fixture", "robot", "robot.thermal", "robot/camera", 0, 0, 1, 10_000_000)
    rays = calibrated_rays(PinholeCalibration(1, 1, 1, 1, 0.5, 0.5), stamp)
    before = (data.qpos.copy(), data.qvel.copy(), data.time)
    hit = MujocoRasterizer(model, data, stamp).cast(rays, max_distance_m=10)
    assert not hit.valid_mask[0, 0], "post-step sphere moved off the optical axis"
    np.testing.assert_array_equal(data.qpos, before[0])
    np.testing.assert_array_equal(data.qvel, before[1])
    assert data.time == before[2]
    moved = calibrated_rays(PinholeCalibration(1, 1, 1, 1, 0.5, 0.5), stamp, base_pose=CameraPose((1, 0, 0)))
    hit = MujocoRasterizer(model, data, stamp).cast(moved, max_distance_m=10)
    assert hit.valid_mask[0, 0] and hit.distances_m[0, 0] == pytest.approx(1.8)


@pytest.mark.parametrize("mismatch", ["generation", "time"])
def test_raster_rejects_stale_source(mismatch):
    from sim.runtime.sensors.measurement_scene import MujocoRasterizer

    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_string("<mujoco/>")
    data = mujoco.MjData(model)
    stamp = SensorSampleStamp("fixture", "robot", "robot.thermal", "robot/camera", 0, 0, 0, 0)
    source = replace(stamp, reset_generation=1) if mismatch == "generation" else stamp
    if mismatch == "time":
        data.time = 0.1
    rays = calibrated_rays(PinholeCalibration(1, 1, 1, 1, 0.5, 0.5), stamp)
    with pytest.raises(ValueError, match="source stamp"):
        MujocoRasterizer(model, data, source).cast(rays, max_distance_m=10)
