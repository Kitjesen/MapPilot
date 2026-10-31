"""Unit-labelled deterministic noise shared by the new measurement producers."""

from dataclasses import replace

import numpy as np
import pytest
from sim.runtime.sensors.samples import SensorSampleStamp


def test_measurement_noise_is_seeded_by_sensor_and_sample_identity():
    from sim.runtime.sensors.measurement_control import MeasurementNoise

    stamp = SensorSampleStamp("session", "robot", "robot.thermal", "robot/camera", 1, 2, 3, 400)
    noise = MeasurementNoise(unit="K", enabled=True, stddev=0.3, seed=123)
    np.testing.assert_array_equal(noise.generator(stamp).normal(size=20), noise.generator(stamp).normal(size=20))
    assert not np.array_equal(noise.generator(stamp).normal(size=20),
                              noise.generator(replace(stamp, sensor_id="robot.other")).normal(size=20))
    assert noise.unit == "K" and noise.stddev == 0.3


@pytest.mark.parametrize("kwargs", [{"enabled": 1}, {"stddev": -1}, {"stddev": float("nan")},
                                    {"seed": True}, {"unit": ""}])
def test_measurement_noise_rejects_ambiguous_configuration(kwargs):
    from sim.runtime.sensors.measurement_control import MeasurementNoise

    with pytest.raises(ValueError):
        MeasurementNoise(**({"unit": "Pa"} | kwargs))
