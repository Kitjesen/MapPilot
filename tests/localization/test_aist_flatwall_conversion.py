import numpy as np
import pytest
from tools.datasets.aist_flatwall_to_ltu1 import convert_bag
from tools.datasets.elevator_bag_to_ltu1 import HEADER, IMU, LTU_POINT

from tests.localization.test_elevator_bag_conversion import fixture_bag


@pytest.mark.parametrize("compression", ["none", "bz2"])
def test_aist_preserves_g_units_timing_and_avia_fields(tmp_path, compression):
    source = tmp_path / "flatwall_01.bag"
    original = fixture_bag(source, compression)
    result = convert_bag(source, tmp_path / "out")
    with (tmp_path / "out/flatwall_01.ltu").open("rb") as stream:
        cloud = HEADER.unpack(stream.read(HEADER.size))
        assert cloud[2] == 100_000_000_123
        points = np.frombuffer(stream.read(cloud[-1]), dtype=LTU_POINT)
        assert points["offset_time_ns"].tolist() == [900, 100]
        for name in ("x", "y", "z", "tag", "line"):
            assert np.array_equal(points[name], original[name])
        header = HEADER.unpack(stream.read(HEADER.size))
        assert header[2] == 100_000_000_456
        assert IMU.unpack(stream.read(header[-1])) == pytest.approx((.1, .2, .3, 0, 0, 1))
        assert not stream.read()
    assert result["imu_raw_acceleration_norm_median"] == 1.0
    assert result["counts"]["unsorted_clouds"] == 1
    assert result["dataset_url"] == "https://zenodo.org/records/7641866"
