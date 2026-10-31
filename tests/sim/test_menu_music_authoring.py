"""The authored menu loops are stereo, quiet and continuous at their seam."""

import array
import wave

import pytest
from sim.tools.assets.author_menu_music import TRACK_NAMES, make_loop, write_tracks


def test_two_distinct_bounded_stereo_loops():
    assert TRACK_NAMES == ("base-arrival", "departure")
    tracks = [make_loop(name, sample_rate=8000) for name in TRACK_NAMES]
    assert tracks[0] != tracks[1]
    for samples in tracks:
        assert len(samples) == 32 * 8000 * 2
        assert 100 < max(abs(value) for value in samples) < 12000
        assert samples[::2] != samples[1::2]
        assert abs(samples[-2] - samples[0]) < 180
        assert abs(samples[-1] - samples[1]) < 180


def test_wavs_preserve_stereo_pcm_and_duration(tmp_path):
    outputs = write_tracks(tmp_path, sample_rate=8000)
    assert len(outputs) == 2
    for path in outputs:
        with wave.open(str(path), "rb") as reader:
            assert reader.getnchannels() == 2
            assert reader.getsampwidth() == 2
            assert reader.getframerate() == 8000
            assert reader.getnframes() == 256000
            pcm = array.array("h", reader.readframes(reader.getnframes()))
            assert pcm == make_loop(path.stem, sample_rate=8000)


def test_unknown_track_is_not_silently_replaced():
    with pytest.raises(ValueError, match="Unknown menu track"):
        make_loop("unknown", sample_rate=8000)
