from __future__ import annotations

from types import SimpleNamespace

from scripts.diagnostics.dds_probe import TopicStats


def test_dds_probe_stats_extracts_frame_and_point_count() -> None:
    stats = TopicStats()
    msg = SimpleNamespace(
        header=SimpleNamespace(frame_id="map"),
        width=3,
        height=2,
    )

    stats.observe(msg)
    stats.observe(msg)

    assert stats.samples == 2
    assert stats.frame_id == "map"
    assert stats.points == 6
    assert stats.hz() >= 0.0
