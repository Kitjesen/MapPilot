from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SLAMD_SOURCE = ROOT / "src" / "localization" / "slam" / "cpp" / "cyclone_runtime.cpp"
SIM_ENV = ROOT / "config" / "runtime_graph" / "envs" / "sim.yaml"


def test_slamd_accepts_only_the_canonical_fastlio2_backend() -> None:
    source = SLAMD_SOURCE.read_text(encoding="utf-8")

    assert 'std::string backend = "fastlio2";' in source
    assert 'if (backend == "fastlio2")' in source
    assert "normalizedBackend" not in source
    assert 'backend == "localizer"' not in source
    assert 'backend == "fastlio"' not in source


def test_localization_is_selected_by_mode_not_backend() -> None:
    source = SLAMD_SOURCE.read_text(encoding="utf-8")
    contract = (SLAMD_SOURCE.parent / "slam.cpp").read_text(encoding="utf-8")

    assert '[--mode mapping|localization]' in source
    assert "const SlamMode runtime_mode = modeFromString(cli.mode);" in source
    assert "backend->setMode(runtime_mode, cli.map_path)" in source
    assert "toString(runtime_mode)" in source
    assert 'value == "mapping"' in contract
    assert 'value == "localization"' in contract
    assert 'value == "localizer"' not in contract
    assert 'value == "nav"' not in contract
    assert 'throw std::invalid_argument("unsupported SLAM mode: " + value)' in contract


def test_sim_product_stops_map_icp_after_initial_alignment() -> None:
    source = SLAMD_SOURCE.read_text(encoding="utf-8")
    sim_env = SIM_ENV.read_text(encoding="utf-8")

    assert '(!product_managed || envOrEmpty("LINGTU_ENV") == "sim")' in source
    assert "const bool repeat_track_against_map = cli.track_against_map_period_s > 0.0;" in source
    assert "track_against_map_enabled = repeat_track_against_map;" in source
    assert "if (out.map_odom_tf.has_value())" in source
    assert "map_tf_publish_allowed" not in source
    assert sim_env.count('--track-against-map-period-s, "0"') == 2
    assert "restart_track_against_map();" in source
    assert source.index("cli.track_against_map_initial_pose") < source.index(
        "loadTrackSeed(cli.track_against_map_seed_file, cli.map_path)"
    )
    assert 'envOrEmpty("LINGTU_SLAM_TRACK_INITIAL_YAW")' in source
    assert source.index('envOrEmpty("LINGTU_SLAM_TRACK_INITIAL_YAW")') < source.index(
        'arg == "--track-against-map-initial-pose"'
    )


def test_rejected_saved_seed_falls_back_to_global_localization() -> None:
    source = SLAMD_SOURCE.read_text(encoding="utf-8")
    degraded_branch = source[source.index("kTrackAgainstMapDegradedFailureCount"):]

    assert "track_against_map_seed.reset();" in degraded_branch
    assert "falling back to global localization" in degraded_branch
