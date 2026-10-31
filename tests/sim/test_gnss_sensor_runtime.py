from __future__ import annotations

from fractions import Fraction

import pytest
from sim.runtime.sensors.contracts import SensorRoute, SensorStreamPlan
from sim.runtime.sensors.gnss import GnssConfig, GnssExtractor
from sim.runtime.sensors.runtime import SensorRuntime

from runtime.msgs.gnss import GnssFixType

pytestmark = [pytest.mark.sim]


def _stream() -> SensorStreamPlan:
    return SensorStreamPlan(
        stream_kind="gnss",
        instance_id="thunder_01",
        sensor_id="thunder_01.gnss",
        frame_id="thunder_01/gnss_antenna",
        message_type="lingtu.dds.GnssFix",
        rate_hz=Fraction(10),
        route=SensorRoute("physics", "mujoco_gnss", "typed_dds"),
    )


def _snapshot(*, time_ns: int = 0, position=(0.0, 0.0, 0.0), reset=0):
    return {
        "session_id": "gnss-contract",
        "model_generation": 1,
        "reset_generation": reset,
        "sim_time_ns": time_ns,
        "bodies": [
            {"stable_id": "thunder_01/base_link", "position_m": list(position), "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0]}
        ],
    }


def _config(**overrides) -> GnssConfig:
    values = dict(
        origin_latitude_deg=0.0,
        origin_longitude_deg=0.0,
        origin_altitude_m=0.0,
        body_frame_id="thunder_01/base_link",
        fix_available=True,
    )
    values.update(overrides)
    return GnssConfig(**values)


def test_gnss_uses_explicit_wgs84_origin_and_simulation_time_not_wall_clock():
    stream = _stream()
    scheduled = (
        SensorRuntime("gnss-contract", (stream,))
        .advance(
            sim_time_ns=0,
            model_generation=1,
            reset_generation=0,
        )
        .samples[0]
    )
    measured = GnssExtractor(_config())(scheduled, _snapshot())
    assert (measured.fix.lat, measured.fix.lon, measured.fix.alt) == pytest.approx((0, 0, 0))
    assert measured.fix.fix_type == GnssFixType.SIMULATION
    assert measured.fix.ts == 0.0
    assert measured.fix.frame_id == stream.frame_id
    assert measured.stamp.session_id == "gnss-contract"
    assert measured.stamp.sim_time_ns == 0
    assert measured.fix.num_sat == measured.fix.num_sat_used == 0
    assert not measured.fix.is_rtk


def test_gnss_applies_antenna_lever_arm_and_map_enu_alignment_without_mutating_truth():
    import copy
    import math

    snapshot = _snapshot(position=(100, 200, 3))
    snapshot["bodies"][0]["quaternion_wxyz"] = [math.sqrt(0.5), 0, 0, math.sqrt(0.5)]
    before = copy.deepcopy(snapshot)
    scheduled = (
        SensorRuntime("gnss-contract", (_stream(),))
        .advance(
            sim_time_ns=0,
            model_generation=1,
            reset_generation=0,
        )
        .samples[0]
    )
    measured = GnssExtractor(
        _config(
            antenna_offset_m=(1, 0, 0),
            world_origin_m=(100, 200, 0),
            world_to_enu_yaw_deg=90,
        )
    )(scheduled, snapshot)
    # At the equator, one metre west at height three metres.
    assert measured.fix.lat == pytest.approx(0, abs=1e-10)
    assert measured.fix.lon == pytest.approx(-8.9831486e-6, abs=1e-11)
    assert measured.fix.alt == pytest.approx(3, abs=1e-5)
    assert snapshot == before


def test_gnss_noise_switch_is_replayable_per_sample_and_never_changes_truth():
    import copy

    config = _config(noise_enabled=True, horizontal_std_m=2.0, vertical_std_m=3.0, seed=17)
    scheduled = (
        SensorRuntime("gnss-contract", (_stream(),))
        .advance(
            sim_time_ns=0,
            model_generation=1,
            reset_generation=0,
        )
        .samples[0]
    )
    snapshot = _snapshot()
    before = copy.deepcopy(snapshot)
    extractor = GnssExtractor(config)
    noisy = extractor(scheduled, snapshot)
    assert (noisy.fix.lat, noisy.fix.lon, noisy.fix.alt) != (0, 0, 0)
    assert noisy.fix.covariance == pytest.approx((4, 0, 0, 0, 4, 0, 0, 0, 9), abs=1e-5)
    assert noisy.configuration == config
    assert noisy == GnssExtractor(config)(scheduled, snapshot)
    extractor.set_noise_enabled(False)
    clean = extractor(scheduled, snapshot)
    assert (clean.fix.lat, clean.fix.lon, clean.fix.alt) == pytest.approx((0, 0, 0))
    assert clean.fix.covariance == (0.0,) * 9
    assert clean.configuration.noise_enabled is False
    extractor.set_noise_enabled(True)
    assert extractor(scheduled, snapshot) == noisy
    assert snapshot == before


def test_gnss_outage_publishes_no_fix_without_leaking_last_known_truth():
    import math

    scheduled = (
        SensorRuntime("gnss-contract", (_stream(),))
        .advance(
            sim_time_ns=0,
            model_generation=1,
            reset_generation=0,
        )
        .samples[0]
    )
    extractor = GnssExtractor(_config())
    assert extractor(scheduled, _snapshot()).fix.fix_type == GnssFixType.SIMULATION
    extractor.set_fix_available(False)
    lost = extractor(scheduled, _snapshot(position=(100, 200, 3)))
    assert lost.fix.fix_type == GnssFixType.NO_FIX
    assert all(math.isnan(value) for value in (lost.fix.lat, lost.fix.lon, lost.fix.alt))
    assert all(math.isnan(value) for value in lost.fix.covariance)
    assert lost.stamp == scheduled_stamp(scheduled)
    assert lost.configuration.fix_available is False
    extractor.set_fix_available(True)
    assert extractor(scheduled, _snapshot()).fix.fix_type == GnssFixType.SIMULATION


def scheduled_stamp(scheduled):
    from sim.runtime.sensors.samples import SensorSampleStamp

    return SensorSampleStamp.from_scheduled(scheduled)


def test_gnss_compiled_plan_dispatches_through_existing_session_and_records_model_limits():
    from dataclasses import asdict

    from sim.runtime.sensors.evidence import build_sensor_stream_summary
    from sim.runtime.sensors.factory import GnssEndpointFactory
    from sim.runtime.sensors.session import SessionSensorRuntime

    declaration = dict(
        instance_id="thunder_01",
        sensor_id="thunder_01.gnss",
        frame_id="thunder_01/gnss_antenna",
        message_type="lingtu.dds.GnssFix",
        rate_hz=10,
        owner="physics",
        source="mujoco_gnss",
        transport="typed_dds",
        gnss=asdict(_config(fix_available=False, horizontal_std_m=2, vertical_std_m=3)),
    )
    plan = SensorRuntime.from_plan(
        dict(
            schema="lingtu.sim.sensor-plan.v1",
            env="sim",
            session_id="gnss-contract",
            streams={"gnss": [declaration]},
        )
    )

    class CaptureSink:
        # This proves the runtime contract, not native DDS delivery.
        def __init__(self):
            self.samples = []
            self.closed = False

        def start(self):
            return {"ready": True}

        def publish(self, sample):
            self.samples.append(sample)

        def close(self):
            self.closed = True

    sink = CaptureSink()
    session = SessionSensorRuntime(
        plan=plan,
        allocation=object(),
        endpoint_factory=GnssEndpointFactory(sink_factory=lambda stream, allocation: sink),
    )
    try:
        assert session.prepare().active_sensor_ids == ("thunder_01.gnss",)
        session.process_snapshot(_snapshot())
        session.process_snapshot(_snapshot(time_ns=100_000_000))
        assert len(sink.samples) == 2
        assert [sample.fix.ts for sample in sink.samples] == [0.0, 0.1]
        assert all(sample.fix.fix_type == GnssFixType.NO_FIX for sample in sink.samples)
        summary = build_sensor_stream_summary(
            plan,
            model_generation=1,
            reset_generation=0,
            observations=session.evidence_observations(),
        )
        fidelity = summary["streams"]["thunder_01.gnss"]["fidelity"]
        assert fidelity["qualification"] == "EXPERIMENTAL"
        assert fidelity["fix_type"] == 0
        assert fidelity["configuration"]["fix_available"] is False
        import json

        json.dumps(summary, allow_nan=False)
        # A live model may toggle outage/noise, never silently move the map origin.
        observations = list(session.evidence_observations())
        observations[0]["fidelity"]["configuration"]["origin_longitude_deg"] = 42
        from sim.runtime.sensors.evidence import SensorEvidenceError

        with pytest.raises(SensorEvidenceError, match="configuration"):
            build_sensor_stream_summary(
                plan,
                observations,
                model_generation=1,
                reset_generation=0,
            )
        session.set_noise_enabled("thunder_01.gnss", True)
        session.set_gnss_fix_available("thunder_01.gnss", True)
        session.process_snapshot(_snapshot(time_ns=200_000_000))
        assert sink.samples[-1].fix.fix_type == GnssFixType.SIMULATION
        assert sink.samples[-1].configuration.noise_enabled is True
        assert sink.samples[-1].fix.covariance[0] > 0
    finally:
        session.close()
    assert sink.closed


@pytest.mark.parametrize(
    "field,value",
    [
        ("origin_latitude_deg", 91),
        ("origin_longitude_deg", -181),
        ("origin_altitude_m", float("nan")),
        ("world_to_enu_yaw_deg", True),
        ("horizontal_std_m", -1),
        ("vertical_std_m", float("inf")),
        ("noise_enabled", 1),
        ("fix_available", "true"),
        ("seed", -1),
        ("seed", True),
        ("body_frame_id", ""),
        ("antenna_offset_m", (0, 0)),
    ],
)
def test_gnss_rejects_invalid_georeference_and_noise_knobs(field, value):
    with pytest.raises(ValueError):
        _config(**{field: value})


@pytest.mark.parametrize(
    "field,value",
    [
        ("session_id", "other-session"),
        ("model_generation", 2),
        ("reset_generation", 1),
        ("sim_time_ns", 1),
        ("bodies", []),
        ("bodies", [{"stable_id": "other/base_link"}]),
    ],
)
def test_gnss_never_samples_another_session_generation_deadline_or_body(field, value):
    from sim.runtime.sensors.extractors import SensorSampleError

    scheduled = (
        SensorRuntime("gnss-contract", (_stream(),))
        .advance(
            sim_time_ns=0,
            model_generation=1,
            reset_generation=0,
        )
        .samples[0]
    )
    snapshot = _snapshot()
    snapshot[field] = value
    with pytest.raises(SensorSampleError):
        GnssExtractor(_config())(scheduled, snapshot)


@pytest.mark.parametrize("origin", [(45, 120, 300), (-35, -179.99, 50), (90, 30, 10), (-90, -60, 10)])
def test_gnss_geodetic_result_recovers_the_measured_enu_including_poles(origin):
    import math

    def ecef(lat_deg, lon_deg, altitude):
        lat, lon = math.radians(lat_deg), math.radians(lon_deg)
        eccentricity_squared = 0.0066943799901413165
        radius = 6378137 / math.sqrt(1 - eccentricity_squared * math.sin(lat) ** 2)
        return (
            (radius + altitude) * math.cos(lat) * math.cos(lon),
            (radius + altitude) * math.cos(lat) * math.sin(lon),
            (radius * (1 - eccentricity_squared) + altitude) * math.sin(lat),
        )

    config = _config(origin_latitude_deg=origin[0], origin_longitude_deg=origin[1], origin_altitude_m=origin[2])
    scheduled = (
        SensorRuntime("gnss-contract", (_stream(),))
        .advance(
            sim_time_ns=0,
            model_generation=1,
            reset_generation=0,
        )
        .samples[0]
    )
    fix = GnssExtractor(config)(scheduled, _snapshot(position=(1000, -500, 30))).fix
    dx, dy, dz = (b - a for a, b in zip(ecef(*origin), ecef(fix.lat, fix.lon, fix.alt)))
    lat, lon = map(math.radians, origin[:2])
    recovered = (
        -math.sin(lon) * dx + math.cos(lon) * dy,
        -math.sin(lat) * math.cos(lon) * dx - math.sin(lat) * math.sin(lon) * dy + math.cos(lat) * dz,
        math.cos(lat) * math.cos(lon) * dx + math.cos(lat) * math.sin(lon) * dy + math.sin(lat) * dz,
    )
    assert recovered == pytest.approx((1000, -500, 30), abs=1e-5)


def test_gnss_default_has_no_fix_and_generation_reset_does_not_reuse_noise():
    config = GnssConfig(0, 0, 0, "thunder_01/base_link")
    assert config.fix_available is config.noise_enabled is False
    runtime = SensorRuntime("gnss-contract", (_stream(),))
    first = runtime.advance(sim_time_ns=0, model_generation=1, reset_generation=0).samples[0]
    after_reset = runtime.advance(sim_time_ns=0, model_generation=1, reset_generation=1).samples[0]
    extractor = GnssExtractor(_config(noise_enabled=True, horizontal_std_m=1, vertical_std_m=1, seed=17))
    old = extractor(first, _snapshot())
    new = extractor(after_reset, _snapshot(reset=1))
    assert old.stamp.sequence == new.stamp.sequence == 0
    assert old.stamp.reset_generation == 0 and new.stamp.reset_generation == 1
    assert (old.fix.lat, old.fix.lon, old.fix.alt) != (new.fix.lat, new.fix.lon, new.fix.alt)


def test_gnss_covariance_is_enu_at_the_fix_not_the_distant_map_origin():
    scheduled = (
        SensorRuntime("gnss-contract", (_stream(),))
        .advance(
            sim_time_ns=0,
            model_generation=1,
            reset_generation=0,
        )
        .samples[0]
    )
    fix = GnssExtractor(
        _config(
            noise_enabled=True,
            horizontal_std_m=2,
            vertical_std_m=5,
            seed=17,
        )
    )(scheduled, _snapshot(position=(1_000_000, 0, 0))).fix
    assert abs(fix.covariance[2]) > 0.1
    assert fix.covariance[2] == pytest.approx(fix.covariance[6])
    assert sum(fix.covariance[i] for i in (0, 4, 8)) == pytest.approx(4 + 4 + 25)
