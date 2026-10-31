"""Serialized controls exercised through real scheduler/session interfaces."""

import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from concurrent.futures import TimeoutError as FutureTimeoutError
from dataclasses import replace
from fractions import Fraction
from pathlib import Path
from threading import Event
from types import SimpleNamespace

import pytest
from sim.runtime.sensors.contracts import SensorRoute, SensorStreamPlan
from sim.runtime.sensors.evidence import build_sensor_stream_summary
from sim.runtime.sensors.measurement_control import MeasurementNoise
from sim.runtime.sensors.runtime import SensorRuntime
from sim.runtime.sensors.samples import SensorSampleStamp
from sim.runtime.sensors.session import SensorEndpoint, SensorSessionError, SessionSensorRuntime


class Sink:
    def __init__(self):
        self.samples = []
        self.closed = 0

    def start(self):
        return {"ready": True}

    def publish(self, sample):
        self.samples.append(sample)

    def close(self):
        self.closed += 1


def plan_for(kind="imu", *, transport="typed_dds", rate=10):
    return SensorRuntime(
        "controls/test",
        (
            SensorStreamPlan(
                stream_kind=kind,
                instance_id="robot",
                sensor_id="robot.sensor",
                frame_id="robot/sensor",
                message_type="test.Sample",
                rate_hz=Fraction(rate),
                route=SensorRoute(owner="physics", source="test", transport=transport),
            ),
        ),
    )


def snapshot(time_ns, *, model=0, reset=0):
    return dict(session_id="controls/test", sim_time_ns=time_ns, model_generation=model, reset_generation=reset)


def extract(scheduled, source):
    return SimpleNamespace(stamp=SensorSampleStamp.from_scheduled(scheduled))


def session_for(plan, *, extractor=extract, sink=None):
    sink = Sink() if sink is None else sink
    runtime = SessionSensorRuntime(
        plan=plan,
        allocation=object(),
        endpoint_factory=lambda stream, allocation: SensorEndpoint(
            source_id="test/source",
            sink=sink,
            extractor=extractor,
        ),
    )
    runtime.prepare()
    return runtime, sink


def test_disable_stops_extraction_retains_transport_and_blocks_required_readiness():
    plan = plan_for()
    extracted = []

    def measured(scheduled, source):
        extracted.append(scheduled)
        return extract(scheduled, source)

    runtime, sink = session_for(plan, extractor=measured)
    runtime.process_snapshot(snapshot(0))
    runtime.set_enabled("robot.sensor", False)
    runtime.process_snapshot(snapshot(1_000_000_000))
    assert len(extracted) == len(sink.samples) == 1
    assert sink.closed == 0
    assert runtime.active_sensor_ids == ()
    observations = runtime.evidence_observations()
    assert observations[0]["state"] == "DISABLED"
    assert observations[0]["last_sample_sim_time_ns"] == 0
    summary = build_sensor_stream_summary(plan, observations, model_generation=0, reset_generation=0)
    assert not summary["is_ready"]
    assert summary["blocking_reasons"] == {"robot.sensor": "stream is DISABLED"}
    runtime.set_enabled("robot.sensor", True)
    runtime.process_snapshot(snapshot(1_000_000_000))
    assert len(sink.samples) == 1
    runtime.process_snapshot(snapshot(1_100_000_000))
    assert len(sink.samples) == 2
    assert sink.samples[-1].stamp.sequence > sink.samples[0].stamp.sequence
    assert sink.samples[-1].stamp.sim_time_ns == 1_100_000_000
    runtime.close()
    assert sink.closed == 1


def test_scheduler_rate_change_anchors_after_source_clock_without_old_catchup():
    runtime = plan_for(rate=10)
    runtime.advance(sim_time_ns=0, reset_generation=0)
    before = runtime.advance(sim_time_ns=250_000_000, reset_generation=0)
    assert [sample.sequence for sample in before.samples] == [1, 2]
    runtime.set_rate_hz("robot.sensor", 20)
    assert runtime.streams[0].rate_hz == 10
    assert runtime.status("robot.sensor")["rate_hz"] == 20
    assert not runtime.advance(sim_time_ns=250_000_000, reset_generation=0).samples
    after = runtime.advance(sim_time_ns=350_000_000, reset_generation=0)
    assert [(sample.sequence, sample.deadline_ns) for sample in after.samples] == [
        (3, 300_000_000),
        (4, 350_000_000),
    ]
    runtime.set_enabled("robot.sensor", False)
    runtime.advance(sim_time_ns=1_000_000_000, reset_generation=0)
    runtime.set_rate_hz("robot.sensor", 4)
    runtime.set_enabled("robot.sensor", True)
    assert not runtime.advance(sim_time_ns=1_000_000_000, reset_generation=0).samples
    resumed = runtime.advance(sim_time_ns=1_250_000_000, reset_generation=0).samples
    assert len(resumed) == 1
    assert resumed[0].deadline_ns == 1_250_000_000
    assert resumed[0].sequence > 4
    reset = runtime.advance(sim_time_ns=0, reset_generation=1).samples
    assert [(sample.sequence, sample.deadline_ns) for sample in reset] == [(0, 0)]
    assert runtime.status("robot.sensor")["rate_hz"] == 4


def test_session_rate_changes_require_producer_support_and_keep_rejections_atomic():
    class OnDemand:
        def __init__(self):
            self.rate = 10

        def set_rate_hz(self, rate_hz):
            if rate_hz != 20:
                raise ValueError("unsupported waveform block rate")
            self.rate = rate_hz

        def __call__(self, scheduled, source):
            return extract(scheduled, source)

    producer = OnDemand()
    plan = plan_for()
    runtime, sink = session_for(plan, extractor=producer)
    runtime.process_snapshot(snapshot(0))
    runtime.set_rate_hz("robot.sensor", 20)
    assert producer.rate == runtime.status("robot.sensor")["rate_hz"] == 20
    previous = runtime.status()
    for invalid in (True, 0, -1, float("nan"), float("inf"), 1_000_000_001):
        with pytest.raises(ValueError, match="rate_hz"):
            runtime.set_rate_hz("robot.sensor", invalid)
        assert runtime.status() == previous
        assert producer.rate == 20
    with pytest.raises(ValueError, match="waveform block"):
        runtime.set_rate_hz("robot.sensor", 40)
    assert runtime.status() == previous
    runtime.process_snapshot(snapshot(50_000_000))
    assert sink.samples[-1].stamp.sim_time_ns == 50_000_000
    assert sink.closed == 0
    assert plan.streams[0].rate_hz == 10
    unsupported, untouched = session_for(plan_for())
    before = unsupported.status()
    with pytest.raises(SensorSessionError, match="set_rate_hz"):
        unsupported.set_rate_hz("robot.sensor", 20)
    assert unsupported.status() == before
    assert untouched.closed == 0


def test_noise_configuration_dispatches_typed_model_and_reports_only_accepted_changes():
    class Measured:
        def __init__(self):
            self.noise = MeasurementNoise(unit="K")

        def configure_noise(self, noise):
            if noise.unit != "K":
                raise ValueError("noise unit must be K")
            self.noise = noise

        def __call__(self, scheduled, source):
            return extract(scheduled, source)

    producer = Measured()
    runtime, sink = session_for(plan_for("thermal"), extractor=producer)
    noise = MeasurementNoise(unit="K", enabled=True, stddev=0.25, seed=42)
    runtime.configure_noise("robot.sensor", noise)
    assert producer.noise is noise
    assert runtime.status("robot.sensor")["noise"] == noise.to_dict()
    previous = runtime.status()
    for invalid in ({"unit": "K"}, None, True):
        with pytest.raises(ValueError, match="MeasurementNoise"):
            runtime.configure_noise("robot.sensor", invalid)
    with pytest.raises(ValueError, match="unit must be K"):
        runtime.configure_noise("robot.sensor", MeasurementNoise(unit="Pa"))
    assert producer.noise is noise
    assert runtime.status() == previous
    unsupported, untouched = session_for(plan_for())
    before = unsupported.status()
    with pytest.raises(SensorSessionError, match="configure_noise"):
        unsupported.configure_noise("robot.sensor", noise)
    assert unsupported.status() == before
    assert sink.closed == untouched.closed == 0


def test_recording_control_dispatches_only_to_sink_and_preserves_publication():
    class Recorder(Sink):
        def __init__(self):
            super().__init__()
            self.recording = False
            self.recorded = []

        def set_recording(self, enabled):
            self.recording = enabled

        def publish(self, sample):
            super().publish(sample)
            if self.recording:
                self.recorded.append(sample)

    sink = Recorder()
    runtime, _ = session_for(plan_for(), sink=sink)
    runtime.process_snapshot(snapshot(0))
    runtime.set_recording("robot.sensor", True)
    assert runtime.status("robot.sensor")["recording"] is True
    runtime.process_snapshot(snapshot(100_000_000))
    runtime.set_recording("robot.sensor", False)
    runtime.process_snapshot(snapshot(200_000_000))
    assert [sample.stamp.sequence for sample in sink.samples] == [0, 1, 2]
    assert [sample.stamp.sequence for sample in sink.recorded] == [1]
    assert runtime.status("robot.sensor")["recording"] is False
    with pytest.raises(ValueError, match="boolean"):
        runtime.set_recording("robot.sensor", 1)
    unsupported, untouched = session_for(plan_for())
    previous = unsupported.status()
    with pytest.raises(SensorSessionError, match="set_recording"):
        unsupported.set_recording("robot.sensor", True)
    assert unsupported.status() == previous
    assert sink.closed == untouched.closed == 0


@pytest.mark.parametrize("kind", ["thermal", "semantic", "instance", "acoustic"])
@pytest.mark.parametrize("transport", ["typed_dds", "in_process"])
def test_live_measurements_publish_only_latest_using_actual_source_time(kind, transport):
    plan = plan_for(kind, transport=transport)

    def live(scheduled, source):
        stamp = replace(SensorSampleStamp.from_scheduled(scheduled), sim_time_ns=source["sim_time_ns"])
        return SimpleNamespace(stamp=stamp)

    runtime, sink = session_for(plan, extractor=live)
    runtime.process_snapshot(snapshot(0))
    runtime.process_snapshot(snapshot(350_000_000))
    assert [(sample.stamp.sequence, sample.stamp.sim_time_ns) for sample in sink.samples] == [
        (0, 0),
        (3, 350_000_000),
    ]
    observation = runtime.evidence_observations()[0]
    assert observation["last_sample_sim_time_ns"] == 350_000_000
    assert observation["last_sample_truth_sequence"] == 3
    assert observation["sample_count"] == 2


def test_prepared_session_can_handoff_to_background_dispatch_without_closing_endpoints():
    runtime, sink = session_for(plan_for())
    runtime.process_snapshot(snapshot(0))
    with ThreadPoolExecutor(max_workers=1) as pool:
        pool.submit(runtime.process_snapshot, snapshot(100_000_000)).result()
        pool.submit(runtime.set_enabled, "robot.sensor", False).result()
        assert pool.submit(runtime.status, "robot.sensor").result()["state"] == "DISABLED"
    assert sink.closed == 0
    assert [sample.stamp.sequence for sample in sink.samples] == [0, 1]


def test_control_waits_for_inflight_extraction_and_never_mutates_it_mid_sample():
    started, release, control_called = Event(), Event(), Event()

    def blocked(scheduled, source):
        started.set()
        assert release.wait(2)
        return extract(scheduled, source)

    runtime, sink = session_for(plan_for(), extractor=blocked)

    def disable():
        control_called.set()
        runtime.set_enabled("robot.sensor", False)

    with ThreadPoolExecutor(max_workers=2) as pool:
        sampling = pool.submit(runtime.process_snapshot, snapshot(0))
        assert started.wait(1)
        control = pool.submit(disable)
        assert control_called.wait(1)
        try:
            with pytest.raises(FutureTimeoutError):
                control.result(timeout=0.05)
        finally:
            release.set()
        sampling.result()
        control.result()
    runtime.process_snapshot(snapshot(100_000_000))
    assert len(sink.samples) == 1 and runtime.status("robot.sensor")["state"] == "DISABLED"


def test_existing_noise_switch_keeps_effective_noise_status_in_sync():
    class Measured:
        def __init__(self):
            self.noise = MeasurementNoise(unit="K")

        def configure_noise(self, noise):
            self.noise = noise

        def set_noise_enabled(self, enabled):
            self.noise = replace(self.noise, enabled=enabled)

        def __call__(self, scheduled, source):
            return extract(scheduled, source)

    producer = Measured()
    runtime, sink = session_for(plan_for("thermal"), extractor=producer)
    runtime.configure_noise("robot.sensor", MeasurementNoise(unit="K", enabled=True, stddev=1, seed=7))
    runtime.set_noise_enabled("robot.sensor", False)
    assert not producer.noise.enabled
    assert runtime.status("robot.sensor")["noise"] == producer.noise.to_dict()
    assert runtime.status("robot.sensor")["noise_enabled"] is False
    previous = runtime.status()
    with pytest.raises(ValueError, match="boolean"):
        runtime.set_noise_enabled("robot.sensor", "yes")
    assert runtime.status() == previous
    assert sink.closed == 0


@pytest.mark.parametrize("transport", ["typed_dds", "in_process"])
@pytest.mark.parametrize("kind", ["thermal", "semantic", "instance", "acoustic"])
@pytest.mark.parametrize(
    "field,value",
    [
        ("sim_time_ns", 300_000_000),
        ("sim_time_ns", 350_000_001),
        ("session_id", "another/session"),
        ("model_generation", 1),
        ("reset_generation", 1),
        ("sequence", 2),
        ("sensor_id", "another.sensor"),
        ("frame_id", "another/frame"),
        ("instance_id", "another_robot"),
    ],
)
def test_live_measurements_reject_forged_source_stamp(kind, transport, field, value):
    def forged(scheduled, source):
        stamp = replace(SensorSampleStamp.from_scheduled(scheduled), sim_time_ns=source["sim_time_ns"])
        return SimpleNamespace(stamp=replace(stamp, **{field: value}))

    runtime, sink = session_for(plan_for(kind, transport=transport), extractor=forged)
    with pytest.raises(SensorSessionError, match=f"sample {field}"):
        runtime.process_snapshot(snapshot(350_000_000))
    assert not sink.samples
    assert sink.closed == 1


@pytest.mark.parametrize("kind,transport", [("imu", "typed_dds"), ("probe", "in_process")])
def test_other_kinds_still_require_deadline_stamps_and_in_process_defaults_to_latest_only(kind, transport):
    plan = plan_for(kind, transport=transport)
    if transport == "in_process":
        assert plan.status("robot.sensor")["policy"] == "drop_intermediate"

    def wrong_time(scheduled, source):
        return SimpleNamespace(
            stamp=replace(
                SensorSampleStamp.from_scheduled(scheduled),
                sim_time_ns=source["sim_time_ns"],
            )
        )

    runtime, sink = session_for(plan, extractor=wrong_time)
    with pytest.raises(SensorSessionError, match="sample sim_time_ns"):
        runtime.process_snapshot(snapshot(350_000_000))
    assert not sink.samples


def test_disabled_before_first_sample_and_across_reset_has_truthful_zero_count_evidence():
    plan = plan_for("thermal", transport="in_process")
    runtime, sink = session_for(plan)
    runtime.set_enabled("robot.sensor", False)
    for source in (snapshot(250_000_000), snapshot(0, reset=1)):
        runtime.process_snapshot(source)
        observation = runtime.evidence_observations()[0]
        assert observation["state"] == "DISABLED"
        assert observation["sample_count"] == 0
        assert observation["last_sample_sim_time_ns"] is None
        summary = build_sensor_stream_summary(
            plan,
            [observation],
            model_generation=source["model_generation"],
            reset_generation=source["reset_generation"],
        )
        assert not summary["is_ready"]
    assert not sink.samples
    runtime.close()
    assert sink.closed == 1


def test_unused_measurement_controls_do_not_load_measurement_model():
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            (
                "import sys; sys.path.insert(0, 'src'); "
                "sys.modules['sim.runtime.sensors.measurement_control'] = None; "
                "from sim.runtime.sensors.session import SessionSensorRuntime"
            ),
        ],
        cwd=Path(__file__).resolve().parents[2],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr


def test_status_queries_initial_and_effective_extractor_settings_and_sink_recording():
    class Measured:
        def __init__(self):
            self.noise = MeasurementNoise(unit="K", enabled=True, stddev=0.5, seed=19)
            self.parameters = {"camera": {"width": 320, "gimbal": [0.0, 0.0]}}

        def measurement_settings(self):
            return {"noise": self.noise.to_dict(), **self.parameters}

        def configure_noise(self, noise):
            self.noise = noise

        def __call__(self, scheduled, source):
            return extract(scheduled, source)

    class Recorder(Sink):
        recording = True

        def set_recording(self, enabled):
            self.recording = enabled

    producer, sink = Measured(), Recorder()
    runtime, _ = session_for(plan_for("thermal", transport="in_process"), extractor=producer, sink=sink)
    initial = runtime.status("robot.sensor")
    assert initial["noise"] == producer.noise.to_dict()
    assert initial["noise_enabled"] is True
    assert initial["recording"] is True
    assert initial["measurement_settings"] == producer.measurement_settings()
    initial["measurement_settings"]["camera"]["gimbal"][0] = 99.0
    assert producer.parameters["camera"]["gimbal"] == [0.0, 0.0]
    runtime.configure_noise("robot.sensor", MeasurementNoise(unit="K", stddev=0.25, seed=7))
    runtime.set_recording("robot.sensor", False)
    # Settings may change through supported producer/sink controls outside this facade.
    producer.noise = MeasurementNoise(unit="K", enabled=True, stddev=0.75, seed=11)
    sink.recording = True
    effective = runtime.status("robot.sensor")
    assert effective["noise"] == producer.noise.to_dict()
    assert effective["noise_enabled"] is True
    assert effective["recording"] is True
    unsupported, _ = session_for(plan_for())
    unknown = unsupported.status("robot.sensor")
    assert unknown["noise"] is unknown["noise_enabled"] is unknown["recording"] is None
    assert unknown["measurement_settings"] is None
