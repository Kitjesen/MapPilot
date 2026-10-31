"""Opt-in GNSS through the public Product compiler, without native launches."""

from __future__ import annotations

import math

import pytest
from sim.runtime.sensors.contracts import SensorRoute
from sim.runtime.sensors.gnss import GnssExtractor
from sim.runtime.sensors.runtime import SensorRuntime

from lingtu.assembly.compile_plan import compile_run_plan
from runtime.msgs.gnss import GnssFixType

GNSS_SESSION = "sim/sessions/products/doso/thunder_v4/gnss.yaml"
NAV_SESSION = "sim/sessions/products/doso/thunder_v4/nav.yaml"
pytestmark = pytest.mark.usefixtures("allow_unbuilt_process_artifacts")


@pytest.mark.parametrize("process_platform", ["windows", "linux"])
def test_formal_gnss_session_preserves_navigation_process_contract(
    process_platform: str,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr("lingtu.assembly.graph.processes._host_process_platform", lambda: process_platform)
    baseline = compile_run_plan("nav", "sim", robot="doso/thunder_v4", env_config={"backend": "mujoco"})
    opted_in = compile_run_plan(
        "nav",
        "sim",
        robot="doso/thunder_v4",
        env_config={"backend": "mujoco", "session": GNSS_SESSION},
    )

    assert opted_in.product == "nav"
    assert opted_in.env == "sim"
    assert opted_in.robot == "doso/thunder_v4"
    assert opted_in.process_control == "subprocess"
    assert opted_in.simulation["session_source"] == GNSS_SESSION
    assert baseline.simulation["session_source"] == NAV_SESSION
    assert [process.name for process in opted_in.processes] == [
        "driver_bridge",
        "imu_publisher",
        "lidar_publisher",
        "map_runtime",
        "mujoco_feeder",
        "nav_runtime",
        "slam_runtime",
        "host_runtime",
    ]
    # The formal feeder shares the existing authenticated IMU publisher connection;
    # GNSS adds records, not a new process, command, role, or environment variable.
    assert opted_in.processes == baseline.processes
    assert opted_in.available_processes == baseline.available_processes
    assert opted_in.support_processes == baseline.support_processes
    assert opted_in.stop_before_start == baseline.stop_before_start
    assert opted_in.process_environment == baseline.process_environment
    assert opted_in.native_process_environment == baseline.native_process_environment
    assert opted_in.process("imu").name == "imu_publisher"
    assert not opted_in.has_process("gnss")


def test_formal_gnss_is_additive_and_does_not_change_default_navigation() -> None:
    baseline = compile_run_plan("nav", "sim", robot="doso/thunder_v4", env_config={"backend": "mujoco"})
    opted_in = compile_run_plan(
        "nav",
        "sim",
        robot="doso/thunder_v4",
        env_config={"backend": "mujoco", "session": GNSS_SESSION},
    )
    unchanged = compile_run_plan("nav", "sim", robot="doso/thunder_v4", env_config={"backend": "mujoco"})

    assert unchanged == baseline
    navigation = baseline.simulation
    gnss = opted_in.simulation
    expected_rates = {"rgb": 1, "depth": 1, "imu": 200, "mid360": 10, "truth_odom": 100}
    assert set(navigation["sensor_plan"]["streams"]) == set(expected_rates)
    assert set(gnss["sensor_plan"]["streams"]) == set(expected_rates) | {"gnss"}
    for kind, rate in expected_rates.items():
        (original,) = navigation["sensor_plan"]["streams"][kind]
        (added,) = gnss["sensor_plan"]["streams"][kind]
        assert added == original
        assert added["rate_hz"] == rate

    nav_session = navigation["session"]
    gnss_session = gnss["session"]
    assert nav_session["session_id"] == "thunderv4_factory_campus_nav"
    assert gnss_session["session_id"] == "thunderv4_factory_campus_nav_gnss"
    assert gnss_session["robots"][0]["sensor_rig"] == "thunderv4_headless_gnss@1.0.0"
    assert nav_session["robots"][0]["sensor_rig"] == "thunderv4_headless_navigation@1.0.0"
    assert gnss_session["world"] == nav_session["world"] == "factory_workshop@2.0.0"
    gnss_session["session_id"] = nav_session["session_id"]
    gnss_session["robots"][0]["sensor_rig"] = nav_session["robots"][0]["sensor_rig"]
    assert gnss_session == nav_session
    assert gnss["control_plan"]["controllers"] == navigation["control_plan"]["controllers"]
    assert gnss["control_plan"]["command_channels"] == navigation["control_plan"]["command_channels"]

    original_channels = navigation["transport_intent"]["channels"]
    assert all(channel["payload_role"] != "gnss" for channel in original_channels)
    assert [channel for channel in gnss["transport_intent"]["channels"] if channel["payload_role"] != "gnss"] == (
        original_channels
    )


def test_formal_gnss_plan_routes_10hz_payload_mount_with_synthetic_no_fix_defaults() -> None:
    plan = compile_run_plan(
        "nav",
        "sim",
        robot="doso/thunder_v4",
        env_config={"backend": "mujoco", "session": GNSS_SESSION},
    )
    simulation = plan.simulation
    (declaration,) = simulation["sensor_plan"]["streams"]["gnss"]
    assert declaration["sensor_id"] == "thunder_01.gnss"
    assert declaration["rate_hz"] == 10
    assert declaration["frame_id"] == "thunder_01/payload_top"
    assert declaration["parent_frame_id"] == "thunder_01/base_link"
    assert declaration["extrinsic"] == {
        "position_m": [0, 0, 0.14],
        "quaternion_wxyz": [1, 0, 0, 0],
    }
    assert declaration["gnss"] == {
        "origin_latitude_deg": 0.0,
        "origin_longitude_deg": 0.0,
        "origin_altitude_m": 0.0,
        "body_frame_id": "thunder_01/base_link",
        "antenna_offset_m": [0.0, 0.0, 0.14],
        "world_origin_m": [0.0, 0.0, 0.0],
        "world_to_enu_yaw_deg": 0.0,
        "noise_enabled": False,
        "horizontal_std_m": 0.0,
        "vertical_std_m": 0.0,
        "seed": 0,
        "fix_available": False,
    }
    (channel,) = [
        channel for channel in simulation["transport_intent"]["channels"] if channel["payload_role"] == "gnss"
    ]
    assert channel == {
        "channel_id": "thunder_01.gnss",
        "direction": "publish",
        "owner": "physics",
        "source": "mujoco_gnss",
        "transport": "typed_dds",
        "delivery": "ordered_no_drop",
        "message_type": "lingtu.dds.GnssFix",
        "payload_role": "gnss",
        "contract": "field_dds_v1",
        "frame_policy": "source_time",
    }

    runtime = SensorRuntime.from_plan(simulation["sensor_plan"])
    (stream,) = [stream for stream in runtime.streams if stream.stream_kind == "gnss"]
    assert stream.rate_hz == 10
    assert stream.route == SensorRoute("physics", "mujoco_gnss", "typed_dds")
    assert stream.gnss is not None
    assert stream.gnss.noise_enabled is False
    assert stream.gnss.fix_available is False
    assert stream.gnss.antenna_offset_m == pytest.approx((0, 0, 0.14))
    extractor = GnssExtractor(stream.gnss)
    for sequence, time_ns in enumerate((0, 100_000_000)):
        (scheduled,) = runtime.advance(
            sim_time_ns=time_ns,
            model_generation=1,
            reset_generation=0,
        ).for_sensor("thunder_01.gnss")
        assert scheduled.deadline_ns == time_ns
        fix = extractor(
            scheduled,
            {
                "session_id": simulation["session"]["session_id"],
                "model_generation": 1,
                "reset_generation": 0,
                "sim_time_ns": time_ns,
                "bodies": [
                    {
                        "stable_id": "thunder_01/base_link",
                        "position_m": [61, 16.5, 0.02],
                        "quaternion_wxyz": [1, 0, 0, 0],
                    }
                ],
            },
        ).fix
        assert fix.fix_type == GnssFixType.NO_FIX
        assert all(math.isnan(value) for value in (fix.lat, fix.lon, fix.alt, *fix.covariance))
        assert fix.num_sat == fix.num_sat_used == 0
        assert fix.frame_id == "thunder_01/payload_top"
        assert fix.seq == sequence
        assert fix.ts == time_ns / 1_000_000_000
