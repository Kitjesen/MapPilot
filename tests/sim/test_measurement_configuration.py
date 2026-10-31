"""Declared measurements use existing extractors and real MuJoCo geometry."""

from contextlib import contextmanager
from fractions import Fraction

import mujoco
import numpy as np
import pytest
from sim.runtime.sensors.contracts import DeadlinePolicy, ScheduledSensorSample, SensorRoute, SensorStreamPlan
from sim.runtime.sensors.measurement_configuration import make_measurement_extractor
from sim.runtime.sensors.measurement_extractors import ThermalCameraExtractor


def _stream(kind="thermal", transport="in_process", rate_hz=None):
    return SensorStreamPlan(
        kind,
        "robot",
        f"robot.{kind}",
        f"robot/{kind}",
        "measurement",
        Fraction(rate_hz or (100 if kind == "acoustic" else 30)),
        SensorRoute("physics", "mujoco_measurement", transport),
    )


def _configuration(kind="thermal"):
    result = {"body_frame_id": "robot/receiver"}
    if kind in {"thermal", "semantic"}:
        result["camera"] = {
            "model": "pinhole",
            "width": 3,
            "height": 3,
            "fx_px": 3,
            "fy_px": 3,
            "cx_px": 1.5,
            "cy_px": 1.5,
        }
    if kind == "thermal":
        result["thermal"] = {"background_temperature_k": 293.15, "calibration_id": "declared-lens"}
    return result


def _world():
    model = mujoco.MjModel.from_xml_string("""<mujoco><worldbody>
      <body name="receiver"><geom type="sphere" size=".01"/></body>
      <body name="pump" pos="2 0 0"><geom type="sphere" size=".1"/></body>
      <geom name="wall" type="box" pos="0 0 4" size="4 4 .1" rgba="0 0 1 1"/>
      </worldbody></mujoco>""")
    return model, mujoco.MjData(model)


def _sample(extractor, stream, model, data, time_ns=0):
    data.time = time_ns / 1e9
    request = ScheduledSensorSample("fixture", stream, 2, 3, 0, time_ns, DeadlinePolicy.DROP_INTERMEDIATE)
    snapshot = dict(session_id="fixture", model_generation=2, reset_generation=3, sim_time_ns=time_ns)
    return extractor(request, snapshot)


@pytest.mark.parametrize("transport", ["in_process", "typed_dds"])
def test_thermal_configuration_produces_declared_kelvin_without_acquiring_state_at_construction(transport):
    model, data = _world()
    leases = []

    @contextmanager
    def read_state():
        leases.append("enter")
        yield model, data
        leases.append("exit")

    configuration = _configuration()
    configuration["mount_pose"] = {"position_m": [0, 0, 0.1], "orientation_wxyz": [1, 0, 0, 0]}
    scene = {"thermal_surfaces": {"wall": {"temperature_k": 400, "emissivity": 1}}}
    extractor = make_measurement_extractor(_stream(transport=transport), configuration, scene, read_state)
    assert isinstance(extractor, ThermalCameraExtractor)
    assert leases == []
    result = _sample(extractor, _stream(transport=transport), model, data)
    np.testing.assert_array_equal(result.temperature_k, 400)
    calibration = result.metadata["camera_calibration"]
    assert calibration["calibration_id"] == "declared-lens"
    assert calibration["ray_calibration"]["mount_pose"]["position_m"] == [0, 0, 0.1]
    assert result.stamp.sim_time_ns == 0
    assert leases == ["enter", "exit"]


@pytest.mark.parametrize(
    "camera",
    [
        {
            "model": "equidistant_fisheye",
            "width": 3,
            "height": 3,
            "focal_px": 3,
            "cx_px": 1.5,
            "cy_px": 1.5,
            "fov_deg": 180,
            "valid_radius_px": 3,
        },
        {
            "model": "equirectangular",
            "width": 3,
            "height": 3,
            "longitude_min_deg": -20,
            "longitude_max_deg": 20,
            "latitude_min_deg": -20,
            "latitude_max_deg": 20,
        },
    ],
)
def test_wide_angle_intrinsics_agree_with_real_thermal_calibration(camera):
    from contextlib import nullcontext

    model, data = _world()
    configuration = _configuration()
    configuration["camera"] = camera
    extractor = make_measurement_extractor(
        _stream(),
        configuration,
        {"thermal_surfaces": {"wall": {"temperature_k": 410, "emissivity": 1}}},
        lambda: nullcontext((model, data)),
    )
    assert "ray_calibration" not in extractor.config.camera_calibration
    result = _sample(extractor, _stream(), model, data, 34_000_000)
    np.testing.assert_array_equal(result.temperature_k, 410)
    assert result.metadata["camera_calibration"]["model"] == camera["model"]
    assert result.stamp.sim_time_ns == 34_000_000


@pytest.mark.parametrize("transport", ["in_process", "typed_dds"])
def test_semantic_configuration_maps_explicit_ids_and_preserves_gimbal_noise(transport):
    from contextlib import nullcontext

    from sim.runtime.sensors.measurement_extractors import SemanticCameraExtractor

    model, data = _world()
    stream = _stream("semantic", transport)
    configuration = _configuration("semantic")
    configuration["gimbal"] = {"pan_deg": 0, "tilt_deg": 0, "pan_limits_deg": [-45, 45], "tilt_limits_deg": [-10, 10]}
    configuration["noise"] = {"unit": "label", "enabled": False, "stddev": 0.2, "seed": 19}
    extractor = make_measurement_extractor(
        stream,
        configuration,
        {"semantic_labels": {"wall": {"class_id": 7, "instance_id": 12}}},
        lambda: nullcontext((model, data)),
    )
    assert isinstance(extractor, SemanticCameraExtractor)
    result = _sample(extractor, stream, model, data)
    np.testing.assert_array_equal(result.class_ids, 7)
    np.testing.assert_array_equal(result.instance_ids, 12)
    assert result.ground_truth and result.simulation
    assert extractor.measurement_settings()["noise"] == configuration["noise"]
    assert result.calibration_metadata["gimbal"]["pan_limits_deg"] == [-45, 45]


def _acoustic_configuration():
    return {
        "body_frame_id": "robot/receiver",
        "microphone": {
            "sample_rate_hz": 8000,
            "block_samples": 80,
            "channel_offsets_m": [[0, 0, 0], [1, 0, 0]],
            "speed_of_sound_mps": 343,
            "sensitivity_volts_per_pa": 0.02,
            "calibration_metadata": {"calibration_id": "mic-1"},
        },
    }


def _acoustic_scene():
    return {
        "acoustic_sources": [
            {
                "source_id": "pump-signal",
                "body_name": "robot/pump",
                "pressure_pa": [1.0] * 8000,
                "sample_rate_hz": 8000,
                "start_sim_time_ns": 0,
                "reference_distance_m": 1,
                "local_offset_m": [0, 0, 0],
            }
        ]
    }


@pytest.mark.parametrize("transport", ["in_process", "typed_dds"])
def test_acoustic_configuration_applies_mount_and_has_finite_duration(transport):
    from contextlib import nullcontext

    from sim.runtime.sensors.measurement_extractors import AcousticExtractor

    model, data = _world()
    configuration = _acoustic_configuration()
    configuration["mount_pose"] = {"position_m": [0, 1, 0], "orientation_wxyz": [0, 0, 0, 1]}
    stream = _stream("acoustic", transport)
    extractor = make_measurement_extractor(
        stream,
        configuration,
        _acoustic_scene(),
        lambda: nullcontext((model, data)),
    )
    assert isinstance(extractor, AcousticExtractor)
    result = _sample(extractor, stream, model, data, 100_000_000)
    assert result.valid_mask.all()
    np.testing.assert_allclose(result.pressure_pa[0], 1 / np.sqrt(5))
    np.testing.assert_allclose(result.pressure_pa[1], 1 / np.sqrt(10))
    np.testing.assert_allclose(result.voltage_v, result.pressure_pa * 0.02)
    np.testing.assert_allclose(result.metadata["channel_positions_world_m"], [[0, 1, 0], [-1, 1, 0]])
    assert result.metadata["sources"][0]["source_id"] == "pump-signal"
    after_end = _sample(extractor, stream, model, data, 2_000_000_000)
    np.testing.assert_array_equal(after_end.pressure_pa, 0)
    assert after_end.valid_mask.all()


def test_explicit_loop_configuration_remains_measurable_after_arrival_time():
    from contextlib import nullcontext

    model, data = _world()
    scene = _acoustic_scene()
    scene["acoustic_sources"][0]["loop"] = True
    stream = _stream("acoustic")
    extractor = make_measurement_extractor(
        stream, _acoustic_configuration(), scene, lambda: nullcontext((model, data))
    )
    result = _sample(extractor, stream, model, data, 10_000_000_000)
    assert result.valid_mask.all()
    assert np.max(np.abs(result.pressure_pa)) > 0
    assert result.metadata["sources"][0]["loop"] is True


def _no_state():
    pytest.fail("factory must not read physics state")


@pytest.mark.parametrize(
    "sources",
    [
        None,
        {},
        "pump",
        [1],
        [
            {
                "source_id": "pump",
                "body_name": "pump",
                "pressure_pa": [True],
                "sample_rate_hz": 8000,
                "start_sim_time_ns": 0,
                "reference_distance_m": 1,
            }
        ],
        [
            {
                "source_id": "pump",
                "body_name": "pump",
                "pressure_pa": np.ones(8),
                "sample_rate_hz": 8000,
                "start_sim_time_ns": 0,
                "reference_distance_m": 1,
            }
        ],
    ],
)
def test_invalid_acoustic_source_declarations_fail_before_physics(sources):
    with pytest.raises(ValueError):
        make_measurement_extractor(
            _stream("acoustic"),
            _acoustic_configuration(),
            {"acoustic_sources": sources},
            _no_state,
        )


@pytest.mark.parametrize(
    "field,value",
    [
        ("body_frame_id", None),
        ("body_frame_id", "receiver"),
        ("body_frame_id", "other/receiver"),
        ("body_frame_id", "robotic/receiver"),
        ("body_frame_id", "robot/"),
        ("ui_camera", {}),
        ("mount_pose", {"yaw": 0}),
        ("gimbal", {"roll_deg": 1}),
        ("camera", {"model": []}),
        ("max_distance_m", "far"),
        ("max_distance_m", True),
    ],
)
def test_invalid_configuration_fields_fail_before_physics(field, value):
    configuration = _configuration()
    configuration[field] = value
    with pytest.raises(ValueError):
        make_measurement_extractor(_stream(), configuration, {}, _no_state)


@pytest.mark.parametrize("kind,section", [("thermal", "thermal_surfaces"), ("semantic", "semantic_labels")])
def test_scene_geom_names_must_be_explicit_trimmed_names(kind, section):
    value = {"temperature_k": 400, "emissivity": 1} if kind == "thermal" else {"class_id": 1, "instance_id": 1}
    with pytest.raises(ValueError, match="name"):
        make_measurement_extractor(_stream(kind), _configuration(kind), {section: {3: value}}, _no_state)


@pytest.mark.parametrize("rate", [Fraction(200), Fraction(100 * 10**20 + 1, 10**20)])
def test_acoustic_initial_rate_must_exactly_equal_sample_rate_over_block(rate):
    with pytest.raises(ValueError, match="cadence"):
        make_measurement_extractor(_stream("acoustic", rate_hz=rate), _acoustic_configuration(), {}, _no_state)


@pytest.mark.parametrize("kind,unit", [("thermal", "K"), ("acoustic", "Pa")])
def test_central_noise_parameters_control_deterministic_measurements(kind, unit):
    from contextlib import nullcontext

    model, data = _world()
    configuration = _configuration() if kind == "thermal" else _acoustic_configuration()
    scene = (
        {"thermal_surfaces": {"wall": {"temperature_k": 400, "emissivity": 1}}}
        if kind == "thermal"
        else _acoustic_scene()
    )
    configuration["noise"] = {"unit": unit, "enabled": True, "stddev": 0.5, "seed": 73}

    def measure():
        extractor = make_measurement_extractor(
            _stream(kind),
            configuration,
            scene,
            lambda: nullcontext((model, data)),
        )
        assert extractor.measurement_settings()["noise"] == configuration["noise"]
        frame = _sample(extractor, _stream(kind), model, data, 100_000_000)
        return frame.temperature_k if kind == "thermal" else frame.pressure_pa

    first = measure()
    np.testing.assert_array_equal(measure(), first)
    configuration["noise"]["seed"] = 74
    assert not np.array_equal(measure(), first)
    configuration["noise"]["enabled"] = False
    clean = measure()
    assert not np.array_equal(first, clean)
    configuration["noise"].update(enabled=True, stddev=0)
    np.testing.assert_array_equal(measure(), clean)


@pytest.mark.parametrize("kind", ["thermal", "semantic", "acoustic"])
@pytest.mark.parametrize(
    "noise",
    [
        {"enabled": False},
        {"unit": "wrong"},
        {"unit": "K", "stddev": -1},
        {"unit": "K", "enabled": 1},
        {"unit": "K", "seed": -1},
        {"unit": "K", "extra": 1},
    ],
)
def test_invalid_noise_is_rejected_before_physics(kind, noise):
    configuration = _acoustic_configuration() if kind == "acoustic" else _configuration(kind)
    configuration["noise"] = noise
    with pytest.raises(ValueError):
        make_measurement_extractor(_stream(kind), configuration, {}, _no_state)


@pytest.mark.parametrize(
    "section,extra",
    [
        ("thermal", {"camera_calibration": {}}),
        ("thermal", {"noise": {"unit": "K"}}),
        ("thermal", {"material": "metal"}),
        ("camera", {"fov": 90}),
        ("mount_pose", {"scale": 1}),
        ("gimbal", {"speed": 1}),
    ],
)
def test_nested_unknown_fields_are_not_silently_ignored(section, extra):
    configuration = _configuration()
    configuration.setdefault(section, {}).update(extra)
    with pytest.raises(ValueError, match="unsupported"):
        make_measurement_extractor(_stream(), configuration, {}, _no_state)


@pytest.mark.parametrize("missing", ["background_temperature_k", "calibration_id"])
def test_thermal_requires_explicit_background_and_calibration_id(missing):
    configuration = _configuration()
    del configuration["thermal"][missing]
    with pytest.raises(ValueError, match=missing):
        make_measurement_extractor(_stream(), configuration, {}, _no_state)


@pytest.mark.parametrize(
    "route",
    [
        SensorRoute("visual", "mujoco_measurement", "in_process"),
        SensorRoute("physics", "render", "typed_dds"),
        SensorRoute("physics", "mujoco_measurement", "dds"),
    ],
)
def test_unsupported_route_does_not_construct_a_measurement(route):
    from dataclasses import replace

    with pytest.raises(ValueError, match="route"):
        make_measurement_extractor(replace(_stream(), route=route), _configuration(), {}, _no_state)


@pytest.mark.parametrize("kind", ["rgb", "depth", "instance", "", "imu"])
def test_only_declared_measurement_stream_kinds_are_supported(kind):
    with pytest.raises(ValueError, match="kind"):
        make_measurement_extractor(_stream(kind), {}, {}, _no_state)


@pytest.mark.parametrize(
    "section,values",
    [
        ("camera", {"cx_px": "1.5"}),
        ("camera", {"fx_px": True}),
        ("mount_pose", {"position_m": ["0", 0, 0]}),
        ("gimbal", {"pan_deg": True}),
    ],
)
def test_geometry_rejects_non_numeric_declaration_values_before_physics(section, values):
    configuration = _configuration()
    configuration.setdefault(section, {}).update(values)
    with pytest.raises(ValueError):
        make_measurement_extractor(_stream(), configuration, {}, _no_state)


@pytest.mark.parametrize("kind", ["thermal", "semantic"])
def test_undeclared_surfaces_are_not_inferred_from_render_color_or_material(kind):
    from contextlib import nullcontext

    model, data = _world()
    extractor = make_measurement_extractor(
        _stream(kind),
        _configuration(kind),
        {},
        lambda: nullcontext((model, data)),
    )
    result = _sample(extractor, _stream(kind), model, data)
    assert not result.valid_mask.any()
    if kind == "thermal":
        assert np.isnan(result.temperature_k).all()
    else:
        assert not result.class_ids.any() and not result.instance_ids.any()


def test_all_declared_thermal_parameters_reach_existing_radiometry():
    from contextlib import nullcontext

    model, data = _world()
    configuration = _configuration()
    configuration["thermal"].update(
        temperature_mode="surface_corrected",
        transmittance=0.7,
        atmospheric_temperature_k=290,
        min_temperature_k=300,
        max_temperature_k=450,
    )
    extractor = make_measurement_extractor(
        _stream(),
        configuration,
        {"thermal_surfaces": {"wall": {"temperature_k": 400, "emissivity": 0.6}}},
        lambda: nullcontext((model, data)),
    )
    result = _sample(extractor, _stream(), model, data)
    np.testing.assert_array_equal(result.temperature_k, 400)
    assert extractor.config.min_temperature_k == 300
    assert extractor.config.max_temperature_k == 450
    assert extractor.config.transmittance == 0.7
    assert extractor.config.atmospheric_temperature_k == 290


@pytest.mark.parametrize(
    "change", [{"max_distance_m": 1}, {"gimbal": {"pan_deg": 180}}, {"mount_pose": {"orientation_wxyz": [0, 0, 1, 0]}}]
)
def test_declared_range_gimbal_and_mount_rotation_change_actual_visibility(change):
    from contextlib import nullcontext

    model, data = _world()
    configuration = _configuration("semantic")
    configuration.update(change)
    extractor = make_measurement_extractor(
        _stream("semantic"),
        configuration,
        {"semantic_labels": {"wall": {"class_id": 1, "instance_id": 1}}},
        lambda: nullcontext((model, data)),
    )
    assert not _sample(extractor, _stream("semantic"), model, data).valid_mask.any()


def test_name_lookup_is_deferred_to_the_real_extractor_lease():
    model, data = _world()
    leases = []

    @contextmanager
    def read_state():
        leases.append("enter")
        try:
            yield model, data
        finally:
            leases.append("exit")

    extractor = make_measurement_extractor(
        _stream(),
        _configuration(),
        {"thermal_surfaces": {"missing": {"temperature_k": 400, "emissivity": 1}}},
        read_state,
    )
    assert not leases
    with pytest.raises(ValueError, match="geom not found"):
        _sample(extractor, _stream(), model, data)
    assert leases == ["enter", "exit"]


def test_semantic_enabled_noise_and_conflicting_instance_classes_fail_before_physics():
    configuration = _configuration("semantic")
    configuration["noise"] = {"unit": "label", "enabled": True, "stddev": 1, "seed": 7}
    with pytest.raises(ValueError, match="unsupported"):
        make_measurement_extractor(_stream("semantic"), configuration, {}, _no_state)
    configuration["noise"]["enabled"] = False
    with pytest.raises(ValueError, match="one class per instance"):
        make_measurement_extractor(
            _stream("semantic"),
            configuration,
            {
                "semantic_labels": {
                    "first": {"class_id": 1, "instance_id": 10},
                    "second": {"class_id": 2, "instance_id": 10},
                }
            },
            _no_state,
        )


@pytest.mark.parametrize(
    "field,value",
    [
        ("pressure_pa", []),
        ("pressure_pa", [float("nan")]),
        ("pressure_pa", [[1]]),
        ("loop", "true"),
        ("start_sim_time_ns", -1),
        ("sample_rate_hz", 0),
        ("reference_distance_m", 0),
        ("local_offset_m", [0, 0]),
        ("noise", {"unit": "Pa"}),
    ],
)
def test_invalid_finite_source_fields_are_rejected(field, value):
    scene = _acoustic_scene()
    scene["acoustic_sources"][0][field] = value
    with pytest.raises(ValueError):
        make_measurement_extractor(_stream("acoustic"), _acoustic_configuration(), scene, _no_state)


def test_microphone_cannot_override_central_noise_and_source_ids_are_unique():
    configuration = _acoustic_configuration()
    configuration["microphone"]["noise"] = {"unit": "Pa"}
    with pytest.raises(ValueError, match="unsupported"):
        make_measurement_extractor(_stream("acoustic"), configuration, {}, _no_state)
    del configuration["microphone"]["noise"]
    scene = _acoustic_scene()
    scene["acoustic_sources"].append(dict(scene["acoustic_sources"][0]))
    with pytest.raises(ValueError, match="unique"):
        make_measurement_extractor(_stream("acoustic"), configuration, scene, _no_state)
