"""UE-owned RGB-D stays intact when physical measurements are mounted."""

from pathlib import Path

import pytest
import yaml
from sim.catalog import CatalogResolver

from tests.sim.test_multimodal_sensor_packages import (
    fixture_world,
    isolated_repository,
    repository,
)

ROOT = Path(__file__).resolve().parents[2]
RIG = "thunderv4_multimodal_unreal@1.0.0"


def test_unreal_multimodal_rig_preserves_navigation_cameras_and_adds_physical_measurements():
    catalog = CatalogResolver.from_repository(ROOT)
    rig = catalog.find_package(RIG, kind="sensor_rig").data
    baseline = catalog.find_package("thunderv4_navigation@1.0.0").data
    physical = catalog.find_package("thunderv4_headless_multimodal@1.0.0").data
    assert rig["sensors"][:5] == baseline["sensors"]
    for actual, original in zip(rig["sensors"][5:], physical["sensors"][5:], strict=True):
        for field in ("id", "package", "runtime", "frequency_hz"):
            assert actual[field] == original[field]
    assert len(rig["sensors"]) == 8
    for row in rig["sensors"][2:4]:
        assert row["runtime"] == {"owner": "visual", "source": "unreal_camera"}
        assert row["frequency_hz"] == 30
        assert row["configuration"] == {"width": 640, "height": 480}


@pytest.fixture
def mixed_owner_session(fixture_world):
    import shutil

    resolver, source, world = fixture_world
    directory = Path("sim/packages/sensor_rigs/doso/thunder_v4/multimodal_unreal")
    destination = resolver.repo_root / directory
    if not destination.exists():
        shutil.copytree(ROOT / directory, destination)
    document = yaml.safe_load(source.read_text(encoding="utf-8"))
    document["robots"][0]["sensor_rig"] = RIG
    document["runtime"]["mode"] = "unreal"
    document["runtime"]["required_bindings"] = ["physics", "control", "sensors", "visual"]
    source.write_text(yaml.safe_dump(document), encoding="utf-8")
    return CatalogResolver.from_repository(resolver.repo_root), source, world


def test_mixed_owner_plan_keeps_rendering_and_measurement_contracts_distinct(mixed_owner_session):
    resolver, source, world = mixed_owner_session
    resolved = resolver.resolve(source)
    streams = resolved.sensor_plan["streams"]
    assert sum(map(len, streams.values())) == 8
    for kind in ("rgb", "depth"):
        (row,) = streams[kind]
        assert row["rate_hz"] == 30
        assert (row["owner"], row["source"], row["transport"]) == ("visual", "unreal_camera", "camera_shm")
        assert (row["width"], row["height"]) == (640, 480)
    for kind in ("thermal", "semantic", "acoustic"):
        (row,) = streams[kind]
        assert row["owner"] == "physics"
        assert row["measurement"]["body_frame_id"] == (
            "thunder_01/base_link" if kind == "acoustic" else "thunder_01/camera1_link"
        )
        mount = row["measurement"]["mount_pose"]
        assert mount["position_m"] == pytest.approx([0, 0, 0.14] if kind == "acoustic" else [0, 0, 0])
    assert streams["depth"][0]["requested_encoding"] == "32FC1"
    assert streams["thermal"][0]["measurement"]["noise"]["unit"] == "K"
    assert streams["acoustic"][0]["measurement"]["noise"]["unit"] == "Pa"
    assert resolved.sensor_plan["scene_measurements"] == world["measurements"]


def test_mixed_owner_rig_and_plan_pass_strict_schema(mixed_owner_session):
    from tests.sim.test_multimodal_sensor_packages import _validate

    resolver, source, _ = mixed_owner_session
    _validate(resolver.find_package(RIG).data, "sensor-rig-package")
    _validate(resolver.resolve(source).sensor_plan, "sensor-plan")


@pytest.mark.parametrize(
    "case", ["unknown_field", "raw_mount", "noise_unit", "negative_focal", "missing_body", "wrong_owner"]
)
def test_measurement_rig_override_schema_rejects_bad_contracts(mixed_owner_session, case):
    import copy

    from tests.sim.test_multimodal_sensor_packages import _validate

    jsonschema = pytest.importorskip("jsonschema")
    resolver, _, _ = mixed_owner_session
    rig = copy.deepcopy(resolver.find_package(RIG).data)
    row = rig["sensors"][5]
    config = row["configuration"]["measurement"]
    if case == "unknown_field":
        config["invented"] = 1
    elif case == "raw_mount":
        config["mount_pose"] = {"position_m": [0, 0, 0], "orientation_wxyz": [1, 0, 0, 0]}
    elif case == "noise_unit":
        config["noise"]["unit"] = "Pa"
    elif case == "negative_focal":
        config["camera"]["fx_px"] = -1
    elif case == "missing_body":
        del config["body_frame_id"]
    elif case == "wrong_owner":
        row["runtime"]["owner"] = "visual"
    with pytest.raises(jsonschema.ValidationError):
        _validate(rig, "sensor-rig-package")


def test_mount_body_must_be_a_physical_body_not_just_a_declared_camera(mixed_owner_session):
    from sim.catalog import CatalogError

    resolver, source, _ = mixed_owner_session
    rig = resolver.find_package(RIG).data
    rig["sensors"][5]["configuration"]["measurement"]["body_frame_id"] = "front_camera"
    with pytest.raises(CatalogError, match="declared robot body frame"):
        resolver.resolve(source)


@pytest.mark.parametrize("projection", ["pinhole", "equidistant_fisheye", "equirectangular"])
def test_actual_robot_mount_pose_follows_body_and_gimbal_in_raw_frames(mixed_owner_session, projection, tmp_path):
    import copy
    import xml.etree.ElementTree as ET
    from contextlib import nullcontext

    import numpy as np
    from sim.runtime.recording.measurement_payload import decode_measurement_payload, encode_measurement_payload
    from sim.runtime.sensors import SensorRuntime
    from sim.runtime.sensors.measurement_configuration import make_measurement_extractor

    mujoco = pytest.importorskip("mujoco")
    resolver, source, _ = mixed_owner_session
    world = resolver.find_package("multimodal_contract_fixture@1.0.0")
    world_path = world.manifest_path.parent / world.data["physics"]["mjcf"]
    tree = ET.parse(world_path)
    target_body = tree.getroot().find(".//body[@name='fixture_emitter']")
    target_geom = tree.getroot().find(".//geom[@name='fixture_target']")
    assert target_body is not None and target_geom is not None
    target_body.set("pos", "2 0 0.72")
    target_geom.set("type", "box")
    target_geom.set("size", "0.1 0.65 0.65")
    tree.write(world_path, encoding="utf-8")
    resolver = CatalogResolver.from_repository(resolver.repo_root)
    plan = resolver.resolve(source).sensor_plan
    robot = resolver.find_package("thunderv4@1.0.3")
    spec = mujoco.MjSpec.from_file(str(robot.manifest_path.parent / robot.data["physics"]["mjcf"]))
    emitter = spec.worldbody.add_body(name="fixture_emitter", pos=list(map(float, target_body.get("pos").split())))
    emitter.add_geom(
        name="fixture_target", type=mujoco.mjtGeom.mjGEOM_BOX, size=list(map(float, target_geom.get("size").split()))
    )
    model = spec.compile()
    data = mujoco.MjData(model)
    joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, robot.data["physics"]["root_joint"])
    qadr = int(model.jnt_qposadr[joint])
    data.qpos[qadr : qadr + 3] = [0, 0, 0.6]
    data.qpos[qadr + 3 : qadr + 7] = [1, 0, 0, 0]
    runtime = SensorRuntime.from_plan(plan)
    extractors = {}
    for kind in ("thermal", "semantic"):
        stream = next(row for row in runtime.streams if row.stream_kind == kind)
        config = copy.deepcopy(plan["streams"][kind][0]["measurement"])
        if projection == "equidistant_fisheye":
            config["camera"] = dict(
                model=projection, width=64, height=48, focal_px=15, cx_px=32, cy_px=24, fov_deg=180, valid_radius_px=23
            )
        elif projection == "equirectangular":
            config["camera"] = dict(model=projection, width=96, height=48)
        extractors[kind] = make_measurement_extractor(
            stream, config, plan["scene_measurements"], lambda: nullcontext((model, data))
        )

    def capture(time_s):
        data.time = time_s
        mujoco.mj_forward(model, data)
        ns = round(time_s * 1e9)
        snapshot = dict(session_id=plan["session_id"], model_generation=0, reset_generation=0, sim_time_ns=ns)
        batch = runtime.advance(sim_time_ns=ns, model_generation=0, reset_generation=0)
        samples = {
            request.stream.stream_kind: extractors[request.stream.stream_kind](request, snapshot)
            for request in batch.samples
            if request.stream.stream_kind in extractors
        }
        thermal, semantic = samples["thermal"], samples["semantic"]
        np.testing.assert_array_equal(thermal.valid_mask, semantic.valid_mask)
        np.testing.assert_array_equal(semantic.class_ids[semantic.valid_mask], 2)
        np.testing.assert_array_equal(semantic.instance_ids[semantic.valid_mask], 7)
        assert np.isnan(thermal.temperature_k[~thermal.valid_mask]).all()
        expected_k = (0.9 * 310**4 + 0.1 * 293.15**4) ** 0.25
        np.testing.assert_allclose(thermal.temperature_k[thermal.valid_mask], expected_k, rtol=1e-5)
        rays = thermal.metadata["camera_calibration"]["ray_calibration"]
        assert rays["projection"]["model"] == projection
        assert thermal.stamp.sim_time_ns == ns
        for kind, sample in samples.items():
            encoded = encode_measurement_payload(sample, modality=kind)
            path = tmp_path / f"{ns}-{kind}.npz"
            path.write_bytes(encoded.payload)
            restored = decode_measurement_payload(path.read_bytes(), modality=kind)
            np.testing.assert_array_equal(restored.valid_mask, sample.valid_mask)
            assert restored.stamp == sample.stamp
        # Display projection of these actual arrays, not synthetic sensor imagery.
        from PIL import Image, ImageDraw

        height, width = thermal.valid_mask.shape
        thermal_rgb = np.full((height, width, 3), [218, 223, 229], dtype=np.uint8)
        thermal_rgb[thermal.valid_mask] = [199, 100, 66]
        semantic_rgb = np.full_like(thermal_rgb, [218, 223, 229])
        semantic_rgb[semantic.valid_mask] = [30, 121, 219]
        image = Image.new("RGB", (width * 8, height * 4 + 80), "white")
        image.paste(Image.fromarray(thermal_rgb).resize((width * 4, height * 4), Image.Resampling.NEAREST), (0, 48))
        image.paste(
            Image.fromarray(semantic_rgb).resize((width * 4, height * 4), Image.Resampling.NEAREST), (width * 4, 48)
        )
        draw = ImageDraw.Draw(image)
        draw.text((8, 6), f"{projection} | t={time_s:.1f}s | pan={rays['gimbal']['pan_deg']:g} deg", fill="black")
        draw.text((8, 26), f"THERMAL: {expected_k:.2f} K on valid pixels", fill="black")
        draw.text((width * 4 + 8, 26), "TRUTH: class 2 / instance 7", fill="black")
        draw.text(
            (8, height * 4 + 58),
            f"Gray = invalid | valid {int(thermal.valid_mask.sum())}/{width * height} | controlled target, not Factory",
            fill="black",
        )
        image.save(tmp_path / f"{ns}-preview.png")
        return thermal, rays

    front, rays = capture(0)
    assert front.valid_mask.sum() > 0
    np.testing.assert_allclose(
        rays["camera_pose_world"]["position_m"], [0.423358800364963, -0.000496202974186816, 0.71370714960317]
    )
    data.qpos[qadr + 3 : qadr + 7] = [0, 0, 0, 1]
    turned, rays = capture(0.1)
    np.testing.assert_allclose(
        rays["camera_pose_world"]["position_m"], [-0.423358800364963, 0.000496202974186816, 0.71370714960317]
    )
    assert not np.array_equal(front.valid_mask, turned.valid_mask)
    for extractor in extractors.values():
        extractor.set_gimbal_angles(180, 0)
    compensated, rays = capture(0.2)
    # Turning this nose-mounted optic back through its chassis cannot reveal a target.
    # The housing is excluded, but the rest of the robot remains an occluder.
    assert compensated.valid_mask.sum() == 0
    assert rays["gimbal"]["pan_deg"] == 180
    carrier = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "camera1_link")
    assert rays["base_pose"]["orientation_wxyz"] == pytest.approx(data.xquat[carrier])
    assert data.qpos[qadr + 3 : qadr + 7] == pytest.approx([0, 0, 0, 1])
    data.qpos[qadr + 3 : qadr + 7] = [1, 0, 0, 0]
    for extractor in extractors.values():
        extractor.set_gimbal_angles(15, 0)
    panned, rays = capture(0.3)
    assert panned.valid_mask.sum() > 0
    assert not np.array_equal(front.valid_mask, panned.valid_mask)
    assert rays["gimbal"]["pan_deg"] == 15
    assert data.qpos[qadr + 3 : qadr + 7] == pytest.approx([1, 0, 0, 0])


def test_actual_robot_microphone_mount_retains_pressure_delay_and_raw_recording(mixed_owner_session, tmp_path):
    import xml.etree.ElementTree as ET
    from contextlib import nullcontext

    import numpy as np
    from sim.runtime.recording.measurement_payload import decode_measurement_payload, encode_measurement_payload
    from sim.runtime.sensors import SensorRuntime
    from sim.runtime.sensors.measurement_configuration import make_measurement_extractor

    mujoco = pytest.importorskip("mujoco")
    resolver, source, _ = mixed_owner_session
    world = resolver.find_package("multimodal_contract_fixture@1.0.0")
    world_path = world.manifest_path.parent / world.data["physics"]["mjcf"]
    tree = ET.parse(world_path)
    emitter_body = tree.getroot().find(".//body[@name='fixture_emitter']")
    assert emitter_body is not None
    emitter_body.set("pos", "2 0 2.74")
    tree.write(world_path, encoding="utf-8")
    resolver = CatalogResolver.from_repository(resolver.repo_root)
    plan = resolver.resolve(source).sensor_plan
    robot = resolver.find_package("thunderv4@1.0.3")
    spec = mujoco.MjSpec.from_file(str(robot.manifest_path.parent / robot.data["physics"]["mjcf"]))
    spec.worldbody.add_body(name="fixture_emitter", pos=[2, 0, 2.74])
    model = spec.compile()
    data = mujoco.MjData(model)
    joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, robot.data["physics"]["root_joint"])
    qadr = int(model.jnt_qposadr[joint])
    data.qpos[qadr + 3 : qadr + 7] = [1, 0, 0, 0]
    observed = []
    for label, base_x in (("near", 0), ("far", -2)):
        data.qpos[qadr : qadr + 3] = [base_x, 0, 0.6]
        runtime = SensorRuntime.from_plan(plan)
        stream = next(row for row in runtime.streams if row.stream_kind == "acoustic")
        extractor = make_measurement_extractor(
            stream, plan["streams"]["acoustic"][0]["measurement"], plan["scene_measurements"],
            lambda: nullcontext((model, data)),
        )
        blocks = []
        for ns in (0, 10_000_000):
            data.time = ns / 1e9
            mujoco.mj_forward(model, data)
            qpos = data.qpos.copy()
            batch = runtime.advance(sim_time_ns=ns, model_generation=0, reset_generation=0)
            request = next(item for item in batch.samples if item.stream.stream_kind == "acoustic")
            frame = extractor(request, dict(session_id=plan["session_id"], model_generation=0,
                                            reset_generation=0, sim_time_ns=ns))
            assert frame.sample_rate_hz == 16000
            assert frame.pressure_pa.shape == (1, 160)
            assert frame.valid_mask.all()
            assert frame.metadata["pressure_unit"] == "Pa"
            assert frame.metadata["scene"]["receiver_body_name"] == "base_link"
            np.testing.assert_allclose(frame.metadata["channel_positions_world_m"], [[base_x, 0, 0.74]])
            np.testing.assert_allclose(frame.voltage_v, frame.pressure_pa * 0.02, atol=1e-10)
            np.testing.assert_array_equal(data.qpos, qpos)
            encoded = encode_measurement_payload(frame, modality="acoustic")
            path = tmp_path / f"{label}-{ns}-acoustic.npz"
            path.write_bytes(encoded.payload)
            restored = decode_measurement_payload(path.read_bytes(), modality="acoustic")
            np.testing.assert_array_equal(restored.pressure_pa, frame.pressure_pa)
            assert restored.stamp == frame.stamp
            blocks.append(frame.pressure_pa[0])
        pressure = np.concatenate(blocks)
        distance = float(np.linalg.norm([2 - base_x, 0, 2]))
        # The first non-zero clip sample is one source sample after t=0.
        first = int(np.flatnonzero(pressure)[0])
        assert first == pytest.approx(distance / 343 * 16000, abs=2)
        assert float(np.sum(pressure)) == pytest.approx(0, abs=1e-6)
        assert np.max(np.abs(pressure)) <= 0.1 / distance
        observed.append((first, np.linalg.norm(pressure)))
    assert observed[1][0] > observed[0][0]
    assert observed[1][1] < observed[0][1]
