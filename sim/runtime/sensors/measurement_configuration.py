"""Translate resolved measurement declarations into existing owner-thread extractors.

No engine is built or started here, and construction never acquires physics
state. Catalog owns mount resolution; physical name lookup remains in the
extractors at sampling time.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import asdict, fields, replace
from fractions import Fraction
from typing import Any

from .acoustic_scene import AcousticScene, AcousticSceneSource
from .acoustics import MicrophoneConfig
from .camera_geometry import (
    CameraPose,
    FisheyeCalibration,
    GimbalPose,
    PanoramaCalibration,
    PinholeCalibration,
    compose_camera_pose,
)
from .contracts import SensorRoute, SensorStreamPlan
from .measurement_control import MeasurementNoise
from .measurement_extractors import (
    AcousticExtractor,
    MountedCamera,
    PhysicsRead,
    SemanticCameraExtractor,
    ThermalCameraExtractor,
)
from .semantic_camera import SemanticCameraConfig, SemanticLabel
from .thermal import ThermalConfig, ThermalSurface


def _mapping(value: object, name: str) -> dict:
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be a mapping")
    return dict(value)


def _only(values: Mapping, allowed: set[str], name: str) -> None:
    unknown = values.keys() - allowed
    if unknown:
        raise ValueError(f"unsupported {name} fields: {sorted(map(str, unknown))}")


def _named(values: object, name: str) -> dict:
    declarations = _mapping(values, name)
    if any(not isinstance(key, str) or not key or key != key.strip() for key in declarations):
        raise ValueError(f"{name} keys must be non-empty trimmed physical geom names")
    return declarations


def _numeric(value: object, name: str) -> None:
    if isinstance(value, (tuple, list)):
        for item in value:
            _numeric(item, name)
    elif isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must contain numeric values, not strings or booleans")


def _declaration(cls: type, value: object, name: str, **supplied: Any) -> Any:
    values = _mapping(value, name)
    _only(values, {item.name for item in fields(cls) if item.init} - supplied.keys(), name)
    if cls in (CameraPose, GimbalPose, PinholeCalibration, FisheyeCalibration, PanoramaCalibration):
        for key, item in values.items():
            _numeric(item, f"{name}.{key}")
    try:
        return cls(**values, **supplied)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"invalid {name}: {exc}") from exc


def _body_name(value: object, instance_id: str) -> str:
    prefix = f"{instance_id}/"
    if not isinstance(value, str) or value != value.strip() or not value.startswith(prefix):
        raise ValueError("body_frame_id must be instance-qualified as '<instance>/<body>'")
    name = value[len(prefix) :]
    if not name or name != name.strip():
        raise ValueError("body_frame_id requires a physical body name")
    return name


def make_measurement_extractor(
    stream: SensorStreamPlan,
    configuration: Mapping,
    scene: Mapping,
    read_state: PhysicsRead,
) -> ThermalCameraExtractor | SemanticCameraExtractor | AcousticExtractor:
    """Build a declared measurement endpoint without touching the physics lease.

    Camera parameters are the calibrated dataclass fields plus ``model``.
    Thermal radiometry lives in ``thermal`` with required background temperature
    and calibration ID. ``noise`` is a top-level MeasurementNoise mapping with
    explicit unit K/Pa/label; omission means disabled noise in that unit.
    Microphone fields exclude noise. Its local channel positions are composed
    with mount_pose into the receiver body's frame. Acoustic sources are finite
    pressure lists, not looped generators. Scene geom/body names are looked up
    only by the existing extractors; only this stream's exact instance prefix
    is stripped from qualified body references.

    Unknown fields and invalid declarations raise ValueError before read_state
    can be acquired. The caller must skip this factory when no stream is active.
    """
    if stream.stream_kind not in {"thermal", "semantic", "acoustic"}:
        raise ValueError(f"unsupported measurement stream kind: {stream.stream_kind}")
    if stream.route not in (
        SensorRoute("physics", "mujoco_measurement", "typed_dds"),
        SensorRoute("physics", "mujoco_measurement", "in_process"),
    ):
        raise ValueError(f"unsupported measurement route: {stream.route}")
    config = _mapping(configuration, "configuration")
    allowed = {"body_frame_id", "mount_pose", "noise"}
    if stream.stream_kind == "acoustic":
        allowed.add("microphone")
    else:
        allowed.update({"camera", "gimbal", "max_distance_m"})
    if stream.stream_kind == "thermal":
        allowed.add("thermal")
    _only(config, allowed, "configuration")
    world = _mapping(scene, "scene")
    _only(world, {"thermal_surfaces", "semantic_labels", "acoustic_sources"}, "scene")
    body = _body_name(config.get("body_frame_id"), stream.instance_id)
    mount = _declaration(CameraPose, config.get("mount_pose", {}), "mount_pose")
    units = {"thermal": "K", "semantic": "label", "acoustic": "Pa"}
    noise = _declaration(MeasurementNoise, config.get("noise", {"unit": units[stream.stream_kind]}), "noise")
    if stream.stream_kind == "acoustic":
        microphone = _declaration(MicrophoneConfig, config.get("microphone"), "microphone", noise=noise)
        microphone = replace(
            microphone,
            channel_offsets_m=tuple(
                compose_camera_pose(mount, CameraPose(offset), GimbalPose()).position_m
                for offset in microphone.channel_offsets_m
            ),
        )
        sources = []
        declarations = world.get("acoustic_sources", [])
        if not isinstance(declarations, list):
            raise ValueError("acoustic_sources must be a list of finite source declarations")
        for values in declarations:
            declaration = _mapping(values, "acoustic_sources entry")
            if not isinstance(declaration.get("pressure_pa"), list):
                raise ValueError("pressure_pa must be a finite non-empty mono waveform list")
            source_body = declaration.get("body_name")
            if isinstance(source_body, str) and source_body.startswith(f"{stream.instance_id}/"):
                declaration["body_name"] = _body_name(source_body, stream.instance_id)
            sources.append(_declaration(AcousticSceneSource, declaration, "acoustic_sources entry"))
        acoustic = AcousticExtractor(
            read_state,
            AcousticScene(microphone, receiver_body_name=body, sources=tuple(sources)),
        )
        if stream.rate_hz != Fraction(microphone.sample_rate_hz, microphone.block_samples):
            raise ValueError("acoustic cadence must equal sample_rate_hz / block_samples")
        acoustic.set_rate_hz(float(stream.rate_hz))
        return acoustic
    camera_values = _mapping(config.get("camera"), "camera")
    model = camera_values.pop("model", None)
    if not isinstance(model, str):
        raise ValueError("camera model must name a supported calibrated projection")
    calibration_type = {
        "pinhole": PinholeCalibration,
        "equidistant_fisheye": FisheyeCalibration,
        "equirectangular": PanoramaCalibration,
    }.get(model)
    if calibration_type is None:
        raise ValueError(f"unsupported camera model: {model}")
    calibration = _declaration(calibration_type, camera_values, "camera")
    max_distance = config.get("max_distance_m", 100.0)
    if isinstance(max_distance, bool) or not isinstance(max_distance, (int, float)):
        raise ValueError("max_distance_m must be a finite distance exceeding .001 m")
    camera = MountedCamera(
        body,
        calibration,
        mount,
        _declaration(GimbalPose, config.get("gimbal", {}), "gimbal"),
        max_distance,
    )
    if stream.stream_kind == "semantic":
        # Temporary indices validate label consistency, not physical geom IDs.
        # The extractor resolves the declared names against the leased model.
        labels = {
            name: _declaration(SemanticLabel, values, f"semantic_labels.{name}", geom_id=index)
            for index, (name, values) in enumerate(_named(world.get("semantic_labels", {}), "semantic_labels").items())
        }
        SemanticCameraConfig(calibration, tuple(labels.values()), noise)
        extractor = SemanticCameraExtractor(
            read_state,
            camera,
            {name: (label.class_id, label.instance_id) for name, label in labels.items()},
        )
        extractor.configure_noise(noise)
        return extractor
    thermal = _mapping(config.get("thermal"), "thermal")
    calibration_id = thermal.pop("calibration_id", None)
    intrinsic = {
        "model": model,
        "width": calibration.width,
        "height": calibration.height,
        "calibration_id": calibration_id,
    }
    # Match thermal_calibration's intrinsic normalization without inventing a
    # measurement stamp or world pose. Actual ray metadata is added at sampling.
    if model == "pinhole":
        intrinsic.update({key: asdict(calibration)[f"{key}_px"] for key in ("fx", "fy", "cx", "cy")})
    else:
        projection = {"model": model, **asdict(calibration)}
        if model == "equirectangular":
            projection["angle_convention"] = (
                "longitude=atan2(x,z),latitude=atan2(-y,hypot(x,z)); u increases longitude,v decreases latitude"
            )
        intrinsic["parameters"] = projection
    thermal_config = _declaration(ThermalConfig, thermal, "thermal", camera_calibration=intrinsic, noise=noise)
    surfaces = {
        name: _declaration(ThermalSurface, values, f"thermal_surfaces.{name}")
        for name, values in _named(world.get("thermal_surfaces", {}), "thermal_surfaces").items()
    }
    return ThermalCameraExtractor(read_state, camera, thermal_config, surfaces)
