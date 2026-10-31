"""Author sensor choices as SessionSpec data; ProductControl owns compilation/start."""

from __future__ import annotations

import math
import uuid
from copy import deepcopy
from pathlib import Path
from typing import Any

import yaml
from sim.catalog import CatalogResolver, PackageRecord
from sim.contracts import physics_sensor_timebase_violation

_LABELS = {
    "rgb": "RGB 相机",
    "depth": "深度相机",
    "lidar": "激光雷达",
    "imu": "IMU",
    "truth_odom": "真值里程计",
    "thermal": "热成像相机",
    "semantic": "语义／实例相机",
    "acoustic": "声学传感器",
    "gnss": "GNSS",
}
_NAV_PRODUCTS = {"nav", "map", "inspection", "tracking", "explore", "teleop_avoid"}
_CAMERA_PRESETS = {"pinhole", "equidistant_fisheye", "equirectangular"}
_CHOICE_FIELDS = ("enabled", "frequency_hz", "camera_preset", "pan_deg", "tilt_deg", "gnss_fix_available")


def _record(resolver: CatalogResolver, reference: str, kind: str, base: Path) -> PackageRecord:
    if "/" in reference or "\\" in reference or reference.endswith((".yaml", ".yml")):
        path = (base / reference).resolve()
        matches = [r for r in resolver.records if r.kind == kind and r.manifest_path.resolve() == path]
        if len(matches) != 1:
            raise ValueError(f"{kind} manifest is not in the validated catalog: {reference}")
        return matches[0]
    return resolver.find_package(reference, kind=kind)


def build_sensor_setup(resolver: CatalogResolver, session_source: Path, *, product: str = "nav") -> dict[str, Any]:
    session = yaml.safe_load(session_source.read_text(encoding="utf-8"))
    robots = session["robots"]
    if len(robots) != 1:
        raise ValueError("sensor selection requires one selected robot")
    robot_spec = robots[0]
    robot = _record(resolver, robot_spec["package"], "robot", session_source.parent)
    rig_ref = robot_spec.get("sensor_rig", robot.data["defaults"]["sensor_rig"])
    if not rig_ref:
        raise ValueError("selected robot has no declared sensor rig")
    rig = _record(resolver, rig_ref, "sensor_rig", session_source.parent)
    world = _record(resolver, session["world"], "world", session_source.parent)
    rows = []
    overrides = robot_spec.get("sensor_overrides", {})
    for sensor in rig.data["sensors"]:
        package = _record(resolver, sensor["package"], "sensor", rig.manifest_path.parent)
        modality = package.data["outputs"][0]
        if package.data["sensor_type"] == "lidar":
            modality = "lidar"
        required = product in _NAV_PRODUCTS and modality in {"imu", "lidar", "truth_odom"}
        required |= product == "inspection" and modality == "rgb"
        configuration = {**package.data["configuration"], **sensor.get("configuration", {})}
        option = overrides.get(sensor["id"], {})
        capabilities: dict[str, Any] = {}
        if modality in {"thermal", "semantic", "acoustic"}:
            measurement = configuration["measurement"]
            if "measurements" not in world.data:
                capabilities["unavailable_reason"] = "此 World 未声明测量属性；请先配置温度／标签／声源。"
            if modality in {"thermal", "semantic"}:
                gimbal = measurement.get("gimbal", {})
                capabilities.update(
                    camera_preset=option.get("camera_preset", measurement["camera"]["model"]),
                    pan_deg=option.get("pan_deg", gimbal.get("pan_deg", 0)),
                    tilt_deg=option.get("tilt_deg", gimbal.get("tilt_deg", 0)),
                    pan_limits_deg=gimbal.get("pan_limits_deg", [-180, 180]),
                    tilt_limits_deg=gimbal.get("tilt_limits_deg", [-90, 90]),
                    notice=("Kelvin 原始测量；未知表面像素无效。" if modality == "thermal" else
                            "类别／实例真值，非算法识别结果。") + "软件投影／云台，非实体电机。",
                )
            else:
                microphone = measurement["microphone"]
                capabilities.update(
                    fixed_frequency_hz=microphone["sample_rate_hz"] / microphone["block_samples"],
                    notice=f"{microphone['sample_rate_hz']} Hz、Pa 原始波形；频率为数据块率，非音效播放。",
                )
        if modality == "gnss":
            gnss = configuration["gnss"]
            capabilities.update(
                gnss_fix_available=option.get("gnss_fix_available", gnss.get("fix_available", False)),
                notice=(f"WGS84 原点 {gnss['origin_latitude_deg']:g}°, {gnss['origin_longitude_deg']:g}°，"
                        f"椭球高 {gnss['origin_altitude_m']:g} m（来自安装配置）。"
                        "未定位为 NO_FIX；模拟定位非实测、非 RTK；零原点不是场地测绘。"),
            )
        rows.append(
            dict(
                id=sensor["id"],
                package=package.ref,
                parent_frame=sensor["parent_frame"],
                enabled=overrides.get(sensor["id"], {}).get("enabled", True),
                frequency_hz=overrides.get(sensor["id"], {}).get(
                    "frequency_hz", sensor.get("frequency_hz", package.data["timing"]["rate_hz"])
                ),
                modality=modality,
                label=_LABELS.get(modality, package.data["sensor_type"]),
                required=required,
                **capabilities,
                **(
                    {"physics_timestep_s": world.data["physics"]["global_policy"]["timestep_s"]}
                    if sensor["runtime"]["owner"] == "physics"
                    else {}
                ),
            )
        )
    return dict(
        schema="lingtu.sim.sensor-setup.v1",
        instance_id=robot_spec["instance_id"],
        rig=dict(id=rig.id, version=rig.version),
        sensors=rows,
    )


def validate_sensor_selection(setup: dict[str, Any], selection: object) -> dict[str, dict[str, Any]]:
    if not isinstance(selection, dict) or set(selection) != {"schema", "instance_id", "sensors"}:
        raise ValueError("sensor selection requires schema, instance_id and sensors")
    if selection["schema"] != "lingtu.sim.sensor-selection.v1" or selection["instance_id"] != setup["instance_id"]:
        raise ValueError("sensor selection must match the selected robot instance")
    if not isinstance(selection["sensors"], list):
        raise ValueError("sensor selection sensors must be a list")
    known = {row["id"]: row for row in setup["sensors"]}
    overrides = {}
    for row in selection["sensors"]:
        if (
            not isinstance(row, dict)
            or not {"id", "enabled"} <= set(row)
            or set(row) - {"id", *_CHOICE_FIELDS}
        ):
            raise ValueError("sensor selection row fields are invalid")
        identity = row["id"]
        if not isinstance(identity, str) or identity not in known or identity in overrides:
            raise ValueError("sensor selection contains unknown or duplicate sensor")
        if type(row["enabled"]) is not bool:
            raise ValueError("sensor enabled must be boolean")
        if known[identity]["required"] and not row["enabled"]:
            raise ValueError(f"required navigation/inspection sensor cannot be disabled: {identity}")
        baseline = known[identity]
        if row["enabled"] and baseline.get("unavailable_reason"):
            raise ValueError(f"World cannot enable {identity}: {baseline['unavailable_reason']}")
        rate = row.get("frequency_hz", known[identity]["frequency_hz"])
        if isinstance(rate, bool) or not isinstance(rate, (int, float)) or not math.isfinite(rate) or rate <= 0:
            raise ValueError("frequency_hz must be positive and finite")
        overrides[identity] = dict(enabled=row["enabled"], frequency_hz=rate)
        if "fixed_frequency_hz" in baseline and rate != baseline["fixed_frequency_hz"]:
            raise ValueError(f"{identity} cadence must match its sample_rate_hz / block_samples")
        for key in ("camera_preset", "pan_deg", "tilt_deg", "gnss_fix_available"):
            if key not in baseline:
                if key in row:
                    raise ValueError(f"{identity} does not support {key}")
                continue
            value = row.get(key, baseline[key])
            if key == "camera_preset":
                if not isinstance(value, str) or value not in _CAMERA_PRESETS:
                    raise ValueError("camera_preset must name a supported calibrated projection")
            elif key == "gnss_fix_available":
                if type(value) is not bool:
                    raise ValueError("gnss_fix_available must be boolean")
            else:
                if (isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)
                        or not baseline[key.replace("_deg", "_limits_deg")][0] <= value <= baseline[key.replace("_deg", "_limits_deg")][1]):
                    raise ValueError(f"{identity}.{key} must be within its finite gimbal limits")
            overrides[identity][key] = value
        timestep = known[identity].get("physics_timestep_s")
        if row["enabled"] and timestep is not None:
            violation = physics_sensor_timebase_violation(
                timestep_s=timestep, streams=[{"owner": "physics", "sensor_id": identity, "rate_hz": rate}]
            )
            if violation is not None:
                raise ValueError(f"{identity} at {rate:g} Hz is incompatible with physics timestep {timestep:g} s")
    if set(overrides) != set(known):
        raise ValueError("sensor selection must explicitly cover the declared rig")
    if not any(row["enabled"] for row in overrides.values()):
        raise ValueError("at least one sensor must remain selected")
    return overrides


def write_sensor_selection(
    repo_root: Path, session_source: Path, selection: dict[str, Any] | None, output_dir: Path, *, product: str
) -> Path:
    root = repo_root.resolve()
    source = session_source.resolve()
    source.relative_to(root)
    resolver = CatalogResolver.from_repository(root)
    setup = build_sensor_setup(resolver, source, product=product)
    if selection is None:
        selection = dict(
            schema="lingtu.sim.sensor-selection.v1",
            instance_id=setup["instance_id"],
            sensors=[{key: row[key] for key in ("id", *_CHOICE_FIELDS) if key in row} for row in setup["sensors"]],
        )
    overrides = validate_sensor_selection(setup, selection)
    original = {row["id"]: {key: row[key] for key in _CHOICE_FIELDS if key in row} for row in setup["sensors"]}
    if overrides == original:
        return source
    destination = output_dir.resolve()
    destination.relative_to(root)
    if destination.exists():
        raise ValueError("authored session output must be new")
    document = deepcopy(yaml.safe_load(source.read_text(encoding="utf-8")))
    document["session_id"] = "sensor-selection-" + uuid.uuid4().hex
    robot = document["robots"][0]
    robot["sensor_overrides"] = overrides

    # Exact catalog references survive relocation; rebase explicit manifest paths.
    def reference(value: str, kind: str) -> str:
        return _record(resolver, value, kind, source.parent).ref

    document["world"] = reference(document["world"], "world")
    robot["package"] = reference(robot["package"], "robot")
    for key, kind in (("sensor_rig", "sensor_rig"), ("controller", "controller")):
        if robot.get(key):
            robot[key] = reference(robot[key], kind)
    destination.mkdir(parents=True, exist_ok=False)
    path = destination / "session.yaml"
    path.write_text(yaml.safe_dump(document, allow_unicode=True, sort_keys=False), encoding="utf-8")
    return path
