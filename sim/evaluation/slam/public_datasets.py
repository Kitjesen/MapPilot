"""Public SLAM dataset catalog and native replay manifest helpers.

The catalog describes external data without downloading or parsing it. Dataset
specific tooling may normalize ROS bags offline, while LingTu's replay boundary
stays the native LTU1 stream consumed on canonical LiDAR and IMU topics.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

PUBLIC_DATASET_CATALOG_PATH = Path("sim/evaluation/slam/configs/public_datasets.json")
_CATALOG_SCHEMA = "lingtu.slam.public_datasets.v1"
_REPLAY_SCHEMA = "lingtu.slam.replay.v1"
_POINT_TIME_UNITS = {"ns", "us", "s"}


@dataclass(frozen=True)
class DatasetLicense:
    name: str
    terms_url: str
    redistribution_allowed: bool
    review_required: bool = False


@dataclass(frozen=True)
class LidarStream:
    topic: str
    message_type: str
    point_time_field: str | None = None
    point_time_unit: str | None = None


@dataclass(frozen=True)
class ImuStream:
    topic: str
    message_type: str
    acceleration_unit: str
    angular_velocity_unit: str


@dataclass(frozen=True)
class GroundTruthStream:
    available: bool
    format: str | None = None
    topic: str | None = None
    path: str | None = None
    canonical_format: str | None = None


@dataclass(frozen=True)
class PublicSlamDataset:
    id: str
    title: str
    official_url: str
    license: DatasetLicense
    sensor_model: str
    source_format: str
    lidar: LidarStream
    imu: ImuStream
    ground_truth: GroundTruthStream
    recommended_uses: tuple[str, ...]
    conversion_adapter: str
    download_hint: str | None = None
    notes: str | None = None

    def validate(self) -> list[str]:
        prefix = f"datasets[{self.id or '?'}]"
        errors: list[str] = []
        for field_name, value in (
            ("id", self.id),
            ("title", self.title),
            ("official_url", self.official_url),
            ("license.name", self.license.name),
            ("license.terms_url", self.license.terms_url),
            ("sensor_model", self.sensor_model),
            ("source_format", self.source_format),
            ("lidar.topic", self.lidar.topic),
            ("lidar.message_type", self.lidar.message_type),
            ("imu.topic", self.imu.topic),
            ("imu.message_type", self.imu.message_type),
            ("imu.acceleration_unit", self.imu.acceleration_unit),
            ("imu.angular_velocity_unit", self.imu.angular_velocity_unit),
            ("conversion_adapter", self.conversion_adapter),
        ):
            if not str(value or "").strip():
                errors.append(f"{prefix}.{field_name} is required")

        if self.license.review_required and self.license.redistribution_allowed:
            errors.append(f"{prefix}.license cannot allow redistribution while review_required is true")

        if "lio_frontend" in self.recommended_uses:
            if not self.lidar.point_time_field:
                errors.append(f"{prefix}.lidar.point_time_field is required for lio_frontend replay")
            if self.lidar.point_time_unit not in _POINT_TIME_UNITS:
                errors.append(f"{prefix}.lidar.point_time_unit must be one of {sorted(_POINT_TIME_UNITS)}")

        if self.ground_truth.available:
            if not self.ground_truth.format:
                errors.append(f"{prefix}.ground_truth.format is required when available")
            if not self.ground_truth.canonical_format:
                errors.append(f"{prefix}.ground_truth.canonical_format is required when available")
            if not (self.ground_truth.topic or self.ground_truth.path):
                errors.append(f"{prefix}.ground_truth requires topic or path when available")
        return errors


@dataclass(frozen=True)
class PublicDatasetCatalog:
    schema_version: str
    datasets: tuple[PublicSlamDataset, ...]
    priority_sets: dict[str, tuple[str, ...]]

    def require(self, dataset_id: str) -> PublicSlamDataset:
        for dataset in self.datasets:
            if dataset.id == dataset_id:
                return dataset
        raise KeyError(f"unknown public SLAM dataset: {dataset_id}")

    def validate(self) -> list[str]:
        errors: list[str] = []
        if self.schema_version != _CATALOG_SCHEMA:
            errors.append(f"schema_version must be {_CATALOG_SCHEMA}, got {self.schema_version or '<empty>'}")
        if not self.datasets:
            errors.append("datasets must not be empty")
        if "mid360" not in self.priority_sets:
            errors.append("priority_sets[mid360] is required")

        dataset_ids: set[str] = set()
        for dataset in self.datasets:
            if dataset.id in dataset_ids:
                errors.append(f"duplicate dataset id: {dataset.id}")
            dataset_ids.add(dataset.id)
            errors.extend(dataset.validate())

        for set_name, members in self.priority_sets.items():
            if not members:
                errors.append(f"priority_sets[{set_name}] must not be empty")
            for dataset_id in members:
                if dataset_id not in dataset_ids:
                    errors.append(f"priority_sets[{set_name}] references unknown dataset {dataset_id}")
                    continue
                if set_name == "mid360":
                    dataset = self.require(dataset_id)
                    if "mid360" not in dataset.sensor_model.lower():
                        errors.append(f"priority_sets[mid360] contains non-MID360 dataset {dataset_id}")
        return errors


def _license_from_dict(data: dict[str, Any]) -> DatasetLicense:
    return DatasetLicense(
        name=str(data.get("name", "")),
        terms_url=str(data.get("terms_url", "")),
        redistribution_allowed=bool(data.get("redistribution_allowed", False)),
        review_required=bool(data.get("review_required", False)),
    )


def _lidar_from_dict(data: dict[str, Any]) -> LidarStream:
    return LidarStream(
        topic=str(data.get("topic", "")),
        message_type=str(data.get("message_type", "")),
        point_time_field=(str(data["point_time_field"]) if data.get("point_time_field") is not None else None),
        point_time_unit=(str(data["point_time_unit"]) if data.get("point_time_unit") is not None else None),
    )


def _imu_from_dict(data: dict[str, Any]) -> ImuStream:
    return ImuStream(
        topic=str(data.get("topic", "")),
        message_type=str(data.get("message_type", "")),
        acceleration_unit=str(data.get("acceleration_unit", "")),
        angular_velocity_unit=str(data.get("angular_velocity_unit", "")),
    )


def _ground_truth_from_dict(data: dict[str, Any]) -> GroundTruthStream:
    return GroundTruthStream(
        available=bool(data.get("available", False)),
        format=str(data["format"]) if data.get("format") is not None else None,
        topic=str(data["topic"]) if data.get("topic") is not None else None,
        path=str(data["path"]) if data.get("path") is not None else None,
        canonical_format=(str(data["canonical_format"]) if data.get("canonical_format") is not None else None),
    )


def catalog_from_dict(data: dict[str, Any]) -> PublicDatasetCatalog:
    datasets = []
    for raw in data.get("datasets", ()):
        datasets.append(
            PublicSlamDataset(
                id=str(raw.get("id", "")),
                title=str(raw.get("title", "")),
                official_url=str(raw.get("official_url", "")),
                license=_license_from_dict(raw.get("license", {})),
                sensor_model=str(raw.get("sensor_model", "")),
                source_format=str(raw.get("source_format", "")),
                lidar=_lidar_from_dict(raw.get("lidar", {})),
                imu=_imu_from_dict(raw.get("imu", {})),
                ground_truth=_ground_truth_from_dict(raw.get("ground_truth", {})),
                recommended_uses=tuple(str(value) for value in raw.get("recommended_uses", ())),
                conversion_adapter=str(raw.get("conversion_adapter", "")),
                download_hint=(str(raw["download_hint"]) if raw.get("download_hint") is not None else None),
                notes=(str(raw["notes"]) if raw.get("notes") is not None else None),
            )
        )

    priority_sets = {
        str(name): tuple(str(value) for value in members) for name, members in data.get("priority_sets", {}).items()
    }
    return PublicDatasetCatalog(
        schema_version=str(data.get("schema_version", "")),
        datasets=tuple(datasets),
        priority_sets=priority_sets,
    )


def load_public_dataset_catalog(
    path: str | Path | None = None,
) -> PublicDatasetCatalog:
    if path is None:
        catalog_path = Path(__file__).parent / "configs" / "public_datasets.json"
    else:
        catalog_path = Path(path)
    with catalog_path.open("r", encoding="utf-8") as handle:
        catalog = catalog_from_dict(json.load(handle))
    errors = catalog.validate()
    if errors:
        raise ValueError("invalid public SLAM dataset catalog: " + "; ".join(errors))
    return catalog


def build_replay_manifest(
    dataset: PublicSlamDataset,
    *,
    source_path: str | Path,
    output_dir: str | Path,
    sequence: str | None = None,
) -> dict[str, Any]:
    errors = dataset.validate()
    if errors:
        raise ValueError("invalid public SLAM dataset: " + "; ".join(errors))

    source = Path(source_path)
    target_dir = Path(output_dir)
    sequence_name = sequence or source.stem or dataset.id
    if not sequence_name or sequence_name in {".", ".."} or Path(sequence_name).name != sequence_name:
        raise ValueError("sequence must be a single non-empty path component")

    ground_truth = dataset.ground_truth
    gt_source: dict[str, str] = {}
    if ground_truth.topic:
        gt_source["topic"] = ground_truth.topic
    if ground_truth.path:
        gt_source["path"] = ground_truth.path.replace("{sequence}", sequence_name)

    return {
        "schema_version": _REPLAY_SCHEMA,
        "dataset": {
            "id": dataset.id,
            "title": dataset.title,
            "official_url": dataset.official_url,
            "license": {
                "name": dataset.license.name,
                "terms_url": dataset.license.terms_url,
                "redistribution_allowed": (dataset.license.redistribution_allowed),
                "review_required": dataset.license.review_required,
            },
        },
        "sequence": sequence_name,
        "source": {
            "path": str(source),
            "format": dataset.source_format,
            "conversion_adapter": dataset.conversion_adapter,
        },
        "target": {
            "format": "LTU1",
            "record_path": str(target_dir / f"{sequence_name}.ltu"),
        },
        "streams": {
            "lidar": {
                "source_topic": dataset.lidar.topic,
                "source_message_type": dataset.lidar.message_type,
                "canonical_topic": "/lidar/raw_frame",
                "point_time": {
                    "field": dataset.lidar.point_time_field,
                    "unit": dataset.lidar.point_time_unit,
                },
            },
            "imu": {
                "source_topic": dataset.imu.topic,
                "source_message_type": dataset.imu.message_type,
                "canonical_topic": "/imu/raw",
                "acceleration_unit": dataset.imu.acceleration_unit,
                "angular_velocity_unit": (dataset.imu.angular_velocity_unit),
            },
        },
        "ground_truth_sidecar": {
            "enabled": ground_truth.available,
            "source_format": ground_truth.format,
            "source": gt_source,
            "canonical_format": ground_truth.canonical_format,
            "output_path": (
                str(target_dir / (f"{sequence_name}.ground_truth.{ground_truth.canonical_format}"))
                if ground_truth.available
                else None
            ),
        },
    }
