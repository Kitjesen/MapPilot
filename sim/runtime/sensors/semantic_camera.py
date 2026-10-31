"""Simulation semantic ground truth from physical nearest-hit geometry IDs.

The physics rasterizer owns visibility/occlusion. This module only maps its
integer IDs to declared stable labels: it never classifies RGB or invents hits.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field

import numpy as np

from .camera_geometry import (
    CameraCalibration,
    CameraRaySample,
    FisheyeCalibration,
    PanoramaCalibration,
    PinholeCalibration,
    calibrated_rays,
)
from .measurement_control import MeasurementNoise
from .samples import SensorSampleStamp


@dataclass(frozen=True, slots=True)
class SemanticLabel:
    """One physical geom ID; several geoms may belong to one stable instance."""

    geom_id: int
    class_id: int
    instance_id: int


@dataclass(frozen=True, slots=True)
class SemanticCameraConfig:
    calibration: CameraCalibration
    labels: tuple[SemanticLabel, ...]
    noise: MeasurementNoise = field(default_factory=lambda: MeasurementNoise(unit="label"))

    def __post_init__(self) -> None:
        if not isinstance(self.noise, MeasurementNoise) or self.noise.unit != "label":
            raise ValueError("semantic noise must be MeasurementNoise with unit='label'")
        if self.noise.enabled:
            raise ValueError("semantic label noise is unsupported; disable noise explicitly")
        if not isinstance(self.calibration, (PinholeCalibration, FisheyeCalibration, PanoramaCalibration)):
            raise ValueError("semantic calibration must be a supported calibrated camera")
        if not isinstance(self.labels, tuple) or not all(isinstance(label, SemanticLabel) for label in self.labels):
            raise ValueError("labels must be a tuple of SemanticLabel declarations")
        seen_geoms: set[int] = set()
        instance_classes: dict[int, int] = {}
        for label in self.labels:
            if any(type(value) is not int for value in (label.geom_id, label.class_id, label.instance_id)):
                raise ValueError("label IDs must be integers")
            if label.geom_id < 0 or not 0 < label.class_id < 2**31 or not 0 < label.instance_id < 2**31:
                raise ValueError("label geom IDs must be nonnegative; class/instance labels must be positive int32")
            if label.geom_id in seen_geoms:
                raise ValueError("label declarations must not repeat geom IDs")
            if instance_classes.get(label.instance_id, label.class_id) != label.class_id:
                raise ValueError("label declarations must assign one class per instance")
            seen_geoms.add(label.geom_id)
            instance_classes[label.instance_id] = label.class_id


@dataclass(frozen=True, slots=True)
class SemanticCameraSample:
    stamp: SensorSampleStamp
    source_stamp: SensorSampleStamp
    class_ids: np.ndarray
    instance_ids: np.ndarray
    valid_mask: np.ndarray
    calibration_metadata: dict
    configuration: SemanticCameraConfig
    simulation: bool = True
    ground_truth: bool = True


def semantic_from_geom_ids(
    config: SemanticCameraConfig,
    stamp: SensorSampleStamp,
    geom_ids: np.ndarray,
    *,
    source_stamp: SensorSampleStamp,
    rays: CameraRaySample | None = None,
) -> SemanticCameraSample:
    """Label an already nearest-hit raster at its exact simulation source time."""
    if not isinstance(stamp, SensorSampleStamp) or source_stamp != stamp:
        raise ValueError("source_stamp must exactly match the requested SensorSampleStamp")
    geom_ids = np.asarray(geom_ids)
    expected_shape = (config.calibration.height, config.calibration.width)
    if geom_ids.shape != expected_shape or geom_ids.dtype.kind not in "iu":
        raise ValueError("geom_ids must be an integer nearest-hit image at the calibrated resolution")
    if rays is None:
        rays = calibrated_rays(config.calibration, stamp)
    if not isinstance(rays, CameraRaySample) or rays.stamp != stamp:
        raise ValueError("rays must have the requested source stamp")
    projection = rays.calibration_metadata.get("projection", {})
    if rays.valid_mask.shape != expected_shape or any(
        projection.get(key) != value for key, value in asdict(config.calibration).items()
    ):
        raise ValueError("rays calibration must match the semantic camera calibration")
    classes = np.zeros(rays.valid_mask.shape, dtype=np.int32)
    instances = np.zeros_like(classes)
    for label in config.labels:
        pixels = (geom_ids == label.geom_id) & rays.valid_mask
        classes[pixels] = label.class_id
        instances[pixels] = label.instance_id
    return SemanticCameraSample(
        stamp,
        source_stamp,
        classes,
        instances,
        instances != 0,
        rays.calibration_metadata,
        config,
    )
