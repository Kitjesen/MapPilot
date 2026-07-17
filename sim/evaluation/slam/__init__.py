"""SLAM simulation evaluation primitives.

This package is intentionally independent from ROS runtime code. It provides
small, deterministic building blocks that can be used by future replay scripts,
simulation jobs, and CI checks.
"""

from .inspection_readiness import (
    FAIL,
    INCOMPLETE,
    LOCALIZATION_FAIL,
    LOCALIZATION_INCOMPLETE,
    LOCALIZATION_PASS,
    PASS,
    EvidenceWindow,
    InspectionLocalizationReadinessConfig,
    RelocalizationRecoveryWindow,
    evaluate_inspection_localization_readiness,
)
from .manifest import (
    SlamEvalBackend,
    SlamEvalCase,
    SlamEvalRobot,
    SlamEvalSensorSuite,
    case_from_dict,
    load_case,
)
from .metrics import (
    MatchedPose,
    TrajectoryErrorSummary,
    associate_by_timestamp,
    evaluate_trajectory,
    path_length,
)
from .public_datasets import (
    PUBLIC_DATASET_CATALOG_PATH,
    DatasetLicense,
    GroundTruthStream,
    ImuStream,
    LidarStream,
    PublicDatasetCatalog,
    PublicSlamDataset,
    build_replay_manifest,
    catalog_from_dict,
    load_public_dataset_catalog,
)
from .tum import TumPose, parse_tum_line, read_tum_trajectory, write_tum_trajectory

__all__ = [
    "FAIL",
    "INCOMPLETE",
    "LOCALIZATION_FAIL",
    "LOCALIZATION_INCOMPLETE",
    "LOCALIZATION_PASS",
    "PASS",
    "PUBLIC_DATASET_CATALOG_PATH",
    "DatasetLicense",
    "EvidenceWindow",
    "GroundTruthStream",
    "ImuStream",
    "InspectionLocalizationReadinessConfig",
    "LidarStream",
    "MatchedPose",
    "PublicDatasetCatalog",
    "PublicSlamDataset",
    "RelocalizationRecoveryWindow",
    "SlamEvalBackend",
    "SlamEvalCase",
    "SlamEvalRobot",
    "SlamEvalSensorSuite",
    "TrajectoryErrorSummary",
    "TumPose",
    "associate_by_timestamp",
    "build_replay_manifest",
    "case_from_dict",
    "catalog_from_dict",
    "evaluate_inspection_localization_readiness",
    "evaluate_trajectory",
    "load_case",
    "load_public_dataset_catalog",
    "parse_tum_line",
    "path_length",
    "read_tum_trajectory",
    "write_tum_trajectory",
]
