import importlib.util
import json
from pathlib import Path

from sim.evaluation.slam.public_datasets import (
    PUBLIC_DATASET_CATALOG_PATH,
    build_replay_manifest,
    catalog_from_dict,
    load_public_dataset_catalog,
)

ROOT = Path(__file__).resolve().parents[3]
DATASET_CLI_PATH = ROOT / "tools" / "datasets" / "public_slam_dataset.py"
SLAM_EVALUATION_README_PATH = ROOT / "sim" / "evaluation" / "slam" / "README.md"
DATASET_CLI_SPEC = importlib.util.spec_from_file_location(
    "public_slam_dataset",
    DATASET_CLI_PATH,
)
assert DATASET_CLI_SPEC is not None and DATASET_CLI_SPEC.loader is not None
DATASET_CLI = importlib.util.module_from_spec(DATASET_CLI_SPEC)
DATASET_CLI_SPEC.loader.exec_module(DATASET_CLI)


def test_public_dataset_catalog_is_valid_and_has_mid360_priority_set():
    catalog = load_public_dataset_catalog()

    assert catalog.validate() == []
    assert len(catalog.priority_sets["mid360"]) >= 4
    for dataset_id in catalog.priority_sets["mid360"]:
        dataset = catalog.require(dataset_id)
        assert "mid360" in dataset.sensor_model.lower()


def test_frontend_replay_datasets_declare_point_level_timestamps():
    catalog = load_public_dataset_catalog()

    frontend_datasets = [dataset for dataset in catalog.datasets if "lio_frontend" in dataset.recommended_uses]

    assert frontend_datasets
    for dataset in frontend_datasets:
        assert dataset.lidar.point_time_field
        assert dataset.lidar.point_time_unit in {"ns", "us", "s"}


def test_catalog_rejects_redistribution_when_license_still_needs_review():
    raw = {
        "schema_version": "lingtu.slam.public_datasets.v1",
        "priority_sets": {"mid360": ["needs-review"]},
        "datasets": [
            {
                "id": "needs-review",
                "title": "Needs review",
                "official_url": "https://example.invalid/dataset",
                "license": {
                    "name": "unverified",
                    "terms_url": "https://example.invalid/terms",
                    "redistribution_allowed": True,
                    "review_required": True,
                },
                "sensor_model": "Livox MID360",
                "source_format": "ros2_bag",
                "lidar": {
                    "topic": "/livox/lidar",
                    "message_type": "livox_ros_driver2/msg/CustomMsg",
                    "point_time_field": "offset_time",
                    "point_time_unit": "ns",
                },
                "imu": {
                    "topic": "/livox/imu",
                    "message_type": "sensor_msgs/msg/Imu",
                    "acceleration_unit": "m/s^2",
                    "angular_velocity_unit": "rad/s",
                },
                "ground_truth": {"available": False},
                "recommended_uses": ["lio_frontend"],
                "conversion_adapter": "normalized_jsonl",
            }
        ],
    }

    catalog = catalog_from_dict(raw)

    assert catalog.validate() == [
        "datasets[needs-review].license cannot allow redistribution while review_required is true"
    ]


def test_catalog_requires_a_mid360_priority_set():
    raw = {
        "schema_version": "lingtu.slam.public_datasets.v1",
        "priority_sets": {},
        "datasets": [],
    }

    catalog = catalog_from_dict(raw)

    assert "priority_sets[mid360] is required" in catalog.validate()


def test_replay_manifest_targets_native_ltu1_and_canonical_topics(tmp_path):
    catalog = load_public_dataset_catalog()
    dataset = catalog.require("aist-hard-localization-mid360")

    manifest = build_replay_manifest(
        dataset,
        source_path=tmp_path / "outdoor_hard_01a",
        output_dir=tmp_path / "normalized",
        sequence="outdoor_hard_01a",
    )

    assert manifest["schema_version"] == "lingtu.slam.replay.v1"
    assert manifest["dataset"]["id"] == "aist-hard-localization-mid360"
    assert manifest["sequence"] == "outdoor_hard_01a"
    assert manifest["target"]["format"] == "LTU1"
    assert manifest["target"]["record_path"].endswith("outdoor_hard_01a.ltu")
    assert manifest["streams"]["lidar"]["canonical_topic"] == "/lidar/raw_frame"
    assert manifest["streams"]["imu"]["canonical_topic"] == "/imu/raw"
    assert manifest["streams"]["lidar"]["point_time"]["field"] == "offset_time"
    assert manifest["ground_truth_sidecar"]["enabled"] is True
    assert manifest["ground_truth_sidecar"]["canonical_format"] == "tum"
    assert manifest["ground_truth_sidecar"]["output_path"].endswith("outdoor_hard_01a.ground_truth.tum")


def test_iilabs_catalog_uses_official_topics_without_claiming_unverified_deskew():
    catalog = load_public_dataset_catalog()
    dataset = catalog.require("iilabs3d-mid360")

    assert dataset.lidar.topic == "/eve/lidar3d"
    assert dataset.lidar.message_type == "sensor_msgs/PointCloud2"
    assert dataset.imu.topic == "/eve/livox/imu"
    assert dataset.lidar.point_time_field is None
    assert "lio_frontend" not in dataset.recommended_uses
    assert dataset.license.review_required is True
    assert dataset.license.redistribution_allowed is False


def test_rtk_slam_keeps_sparse_checkpoint_ground_truth_out_of_tum_trajectory(tmp_path):
    catalog = load_public_dataset_catalog()
    dataset = catalog.require("rtk-slam-mid360")

    manifest = build_replay_manifest(
        dataset,
        source_path=tmp_path / "construction_seq2.bag",
        output_dir=tmp_path / "normalized",
        sequence="construction_seq2",
    )

    sidecar = manifest["ground_truth_sidecar"]
    assert sidecar["source_format"] == "checkpoint_csv"
    assert sidecar["canonical_format"] == "checkpoint_csv"
    assert sidecar["source"]["path"] == ("ground_truth/construction_seq2.csv")
    assert sidecar["output_path"].endswith("construction_seq2.ground_truth.checkpoint_csv")


def test_flatwall_ground_truth_is_not_claimed_as_tum_before_conversion():
    catalog = load_public_dataset_catalog()
    dataset = catalog.require("aist-flatwall-avia")

    assert dataset.ground_truth.format == "aist_apriltag_archive"
    assert dataset.ground_truth.path == "gt.tar.gz"
    assert dataset.ground_truth.canonical_format == "source_archive"


def test_aist_driving_is_transport_smoke_not_frontend_accuracy_evidence():
    catalog = load_public_dataset_catalog()
    dataset = catalog.require("aist-driving-mid360")

    assert dataset.lidar.topic == "/livox/lidar"
    assert dataset.lidar.message_type == "sensor_msgs/msg/PointCloud2"
    assert dataset.imu.topic == "/livox/imu"
    assert dataset.ground_truth.available is False
    assert "transport_replay" in dataset.recommended_uses
    assert "lio_frontend" not in dataset.recommended_uses


def test_ros2_bag_catalog_entries_route_to_the_implemented_adapter():
    catalog = load_public_dataset_catalog()
    ros2_datasets = [dataset for dataset in catalog.datasets if dataset.source_format == "ros2_bag"]

    assert ros2_datasets
    for dataset in ros2_datasets:
        assert dataset.conversion_adapter == "ros2_bag_to_normalized_jsonl"


def test_dataset_cli_writes_a_native_replay_manifest(tmp_path):
    manifest_path = tmp_path / "hard-localization.replay.json"

    rc = DATASET_CLI.main(
        [
            "manifest",
            "aist-hard-localization-mid360",
            str(tmp_path / "outdoor_hard_01a"),
            str(tmp_path / "normalized"),
            "--sequence",
            "outdoor_hard_01a",
            "--write",
            str(manifest_path),
        ]
    )

    assert rc == 0
    payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert payload["schema_version"] == "lingtu.slam.replay.v1"
    assert payload["target"]["format"] == "LTU1"
    assert payload["streams"]["lidar"]["point_time"]["field"] == "offset_time"


def test_catalog_path_is_repository_owned():
    assert PUBLIC_DATASET_CATALOG_PATH == Path("sim/evaluation/slam/configs/public_datasets.json")


def test_public_mid360_runbook_reaches_native_dds_and_fastlio2():
    runbook = SLAM_EVALUATION_README_PATH.read_text(encoding="utf-8")

    assert "--stdin-records" in runbook
    assert "--dds" in runbook
    assert "--domain-id 83" in runbook
    assert "LINGTU_SLAM_BIN=" in runbook
    assert "LINGTU_SLAM_MODE=mapping" in runbook
    assert "scripts/deploy/thunder/run_slam_dds.sh" in runbook
