"""Tests for post-inference Habitat object identity evaluation."""

from types import SimpleNamespace

import numpy as np
from tools.datasets.habitat_identity import (
    association_report,
    evaluate_masks,
    oracle_id,
)


def test_oracle_id_requires_one_identity_to_cover_the_predicted_mask() -> None:
    identity = np.array([[10, 10, 0], [10, 20, 20]])
    mask = np.ones(identity.shape, dtype=bool)

    assert oracle_id(mask, identity, 0.5, valid_ids={10, 20}) == 10
    assert oracle_id(mask, identity, 0.6, valid_ids={10, 20}) is None


def test_evaluation_is_read_only_and_reports_mask_attribution() -> None:
    mask = np.array([[1, 1], [1, 0]], dtype=bool)
    observation = SimpleNamespace(mask=mask)
    match = SimpleNamespace(object_id=7)
    identity = np.array([[101, 101], [101, 0]], dtype=np.uint32)

    records = evaluate_masks([observation], [match], identity, valid_ids={101})

    assert records == [{
        "track_id": 7,
        "oracle_id": 101,
        "oracle_majority_id": 101,
        "oracle_pixel_fraction": 1.0,
        "oracle_pixels": 3,
        "mask_pixels": 3,
    }]
    np.testing.assert_array_equal(observation.mask, mask)
    assert match.object_id == 7


def test_association_report_exposes_switches_mixed_tracks_splits_and_coverage() -> None:
    records = [
        {"track_id": 1, "oracle_id": 10},
        {"track_id": 1, "oracle_id": 10},
        {"track_id": 1, "oracle_id": 11},
        {"track_id": 2, "oracle_id": 10},
        {"track_id": 3, "oracle_id": None},
    ]

    report = association_report(records, minimum_fraction=0.7)

    assert report["minimum_oracle_pixel_fraction"] == 0.7
    assert report["attributed_observations"] == 4
    assert report["ambiguous_observations"] == 1
    assert report["attribution_coverage"] == 0.8
    assert report["identity_switch_count"] == 1
    assert report["identity_switches"] == {"1": 1, "2": 0}
    assert report["mixed_track_oracle_ids"] == {"1": [10, 11]}
    assert report["split_oracle_track_ids"] == {"10": [1, 2]}
