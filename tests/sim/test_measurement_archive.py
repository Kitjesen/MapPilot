"""Formal run recording persists raw samples only when its sink requests it."""

import json
from dataclasses import replace

import numpy as np
import pytest
from sim.runtime.recording.measurement_archive import MeasurementArchive
from sim.runtime.recording.measurement_payload import decode_measurement_payload, encode_measurement_payload

from tests.sim.test_measurement_payload import _thermal


def test_archive_is_lazy_raw_lossless_and_binds_the_run(tmp_path):
    original = _thermal()
    payload = encode_measurement_payload(original, modality="thermal")
    root = tmp_path / "measurements"
    archive = MeasurementArchive(root, product_session_id="product", session_id=original.stamp.session_id,
                                 sensor_ids=[original.stamp.sensor_id])
    assert not root.exists()
    archive.append(payload)
    archive.close()
    summary = json.loads((root / "recording.json").read_text())
    assert summary["state"] == "closed" and summary["sample_count"] == 1
    assert summary["product_session_id"] == "product"
    entry = json.loads((root / "index.jsonl").read_text())
    assert entry["sensor_id"] == original.stamp.sensor_id
    decoded = decode_measurement_payload((root / entry["file"]).read_bytes(), modality="thermal")
    np.testing.assert_array_equal(decoded.temperature_k, original.temperature_k)
    np.testing.assert_array_equal(decoded.valid_mask, original.valid_mask)
    assert decoded.stamp == original.stamp
    with pytest.raises(RuntimeError, match="closed"):
        archive.append(payload)


def test_duplicate_and_wrong_source_never_overwrite_a_record(tmp_path):
    original = _thermal()
    payload = encode_measurement_payload(original, modality="thermal")
    archive = MeasurementArchive(tmp_path, product_session_id="product", session_id=original.stamp.session_id,
                                 sensor_ids=[original.stamp.sensor_id])
    archive.append(payload)
    before = (tmp_path / "index.jsonl").read_bytes()
    with pytest.raises(ValueError, match="sequence"):
        archive.append(payload)
    wrong_metadata = dict(payload.metadata, stamp=dict(payload.metadata["stamp"], session_id="wrong"))
    with pytest.raises(ValueError, match="identity"):
        archive.append(replace(payload, metadata=wrong_metadata))
    assert (tmp_path / "index.jsonl").read_bytes() == before
    archive.close()
