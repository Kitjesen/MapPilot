"""Measured temperature and pressure have native types, not depth Image aliases."""

import pytest

from message.generated.schema import MESSAGE_FIELDS
from message.topics import TOPIC_SPECS


@pytest.mark.parametrize(
    "topic,type_name,fields",
    [
        (
            "/sensors/thermal",
            "ThermalImage",
            {"sensor_id", "header", "identity", "width", "height", "temperature_k", "valid_mask", "metadata_json"},
        ),
        (
            "/sensors/acoustic",
            "AcousticBlock",
            {
                "sensor_id",
                "header",
                "identity",
                "sample_rate_hz",
                "channels",
                "samples",
                "pressure_pa",
                "valid_mask",
                "voltage_v",
                "metadata_json",
            },
        ),
        (
            "/sensors/semantic",
            "SemanticImage",
            {
                "sensor_id",
                "header",
                "identity",
                "width",
                "height",
                "class_ids",
                "instance_ids",
                "valid_mask",
                "metadata_json",
            },
        ),
    ],
)
def test_native_measurements_have_distinct_typed_topics(topic, type_name, fields):
    spec = TOPIC_SPECS[topic]
    assert spec.message_type == f"lingtu.dds.{type_name}"
    assert fields == set(MESSAGE_FIELDS[spec.message_type])


def test_native_measurement_identity_preserves_product_source_and_generations():
    assert set(MESSAGE_FIELDS["lingtu.dds.MeasurementIdentity"]) == {
        "product_session_id",
        "session_id",
        "instance_id",
        "model_generation",
        "reset_generation",
        "sample_sequence",
        "sim_time_ns",
    }
