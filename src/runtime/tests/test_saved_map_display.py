from diagnostics.saved_map_display import format_saved_map_artifact_gate_payload


def test_saved_map_display_reports_blockers() -> None:
    output = format_saved_map_artifact_gate_payload(
        {
            "ok": False,
            "map_dir": "/tmp/map",
            "checked_frame_id": "odom",
            "checked_allowed_frame_ids": ["map"],
            "checked_expected": {"data_source": "field", "frame_id": "map"},
            "metadata": {"exists": True},
            "metadata_validation": {"ok": False},
            "blockers": ["frame mismatch"],
        }
    )
    assert "Saved map artifact gate: FAIL" in output
    assert "Map dir" not in output
    assert "Expected: data_source=field frame=map" in output
    assert "Metadata: exists=true ok=false" in output
    assert "Blockers: frame mismatch" in output
