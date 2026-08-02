from __future__ import annotations

import pytest

from maps.map_save import (
    MapSaveError,
    save_nav_map_with_adapter,
    save_slam_map_with_adapter,
)


class _Adapter:
    def __init__(self, result):
        self.result = result

    def save_nav_map(self, pcd_path, *, timeout_sec=30.0):
        del pcd_path, timeout_sec
        return self.result

    def save_slam_map(
        self,
        file_path,
        *,
        save_patches=True,
        timeout_sec=30.0,
    ):
        del file_path, save_patches, timeout_sec
        return self.result


@pytest.mark.parametrize(
    "result",
    [None, True, 1, "queued", {}, {"success": "true"}],
)
@pytest.mark.parametrize(
    "save",
    [save_nav_map_with_adapter, save_slam_map_with_adapter],
)
def test_map_save_adapter_rejects_malformed_ack(result, save) -> None:
    with pytest.raises(MapSaveError, match="invalid acknowledgement"):
        save(_Adapter(result), "map.pcd")


@pytest.mark.parametrize("success", [True, False])
def test_map_save_adapter_preserves_explicit_boolean_ack(success: bool) -> None:
    result = {"success": success, "message": "result"}

    assert save_slam_map_with_adapter(_Adapter(result), "map.pcd") == result
