from __future__ import annotations

from pathlib import Path
from unittest.mock import MagicMock

from lingtu.slam import SLAM


def test_slam_save_map_uses_injected_map_save_adapter(monkeypatch, tmp_path) -> None:
    monkeypatch.setenv("NAV_MAP_DIR", str(tmp_path))

    def fake_adapter(pcd_path, **_kwargs):
        Path(pcd_path).parent.mkdir(parents=True, exist_ok=True)
        Path(pcd_path).write_text("VERSION 0.7\nDATA ascii\n", encoding="utf-8")
        return {"success": True, "source": "fake_adapter"}

    adapter = MagicMock(side_effect=fake_adapter)
    slam = SLAM(map_save_adapter=adapter)
    build_tomogram = MagicMock()
    monkeypatch.setattr(slam, "_build_tomogram", build_tomogram)

    assert slam.save_map("demo") is True

    pcd_path = tmp_path / "demo" / "map.pcd"
    assert pcd_path.exists()
    adapter.assert_called_once()
    assert Path(adapter.call_args.args[0]) == pcd_path
    assert adapter.call_args.kwargs["save_patches"] is True
    build_tomogram.assert_called_once_with("demo")
