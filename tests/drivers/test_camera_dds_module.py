from __future__ import annotations

# ruff: noqa: D103,S101
import time
from importlib import reload
from pathlib import Path

from drivers.real.camera.dds_module import DdsCameraModule
from drivers.real.camera.shm import StreamKind
from runtime.contracts import CAMERA_BACKEND_DDS, CAMERA_ROLE
from runtime.registry import clear, get, register, restore, snapshot
from tests.drivers.test_camera_shm import write_committed_frame


def _wait_until(predicate, timeout_s: float = 3.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("camera SHM condition was not reached before timeout")


def _write_rgbd_set(tmp_path: Path) -> dict[str, Path]:
    paths = {
        "color": tmp_path / "camera_color.shm",
        "depth": tmp_path / "camera_depth.shm",
        "info": tmp_path / "camera_info.shm",
    }
    write_committed_frame(paths["color"])
    write_committed_frame(
        paths["depth"],
        stream_kind=StreamKind.DEPTH,
        encoding="16UC1",
        stride=4,
        payload=(1000).to_bytes(2, "little")
        + (2000).to_bytes(2, "little")
        + (3000).to_bytes(2, "little")
        + (4000).to_bytes(2, "little"),
    )
    write_committed_frame(
        paths["info"],
        stream_kind=StreamKind.INFO,
        encoding="camera_info",
        stride=0,
        payload=b"",
        calibration=(500.0, 501.0, 1.0, 1.5, 0.001, 0.1, 0.2, 0.3, 0.4, 0.5),
    )
    return paths


def test_camera_module_reads_rgbd_frames_from_shm_without_python_dds(tmp_path):
    paths = _write_rgbd_set(tmp_path)
    module = DdsCameraModule(
        color_shm_path=str(paths["color"]),
        depth_shm_path=str(paths["depth"]),
        info_shm_path=str(paths["info"]),
        stale_timeout_s=30.0,
        poll_interval_s=0.001,
    )
    colors = []
    depths = []
    infos = []
    alive = []
    module.color_image.subscribe(colors.append)
    module.depth_image.subscribe(depths.append)
    module.camera_info.subscribe(infos.append)
    module.alive.subscribe(alive.append)

    module.setup()
    try:
        _wait_until(lambda: bool(colors and depths and infos))
        assert colors[-1].data.shape == (2, 2, 3)
        assert colors[-1].format.value == "RGB"
        assert depths[-1].data.shape == (2, 2)
        assert int(depths[-1].data[0, 0]) == 1000
        assert infos[-1].fx == 500.0
        assert infos[-1].fy == 501.0
        assert infos[-1].depth_scale == 0.001
        assert infos[-1].frame_id == "camera_link"

        health = module.health()
        assert alive[0] is True
        assert health["backend"] == CAMERA_BACKEND_DDS
        assert health["status"] == "ready"
        assert health["ready"] is True
        assert health["frames"] == {"color": 1, "depth": 1, "info": 1}
        assert health["rejected_frames"] == {"color": 0, "depth": 0, "info": 0}
        assert health["error"] is None
        assert health["transport"]["frame_data"] == "posix_shm"
        assert health["shm_schema"] == "lingtu.camera.shm_frame.v1"
        assert health["shm_paths"] == {name: str(path) for name, path in paths.items()}
    finally:
        module.stop()


def test_camera_module_rejects_corrupt_shm_without_stopping_other_streams(tmp_path):
    paths = _write_rgbd_set(tmp_path)
    write_committed_frame(paths["color"], payload_crc32=123)
    module = DdsCameraModule(
        color_shm_path=str(paths["color"]),
        depth_shm_path=str(paths["depth"]),
        info_shm_path=str(paths["info"]),
        stale_timeout_s=30.0,
        poll_interval_s=0.001,
    )
    colors = []
    depths = []
    module.color_image.subscribe(colors.append)
    module.depth_image.subscribe(depths.append)

    module.setup()
    try:
        _wait_until(lambda: module.health()["rejected_frames"]["color"] > 0)
        _wait_until(lambda: bool(depths))
        _wait_until(lambda: module.health()["frames"]["info"] == 1)
        health = module.health()
        assert colors == []
        assert health["status"] == "running"
        assert health["ready"] is False
        assert health["frames"]["depth"] == 1
        assert health["frames"]["info"] == 1
        assert health["rejected_frames"]["color"] >= 1
        assert "CRC mismatch" in str(health["error"])
        rejected = health["rejected_frames"]["color"]
        time.sleep(0.02)
        assert module.health()["rejected_frames"]["color"] == rejected
    finally:
        module.stop()


def test_camera_module_source_has_no_python_dds_reader_dependency():
    source = Path("src/drivers/real/camera/dds_module.py").read_text(encoding="utf-8")

    assert "runtime.adapters.dds" not in source
    assert "import cyclonedds" not in source.lower()
    assert "from cyclonedds" not in source.lower()


def test_camera_module_health_matches_camera_service_boundary(tmp_path):
    paths = _write_rgbd_set(tmp_path)
    module = DdsCameraModule(
        color_shm_path=str(paths["color"]),
        depth_shm_path=str(paths["depth"]),
        info_shm_path=str(paths["info"]),
    )
    health = module.health()

    assert health["source_service"] == "camera"
    assert health["source_unit"] == "lingtu-camera-dds.service"
    assert health["transport"] == {
        "frame_data": "posix_shm",
        "metadata": "posix_shm_and_optional_typed_dds",
    }


def test_dds_camera_registers_as_camera_backend():
    state = snapshot()
    try:
        clear()
        import drivers.real.camera.dds_module as module

        reload(module)
        cls = get(CAMERA_ROLE, CAMERA_BACKEND_DDS)
        assert cls.__name__ == "DdsCameraModule"
        assert cls.__module__ == "drivers.real.camera.dds_module"
    finally:
        restore(state)


def test_camera_gateway_resolves_dds_backend():
    from lingtu.plugin_seed import seed_builtin_plugins
    from runtime.adapters.perception_gateway import camera_module

    state = snapshot()
    try:
        clear()
        seed_builtin_plugins(groups=("camera",), reload_loaded=True, strict=True)
        cls = camera_module(backend=CAMERA_BACKEND_DDS)
        assert cls is not None
        assert cls.__name__ == "DdsCameraModule"
        assert cls.__module__ == "drivers.real.camera.dds_module"
    finally:
        restore(state)


def test_camera_gateway_does_not_fallback_to_orbbec_for_missing_dds(monkeypatch):
    import runtime.adapters.perception_gateway as gateway

    state = snapshot()
    try:
        clear()

        @register("camera_bridge", "default")
        class LegacyCameraBridge:
            pass

        monkeypatch.setattr(gateway, "seed_registered_plugins", lambda *a, **kw: None)
        monkeypatch.setattr(gateway, "_reload_camera_candidates", lambda: None)
        assert gateway.camera_module(backend=CAMERA_BACKEND_DDS) is None
    finally:
        restore(state)
