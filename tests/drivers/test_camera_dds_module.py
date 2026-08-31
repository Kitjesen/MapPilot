from __future__ import annotations

# ruff: noqa: D103,S101
import time
from importlib import reload
from pathlib import Path

from drivers.real.camera.dds_module import DdsCameraModule
from drivers.real.camera.shm import ShmFrameWriter, StreamKind
from runtime.contracts import CAMERA_BACKEND_DDS, CAMERA_BACKEND_ORBBEC, CAMERA_ROLE
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
    module.start()
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


def test_camera_module_uses_session_root_file_rings_in_sim(monkeypatch, tmp_path):
    monkeypatch.setenv("LINGTU_ENV", "sim")
    monkeypatch.setenv("LINGTU_SESSION_ROOT", str(tmp_path.resolve()))

    module = DdsCameraModule()

    assert module._shm_paths == {
        "color": tmp_path / "camera_color.shm",
        "depth": tmp_path / "camera_depth.shm",
        "info": tmp_path / "camera_info.shm",
    }


def test_camera_module_explicit_shm_paths_override_sim_session_root(monkeypatch, tmp_path):
    session_root = tmp_path / "session"
    session_root.mkdir()
    explicit = tmp_path / "explicit_color.shm"
    monkeypatch.setenv("LINGTU_ENV", "sim")
    monkeypatch.setenv("LINGTU_SESSION_ROOT", str(session_root.resolve()))

    module = DdsCameraModule(color_shm_path=str(explicit))

    assert module._shm_paths["color"] == explicit
    assert module._shm_paths["depth"] == session_root / "camera_depth.shm"


def test_camera_module_real_defaults_remain_posix_shm(monkeypatch):
    monkeypatch.setenv("LINGTU_ENV", "real")
    monkeypatch.setenv("LINGTU_SESSION_ROOT", "/tmp/ignored-session")
    monkeypatch.delenv("LINGTU_CAMERA_COLOR_SHM", raising=False)
    monkeypatch.delenv("LINGTU_CAMERA_DEPTH_SHM", raising=False)
    monkeypatch.delenv("LINGTU_CAMERA_INFO_SHM", raising=False)

    module = DdsCameraModule()

    assert module._shm_paths == {
        "color": Path("/dev/shm/lingtu_camera_color"),
        "depth": Path("/dev/shm/lingtu_camera_depth"),
        "info": Path("/dev/shm/lingtu_camera_info"),
    }


def test_camera_module_startup_readiness_requires_three_fresh_streams(tmp_path):
    paths = {
        "color": tmp_path / "camera_color.shm",
        "depth": tmp_path / "camera_depth.shm",
        "info": tmp_path / "camera_info.shm",
    }
    module = DdsCameraModule(
        color_shm_path=str(paths["color"]),
        depth_shm_path=str(paths["depth"]),
        info_shm_path=str(paths["info"]),
        stale_timeout_s=30.0,
        poll_interval_s=0.001,
    )

    assert module.startup_readiness() == "not_running"
    module.setup()
    module.start()
    try:
        assert module.startup_readiness() == "camera_streams_missing:color,depth,info"
        write_committed_frame(paths["color"])
        _wait_until(lambda: module.health()["frames"]["color"] == 1)
        assert module.startup_readiness() == "camera_streams_missing:depth,info"
        write_committed_frame(
            paths["depth"],
            stream_kind=StreamKind.DEPTH,
            encoding="16UC1",
            stride=4,
            payload=bytes(8),
        )
        write_committed_frame(
            paths["info"],
            stream_kind=StreamKind.INFO,
            encoding="camera_info",
            stride=0,
            payload=b"",
        )
        _wait_until(lambda: module.health()["ready"] is True)
        assert module.startup_readiness() is None
    finally:
        module.stop()


def test_camera_module_startup_readiness_rejects_stale_stream(tmp_path):
    paths = _write_rgbd_set(tmp_path)
    module = DdsCameraModule(
        color_shm_path=str(paths["color"]),
        depth_shm_path=str(paths["depth"]),
        info_shm_path=str(paths["info"]),
        stale_timeout_s=1.0,
        poll_interval_s=0.001,
    )
    module.setup()
    module.start()
    try:
        _wait_until(lambda: module.health()["ready"] is True)
        time.sleep(1.05)
        assert module.startup_readiness() == "camera_streams_stale:color,depth"
    finally:
        module.stop()


def test_camera_module_keeps_static_info_while_dynamic_streams_stay_fresh(tmp_path):
    paths = {
        "color": tmp_path / "camera_color.shm",
        "depth": tmp_path / "camera_depth.shm",
        "info": tmp_path / "camera_info.shm",
    }
    color = ShmFrameWriter(paths["color"], stream_kind=StreamKind.COLOR, slot_capacity=12)
    depth = ShmFrameWriter(paths["depth"], stream_kind=StreamKind.DEPTH, slot_capacity=8)
    info = ShmFrameWriter(paths["info"], stream_kind=StreamKind.INFO, slot_capacity=1)

    def publish_dynamic() -> None:
        timestamp_ns = time.time_ns()
        color.publish(
            timestamp_ns=timestamp_ns,
            width=2,
            height=2,
            stride=6,
            encoding="rgb8",
            frame_id="camera_link",
            payload=bytes(12),
        )
        depth.publish(
            timestamp_ns=timestamp_ns,
            width=2,
            height=2,
            stride=4,
            encoding="16UC1",
            frame_id="camera_link",
            payload=bytes(8),
        )

    publish_dynamic()
    info.publish(
        timestamp_ns=time.time_ns(),
        width=2,
        height=2,
        stride=0,
        encoding="camera_info",
        frame_id="camera_link",
        payload=b"",
        fx=500.0,
        fy=501.0,
        cx=1.0,
        cy=1.0,
    )
    module = DdsCameraModule(
        color_shm_path=str(paths["color"]),
        depth_shm_path=str(paths["depth"]),
        info_shm_path=str(paths["info"]),
        stale_timeout_s=0.5,
        poll_interval_s=0.001,
    )
    module.setup()
    module.start()
    try:
        _wait_until(lambda: module.health()["ready"] is True)
        time.sleep(0.55)
        publish_dynamic()
        _wait_until(lambda: module.health()["frames"]["color"] == 2)
        _wait_until(lambda: module.health()["frames"]["depth"] == 2)

        health = module.health()
        assert health["ready"] is True
        assert health["stale_ms"]["info"] > 500
        assert module.startup_readiness() is None
    finally:
        module.stop()
        color.close()
        depth.close()
        info.close()


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
    module.start()
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
    assert health["source_unit"] == "lt-camera.service"
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
