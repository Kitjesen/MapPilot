from __future__ import annotations

from pathlib import Path


def test_reconstruction_server_health_reports_backend_status(tmp_path: Path):
    from perception.reconstruction.server.recon_server import ReconServer

    server = ReconServer(backend="tsdf", output_dir=tmp_path)

    health = server.health()
    assert health["status"] == "ok"
    assert health["configured_backend"] == "tsdf"
    assert health["backend"] == "tsdf"
    assert health["degraded"] is False
    assert health["degraded_reason"] == ""
