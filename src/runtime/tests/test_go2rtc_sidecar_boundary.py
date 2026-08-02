from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


def _read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def _read_all(paths: list[Path]) -> str:
    return "\n".join(path.read_text(encoding="utf-8") for path in paths)


def test_go2rtc_is_not_a_product_or_readiness_service() -> None:
    """go2rtc stays outside Product/RunPlan resolution and field readiness."""

    runtime_graph_files = sorted(
        path
        for root in ("config/runtime_graph/products", "config/runtime_graph/envs")
        for path in (ROOT / root).rglob("*")
        if path.is_file()
    )
    scanned = {
        "runtime_graph": _read_all(runtime_graph_files),
        "thunder_catalog": _read("src/runtime/service_catalogs/thunder.py"),
        "robot_cli": _read("scripts/lingtu"),
    }

    violations = [name for name, text in scanned.items() if "go2rtc" in text.lower()]

    assert violations == []


def test_go2rtc_installer_documents_machine_level_sidecar_boundary() -> None:
    installer = _read("scripts/webrtc/install_go2rtc.sh")
    contract = " ".join(line.removeprefix("# ").strip() for line in installer.splitlines())

    assert "optional machine-level external media sidecar" in contract
    assert "stays outside ProductControl" in contract
    assert "not part of any Product or RunPlan" in contract
    assert "does not participate in Product readiness" in contract
    assert "falls back to Gateway JPEG" in contract
    assert "systemctl daemon-reload" in installer
    assert "systemctl enable --now go2rtc" in installer
    assert "systemctl restart go2rtc" in installer


def test_camera_transport_adr_locks_whep_and_jpeg_fallback_contract() -> None:
    adr = _read("docs/architecture/CAMERA_TRANSPORT_DECISION.md")
    contract = " ".join(adr.split())

    assert "go2rtc WHEP" in contract
    assert "Gateway JPEG-over-WebSocket as the fallback" in contract
    assert "optional machine-level external media sidecar" in contract
    assert "outside ProductControl" in contract
    assert "not part of any Product, RunPlan, or Product readiness gate" in contract
    assert "falls directly back to JPEG" in contract
