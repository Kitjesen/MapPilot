from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "deploy" / "thunder" / "verify_gateway_grpc_preflight.py"
_spec = importlib.util.spec_from_file_location("gateway_grpc_preflight", SCRIPT)
assert _spec and _spec.loader
_module = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = _module
_spec.loader.exec_module(_module)
inspect = _module.inspect


def _fixture(tmp_path: Path, *, secure: bool = True, signed: bool = True) -> dict[str, Path]:
    root = tmp_path / "current"
    for relative in (
        "bin/lingtu_dds_probe",
        "lib/liblingtu_nav_client.so",
        "lib/liblingtu_inspection_evidence_bridge.so",
    ):
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"fixture")
    service = tmp_path / "lt-host.service"
    service.write_text(
        "EnvironmentFile=/run/lingtu/session.env\n"
        "ExecStartPre=/opt/lingtu/current/scripts/deploy/thunder/require_product_session.sh host\n",
        encoding="utf-8",
    )
    env = tmp_path / "gateway.env"
    cert = tmp_path / "server.crt"
    key = tmp_path / "server.key"
    cert.write_text("certificate", encoding="utf-8")
    key.write_text("private-key", encoding="utf-8")
    lines = [
        "LINGTU_GRPC_REQUIRE_API_KEY=1",
        "LINGTU_GRPC_API_KEY=" + "a" * 64,
    ]
    if secure:
        lines.extend([
            f"LINGTU_GRPC_TLS_CERT_FILE={cert}",
            f"LINGTU_GRPC_TLS_KEY_FILE={key}",
        ])
    env.write_text("\n".join(lines) + "\n", encoding="utf-8")
    manifest = tmp_path / "manifest.json"
    manifest.write_text(json.dumps({
        "artifacts": [{"hw_compat": ["s100p", "aarch64"]}],
        "signature": "signed" if signed else "",
    }), encoding="utf-8")
    return {"root": root, "service": service, "env": env, "manifest": manifest}


def test_secure_release_preflight_passes(tmp_path: Path) -> None:
    fixture = _fixture(tmp_path)
    checks = inspect(
        current_root=fixture["root"],
        service_file=fixture["service"],
        gateway_env=fixture["env"],
        manifest=fixture["manifest"],
        session_file=None,
        expect_listening=False,
    )

    assert all(check.ok for check in checks)


def test_preflight_blocks_insecure_or_unsigned_release(tmp_path: Path) -> None:
    fixture = _fixture(tmp_path, secure=False, signed=False)
    checks = inspect(
        current_root=fixture["root"],
        service_file=fixture["service"],
        gateway_env=fixture["env"],
        manifest=fixture["manifest"],
        session_file=None,
        expect_listening=False,
    )
    failed = {check.name: check.detail for check in checks if not check.ok}

    assert "grpc_tls_pair" in failed
    assert "native_manifest_signature" in failed
    assert "a" * 64 not in json.dumps([check.__dict__ for check in checks])


def test_runtime_preflight_requires_product_session(tmp_path: Path) -> None:
    fixture = _fixture(tmp_path)
    session = tmp_path / "session.env"
    checks = inspect(
        current_root=fixture["root"],
        service_file=fixture["service"],
        gateway_env=fixture["env"],
        manifest=fixture["manifest"],
        session_file=session,
        expect_listening=False,
    )

    assert any(check.name == "product_session" and not check.ok for check in checks)
