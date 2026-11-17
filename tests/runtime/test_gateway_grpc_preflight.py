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

_LOADED = {"LoadState": "loaded", "UnitFileState": "static"}


def _service_text() -> str:
    return (
        "EnvironmentFile=/run/lingtu/session.env\n"
        "ExecStartPre=/opt/lingtu/current/scripts/deploy/thunder/require_product_session.sh host\n"
    )


def _fixture(
    tmp_path: Path,
    *,
    secure: bool = True,
    signed: bool = True,
    require_api_key: str = "true",
    architecture: str = "aarch64",
    libraries: tuple[str, ...] = ("libfrom-manifest.so",),
    key_mode: int = 0o600,
    env_mode: int = 0o600,
) -> dict[str, Path]:
    root = tmp_path / "current"
    for relative in libraries:
        path = root / "lib" / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"fixture")
    service = tmp_path / "lt-host.service"
    service.write_text(_service_text(), encoding="utf-8")
    env = tmp_path / "gateway.env"
    cert = tmp_path / "server.crt"
    key = tmp_path / "server.key"
    cert.write_text("certificate", encoding="utf-8")
    key.write_text("private-key", encoding="utf-8")
    key.chmod(key_mode)
    lines = [
        f"LINGTU_GRPC_REQUIRE_API_KEY={require_api_key}",
        "LINGTU_GRPC_API_KEY=" + "a" * 64,
    ]
    if secure:
        lines.extend([
            f"LINGTU_GRPC_TLS_CERT_FILE={cert}",
            f"LINGTU_GRPC_TLS_KEY_FILE={key}",
        ])
    env.write_text("\n".join(lines) + "\n", encoding="utf-8")
    env.chmod(env_mode)
    manifest = tmp_path / "manifest.json"
    manifest.write_text(json.dumps({
        "architecture": architecture,
        "requiredLibraries": list(libraries),
        "signature": "signed-manifest" if signed else "",
    }), encoding="utf-8")
    session = tmp_path / "session.env"
    session.write_text("LINGTU_PRODUCT=nav\n", encoding="utf-8")
    return {
        "root": root,
        "service": service,
        "env": env,
        "manifest": manifest,
        "session": session,
        "key": key,
    }


def _checks(fixture: dict[str, Path], **overrides: object) -> list[object]:
    arguments = {
        "current_root": fixture["root"],
        "service_file": fixture["service"],
        "gateway_env": fixture["env"],
        "manifest": fixture["manifest"],
        "session_file": fixture["session"],
        "expect_listening": False,
        "unit_state": _LOADED,
    }
    arguments.update(overrides)
    return inspect(**arguments)


def test_secure_release_preflight_passes(tmp_path: Path) -> None:
    checks = _checks(_fixture(tmp_path))

    assert all(check.ok for check in checks)
    assert not any(check.name == "grpc_gateway_listening" for check in checks)


def test_preflight_reads_libraries_from_the_manifest(tmp_path: Path) -> None:
    library = "vendor/libonly-in-manifest.so"
    fixture = _fixture(tmp_path, libraries=(library,))

    checks = _checks(fixture)
    failed = {check.name for check in checks if not check.ok}

    assert "required_libraries_present" not in failed
    assert (fixture["root"] / "lib" / "liblingtu_nav_client.so").exists() is False


def test_preflight_blocks_unsigned_insecure_and_numeric_api_key(tmp_path: Path) -> None:
    fixture = _fixture(tmp_path, secure=False, signed=False, require_api_key="1")
    checks = _checks(fixture)
    failed = {check.name: check.detail for check in checks if not check.ok}

    assert "grpc_tls_pair" in failed
    assert "native_manifest_signature" in failed
    assert failed["grpc_api_key_required"] == "LINGTU_GRPC_REQUIRE_API_KEY must be true"
    rendered = json.dumps([check.__dict__ for check in checks])
    assert "a" * 64 not in rendered
    assert "private-key" not in rendered


def test_preflight_accepts_arch_alias_and_signed_flag(tmp_path: Path) -> None:
    fixture = _fixture(tmp_path)
    fixture["manifest"].write_text(json.dumps({
        "arch": "arm64",
        "requiredLibraries": ["libfrom-manifest.so"],
        "signature": "",
        "signed": True,
    }), encoding="utf-8")

    checks = _checks(fixture)

    assert all(check.ok for check in checks)


def test_preflight_rejects_hardware_compat_without_required_libraries(tmp_path: Path) -> None:
    fixture = _fixture(tmp_path)
    fixture["manifest"].write_text(json.dumps({
        "artifacts": [{"hw_compat": ["s100p", "aarch64"]}],
        "signature": "signed-manifest",
    }), encoding="utf-8")

    checks = _checks(fixture)
    failed = {check.name for check in checks if not check.ok}

    assert "native_manifest_architecture" in failed
    assert "required_libraries" in failed


def test_preflight_blocks_missing_declared_library_and_private_key_mode(tmp_path: Path) -> None:
    fixture = _fixture(tmp_path, libraries=("libmissing.so",), key_mode=0o644)
    (fixture["root"] / "lib" / "libmissing.so").unlink()

    checks = _checks(fixture)
    failed = {check.name for check in checks if not check.ok}

    assert "required_libraries_present" in failed
    assert "grpc_tls_private_key_mode" in failed


def test_preflight_requires_an_existing_session_and_does_not_create_one(tmp_path: Path) -> None:
    fixture = _fixture(tmp_path)
    missing = tmp_path / "missing-session.env"
    checks = _checks(fixture, session_file=missing)

    assert any(check.name == "product_session" and not check.ok for check in checks)
    assert not missing.exists()


def test_preflight_blocks_boot_enabled_host(tmp_path: Path) -> None:
    fixture = _fixture(tmp_path)
    fixture["service"].write_text(
        _service_text() + "[Install]\nWantedBy=multi-user.target\n",
        encoding="utf-8",
    )

    checks = _checks(fixture, unit_state={"LoadState": "loaded", "UnitFileState": "enabled"})
    failed = {check.name for check in checks if not check.ok}

    assert "host_unit_not_boot_enabled" in failed
    assert "host_unit_file_state" in failed


def test_preflight_accepts_disabled_unit_file_state(tmp_path: Path) -> None:
    checks = _checks(
        _fixture(tmp_path),
        unit_state={"LoadState": "loaded", "UnitFileState": "disabled"},
    )

    assert all(check.ok for check in checks)


def test_preflight_source_is_read_only() -> None:
    source = SCRIPT.read_text(encoding="utf-8")

    assert '["systemctl", "show"' in source
    for verb in ("start", "restart", "enable", "disable", "daemon-reload", "isolate"):
        assert f'["systemctl", "{verb}"' not in source
    assert "write_text" not in source
    assert "mkdir" not in source
    assert '== "true"' in source
    assert "liblingtu_nav_client.so" not in source
