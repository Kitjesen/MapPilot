#!/usr/bin/env python3
"""Read-only preflight for the formal LingTu gRPC Gateway release.

The deployment gate consumes one JSON manifest. That object must carry
``architecture`` or ``arch`` in {aarch64, arm64}, a non-empty ``signature``
or ``signed is True``, and ``requiredLibraries`` as the library names to
check under ``<current>/lib``. Library names come from the manifest. This
script does not keep a second list, does not start a service, and does not
create a Product session.
"""

from __future__ import annotations

import argparse
import json
import shutil
import socket
import subprocess
import sys
from collections.abc import Mapping
from dataclasses import asdict, dataclass
from pathlib import Path

ACCEPTED_ARCHITECTURES = {"aarch64", "arm64"}
ACCEPTED_UNIT_FILE_STATES = {"disabled", "static"}
_LOCAL_GATEWAY_PORT = 5052


@dataclass(frozen=True)
class Check:
    name: str
    ok: bool
    detail: str


def _env(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        name, value = line.split("=", 1)
        value = value.strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in {"'", '"'}:
            value = value[1:-1]
        values[name.strip()] = value
    return values


def _regular_file(path: Path) -> bool:
    return path.is_file() and not path.is_symlink()


def _owner_only(path: Path) -> bool:
    return path.stat().st_mode & 0o077 == 0


def _library_file(current_root: Path, name: str) -> Path | None:
    relative = Path(name)
    if relative.is_absolute() or not relative.parts or ".." in relative.parts:
        return None
    return current_root / "lib" / relative


def _boot_enabled(text: str) -> bool:
    for raw in text.splitlines():
        line = raw.strip()
        if line == "[Install]" or line.startswith("WantedBy="):
            return True
    return False


def _query_unit_state(unit_name: str) -> dict[str, str]:
    if shutil.which("systemctl") is None:
        return {}
    try:
        completed = subprocess.run(
            ["systemctl", "show", unit_name, "-p", "LoadState", "-p", "UnitFileState"],
            check=False,
            capture_output=True,
            text=True,
        )
    except OSError:
        return {}
    state: dict[str, str] = {}
    for raw in completed.stdout.splitlines():
        if "=" not in raw:
            continue
        key, value = raw.split("=", 1)
        state[key.strip()] = value.strip()
    return state


def _manifest_checks(manifest: Path | None, current_root: Path, checks: list[Check]) -> None:
    if manifest is None:
        checks.append(Check("native_manifest", False, "--manifest is required"))
        return
    if not _regular_file(manifest):
        checks.append(Check("native_manifest", False, f"missing: {manifest}"))
        return
    try:
        payload = json.loads(manifest.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        checks.append(Check("native_manifest", False, f"invalid: {type(exc).__name__}"))
        return
    if not isinstance(payload, dict):
        checks.append(Check("native_manifest", False, "manifest must be a JSON object"))
        return
    checks.append(Check("native_manifest", True, "readable"))

    architecture = payload.get("architecture", payload.get("arch"))
    architecture_ok = architecture in ACCEPTED_ARCHITECTURES
    checks.append(Check(
        "native_manifest_architecture",
        architecture_ok,
        architecture if architecture_ok else "architecture must be aarch64 or arm64",
    ))

    signature = payload.get("signature")
    signature_ok = isinstance(signature, str) and bool(signature.strip())
    signed_ok = payload.get("signed") is True
    checks.append(Check(
        "native_manifest_signature",
        signature_ok or signed_ok,
        "signed" if signature_ok or signed_ok else "signature missing",
    ))

    libraries = payload.get("requiredLibraries")
    libraries_ok = (
        isinstance(libraries, list)
        and bool(libraries)
        and all(isinstance(name, str) and bool(name) for name in libraries)
    )
    checks.append(Check(
        "required_libraries",
        libraries_ok,
        "declared" if libraries_ok else "requiredLibraries must be a non-empty list of strings",
    ))
    if not libraries_ok:
        checks.append(Check("required_libraries_present", False, "requiredLibraries unavailable"))
        return
    missing = [name for name in libraries if not _library_present(current_root, name)]
    checks.append(Check(
        "required_libraries_present",
        not missing,
        "present" if not missing else "missing: " + ", ".join(missing),
    ))


def _library_present(current_root: Path, name: str) -> bool:
    path = _library_file(current_root, name)
    return path is not None and path.is_file()


def _credential_checks(gateway_env: Path, checks: list[Check]) -> None:
    checks.append(Check(
        "gateway.env",
        _regular_file(gateway_env),
        "present" if _regular_file(gateway_env) else f"missing: {gateway_env}",
    ))
    if not _regular_file(gateway_env):
        checks.extend([
            Check("gateway_env_mode", False, "gateway.env unavailable"),
            Check("grpc_api_key_required", False, "gateway.env unavailable"),
            Check("grpc_api_key", False, "gateway.env unavailable"),
            Check("grpc_tls_pair", False, "gateway.env unavailable"),
        ])
        return

    checks.append(Check(
        "gateway_env_mode",
        _owner_only(gateway_env),
        "owner-only" if _owner_only(gateway_env) else "gateway.env must not be group or world accessible",
    ))
    try:
        values = _env(gateway_env)
    except (OSError, UnicodeError) as exc:
        checks.append(Check("gateway.env_parse", False, type(exc).__name__))
        values = {}
    # The deployment gate expects the string true. A numeric 1 is not accepted.
    require_key = values.get("LINGTU_GRPC_REQUIRE_API_KEY") == "true"
    checks.append(Check(
        "grpc_api_key_required",
        require_key,
        "enabled" if require_key else "LINGTU_GRPC_REQUIRE_API_KEY must be true",
    ))
    key = values.get("LINGTU_GRPC_API_KEY", "")
    key_ok = bool(key.strip()) and key.isascii()
    checks.append(Check(
        "grpc_api_key",
        key_ok,
        "configured (value hidden)" if key_ok else "missing or non-ASCII",
    ))

    cert = values.get("LINGTU_GRPC_TLS_CERT_FILE", "")
    private_key = values.get("LINGTU_GRPC_TLS_KEY_FILE", "")
    pair = bool(cert) and bool(private_key)
    checks.append(Check(
        "grpc_tls_pair",
        pair,
        "configured" if pair else "certificate and key must be configured together",
    ))
    if not pair:
        return
    cert_path = Path(cert)
    key_path = Path(private_key)
    cert_ok = _regular_file(cert_path)
    key_ok = _regular_file(key_path)
    checks.append(Check(
        "grpc_tls_certificate",
        cert_ok,
        "readable" if cert_ok else f"missing: {cert_path}",
    ))
    checks.append(Check(
        "grpc_tls_private_key",
        key_ok,
        "readable" if key_ok else f"missing: {key_path}",
    ))
    if key_ok:
        mode_ok = _owner_only(key_path)
        checks.append(Check(
            "grpc_tls_private_key_mode",
            mode_ok,
            "owner-only" if mode_ok else "private key must not be group or world accessible",
        ))


def inspect(
    *,
    current_root: Path,
    service_file: Path,
    gateway_env: Path,
    manifest: Path | None,
    session_file: Path,
    expect_listening: bool = False,
    unit_state: Mapping[str, str] | None = None,
) -> list[Check]:
    checks: list[Check] = []
    checks.append(Check(
        "current_root",
        current_root.is_dir(),
        "present" if current_root.is_dir() else f"missing: {current_root}",
    ))
    _manifest_checks(manifest, current_root, checks)

    service_ok = _regular_file(service_file)
    checks.append(Check(
        "lt-host.service",
        service_ok,
        "present" if service_ok else f"missing: {service_file}",
    ))
    service_text = ""
    read_error = ""
    if service_ok:
        try:
            service_text = service_file.read_text(encoding="utf-8")
        except (OSError, UnicodeError) as exc:
            read_error = type(exc).__name__
    gated = (
        "require_product_session.sh host" in service_text
        and "EnvironmentFile=/run/lingtu/session.env" in service_text
    )
    checks.append(Check(
        "product_session_gate",
        gated,
        read_error or ("present" if gated else "lt-host is not guarded by a ProductControl session"),
    ))
    boot_enabled = _boot_enabled(service_text) if service_text else True
    boot_ok = service_ok and not boot_enabled
    checks.append(Check(
        "host_unit_not_boot_enabled",
        boot_ok,
        "unit has no boot install section" if boot_ok else "lt-host must not install a boot target",
    ))

    state = dict(unit_state) if unit_state is not None else _query_unit_state(service_file.name)
    load_state = state.get("LoadState", "")
    unit_file_state = state.get("UnitFileState", "")
    checks.append(Check(
        "host_load_state",
        load_state == "loaded",
        "loaded" if load_state == "loaded" else f"LoadState={load_state or 'unknown'}",
    ))
    unit_file_ok = unit_file_state in ACCEPTED_UNIT_FILE_STATES
    checks.append(Check(
        "host_unit_file_state",
        unit_file_ok,
        unit_file_state if unit_file_ok else "Host must be disabled or static, not enabled",
    ))

    _credential_checks(gateway_env, checks)

    session_ok = _regular_file(session_file)
    checks.append(Check(
        "product_session",
        session_ok,
        "present" if session_ok else f"missing: {session_file}",
    ))

    if expect_listening:
        listening = False
        try:
            with socket.create_connection(("127.0.0.1", _LOCAL_GATEWAY_PORT), timeout=0.5):
                listening = True
        except OSError:
            pass
        checks.append(Check(
            "grpc_gateway_listening",
            listening,
            "listening" if listening else "not listening",
        ))
    return checks


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--current-root", type=Path, default=Path("/opt/lingtu/current"))
    parser.add_argument("--service-file", type=Path, default=Path("/etc/systemd/system/lt-host.service"))
    parser.add_argument("--gateway-env", type=Path, default=Path("/etc/lingtu/gateway.env"))
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--session-file", type=Path, default=Path("/run/lingtu/session.env"))
    parser.add_argument(
        "--expect-listening",
        action="store_true",
        help="also observe whether the local gateway port is already open; does not start it",
    )
    parser.add_argument("--json", action="store_true", help="emit structured results")
    args = parser.parse_args(argv)
    checks = inspect(
        current_root=args.current_root,
        service_file=args.service_file,
        gateway_env=args.gateway_env,
        manifest=args.manifest,
        session_file=args.session_file,
        expect_listening=args.expect_listening,
    )
    passed = all(check.ok for check in checks)
    result = {"ready": passed, "checks": [asdict(check) for check in checks]}
    if args.json:
        print(json.dumps(result, ensure_ascii=False, indent=2))
    else:
        for check in checks:
            print(f"{'PASS' if check.ok else 'BLOCK'} {check.name}: {check.detail}")
        print("READY" if passed else "BLOCKED")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
