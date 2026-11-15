#!/usr/bin/env python3
"""Read-only preflight for the credentialed DOSO gRPC Gateway on 5052."""

from __future__ import annotations

import argparse
import json
import socket
import sys
from dataclasses import asdict, dataclass
from pathlib import Path


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


def _check_file(path: Path, name: str, checks: list[Check]) -> None:
    checks.append(Check(name, path.is_file(), "present" if path.is_file() else f"missing: {path}"))


def inspect(
    *,
    current_root: Path,
    service_file: Path,
    gateway_env: Path,
    manifest: Path | None,
    session_file: Path | None,
    expect_listening: bool,
) -> list[Check]:
    checks: list[Check] = []
    checks.append(Check("current_root", current_root.is_dir(), "present" if current_root.is_dir() else f"missing: {current_root}"))

    for relative in (
        "bin/lingtu_dds_probe",
        "lib/liblingtu_nav_client.so",
        "lib/liblingtu_inspection_evidence_bridge.so",
    ):
        _check_file(current_root / relative, f"runtime:{relative}", checks)

    _check_file(service_file, "lt-host.service", checks)
    if service_file.is_file():
        service = service_file.read_text(encoding="utf-8")
        checks.append(Check(
            "product_session_gate",
            "require_product_session.sh host" in service and "EnvironmentFile=/run/lingtu/session.env" in service,
            "present" if "require_product_session.sh host" in service and "EnvironmentFile=/run/lingtu/session.env" in service else "lt-host is not guarded by ProductControl session",
        ))
    else:
        checks.append(Check("product_session_gate", False, "service file missing"))

    _check_file(gateway_env, "gateway.env", checks)
    values: dict[str, str] = {}
    if gateway_env.is_file():
        try:
            values = _env(gateway_env)
        except (OSError, UnicodeError) as exc:
            checks.append(Check("gateway.env_parse", False, type(exc).__name__))
    if values:
        key = values.get("LINGTU_GRPC_API_KEY", "")
        require_key = values.get("LINGTU_GRPC_REQUIRE_API_KEY") == "1"
        checks.append(Check("grpc_api_key_required", require_key, "enabled" if require_key else "LINGTU_GRPC_REQUIRE_API_KEY is not 1"))
        checks.append(Check("grpc_api_key", bool(key.strip()) and key.isascii(), "configured (value hidden)" if key.strip() and key.isascii() else "missing or non-ASCII"))
        cert = values.get("LINGTU_GRPC_TLS_CERT_FILE", "")
        private_key = values.get("LINGTU_GRPC_TLS_KEY_FILE", "")
        pair = bool(cert) and bool(private_key)
        checks.append(Check("grpc_tls_pair", pair, "configured" if pair else "certificate/key must be configured together"))
        if pair:
            cert_path = Path(cert)
            key_path = Path(private_key)
            checks.append(Check("grpc_tls_certificate", cert_path.is_file(), "readable" if cert_path.is_file() else f"missing: {cert_path}"))
            checks.append(Check("grpc_tls_private_key", key_path.is_file(), "readable" if key_path.is_file() else f"missing: {key_path}"))
    else:
        checks.extend([
            Check("grpc_api_key_required", False, "gateway.env unavailable"),
            Check("grpc_api_key", False, "gateway.env unavailable"),
            Check("grpc_tls_pair", False, "gateway.env unavailable"),
        ])

    if manifest is None:
        checks.append(Check("native_manifest", False, "--manifest is required"))
    elif not manifest.is_file():
        checks.append(Check("native_manifest", False, f"missing: {manifest}"))
    else:
        try:
            payload = json.loads(manifest.read_text(encoding="utf-8"))
            artifacts = payload.get("artifacts") or []
            compat = set(artifacts[0].get("hw_compat") or []) if artifacts and isinstance(artifacts[0], dict) else set()
            checks.append(Check("native_manifest", "aarch64" in compat or "arm64" in compat, "aarch64 compatibility declared" if "aarch64" in compat or "arm64" in compat else "aarch64 compatibility missing"))
            checks.append(Check("native_manifest_signature", bool(payload.get("signature")), "signed" if payload.get("signature") else "signature missing"))
        except (OSError, ValueError, TypeError, IndexError) as exc:
            checks.append(Check("native_manifest", False, f"invalid: {type(exc).__name__}"))

    if session_file is not None:
        checks.append(Check("product_session", session_file.is_file(), "present" if session_file.is_file() else f"missing: {session_file}"))

    if expect_listening:
        listening = False
        try:
            with socket.create_connection(("127.0.0.1", 5052), timeout=0.5):
                listening = True
        except OSError:
            pass
        checks.append(Check("grpc_5052", listening, "listening" if listening else "not listening"))

    return checks


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--current-root", type=Path, default=Path("/opt/lingtu/current"))
    parser.add_argument("--service-file", type=Path, default=Path("/etc/systemd/system/lt-host.service"))
    parser.add_argument("--gateway-env", type=Path, default=Path("/etc/lingtu/gateway.env"))
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--session-file", type=Path)
    parser.add_argument("--expect-listening", action="store_true")
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
