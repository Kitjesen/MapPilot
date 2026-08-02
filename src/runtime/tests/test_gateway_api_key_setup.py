"""Behavior tests for scoped Gateway API-key provisioning."""

from __future__ import annotations

import os
import re
import shutil
import stat
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
THUNDER_DEPLOY = REPO_ROOT / "scripts" / "deploy" / "thunder"
SCRIPT = THUNDER_DEPLOY / "configure_gateway_api_key.sh"
BASH = shutil.which("bash")
POSIX_BASH = os.name == "posix" and BASH is not None
requires_posix_bash = pytest.mark.skipif(
    not POSIX_BASH,
    reason="native POSIX bash is required for permission-sensitive shell tests",
)


def _run_setup(
    gateway_target: Path,
    map_target: Path,
    *args: str,
) -> subprocess.CompletedProcess[str]:
    if BASH is None:
        raise RuntimeError("bash is unavailable")
    env = os.environ.copy()
    env.pop("LINGTU_RMF_API_KEY", None)
    env["LINGTU_GATEWAY_ENV_FILE"] = str(gateway_target)
    env["LINGTU_MAP_CLIENT_ENV_FILE"] = str(map_target)
    return subprocess.run(  # noqa: S603 - Runs a repository-owned helper in an isolated test.
        [BASH, str(SCRIPT), *args],
        cwd=REPO_ROOT,
        env=env,
        check=False,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )


def _file_key(target: Path, expected_name: str) -> str:
    content = target.read_text(encoding="utf-8")
    prefix = f"{expected_name}="
    assert content.startswith(prefix)
    assert content.endswith("\n")
    assert content.count("\n") == 1
    value = content[len(prefix) : -1]
    assert len(value) >= 64
    assert re.fullmatch(r"[A-Za-z0-9_-]+", value)
    return value


def _credentials(gateway_target: Path, map_target: Path) -> tuple[str, str]:
    return (
        _file_key(gateway_target, "LINGTU_API_KEY"),
        _file_key(map_target, "LINGTU_MAP_API_KEY"),
    )


def _assert_secure_file(target: Path, expected_mode: int) -> None:
    metadata = target.lstat()
    assert stat.S_ISREG(metadata.st_mode)
    assert not target.is_symlink()
    assert stat.S_IMODE(metadata.st_mode) == expected_mode
    assert metadata.st_uid == os.geteuid()
    assert metadata.st_gid == os.getegid()
    assert metadata.st_nlink == 1


def _assert_secrets_hidden(
    result: subprocess.CompletedProcess[str],
    *tokens: str,
) -> None:
    for token in tokens:
        assert token not in result.stdout
        assert token not in result.stderr


def _write_gateway(
    target: Path,
    operator_key: str,
    embedded_map_key: str | None = None,
) -> None:
    content = f"LINGTU_API_KEY={operator_key}\n"
    if embedded_map_key is not None:
        content += f"LINGTU_MAP_API_KEY={embedded_map_key}\n"
    target.write_text(content, encoding="utf-8")
    target.chmod(0o600)


def _write_map(target: Path, map_key: str) -> None:
    target.write_text(f"LINGTU_MAP_API_KEY={map_key}\n", encoding="utf-8")
    target.chmod(0o640)


@requires_posix_bash
def test_setup_creates_separate_scoped_credentials_with_private_permissions(
    tmp_path: Path,
) -> None:
    gateway_target = tmp_path / "gateway.env"
    map_target = tmp_path / "map-client.env"

    result = _run_setup(gateway_target, map_target)

    assert result.returncode == 0, result.stderr
    operator_key, map_key = _credentials(gateway_target, map_target)
    assert operator_key != map_key
    _assert_secrets_hidden(result, operator_key, map_key)
    _assert_secure_file(gateway_target, 0o600)
    _assert_secure_file(map_target, 0o640)


@requires_posix_bash
def test_current_split_state_is_left_unchanged(tmp_path: Path) -> None:
    gateway_target = tmp_path / "gateway.env"
    map_target = tmp_path / "map-client.env"
    created = _run_setup(gateway_target, map_target)
    assert created.returncode == 0, created.stderr
    before_gateway = gateway_target.read_bytes()
    before_map = map_target.read_bytes()
    keys = _credentials(gateway_target, map_target)

    repeated = _run_setup(gateway_target, map_target)

    assert repeated.returncode == 0, repeated.stderr
    assert gateway_target.read_bytes() == before_gateway
    assert map_target.read_bytes() == before_map
    assert "left unchanged" in repeated.stderr
    assert "--rotate" in repeated.stderr
    _assert_secrets_hidden(repeated, *keys)


@requires_posix_bash
def test_rotate_replaces_both_scoped_credentials(tmp_path: Path) -> None:
    gateway_target = tmp_path / "gateway.env"
    map_target = tmp_path / "map-client.env"
    created = _run_setup(gateway_target, map_target)
    assert created.returncode == 0, created.stderr
    old_operator_key, old_map_key = _credentials(gateway_target, map_target)

    rotated = _run_setup(gateway_target, map_target, "--rotate")

    assert rotated.returncode == 0, rotated.stderr
    operator_key, map_key = _credentials(gateway_target, map_target)
    assert operator_key != old_operator_key
    assert map_key != old_map_key
    assert operator_key != map_key
    _assert_secure_file(gateway_target, 0o600)
    _assert_secure_file(map_target, 0o640)
    _assert_secrets_hidden(
        rotated,
        old_operator_key,
        old_map_key,
        operator_key,
        map_key,
    )


@requires_posix_bash
def test_rotate_map_replaces_only_the_map_client_credential(tmp_path: Path) -> None:
    gateway_target = tmp_path / "gateway.env"
    map_target = tmp_path / "map-client.env"
    created = _run_setup(gateway_target, map_target)
    assert created.returncode == 0, created.stderr
    old_operator_key, old_map_key = _credentials(gateway_target, map_target)
    before_gateway = gateway_target.read_bytes()

    rotated = _run_setup(gateway_target, map_target, "--rotate-map")

    assert rotated.returncode == 0, rotated.stderr
    operator_key, map_key = _credentials(gateway_target, map_target)
    assert gateway_target.read_bytes() == before_gateway
    assert operator_key == old_operator_key
    assert map_key != old_map_key
    assert map_key != operator_key
    assert "Gateway key was preserved" in rotated.stdout
    _assert_secrets_hidden(rotated, old_operator_key, old_map_key, map_key)


@requires_posix_bash
def test_legacy_operator_file_gains_a_map_credential_without_rotation(
    tmp_path: Path,
) -> None:
    gateway_target = tmp_path / "gateway.env"
    map_target = tmp_path / "map-client.env"
    operator_key = "a" * 64
    _write_gateway(gateway_target, operator_key)
    before_gateway = gateway_target.read_bytes()

    migrated = _run_setup(gateway_target, map_target)

    assert migrated.returncode == 0, migrated.stderr
    resulting_operator_key, map_key = _credentials(gateway_target, map_target)
    assert gateway_target.read_bytes() == before_gateway
    assert resulting_operator_key == operator_key
    assert map_key != operator_key
    assert "Migrated" in migrated.stdout
    _assert_secure_file(gateway_target, 0o600)
    _assert_secure_file(map_target, 0o640)
    _assert_secrets_hidden(migrated, operator_key, map_key)


@requires_posix_bash
@pytest.mark.parametrize("map_already_split", (False, True))
def test_embedded_map_key_is_migrated_then_repeated_runs_converge(
    tmp_path: Path,
    map_already_split: bool,
) -> None:
    gateway_target = tmp_path / "gateway.env"
    map_target = tmp_path / "map-client.env"
    operator_key = "a" * 64
    map_key = "b" * 64
    _write_gateway(gateway_target, operator_key, map_key)
    if map_already_split:
        _write_map(map_target, map_key)

    migrated = _run_setup(gateway_target, map_target)

    assert migrated.returncode == 0, migrated.stderr
    assert _credentials(gateway_target, map_target) == (operator_key, map_key)
    assert gateway_target.read_text(encoding="utf-8") == (
        f"LINGTU_API_KEY={operator_key}\n"
    )
    _assert_secure_file(gateway_target, 0o600)
    _assert_secure_file(map_target, 0o640)
    _assert_secrets_hidden(migrated, operator_key, map_key)
    before_gateway = gateway_target.read_bytes()
    before_map = map_target.read_bytes()

    repeated = _run_setup(gateway_target, map_target)

    assert repeated.returncode == 0, repeated.stderr
    assert gateway_target.read_bytes() == before_gateway
    assert map_target.read_bytes() == before_map
    assert "left unchanged" in repeated.stderr
    _assert_secrets_hidden(repeated, operator_key, map_key)


@requires_posix_bash
def test_conflicting_split_map_key_blocks_implicit_embedded_migration(
    tmp_path: Path,
) -> None:
    gateway_target = tmp_path / "gateway.env"
    map_target = tmp_path / "map-client.env"
    operator_key = "a" * 64
    embedded_map_key = "b" * 64
    split_map_key = "c" * 64
    _write_gateway(gateway_target, operator_key, embedded_map_key)
    _write_map(map_target, split_map_key)
    before_gateway = gateway_target.read_bytes()
    before_map = map_target.read_bytes()

    rejected = _run_setup(gateway_target, map_target)

    assert rejected.returncode == 78
    assert "conflicting" in rejected.stderr
    assert gateway_target.read_bytes() == before_gateway
    assert map_target.read_bytes() == before_map
    _assert_secrets_hidden(
        rejected,
        operator_key,
        embedded_map_key,
        split_map_key,
    )


@requires_posix_bash
def test_rotate_map_resolves_a_conflict_without_rotating_the_operator_key(
    tmp_path: Path,
) -> None:
    gateway_target = tmp_path / "gateway.env"
    map_target = tmp_path / "map-client.env"
    operator_key = "a" * 64
    embedded_map_key = "b" * 64
    split_map_key = "c" * 64
    _write_gateway(gateway_target, operator_key, embedded_map_key)
    _write_map(map_target, split_map_key)

    rotated = _run_setup(gateway_target, map_target, "--rotate-map")

    assert rotated.returncode == 0, rotated.stderr
    resulting_operator_key, map_key = _credentials(gateway_target, map_target)
    assert resulting_operator_key == operator_key
    assert map_key not in {operator_key, embedded_map_key, split_map_key}
    _assert_secrets_hidden(
        rotated,
        operator_key,
        embedded_map_key,
        split_map_key,
        map_key,
    )


@requires_posix_bash
@pytest.mark.parametrize("target_kind", ("gateway", "map"))
@pytest.mark.parametrize("unsafe_kind", ("wrong_mode", "symlink", "fifo", "hardlink"))
def test_rotation_refuses_unsafe_existing_targets(
    tmp_path: Path,
    target_kind: str,
    unsafe_kind: str,
) -> None:
    gateway_target = tmp_path / "gateway.env"
    map_target = tmp_path / "map-client.env"
    created = _run_setup(gateway_target, map_target)
    assert created.returncode == 0, created.stderr
    operator_key, map_key = _credentials(gateway_target, map_target)
    target = gateway_target if target_kind == "gateway" else map_target
    decoy = tmp_path / f"{target_kind}-decoy.env"
    expected_mode = 0o600 if target_kind == "gateway" else 0o640

    if unsafe_kind == "wrong_mode":
        target.chmod(0o644 if target_kind == "gateway" else 0o600)
    elif unsafe_kind == "symlink":
        target.unlink()
        decoy.write_text("do-not-overwrite\n", encoding="utf-8")
        target.symlink_to(decoy)
    elif unsafe_kind == "fifo":
        target.unlink()
        os.mkfifo(target)
    else:
        content = target.read_bytes()
        target.unlink()
        decoy.write_bytes(content)
        decoy.chmod(expected_mode)
        os.link(decoy, target)

    rejected = _run_setup(gateway_target, map_target, "--rotate")

    assert rejected.returncode == 78
    assert "unsafe" in rejected.stderr or "non-symlink" in rejected.stderr
    _assert_secrets_hidden(rejected, operator_key, map_key)
    if unsafe_kind == "symlink":
        assert target.is_symlink()
        assert decoy.read_text(encoding="utf-8") == "do-not-overwrite\n"
    elif unsafe_kind == "hardlink":
        assert target.stat().st_nlink == 2


@requires_posix_bash
def test_setup_refuses_the_same_target_for_both_credentials(tmp_path: Path) -> None:
    target = tmp_path / "credential.env"

    rejected = _run_setup(target, target)

    assert rejected.returncode == 78
    assert "must differ" in rejected.stderr
    assert not target.exists()


@requires_posix_bash
def test_setup_refuses_a_symlink_map_client_parent(tmp_path: Path) -> None:
    real_parent = tmp_path / "real-parent"
    real_parent.mkdir()
    linked_parent = tmp_path / "linked-parent"
    linked_parent.symlink_to(real_parent, target_is_directory=True)
    gateway_target = tmp_path / "gateway.env"

    rejected = _run_setup(gateway_target, linked_parent / "map-client.env")

    assert rejected.returncode == 78
    assert "symlink" in rejected.stderr
    assert not gateway_target.exists()
    assert not (real_parent / "map-client.env").exists()


@requires_posix_bash
def test_rotation_modes_are_mutually_exclusive(tmp_path: Path) -> None:
    gateway_target = tmp_path / "gateway.env"
    map_target = tmp_path / "map-client.env"

    rejected = _run_setup(
        gateway_target,
        map_target,
        "--rotate",
        "--rotate-map",
    )

    assert rejected.returncode == 64
    assert "mutually exclusive" in rejected.stderr
    assert not gateway_target.exists()
    assert not map_target.exists()


def test_deployment_contract_uses_two_least_privilege_environment_files() -> None:
    script = SCRIPT.read_text(encoding="utf-8")
    service_lines = (THUNDER_DEPLOY / "lingtu.service").read_text(
        encoding="utf-8"
    ).splitlines()

    assert 'DEFAULT_GATEWAY_ENV_FILE="/etc/lingtu/gateway.env"' in script
    assert 'DEFAULT_MAP_CLIENT_ENV_FILE="/etc/lingtu/map-client.env"' in script
    assert "gateway_expected_uid=0" in script
    assert "gateway_expected_gid=0" in script
    assert "map_expected_uid=0" in script
    assert 'map_expected_gid="$(id -g sunrise 2>/dev/null)"' in script
    assert 'stat -c \'%u:%g:%a:%h\'' in script
    assert '"${expected_uid}:${expected_gid}:${expected_mode}:1"' in script
    assert '[[ -L "${target}" || ! -f "${target}" ]]' in script
    assert '"${gateway_expected_gid}" 600' in script
    assert '"${map_expected_gid}" 640' in script
    assert "secrets.token_urlsafe(48)" in script
    assert "--rotate-map" in script
    assert "EnvironmentFile=-/etc/lingtu/gateway.env" in service_lines
    assert "EnvironmentFile=-/etc/lingtu/map-client.env" in service_lines
    assert "${EGID}" not in script
    map_install = script.index(
        '"${map_temp}" "${map_target}" "${map_exists}" "${map_snapshot}"'
    )
    gateway_install = script.index(
        '"${gateway_temp}" "${gateway_target}" "${gateway_exists}"'
    )
    assert map_install < gateway_install
    assert "scripts/lingtu --env real svc restart host" in script
