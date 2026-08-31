import os
import shlex
import shutil
import subprocess
import textwrap
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
THUNDER_DEPLOY = REPO_ROOT / "scripts" / "deploy" / "thunder"


def _read(name: str) -> str:
    return (THUNDER_DEPLOY / name).read_text(encoding="utf-8")


def _bash_path(path: Path) -> str:
    if os.name != "nt":
        return str(path)
    probe = subprocess.run(
        ["bash", "-lc", "command -v wslpath"],
        check=False,
        cwd=REPO_ROOT,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    if probe.returncode != 0:
        pytest.skip("WSL bash with wslpath is required")
    result = subprocess.run(
        ["bash", "-lc", f"wslpath -a {shlex.quote(str(path))}"],
        check=True,
        cwd=REPO_ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        encoding="utf-8",
        errors="ignore",
    )
    return result.stdout.strip()


def _write_shell(path: Path, source: str) -> None:
    path.write_text(textwrap.dedent(source).lstrip(), encoding="utf-8", newline="\n")
    path.chmod(0o755)


PRODUCT_MODE_UNITS = (
    "lt-lidar.service",
    "lt-slam.service",
    "lt-maps.service",
    "lt-terrain.service",
    "lt-nav.service",
    "lt-explore.service",
    "lt-camera.service",
    "lt-host.service",
)


def test_host_runtime_no_longer_pulls_in_product_role_units() -> None:
    text = _read("lt-host.service")
    wants = next(line for line in text.splitlines() if line.startswith("Wants="))

    assert wants == "Wants=network-online.target"
    assert "lt-slam.service" not in wants
    assert "lt-nav.service" not in wants
    assert "lt-driver.service" not in wants
    assert "brainstem.service" not in wants
    assert "robot-brainstem.service" not in wants
    assert "slam.service" not in wants


def test_gateway_runtime_is_direct_product_control_process() -> None:
    text = _read("lt-host.service")

    assert "Type=simple" in text
    assert "NotifyAccess=all" not in text
    assert "WatchdogSec=" not in text
    assert "WatchdogSignal=" not in text
    assert "KillMode=control-group" not in text
    assert "run_http_watchdog.sh" not in text
    assert "ExecStart=/bin/bash -lc" in text


@pytest.mark.parametrize("unit_name", PRODUCT_MODE_UNITS)
def test_product_mode_units_do_not_self_manage_lifecycle(unit_name: str) -> None:
    text = _read(unit_name)

    assert "WantedBy=multi-user.target" not in text
    assert "Restart=on-failure" not in text
    assert "run_status_file_watchdog.sh" not in text
    assert "run_http_watchdog.sh" not in text
    assert "KillMode=control-group" not in text


def test_persistent_driver_keeps_motion_output_supervision() -> None:
    text = _read("lt-driver.service")

    assert "Type=notify" in text
    assert "NotifyAccess=all" in text
    assert "WatchdogSec=6s" in text
    assert "WatchdogSignal=SIGTERM" in text
    assert "KillMode=control-group" in text
    assert "Restart=on-failure" in text
    assert "run_status_file_watchdog.sh --status-file ${LINGTU_DRIVER_STATUS_FILE}" in text
    assert "After=network-online.target" in text
    assert "Wants=network-online.target" in text
    assert "lt-nav.service" not in text
    assert "WantedBy=multi-user.target" in text


def test_field_driver_reads_selected_backend_from_product_session() -> None:
    text = _read("lt-driver.service")

    assert "Environment=LINGTU_BRAINSTEM_HOST=127.0.0.1" not in text
    assert "EnvironmentFile=/run/lingtu/session.env" in text
    assert "require_product_session.sh driver" in text
    assert "run_driver.sh --require-remote" not in text


def test_go2_launcher_validates_network_routing_without_rewriting_dds() -> None:
    text = _read("run_driver.sh")

    assert '[[ ! -r "/sys/class/net/${GO2_INTERFACE}/carrier" ]]' in text
    assert '[[ "${LINGTU_DDS_NETWORK_INTERFACE:-}" != "${GO2_INTERFACE}" ]]' in text
    assert "<NetworkInterfaceAddress>${GO2_INTERFACE}</NetworkInterfaceAddress>" not in text
    assert 'export LINGTU_DDS_NETWORK_INTERFACE=' not in text
    assert 'export CYCLONEDDS_URI=' not in text


def test_driver_installation_uses_robot_config_and_the_service_catalog() -> None:
    install_all = _read("install_services.sh")
    install_driver = _read("install_driver_service.sh")

    assert '[ "${service}" = "driver" ]' in install_all
    assert 'bash "${SCRIPT_DIR}/install_driver_service.sh"' in install_all
    assert 'bash "${SCRIPT_DIR}/install_catalog_service.sh" "${service}"' in install_all
    assert "load_config" in install_driver
    assert "validate_config" in install_driver
    assert 'install_catalog_service.sh" driver' in install_driver
    assert "brainstem.env" not in install_driver
    assert "systemctl disable --now" in install_driver
    assert '"driver.service"' in install_driver
    assert "driver.service" in _read("lt-driver.service")


@pytest.mark.skipif(shutil.which("bash") is None, reason="bash is required")
@pytest.mark.parametrize("host", ("", "localhost", "127.0.0.1", "::1", "brainstem.local"))
def test_field_driver_launcher_rejects_missing_or_loopback_brainstem(
    tmp_path: Path,
    host: str,
) -> None:
    invoked = tmp_path / "driver.invoked"
    fake_driver = tmp_path / "lingtu_driver"
    harness = tmp_path / "driver-launcher-harness.sh"
    _write_shell(
        fake_driver,
        f"""
        #!/usr/bin/env bash
        printf invoked > {_bash_path(invoked)!r}
        """,
    )
    _write_shell(
        harness,
        f"""
        #!/usr/bin/env bash
        export LINGTU_DRIVER_BIN={shlex.quote(_bash_path(fake_driver))}
        export LINGTU_DRIVER_BACKEND=doso
        export LINGTU_BRAINSTEM_HOST={shlex.quote(host)}
        exec bash {shlex.quote(_bash_path(THUNDER_DEPLOY / "run_driver.sh"))}
        """,
    )

    result = subprocess.run(
        ["bash", _bash_path(harness)],
        cwd=REPO_ROOT,
        timeout=10,
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        encoding="utf-8",
        errors="replace",
    )

    assert result.returncode == 64
    assert "remote Brainstem" in result.stderr
    assert not invoked.exists()


@pytest.mark.skipif(shutil.which("bash") is None, reason="bash is required")
def test_field_driver_launcher_accepts_explicit_remote_brainstem(
    tmp_path: Path,
) -> None:
    observed_host = tmp_path / "observed-host"
    fake_driver = tmp_path / "lingtu_driver"
    harness = tmp_path / "driver-launcher-harness.sh"
    ca_file = tmp_path / "ca.pem"
    cert_file = tmp_path / "client.pem"
    key_file = tmp_path / "client.key"
    for path in (ca_file, cert_file, key_file):
        path.write_text("test-only-pem", encoding="utf-8")
    _write_shell(
        fake_driver,
        f"""
        #!/usr/bin/env bash
        printf '%s' "${{LINGTU_BRAINSTEM_HOST:-}}" > {_bash_path(observed_host)!r}
        """,
    )
    _write_shell(
        harness,
        f"""
        #!/usr/bin/env bash
        export LINGTU_DRIVER_BIN={shlex.quote(_bash_path(fake_driver))}
        export LINGTU_DRIVER_BACKEND=doso
        export LINGTU_BRAINSTEM_HOST=192.168.114.50
        export LINGTU_DRIVER_TLS_CA_FILE={shlex.quote(_bash_path(ca_file))}
        export LINGTU_DRIVER_TLS_CERT_FILE={shlex.quote(_bash_path(cert_file))}
        export LINGTU_DRIVER_TLS_KEY_FILE={shlex.quote(_bash_path(key_file))}
        exec bash {shlex.quote(_bash_path(THUNDER_DEPLOY / "run_driver.sh"))}
        """,
    )

    result = subprocess.run(
        ["bash", _bash_path(harness)],
        cwd=REPO_ROOT,
        timeout=10,
        check=False,
    )

    assert result.returncode == 0
    assert observed_host.read_text(encoding="utf-8") == "192.168.114.50"


@pytest.mark.skipif(shutil.which("bash") is None, reason="bash is required")
def test_field_driver_launcher_accepts_go2_network_interface(tmp_path: Path) -> None:
    observed_interface = tmp_path / "observed-interface"
    observed_dds_uri = tmp_path / "observed-dds-uri"
    fake_driver = tmp_path / "lingtu_driver"
    harness = tmp_path / "go2-driver-launcher-harness.sh"
    _write_shell(
        fake_driver,
        f"""
        #!/usr/bin/env bash
        printf '%s' "${{LINGTU_DRIVER_NETWORK_INTERFACE:-}}" > {_bash_path(observed_interface)!r}
        printf '%s' "${{CYCLONEDDS_URI:-}}" > {_bash_path(observed_dds_uri)!r}
        """,
    )
    _write_shell(
        harness,
        f"""
        #!/usr/bin/env bash
        export LINGTU_DRIVER_BIN={shlex.quote(_bash_path(fake_driver))}
        export LINGTU_DRIVER_BACKEND=go2
        export LINGTU_DRIVER_NETWORK_INTERFACE=lo
        export LINGTU_DRIVER_NETWORK_ADDRESS=127.0.0.1/8
        export LINGTU_DRIVER_PROBE_IP=127.0.0.1
        export LINGTU_DDS_NETWORK_INTERFACE=lo
        export CYCLONEDDS_URI=file:///run/lingtu/cyclonedds.xml
        exec bash {shlex.quote(_bash_path(THUNDER_DEPLOY / "run_driver.sh"))}
        """,
    )

    result = subprocess.run(
        ["bash", _bash_path(harness)],
        cwd=REPO_ROOT,
        timeout=10,
        check=False,
    )

    assert result.returncode == 0
    assert observed_interface.read_text(encoding="utf-8") == "lo"
    assert observed_dds_uri.read_text(encoding="utf-8") == "file:///run/lingtu/cyclonedds.xml"


@pytest.mark.skipif(shutil.which("bash") is None, reason="bash is required")
def test_field_driver_launcher_rejects_remote_brainstem_without_mtls(
    tmp_path: Path,
) -> None:
    fake_driver = tmp_path / "lingtu_driver"
    harness = tmp_path / "driver-launcher-no-mtls.sh"
    _write_shell(fake_driver, "#!/usr/bin/env bash\nexit 0\n")
    _write_shell(
        harness,
        f"""
        #!/usr/bin/env bash
        export LINGTU_DRIVER_BIN={shlex.quote(_bash_path(fake_driver))}
        export LINGTU_DRIVER_BACKEND=doso
        export LINGTU_BRAINSTEM_HOST=192.168.114.50
        unset LINGTU_DRIVER_TLS_CA_FILE
        unset LINGTU_DRIVER_TLS_CERT_FILE
        unset LINGTU_DRIVER_TLS_KEY_FILE
        exec bash {shlex.quote(_bash_path(THUNDER_DEPLOY / "run_driver.sh"))}
        """,
    )

    result = subprocess.run(
        ["bash", _bash_path(harness)],
        cwd=REPO_ROOT,
        timeout=10,
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        encoding="utf-8",
        errors="replace",
    )

    assert result.returncode == 64
    assert "mTLS" in result.stderr


@pytest.mark.skipif(shutil.which("bash") is None, reason="bash is required")
def test_status_watchdog_pings_only_when_heartbeat_advances_and_forwards_stop(
    tmp_path: Path,
) -> None:
    notify_log = tmp_path / "notify.log"
    stopped_marker = tmp_path / "child.stopped"
    fake_notify = tmp_path / "systemd-notify"
    child = tmp_path / "heartbeat-child.sh"
    harness = tmp_path / "watchdog-harness.sh"
    _write_shell(
        fake_notify,
        """
        #!/usr/bin/env bash
        set -euo pipefail
        printf '%s\n' "$*" >> "${NOTIFY_LOG:?}"
        """,
    )
    _write_shell(
        child,
        """
        #!/usr/bin/env bash
        set -euo pipefail
        status_file="$1"
        stopped_marker="$2"
        trap 'printf stopped > "${stopped_marker}"; exit 0' TERM INT
        sleep 0.05
        printf one > "${status_file}.tmp"
        mv "${status_file}.tmp" "${status_file}"
        sleep 0.15
        printf two > "${status_file}.tmp"
        mv "${status_file}.tmp" "${status_file}"
        while :; do sleep 1; done
        """,
    )

    wrapper = THUNDER_DEPLOY / "run_status_file_watchdog.sh"
    temp_dir = _bash_path(tmp_path)
    wrapper_path = _bash_path(wrapper)
    _write_shell(
        harness,
        f"""
        #!/usr/bin/env bash
        set -euo pipefail
        export LINGTU_SYSTEMD_NOTIFY_BIN={shlex.quote(f"{temp_dir}/systemd-notify")}
        export NOTIFY_LOG={shlex.quote(f"{temp_dir}/notify.log")}
        bash {shlex.quote(wrapper_path)} \\
            --status-file {shlex.quote(f"{temp_dir}/status.json")} \\
            --startup-timeout-s 3 --poll-interval-s 0.02 -- \\
            bash {shlex.quote(f"{temp_dir}/heartbeat-child.sh")} \\
            {shlex.quote(f"{temp_dir}/status.json")} \\
            {shlex.quote(f"{temp_dir}/child.stopped")} &
        wrapper_pid=$!
        observed=0
        for _ in $(seq 1 250); do
            observed=$(grep -c 'WATCHDOG=1' {shlex.quote(f"{temp_dir}/notify.log")} 2>/dev/null || true)
            if [ "${{observed}}" -ge 2 ]; then
                break
            fi
            sleep 0.02
        done
        if [ "${{observed}}" -lt 2 ]; then
            kill -TERM "${{wrapper_pid}}" 2>/dev/null || true
            wait "${{wrapper_pid}}" 2>/dev/null || true
            exit 1
        fi
        kill -TERM "${{wrapper_pid}}"
        set +e
        wait "${{wrapper_pid}}"
        code=$?
        set -e
        [ "${{code}}" -eq 143 ]
        """,
    )
    subprocess.run(
        ["bash", _bash_path(harness)],
        check=True,
        cwd=REPO_ROOT,
        timeout=10,
    )

    notifications = notify_log.read_text(encoding="utf-8").splitlines()
    watchdog_pings = [line for line in notifications if "WATCHDOG=1" in line]
    assert len(watchdog_pings) == 2
    assert "READY=1" in watchdog_pings[0]
    assert stopped_marker.read_text(encoding="utf-8") == "stopped"


@pytest.mark.skipif(shutil.which("bash") is None, reason="bash is required")
def test_status_watchdog_fails_startup_and_stops_child_without_heartbeat(
    tmp_path: Path,
) -> None:
    stopped_marker = tmp_path / "child.stopped"
    child = tmp_path / "silent-child.sh"
    harness = tmp_path / "missing-heartbeat-harness.sh"
    _write_shell(
        child,
        """
        #!/usr/bin/env bash
        set -euo pipefail
        stopped_marker="$1"
        trap 'printf stopped > "${stopped_marker}"; exit 0' TERM INT
        while :; do sleep 1; done
        """,
    )

    temp_dir = _bash_path(tmp_path)
    wrapper_path = _bash_path(THUNDER_DEPLOY / "run_status_file_watchdog.sh")
    _write_shell(
        harness,
        f"""
        #!/usr/bin/env bash
        set -u
        export LINGTU_SYSTEMD_NOTIFY_BIN=/bin/true
        bash {shlex.quote(wrapper_path)} \\
            --status-file {shlex.quote(f"{temp_dir}/missing-status.json")} \\
            --startup-timeout-s 1 --poll-interval-s 0.02 -- \\
            bash {shlex.quote(f"{temp_dir}/silent-child.sh")} \\
            {shlex.quote(f"{temp_dir}/child.stopped")}
        code=$?
        [ "${{code}}" -eq 70 ]
        """,
    )
    subprocess.run(
        ["bash", _bash_path(harness)],
        check=True,
        cwd=REPO_ROOT,
        timeout=10,
    )

    assert stopped_marker.read_text(encoding="utf-8") == "stopped"
