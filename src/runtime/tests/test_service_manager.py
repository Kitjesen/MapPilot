from __future__ import annotations

import subprocess
from dataclasses import dataclass

import pytest


@dataclass
class _RunResult:
    returncode: int = 0
    stdout: str = ""
    stderr: bytes = b""


class _FakeSystemctl:
    def __init__(self, *, active: set[str] | None = None, loaded: set[str] | None = None):
        self.active = set(active or set())
        self.loaded = set(loaded or set())
        self.commands: list[list[str]] = []
        self.kwargs: list[dict] = []

    def __call__(self, cmd, **kwargs):
        command = list(cmd)
        self.commands.append(command)
        self.kwargs.append(dict(kwargs))

        if command[:3] == ["systemctl", "is-active", "--quiet"]:
            return _RunResult(returncode=0 if command[3] in self.active else 3)

        if command[:5] == ["systemctl", "show", "-p", "LoadState", "--value"]:
            unit = command[5]
            state = "loaded" if unit in self.loaded else "not-found"
            return _RunResult(returncode=0, stdout=state)

        if command[:3] == ["sudo", "systemctl", "start"]:
            unit = command[3]
            if unit not in self.loaded:
                raise subprocess.CalledProcessError(
                    5,
                    command,
                    stderr=b"Unit not found",
                )
            self.active.add(unit)
            return _RunResult()

        if command[:3] == ["sudo", "systemctl", "stop"]:
            self.active.discard(command[3])
            return _RunResult()

        raise AssertionError(f"unexpected command: {command!r}")


def test_logical_status_detects_robot_service_aliases(monkeypatch):
    from runtime.service_manager import ServiceManager

    fake = _FakeSystemctl(
        active={
            "robot-lidar.service",
            "robot-fastlio2.service",
            "robot-localizer.service",
            "robot-genz-icp.service",
            "robot-hba.service",
        }
    )
    monkeypatch.setattr(subprocess, "run", fake)

    svc = ServiceManager()

    assert svc.status(
        "lidar",
        "slam",
        "slam_pgo",
        "localizer",
        "genz_icp",
        "hba",
    ) == {
        "lidar": "running",
        "slam": "running",
        "slam_pgo": "stopped",
        "localizer": "running",
        "genz_icp": "running",
        "hba": "running",
    }


def test_logical_status_keeps_legacy_service_fallback(monkeypatch):
    from runtime.service_manager import ServiceManager

    fake = _FakeSystemctl(active={"localization.service", "localizer.service"})
    monkeypatch.setattr(subprocess, "run", fake)

    svc = ServiceManager()

    assert svc.status("slam", "localizer") == {
        "slam": "running",
        "localizer": "running",
    }


def test_canonical_robot_unit_wins_over_active_legacy_alias(monkeypatch):
    from runtime.service_manager import ServiceManager

    fake = _FakeSystemctl(
        active={"camera.service"},
        loaded={"lingtu-camera-dds.service", "camera.service"},
    )
    monkeypatch.setattr(subprocess, "run", fake)

    svc = ServiceManager()

    assert svc.status("camera") == {"camera": "stopped"}
    assert svc.start("camera") == ["camera"]
    assert "lingtu-camera-dds.service" in fake.active
    assert ["sudo", "systemctl", "start", "lingtu-camera-dds.service"] in fake.commands
    assert ["sudo", "systemctl", "start", "camera.service"] not in fake.commands


def test_managed_product_rejects_direct_service_mutation(monkeypatch):
    from runtime.service_manager import ServiceManager

    fake = _FakeSystemctl(
        active={"lingtu-slam-dds.service"},
        loaded={"lingtu-slam-dds.service"},
    )
    monkeypatch.setattr(subprocess, "run", fake)
    svc = ServiceManager(
        environment={
            "LINGTU_PRODUCT": "nav",
            "LINGTU_RUN_PLAN_FINGERPRINT": "a" * 64,
        }
    )

    assert svc.status("slam") == {"slam": "running"}
    with pytest.raises(RuntimeError, match="use ProductControl"):
        svc.start("slam")
    with pytest.raises(RuntimeError, match="use ProductControl"):
        svc.stop("slam")

    assert not any(command[:3] == ["sudo", "systemctl", "start"] for command in fake.commands)
    assert not any(command[:3] == ["sudo", "systemctl", "stop"] for command in fake.commands)


@pytest.mark.parametrize(
    "environment",
    [
        {"LINGTU_PRODUCT": "nav"},
        {"LINGTU_PRODUCT_SESSION_ID": "session-1"},
        {"LINGTU_RUN_PLAN": "/run/lingtu/plan.json"},
        {"LINGTU_RUN_PLAN_FINGERPRINT": "a" * 64},
        {"LINGTU_SYSTEMD_UNIT": "lingtu.service"},
    ],
)
def test_partial_managed_identity_fails_closed(environment):
    from runtime.service_manager import ServiceManager

    svc = ServiceManager(environment=environment)

    with pytest.raises(RuntimeError, match="use ProductControl"):
        svc.assert_local_mutation("start services")


def test_status_details_reports_concrete_units_and_native_precedence(monkeypatch):
    from runtime.service_manager import ServiceManager

    fake = _FakeSystemctl(
        active={"robot-fastlio2.service"},
        loaded={"lingtu-slam-dds.service", "robot-fastlio2.service"},
    )
    monkeypatch.setattr(subprocess, "run", fake)

    svc = ServiceManager()
    details = svc.status_details("slam")["slam"]

    assert details["status"] == "stopped"
    assert details["canonical_unit"] == "lingtu-slam-dds.service"
    assert details["selected_unit"] == "robot-fastlio2.service"
    assert details["installed_units"] == [
        "lingtu-slam-dds.service",
        "robot-fastlio2.service",
    ]
    assert details["active_units"] == ["robot-fastlio2.service"]


def test_start_slam_does_not_fallback_to_legacy_unit(monkeypatch):
    from runtime.service_manager import ServiceManager

    fake = _FakeSystemctl(loaded={"robot-fastlio2.service", "localization.service"})
    monkeypatch.setattr(subprocess, "run", fake)

    svc = ServiceManager()

    assert svc.start("slam") == []
    assert "robot-fastlio2.service" not in fake.active
    assert ["sudo", "systemctl", "start", "lingtu-slam-dds.service"] in fake.commands
    assert svc._started == []


def test_start_legacy_slam_uses_explicit_compat_alias(monkeypatch):
    from runtime.service_manager import ServiceManager

    fake = _FakeSystemctl(loaded={"robot-fastlio2.service", "localization.service"})
    monkeypatch.setattr(subprocess, "run", fake)

    svc = ServiceManager()

    assert svc.start("legacy_slam") == ["legacy_slam"]
    assert "robot-fastlio2.service" in fake.active
    assert ["sudo", "systemctl", "start", "robot-fastlio2.service"] in fake.commands
    assert svc._started == ["robot-fastlio2.service"]


def test_start_slam_prefers_native_dds_when_installed(monkeypatch):
    from runtime.service_manager import ServiceManager

    fake = _FakeSystemctl(
        loaded={
            "lingtu-slam-dds.service",
            "robot-fastlio2.service",
            "localization.service",
        }
    )
    monkeypatch.setattr(subprocess, "run", fake)

    svc = ServiceManager()

    assert svc.start("slam") == ["slam"]
    assert "lingtu-slam-dds.service" in fake.active
    assert ["sudo", "systemctl", "start", "lingtu-slam-dds.service"] in fake.commands
    assert ["sudo", "systemctl", "start", "robot-fastlio2.service"] not in fake.commands
    assert svc._started == ["lingtu-slam-dds.service"]


def test_start_lidar_prefers_native_dds_when_installed(monkeypatch):
    from runtime.service_manager import ServiceManager

    fake = _FakeSystemctl(
        loaded={
            "lingtu-livox-dds.service",
            "robot-lidar.service",
            "lidar.service",
        }
    )
    monkeypatch.setattr(subprocess, "run", fake)

    svc = ServiceManager()

    assert svc.start("lidar") == ["lidar"]
    assert "lingtu-livox-dds.service" in fake.active
    assert ["sudo", "systemctl", "start", "lingtu-livox-dds.service"] in fake.commands
    assert ["sudo", "systemctl", "start", "robot-lidar.service"] not in fake.commands
    assert svc._started == ["lingtu-livox-dds.service"]


def test_start_legacy_lidar_uses_explicit_compat_alias(monkeypatch):
    from runtime.service_manager import ServiceManager

    fake = _FakeSystemctl(
        loaded={
            "robot-lidar.service",
            "lidar.service",
        }
    )
    monkeypatch.setattr(subprocess, "run", fake)

    svc = ServiceManager()

    assert svc.start("legacy_lidar") == ["legacy_lidar"]
    assert "robot-lidar.service" in fake.active
    assert ["sudo", "systemctl", "start", "robot-lidar.service"] in fake.commands
    assert ["sudo", "systemctl", "start", "lidar.service"] not in fake.commands
    assert svc._started == ["robot-lidar.service"]


def test_untracked_start_does_not_release_external_service(monkeypatch):
    from runtime.service_manager import ServiceManager

    fake = _FakeSystemctl(loaded={"robot-fastlio2.service", "localization.service"})
    monkeypatch.setattr(subprocess, "run", fake)

    svc = ServiceManager()

    assert svc.start("legacy_slam", track_started=False) == ["legacy_slam"]
    assert "robot-fastlio2.service" in fake.active
    assert svc._started == []

    svc.stop_all_started()

    assert "robot-fastlio2.service" in fake.active
    assert ["sudo", "systemctl", "stop", "robot-fastlio2.service"] not in fake.commands


def test_optional_super_lio_integration_has_no_service_manager_contract():
    import runtime.service_manager as service_manager

    integration_names = {"super_lio", "super_lio_relocation"}

    assert integration_names.isdisjoint(service_manager.SERVICE_ALIASES)
    assert integration_names.isdisjoint(service_manager.SERVICE_START_ALIASES)
    assert not hasattr(service_manager, "SERVICES_SUPER_LIO")
    assert not hasattr(service_manager, "SERVICES_SUPER_LIO_RELOCATION")


def test_start_genz_icp_and_hba_prefer_robot_units_when_installed(monkeypatch):
    from runtime.service_manager import ServiceManager

    fake = _FakeSystemctl(
        loaded={
            "robot-genz-icp.service",
            "genz_icp.service",
            "robot-hba.service",
            "hba.service",
        }
    )
    monkeypatch.setattr(subprocess, "run", fake)

    svc = ServiceManager()

    assert svc.start("genz_icp", "hba") == ["genz_icp", "hba"]
    assert "robot-genz-icp.service" in fake.active
    assert "robot-hba.service" in fake.active
    assert ["sudo", "systemctl", "start", "robot-genz-icp.service"] in fake.commands
    assert ["sudo", "systemctl", "start", "robot-hba.service"] in fake.commands
    assert ["sudo", "systemctl", "start", "genz_icp.service"] not in fake.commands
    assert ["sudo", "systemctl", "start", "hba.service"] not in fake.commands
    assert svc._started == ["robot-genz-icp.service", "robot-hba.service"]


def test_stop_clears_new_and_legacy_aliases(monkeypatch):
    from runtime.service_manager import ServiceManager

    fake = _FakeSystemctl(
        active={
            "lingtu-slam-dds.service",
            "robot-fastlio2.service",
            "localization.service",
            "robot-localizer.service",
            "localizer.service",
            "robot-genz-icp.service",
            "genz_icp.service",
            "genz-icp.service",
            "robot-hba.service",
            "hba.service",
        }
    )
    monkeypatch.setattr(subprocess, "run", fake)

    svc = ServiceManager()
    svc._started = [
        "lingtu-slam-dds.service",
        "robot-fastlio2.service",
        "localization.service",
    ]

    svc.stop(
        "slam",
        "localizer",
        "genz_icp",
        "hba",
    )

    assert "lingtu-slam-dds.service" not in fake.active
    assert "robot-fastlio2.service" not in fake.active
    assert "localization.service" not in fake.active
    assert "robot-localizer.service" not in fake.active
    assert "localizer.service" not in fake.active
    assert "robot-genz-icp.service" not in fake.active
    assert "genz_icp.service" not in fake.active
    assert "genz-icp.service" not in fake.active
    assert "robot-hba.service" not in fake.active
    assert "hba.service" not in fake.active
    assert ["sudo", "systemctl", "stop", "lingtu-slam-dds.service"] in fake.commands
    assert ["sudo", "systemctl", "stop", "robot-fastlio2.service"] in fake.commands
    assert ["sudo", "systemctl", "stop", "localization.service"] in fake.commands
    assert ["sudo", "systemctl", "stop", "robot-genz-icp.service"] in fake.commands
    assert ["sudo", "systemctl", "stop", "genz-icp.service"] in fake.commands
    assert ["sudo", "systemctl", "stop", "robot-hba.service"] in fake.commands
    assert svc._started == []


def test_text_systemctl_calls_use_stable_utf8_decode(monkeypatch):
    from runtime.service_manager import ServiceManager

    fake = _FakeSystemctl(loaded={"robot-camera.service"})
    monkeypatch.setattr(subprocess, "run", fake)

    svc = ServiceManager()

    assert svc._unit_exists("robot-camera.service") is True
    show_index = fake.commands.index(["systemctl", "show", "-p", "LoadState", "--value", "robot-camera.service"])
    assert fake.kwargs[show_index]["encoding"] == "utf-8"
    assert fake.kwargs[show_index]["errors"] == "replace"


def test_start_failure_logs_invalid_utf8_stderr_without_crashing(monkeypatch, caplog):
    from runtime.service_manager import ServiceManager

    class FakeSystemctl(_FakeSystemctl):
        def __call__(self, cmd, **kwargs):
            command = list(cmd)
            if command[:3] == ["sudo", "systemctl", "start"]:
                self.commands.append(command)
                self.kwargs.append(dict(kwargs))
                raise subprocess.CalledProcessError(
                    5,
                    command,
                    stderr=b"\xff\xfeunit start failed",
                )
            return super().__call__(cmd, **kwargs)

    fake = FakeSystemctl(loaded={"robot-camera.service"})
    monkeypatch.setattr(subprocess, "run", fake)

    svc = ServiceManager()

    with caplog.at_level("ERROR"):
        assert svc.start("camera") == []
    assert "Failed to start camera" in caplog.text
