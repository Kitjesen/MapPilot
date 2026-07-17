import importlib
import sys
from pathlib import Path
from urllib.parse import urlparse
from xml.etree import ElementTree

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
PACKAGE_ROOT = REPO_ROOT / "integrations" / "open_rmf" / "ros_ws" / "src" / "lingtu_rmf_adapter"


def _adapter_module():
    package_path = str(PACKAGE_ROOT)
    if package_path not in sys.path:
        sys.path.insert(0, package_path)
    return importlib.import_module("lingtu_rmf_adapter.adapter")


def test_open_rmf_package_manifest_and_entrypoint_exist() -> None:
    root = ElementTree.parse(PACKAGE_ROOT / "package.xml").getroot()  # noqa: S314
    setup_text = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")

    assert root.findtext("name") == "lingtu_rmf_adapter"
    assert "rmf_fleet_adapter_python" in {item.text for item in root.findall("exec_depend")}
    assert "lingtu_rmf_adapter.adapter:main" in setup_text


def test_open_rmf_shadow_config_disables_dispatch_and_commands(
    monkeypatch,
) -> None:
    monkeypatch.delenv("LINGTU_RMF_COMMANDS_ENABLED", raising=False)
    monkeypatch.setenv("LINGTU_RMF_API_KEY", "scoped-secret")
    adapter = _adapter_module()

    settings = adapter.load_bridge_settings(PACKAGE_ROOT / "config" / "lingtu_bridge.yaml")

    assert settings.gateway.commands_enabled is False
    assert "scoped-secret" not in repr(settings.gateway)
    assert adapter._fleet_dispatch_enabled(PACKAGE_ROOT / "config" / "lingtu_fleet_shadow.yaml") is False
    assert adapter._fleet_dispatch_enabled(PACKAGE_ROOT / "config" / "lingtu_fleet_single_robot.yaml") is True
    with pytest.raises(RuntimeError, match="shadow mode"):
        adapter.validate_dispatch_mode(
            settings,
            PACKAGE_ROOT / "config" / "lingtu_fleet_single_robot.yaml",
        )


def test_open_rmf_live_config_requires_scoped_key_and_floor_transforms(
    monkeypatch,
) -> None:
    adapter = _adapter_module()
    monkeypatch.setenv("LINGTU_RMF_COMMANDS_ENABLED", "1")
    monkeypatch.delenv("LINGTU_RMF_API_KEY", raising=False)

    with pytest.raises(ValueError, match="scoped API key"):
        adapter.load_bridge_settings(PACKAGE_ROOT / "config" / "lingtu_bridge.yaml")

    monkeypatch.setenv("LINGTU_RMF_API_KEY", "rmf-secret")
    settings = adapter.load_bridge_settings(PACKAGE_ROOT / "config" / "lingtu_bridge.yaml")

    assert settings.gateway.commands_enabled is True
    with pytest.raises(RuntimeError, match="coordinate transform"):
        adapter.validate_dispatch_mode(
            settings,
            PACKAGE_ROOT / "config" / "lingtu_fleet_single_robot.yaml",
        )


def test_open_rmf_live_config_accepts_explicit_per_floor_transforms(
    monkeypatch,
    tmp_path,
) -> None:
    adapter = _adapter_module()
    monkeypatch.setenv("LINGTU_RMF_COMMANDS_ENABLED", "1")
    monkeypatch.setenv("LINGTU_RMF_API_KEY", "rmf-secret")
    settings = adapter.load_bridge_settings(PACKAGE_ROOT / "config" / "lingtu_bridge.yaml")
    fleet_config = tmp_path / "fleet.yaml"
    fleet_config.write_text(
        """rmf_fleet:
  task_capabilities:
    loop: true
  transforms:
    L1:
      translation: [0.0, 0.0]
      rotation: 0.0
      scale: 1.0
    L2:
      translation: [12.5, -4.0]
      rotation: 1.57079632679
      scale: 1.0
""",
        encoding="utf-8",
    )

    adapter.validate_dispatch_mode(settings, fleet_config)


def test_open_rmf_live_config_rejects_invalid_coordinate_transform(
    monkeypatch,
    tmp_path,
) -> None:
    adapter = _adapter_module()
    monkeypatch.setenv("LINGTU_RMF_COMMANDS_ENABLED", "1")
    monkeypatch.setenv("LINGTU_RMF_API_KEY", "rmf-secret")
    settings = adapter.load_bridge_settings(PACKAGE_ROOT / "config" / "lingtu_bridge.yaml")
    fleet_config = tmp_path / "fleet.yaml"
    fleet_config.write_text(
        """rmf_fleet:
  task_capabilities:
    loop: true
  transforms:
    L1:
      translation: [0.0, 0.0]
      rotation: 0.0
      scale: 0.0
    L2:
      translation: [0.0, 0.0]
      rotation: 0.0
      scale: 1.0
""",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="scale"):
        adapter.validate_dispatch_mode(settings, fleet_config)


def test_open_rmf_bridge_sample_uses_safe_gateway_transport() -> None:
    adapter = _adapter_module()
    settings = adapter.load_bridge_settings(PACKAGE_ROOT / "config" / "lingtu_bridge.yaml")
    parsed = urlparse(settings.gateway.base_url)

    assert parsed.scheme == "https" or parsed.hostname in {
        "localhost",
        "127.0.0.1",
        "::1",
    }
    assert settings.gateway.allow_insecure_http is False


def test_open_rmf_sidecar_has_no_direct_velocity_route() -> None:
    source = (PACKAGE_ROOT / "lingtu_rmf_adapter" / "adapter.py").read_text(encoding="utf-8")

    assert "/api/v1/cmd_vel" not in source


def test_open_rmf_installer_pins_and_verifies_ros_apt_source() -> None:
    installer = (REPO_ROOT / "integrations" / "open_rmf" / "scripts" / "install_jazzy_open_rmf.sh").read_text(
        encoding="utf-8"
    )

    assert "releases/latest" not in installer
    assert "ROS_APT_SOURCE_VERSION:-1.2.0" in installer
    assert "0804d9b13db770eb87019be414cd78378835228ad5fa801fc88758596dd8f7e5" in installer
    assert "sha256sum -c" in installer
    assert "LINGTU_ENABLE_EXTERNAL_ROS2_SIDECAR" in installer
