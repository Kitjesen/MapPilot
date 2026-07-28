# ruff: noqa: S101

from __future__ import annotations

import json

from lingtu.control import ProductControl
from lingtu.control import main as control_main
from lingtu.launcher import LaunchReport
from lingtu.product import ProductManifest


class FakeLauncher:
    def __init__(self) -> None:
        self.calls = []

    def restart(self, product, process_name: str, *, dry_run: bool = False):
        self.calls.append((product, process_name, dry_run))
        return LaunchReport(
            product=product.profile,
            endpoint=product.endpoint,
            action="restart",
            ok=True,
            status="active",
        )

    def apply(self, product, *, dry_run: bool = False):
        self.calls.append((product, "apply", dry_run))
        return LaunchReport(
            product=product.profile,
            endpoint=product.endpoint,
            action="apply",
            ok=True,
            status="active",
        )

    def stop(self, product, *, dry_run: bool = False):
        self.calls.append((product, "stop", dry_run))
        return LaunchReport(
            product=product.profile,
            endpoint=product.endpoint,
            action="stop",
            ok=True,
            status="stopped",
        )

    def quiesce(self, product, *, dry_run: bool = False):
        self.calls.append((product, "quiesce", dry_run))
        return LaunchReport(
            product=product.profile,
            endpoint=product.endpoint,
            action="quiesce",
            ok=True,
            status="stopped",
        )


def test_product_control_compiles_environment_once_per_operation() -> None:
    launcher = FakeLauncher()
    control = ProductControl(
        launcher,  # type: ignore[arg-type]
        environment={
            "LINGTU_PROFILE": "nav",
            "LINGTU_ENDPOINT": "thunder_field",
        },
    )

    report = control.restart("slam")

    product, process_name, dry_run = launcher.calls[0]
    assert product.profile == "nav"
    assert product.endpoint == "thunder_field"
    assert process_name == "slam"
    assert dry_run is False
    assert report.ok is True


def test_product_control_requires_explicit_active_profile() -> None:
    control = ProductControl(FakeLauncher(), environment={})  # type: ignore[arg-type]

    try:
        control.restart("slam")
    except RuntimeError as exc:
        assert str(exc) == "LINGTU_PROFILE is required for product process control"
    else:
        raise AssertionError("missing active profile must fail closed")


def test_product_control_reuses_one_compiled_product() -> None:
    control = ProductControl(
        FakeLauncher(),  # type: ignore[arg-type]
        environment={
            "LINGTU_PROFILE": "nav",
            "LINGTU_ENDPOINT": "thunder_field",
        },
    )

    assert control.product() is control.product()


def test_product_control_does_not_own_compatibility_service_operations() -> None:
    legacy_methods = {name for name in dir(ProductControl) if name.startswith("legacy_")}

    assert legacy_methods == set()


def test_product_control_applies_persisted_manifest_without_recompile(tmp_path) -> None:
    launcher = FakeLauncher()
    compiler = ProductControl(
        launcher,  # type: ignore[arg-type]
        environment={
            "LINGTU_PROFILE": "nav",
            "LINGTU_ENDPOINT": "thunder_field",
        },
    )
    manifest = tmp_path / "product.json"
    compiler.write_manifest(manifest)
    runner = ProductControl(launcher, environment={})  # type: ignore[arg-type]

    report = runner.apply_manifest(manifest)

    product, action, dry_run = launcher.calls[-1]
    assert isinstance(product, ProductManifest)
    assert product.fingerprint
    assert action == "apply"
    assert dry_run is False
    assert report.ok is True


def test_product_control_rejects_direct_apply_for_process_only_field_product() -> None:
    launcher = FakeLauncher()
    control = ProductControl(launcher, environment={})  # type: ignore[arg-type]
    product = control.product("map", endpoint="thunder_field")

    try:
        control.apply_product(product)
    except RuntimeError as exc:
        assert "requires ProductControl.switch" in str(exc)
    else:
        raise AssertionError("process-only field Product must not bypass the switch transaction")

    assert launcher.calls == []



def test_product_control_rejects_direct_apply_for_field_host_product() -> None:
    launcher = FakeLauncher()
    control = ProductControl(launcher, environment={})  # type: ignore[arg-type]
    product = control.product("nav", endpoint="thunder_field")

    try:
        control.apply_product(product)
    except RuntimeError as exc:
        assert "requires ProductControl.switch" in str(exc)
    else:
        raise AssertionError("field Product must not bypass the switch transaction")

    assert launcher.calls == []

def test_product_control_cli_plans_switch_without_field_side_effects(capsys) -> None:
    exit_code = control_main(
        [
            "switch",
            "teleop",
            "--current",
            "teleop",
            "--endpoint",
            "thunder_field",
            "--dry-run",
            "--json",
        ]
    )

    payload = json.loads(capsys.readouterr().out)
    assert exit_code == 0
    assert payload["status"] == "planned"
    assert payload["target_profile"] == "teleop"
    assert payload["phases"] == ["preflight"]
