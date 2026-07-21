from __future__ import annotations

from lingtu.control import ProductControl
from lingtu.launcher import LaunchReport


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
