from lingtu.assembly.deployment import product_native_build_scripts


def test_product_build_plan_does_not_resolve_an_environment(monkeypatch) -> None:
    def fail_host_compile(*_args, **_kwargs):
        raise AssertionError("build planning must not resolve Product + env")

    monkeypatch.setattr(
        "lingtu.assembly.products.configuration.resolve_product_host_runtime",
        fail_host_compile,
    )

    scripts = product_native_build_scripts("teleop_avoid")

    assert "scripts/build/build_livox_sdk2_stream.sh" in scripts
    assert "scripts/build/build_camera_dds.sh" not in scripts
    assert scripts[-1] == "scripts/build/build_native_recording.sh"
