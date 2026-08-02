from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

INTEGRATION = ROOT / "integrations" / "super_lio"
SUPER_LIO_NAMES = ("super_lio", "super_lio_relocation", "super-lio")
SUPER_LIO_CONFLICT_UNITS = {
    "robot-super-lio.service",
    "robot-super-lio-relocation.service",
    "super_lio.service",
    "super_lio_relocation.service",
}


def _read(rel_path: str) -> str:
    return (ROOT / rel_path).read_text(encoding="utf-8-sig")


def _section(text: str, start: str, end: str) -> str:
    return text.split(start, 1)[1].split(end, 1)[0]


def test_super_lio_experiment_is_preserved_outside_the_product_runtime() -> None:
    readme = INTEGRATION / "README.md"
    mapping_unit = INTEGRATION / "systemd" / "super_lio.service"
    relocation_unit = INTEGRATION / "systemd" / "super_lio_relocation.service"

    assert readme.is_file()
    assert mapping_unit.is_file()
    assert relocation_unit.is_file()

    guidance = readme.read_text(encoding="utf-8").lower()
    for phrase in (
        "experimental external integration",
        "not a product",
        "not a profile",
        "not installed by default",
        "native release package does not include them",
        "do not run these units in parallel",
        "do not restore `slamcheck`",
    ):
        assert phrase in guidance
    assert "scripts/lingtu --env real svc stop all" in guidance
    assert "scripts/lingtu stop" not in guidance


def test_super_lio_experiment_is_not_a_default_field_control_plane() -> None:
    product_names = {
        path.stem for path in (ROOT / "config" / "runtime_graph" / "products").glob("*.yaml")
    }
    assert "super_lio" not in product_names
    assert "super_lio_relocation" not in product_names

    real_env = _read("config/runtime_graph/envs/real.yaml").lower()
    processes = _section(real_env, "processes:", "conflicts:")
    assert "super_lio" not in processes
    assert "super-lio" not in processes

    conflicts = _section(real_env, "conflicts:", "endpoints:")
    conflict_units = {
        line.removeprefix("- ").strip()
        for raw_line in conflicts.splitlines()
        if (line := raw_line.strip()).startswith("- ")
    }
    assert SUPER_LIO_CONFLICT_UNITS <= conflict_units
    real_env_without_conflicts = real_env.replace(conflicts, "")
    assert "super_lio" not in real_env_without_conflicts
    assert "super-lio" not in real_env_without_conflicts

    from runtime.service_catalogs import thunder

    catalog_text = _read("src/runtime/service_catalogs/thunder.py").lower()
    assert "super_lio" not in catalog_text
    assert "super-lio" not in catalog_text
    for mode in ("field", "nav", "thunder-nav", "field-cpp", "dds-cpp"):
        plan = "\n".join(thunder.thunder_install_plan(mode)).lower()
        services = "\n".join(thunder.thunder_install_services(mode)).lower()
        assert "super_lio" not in plan
        assert "super-lio" not in plan
        assert "super_lio" not in services
        assert "super-lio" not in services

    assert not (ROOT / "scripts/deploy/s100p/super_lio.service").exists()
    assert not (ROOT / "scripts/deploy/s100p/super_lio_relocation.service").exists()


def test_super_lio_conflict_tombstones_reach_the_run_plan_stop_plan() -> None:
    from lingtu.assembly.products import resolve_product_host_runtime
    from lingtu.assembly.profile_builder import compile_run_plan

    resolved = resolve_product_host_runtime("nav", "real")
    plan = compile_run_plan(
        resolved.product,
        resolved.env,
        resolved.config,
    )

    assert SUPER_LIO_CONFLICT_UNITS <= set(plan.stop_targets)
    assert SUPER_LIO_CONFLICT_UNITS.isdisjoint(
        process.target for process in plan.available_processes
    )


def test_super_lio_units_keep_the_documented_ros_topic_adapter_contract() -> None:
    for filename in ("super_lio.service", "super_lio_relocation.service"):
        text = (INTEGRATION / "systemd" / filename).read_text(
            encoding="utf-8-sig"
        )
        assert "SUPER_LIO_LIDAR_TOPIC=/lidar/raw_frame" in text
        assert "SUPER_LIO_IMU_TOPIC=/imu/raw" in text
        assert '-r "/lio/odom:=/slam/odometry"' in text
        assert '-r "/lio/cloud_world:=/slam/map_cloud"' in text


def test_super_lio_relocation_adapts_lingtu_map_path_to_upstream_relative_path() -> None:
    text = (INTEGRATION / "systemd" / "super_lio_relocation.service").read_text(
        encoding="utf-8-sig"
    )

    assert "Environment=SUPER_LIO_ROOT=" in text
    assert "Environment=SUPER_LIO_RELOCATION_SOURCE_MAP_DIR=" in text
    assert "Environment=SUPER_LIO_RELOCATION_EFFECTIVE_MAP_DIR=map/lingtu_active" in text
    assert 'case "$${EFFECTIVE_MAP_DIR}" in /*)' in text
    assert 'ln -sfn "$${SOURCE_MAP_DIR_ABS}" "$${EFFECTIVE_MAP_PATH}"' in text
    assert 'test -f "$${CONFIG}"' in text
    assert 'test -s "$${MAP_PATH}"' in text
    assert 'test -s "$${EFFECTIVE_MAP_PATH}/$${SUPER_LIO_RELOCATION_MAP_NAME}"' in text
    assert 'cd "$${SUPER_LIO_ROOT_ABS}"' in text
    assert '-p "lio.map.save_map_dir:=$${EFFECTIVE_MAP_DIR}"' in text
    assert '-p "lio.map.save_map_dir:=$${SUPER_LIO_RELOCATION_MAP_DIR}"' not in text


def test_super_lio_units_are_lab_only_and_not_systemd_install_targets() -> None:
    for unit in (INTEGRATION / "systemd").glob("super_lio*.service"):
        text = unit.read_text(encoding="utf-8-sig")
        assert "[Install]" not in text
        assert "WantedBy=" not in text
        assert "Install" not in text

