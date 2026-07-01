from pathlib import Path


def test_nav_planning_has_no_legacy_ros_package_entrypoints() -> None:
    repo = Path(__file__).resolve().parents[3]
    pct_root = (
        repo
        / "src"
        / "nav"
        / "services"
        / "plan"
        / "global_planner"
        / "algorithm"
        / "pct"
        / "vendor"
        / "pct_planner"
    )
    adapters_root = (
        repo
        / "src"
        / "nav"
        / "services"
        / "plan"
        / "global_planner"
        / "backends"
        / "pct_adapters"
    )

    forbidden = {
        pct_root: [
            "CMakeLists.txt",
            "package.xml",
            "launch",
            "planner/scripts/global_planner.py",
            "planner/scripts/pct_planner_astar.py",
            "planner/scripts/fake_localization.py",
            "planner/scripts/utils/vis_ros.py",
        ],
        adapters_root: [
            "CMakeLists.txt",
            "package.xml",
            "pct_path_adapter.cpp",
            "config/pct_path_adapter.yaml",
        ],
    }

    assert [
        str(root / rel)
        for root, rels in forbidden.items()
        for rel in rels
        if (root / rel).exists()
    ] == []


def test_global_planning_does_not_need_ros_compat_boundary() -> None:
    from runtime.adapters.ros2.manifest import (
        ROS_COMPAT_IMPORT_BOUNDARIES,
        ROS_SCAN_EXCLUDED_PREFIXES,
    )

    assert all(
        not boundary.prefix.startswith("nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/")
        for boundary in ROS_COMPAT_IMPORT_BOUNDARIES
    )
    assert all(
        not prefix.startswith("nav/services/plan/global_planner/algorithm/pct/vendor/pct_planner/launch/")
        for prefix in ROS_SCAN_EXCLUDED_PREFIXES
    )
