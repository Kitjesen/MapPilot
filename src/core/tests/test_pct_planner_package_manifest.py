from pathlib import Path


def test_pct_planner_cmake_installs_existing_package_scripts() -> None:
    repo = Path(__file__).resolve().parents[3]
    package_dir = repo / "src" / "global_planning" / "pct_planner"
    cmake = (package_dir / "CMakeLists.txt").read_text(encoding="utf-8")

    expected_scripts = [
        "planner/scripts/global_planner.py",
        "planner/scripts/pct_planner_astar.py",
        "planner/scripts/fake_localization.py",
    ]
    for script in expected_scripts:
        assert script in cmake
        assert (package_dir / script).is_file()

    assert "../../../src/legacy/pct_planner" not in cmake
