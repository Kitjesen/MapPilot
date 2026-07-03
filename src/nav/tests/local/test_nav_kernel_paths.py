from pathlib import Path

from nav.kernel import nav_kernel_build_hint, nav_kernel_candidate_dirs


def test_nav_kernel_candidate_dirs_preserve_legacy_order(tmp_path: Path) -> None:
    repo = tmp_path / "repo"
    (repo / "src").mkdir(parents=True)
    (repo / "lingtu.py").write_text("", encoding="utf-8")
    anchor = repo / "src" / "nav" / "kernel" / "runtime.py"
    anchor.parent.mkdir(parents=True)
    anchor.write_text("", encoding="utf-8")

    assert nav_kernel_candidate_dirs(anchor) == [
        str(repo / "src" / "nav" / "kernel" / "build_nb_win"),
        str(repo / "src" / "nav" / "kernel" / "build_nb"),
        str(repo / "src"),
        str(repo / "install" / "nav_kernel" / "lib"),
    ]


def test_nav_kernel_repo_root_ignores_kernel_source_dir(tmp_path: Path) -> None:
    repo = tmp_path / "repo"
    (repo / "src" / "nav" / "kernel" / "src").mkdir(parents=True)
    (repo / "pyproject.toml").write_text("", encoding="utf-8")
    (repo / "AGENTS.md").write_text("", encoding="utf-8")
    anchor = repo / "src" / "nav" / "kernel" / "paths.py"
    anchor.write_text("", encoding="utf-8")

    assert nav_kernel_candidate_dirs(anchor)[0] == str(
        repo / "src" / "nav" / "kernel" / "build_nb_win"
    )


def test_nav_kernel_build_hint_points_to_existing_build_script() -> None:
    hint = nav_kernel_build_hint()

    assert "scripts/build/build_nav_kernel.sh" in hint
    assert "src/nav/kernel/build_nb_win" in hint
