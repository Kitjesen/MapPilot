from pathlib import Path

import pytest

from decision.tasks import skills


def write_skill(root, name="example", body="Run the existing read-only tool."):
    directory = root / name
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "SKILL.md").write_text(f"---\nname: Example\ndescription: A useful task guide.\n---\n{body}", encoding="utf-8")
    return directory


@pytest.fixture
def roots(tmp_path, monkeypatch):
    roots = {"builtin": tmp_path / "builtin", "local": tmp_path / "local"}
    monkeypatch.setattr(skills, "skill_roots", lambda: roots)
    return roots


def test_list_discloses_metadata_only_and_keeps_local_identity(roots):
    for root in roots.values():
        write_skill(root, body="BODY_ONLY_MARKER")
    result = skills.execute_skill_tool("skills_list", {})
    assert {item["name"] for item in result["skills"]} == {"builtin/example", "local/example"}
    assert "BODY_ONLY_MARKER" not in str(result)
    assert not result["errors"]


def test_view_and_reference_read_current_files(roots):
    directory = write_skill(roots["local"])
    (directory / "references").mkdir()
    reference = directory / "references" / "check.md"
    reference.write_text("First", encoding="utf-8")
    result = skills.execute_skill_tool("skill_view", {"name": "local/example"})
    assert "Run the existing" in result["content"]
    args = {"name": "local/example", "file_path": "references/check.md"}
    assert skills.execute_skill_tool("skill_view", args)["content"] == "First"
    reference.write_text("Updated", encoding="utf-8")
    assert skills.execute_skill_tool("skill_view", args)["content"] == "Updated"


@pytest.mark.parametrize("reference", ["../outside.md", "../../outside.md", "/outside.md", "scripts/run.py"])
def test_references_cannot_leave_the_selected_skill_or_execute_code(roots, reference):
    write_skill(roots["local"])
    (roots["local"] / "outside.md").write_text("private", encoding="utf-8")
    result = skills.execute_skill_tool("skill_view", {"name": "local/example", "file_path": reference})
    assert "error" in result and "content" not in result


def test_bad_metadata_and_oversize_are_reported_without_partial_content(roots):
    invalid = write_skill(roots["local"], "invalid")
    (invalid / "SKILL.md").write_text("not a skill", encoding="utf-8")
    write_skill(roots["local"], "large", "a" * 65537)
    result = skills.execute_skill_tool("skills_list", {})
    assert not result["skills"] and len(result["errors"]) == 2
    assert "error" in skills.execute_skill_tool("skill_view", {"name": "local/large"})
    assert "error" in skills.execute_skill_tool("skill_view", {"name": "../../codex.json"})


def test_bundled_guides_load_without_network(monkeypatch, tmp_path):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    result = skills.execute_skill_tool("skills_list", {})
    assert not result["errors"]
    names = {item["name"] for item in result["skills"]}
    assert {"builtin/model-connections", "builtin/map-review"} <= names
    content = skills.execute_skill_tool("skill_view", {"name": "builtin/model-connections", "file_path": "references/verification.md"})
    assert "Inference" in content["content"]
    assert skills.skill_roots()["local"] == Path(tmp_path) / "lingtu" / "skills"
