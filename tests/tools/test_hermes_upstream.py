from tools.upstream.hermes import build_report, compare_trees


def test_report_tracks_renames_and_marks_incomplete_results_unknown():
    manifest = {"baseline": {"release": "v1", "commit": "abc"}, "features": {
        "memory": {"upstream": ["tools/memory"], "local": ["local.py"], "tests": ["test_local.py"],
                   "status": "tested", "differences": "existing owner"},
        "context": {"upstream": ["agent/context"], "local": [], "tests": [], "status": "pending", "differences": "none"}}}
    comparison = {"html_url": "https://github.com/example/compare", "total_commits": 2, "commits": [{}],
                  "files": [{"filename": "tools/new_memory.py", "previous_filename": "tools/memory.py", "status": "renamed"}]}
    text = build_report(manifest, comparison, "v2")
    assert "memory: review required" in text
    assert "context: unknown (partial comparison)" in text
    assert "existing owner" in text and "test_local.py" in text
    comparison["total_commits"] = 1
    assert "context: no mapped file changes" in build_report(manifest, comparison, "v2")


def test_full_tree_comparison_tracks_add_remove_content_and_mode_changes():
    before = {"tree": [{"path": name, "sha": "same", "mode": "100644", "type": "blob"}
                       for name in ("unchanged", "removed", "modified", "executable")]}
    after = {"tree": [{"path": name, "sha": "different" if name == "modified" else "same",
                       "mode": "100755" if name == "executable" else "100644", "type": "blob"}
                      for name in ("unchanged", "added", "modified", "executable")]}
    assert compare_trees(before, after) == [
        {"filename": "added", "status": "added"}, {"filename": "executable", "status": "modified"},
        {"filename": "modified", "status": "modified"}, {"filename": "removed", "status": "removed"}]
    assert compare_trees(before, {**after, "truncated": True}) is None
