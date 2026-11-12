"""Report upstream changes by capability; never overwrite local implementation."""

import argparse
import json
from pathlib import Path
from urllib.parse import quote

import httpx
import yaml

MANIFEST = Path(__file__).resolve().parents[2] / "config/upstream/hermes.yaml"


def compare_trees(before, after):
    """Compare Git's existing object IDs; avoid downloading every upstream source file."""
    if before.get("truncated") or after.get("truncated"):
        return None
    old = {item["path"]: (item["sha"], item["mode"]) for item in before["tree"] if item["type"] != "tree"}
    new = {item["path"]: (item["sha"], item["mode"]) for item in after["tree"] if item["type"] != "tree"}
    return [{"filename": path, "status": "added" if path not in old else "removed" if path not in new else "modified"}
            for path in sorted(old.keys() | new.keys()) if old.get(path) != new.get(path)]


def build_report(manifest, comparison, target, release_notes=""):
    files = comparison.get("files", [])
    # GitHub compare lists at most 300 files, even if more changed.
    incomplete = not comparison.get("files_complete", len(files) < 300 and
        comparison.get("total_commits", 0) <= len(comparison.get("commits", [])))
    lines = ["# Hermes capability update review", "",
             f"Baseline: `{manifest['baseline']['release']}` / `{manifest['baseline']['commit']}`",
             f"Candidate: `{target}`", "", f"[Upstream comparison]({comparison['html_url']})", "",
             "Coverage: partial; unlisted files/capabilities are UNKNOWN. Use a local git diff for a complete review."
             if incomplete else "Coverage: complete changed-file inventory (compare API or full Git tree comparison).", "",
             "This report does not change the baseline, copy code, install dependencies or certify compatibility.", ""]
    assigned = set()
    for name, feature in manifest["features"].items():
        matches = [item for item in files if any(
            path.startswith(prefix) for prefix in feature["upstream"]
            for path in (item["filename"], item.get("previous_filename", "")))]
        assigned.update(item["filename"] for item in matches)
        state = "review required" if matches else "unknown (partial comparison)" if incomplete else "no mapped file changes"
        lines.extend([f"## {name}: {state}", "", f"Local status: `{feature['status']}`",
                      f"Local differences to preserve: {feature['differences']}", "",
                      "Local owners: " + ", ".join(f"`{path}`" for path in feature["local"]),
                      "Validation: `python -m pytest -q " + " ".join(feature["tests"]) + "`", ""])
        lines.extend(f"- `{item['status']}` `{item['filename']}`" for item in matches)
        lines.append("")
    unmapped = [item["filename"] for item in files if item["filename"] not in assigned]
    lines.extend(["## Unmapped changes / new capability candidates", "",
                  "Review release notes and these paths before deciding whether a new capability belongs in LingTu.", ""])
    lines.extend(f"- `{path}`" for path in unmapped[:100])
    if len(unmapped) > 100:
        lines.append(f"\nShowing 100 of {len(unmapped)} unmapped paths; the adjacent JSON contains the full inventory.")
    if release_notes:
        lines.extend(["", "## Upstream release notes (reference data)", "", release_notes])
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ref", help="Release tag or commit; defaults to latest official release")
    parser.add_argument("--output", type=Path, default=Path("artifacts/upstream/hermes-update.md"))
    args = parser.parse_args()
    manifest = yaml.safe_load(MANIFEST.read_text(encoding="utf-8"))
    base = f"https://api.github.com/repos/{manifest['repository']}"
    with httpx.Client(timeout=30, follow_redirects=True) as client:
        release_notes = ""
        target = args.ref
        if not target:
            response = client.get(base + "/releases/latest")
            response.raise_for_status()
            release = response.json()
            target, release_notes = release["tag_name"], release.get("body", "")
        response = client.get(base + "/commits/" + quote(target, safe=""))
        response.raise_for_status()
        resolved = response.json()["sha"]
        response = client.get(base + "/compare/" + quote(manifest["baseline"]["commit"], safe="") + "..." + resolved)
        response.raise_for_status()
        comparison = response.json()
        if len(comparison.get("files", [])) >= 300:
            trees = []
            for revision in (manifest["baseline"]["commit"], resolved):
                response = client.get(base + "/git/trees/" + revision, params={"recursive": "1"})
                response.raise_for_status()
                trees.append(response.json())
            files = compare_trees(*trees)
            if files is not None:
                comparison.update(files=files, files_complete=True, inventory_source="git_trees")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(build_report(manifest, comparison, f"{target} / {resolved}", release_notes), encoding="utf-8")
    args.output.with_suffix(".json").write_text(json.dumps(comparison, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"Read-only comparison saved: {args.output}")


if __name__ == "__main__":
    main()
