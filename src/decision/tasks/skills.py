"""Read task guides on demand; follows Hermes' skills_list / skill_view protocol.

Source comparison and limits: ../llm/SKILLS_NOTICE.md.
"""

import os
from pathlib import Path

import yaml

SKILL_TOOLS = [
    {"type": "function", "function": {
        "name": "skills_list", "description": "List installed task guides by name and description. Read a matching guide with skill_view before applying it.",
        "parameters": {"type": "object", "properties": {}, "additionalProperties": False},
    }},
    {"type": "function", "function": {
        "name": "skill_view", "description": "Read a listed SKILL.md or a Markdown/text reference inside that skill. Reading never executes scripts or grants tool permissions.",
        "parameters": {"type": "object", "properties": {
            "name": {"type": "string", "description": "Exact name returned by skills_list, including builtin/ or local/."},
            "file_path": {"type": "string", "description": "Optional relative .md or .txt reference; defaults to SKILL.md."},
        }, "required": ["name"], "additionalProperties": False},
    }},
]


def skill_roots():
    config = Path(os.environ.get("XDG_CONFIG_HOME") or Path.home() / ".config")
    return {"builtin": Path(__file__).resolve().parents[1] / "skills", "local": config / "lingtu" / "skills"}


def _paths():
    return {f"{origin}/{path.parent.relative_to(root).as_posix()}": (root, path)
            for origin, root in skill_roots().items() for path in sorted(root.rglob("SKILL.md"))}


def _read(path: Path, root: Path):
    if not path.resolve().is_relative_to(root.resolve()):
        raise ValueError("技能文件必须位于对应技能目录内。")
    with path.open("rb") as file:
        data = file.read(65537)
    if len(data) > 65536:
        raise ValueError("技能文件超过 64 KiB，请拆分为正文和参考文件。")
    return data.decode("utf-8-sig")


def _metadata(content: str):
    lines = content.splitlines()
    if not lines or lines[0] != "---" or "---" not in lines[1:]:
        raise ValueError("SKILL.md 缺少 YAML 元数据。")
    end = lines.index("---", 1)
    metadata = yaml.safe_load("\n".join(lines[1:end]))
    if not isinstance(metadata, dict) or any(not isinstance(metadata.get(key), str) or not metadata[key].strip()
                                             for key in ("name", "description")):
        raise ValueError("技能需要非空的 name 和 description。")
    return metadata


def execute_skill_tool(name: str, args: dict):
    try:
        paths = _paths()
        if name == "skills_list":
            skills, errors = [], []
            for identity, (root, path) in paths.items():
                try:
                    metadata = _metadata(_read(path, root))
                    skills.append({"name": identity, "title": metadata["name"], "description": metadata["description"]})
                except (OSError, UnicodeError, ValueError, yaml.YAMLError) as exc:
                    errors.append({"name": identity, "error": str(exc)})
            return {"skills": skills, "errors": errors, "hint": "Use skill_view with an exact listed name."}
        if name != "skill_view":
            return {"error": "unsupported_tool"}
        identity = args.get("name")
        if identity not in paths:
            return {"error": "未找到该技能，请先调用 skills_list。"}
        root, main = paths[identity]
        content = _read(main, root)
        metadata = _metadata(content)
        reference = args.get("file_path", "SKILL.md")
        path = Path(reference)
        if path.is_absolute() or path.suffix.lower() not in {".md", ".txt"}:
            return {"error": "仅支持技能目录内的相对 Markdown 或文本文件路径。"}
        if reference != "SKILL.md":
            content = _read(main.parent / path, main.parent)
        return {"name": identity, "title": metadata["name"], "file_path": reference, "content": content,
                "note": "Task guidance only. Existing user intent, tool permissions and robot execution evidence remain authoritative."}
    except (OSError, UnicodeError, ValueError, yaml.YAMLError) as exc:
        return {"error": str(exc)}
