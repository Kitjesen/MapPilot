"""Generate native defaults from the same component YAML used by Product assembly."""

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path

import yaml


def literal(value):
    if isinstance(value, bool):
        return "bool", "true" if value else "false"
    if isinstance(value, int):
        return "int", str(value)
    if isinstance(value, float) and math.isfinite(value):
        return "double", repr(value)
    if isinstance(value, str):
        return "const char*", json.dumps(value, ensure_ascii=True)
    if isinstance(value, list) and value:
        items = [literal(item) for item in value]
        types = {kind for kind, _ in items}
        kind = "double" if types <= {"int", "double"} and "double" in types else items[0][0]
        if not (types <= {"int", "double"} or len(types) == 1):
            raise ValueError(f"mixed array types: {types}")
        return f"std::array<{kind}, {len(items)}>", "{{" + ", ".join(text for _, text in items) + "}}"
    raise ValueError(f"unsupported setting value: {value!r}")


def declarations(mapping, prefix=""):
    result = []
    for key, value in mapping.items():
        name = key.replace(".", "_")
        if re.fullmatch(r"[a-zA-Z_][a-zA-Z0-9_]*", name) is None:
            raise ValueError(f"invalid C++ setting name: {prefix}{key}")
        if isinstance(value, dict):
            result.append(f"namespace {name} {{")
            result.extend(declarations(value, prefix + key + "."))
            result.append(f"}}  // namespace {name}")
        else:
            kind, text = literal(value)
            result.append(f"inline constexpr {kind} {name} = {text};")
    return result


def generate(directory: Path) -> str:
    lines = ["// Generated from config/system/settings/*.yaml. Do not edit.",
             "#pragma once", "#include <array>", "namespace lingtu::settings {"]
    for path in sorted(directory.glob("*.yaml")):
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        lines.append(f"namespace {path.stem} {{")
        lines.extend(declarations(data))
        lines.append(f"}}  // namespace {path.stem}")
    lines.append("}  // namespace lingtu::settings")
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    content = generate(args.directory)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not args.output.exists() or args.output.read_text(encoding="utf-8") != content:
        with args.output.open("w", encoding="utf-8", newline="\n") as stream:
            stream.write(content)


if __name__ == "__main__":
    main()
