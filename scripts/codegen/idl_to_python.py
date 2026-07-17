#!/usr/bin/env python3
"""IDL -> Python dataclass code generator.

Parses a CycloneDDS-style IDL file and generates Python ``@dataclass`` type
definitions.  No external dependencies are required.

Usage::

    python scripts/codegen/idl_to_python.py src/message/idl/lingtu_slam.idl \
        --output src/message/dds_types_generated/
"""

from __future__ import annotations

import argparse
import os
import re
import sys
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------
# Parsed IDL data structures
# ---------------------------------------------------------------------------


@dataclass
class IdlField:
    """A single field inside an IDL struct."""

    name: str
    raw_type: str
    array_size: int | None = None
    is_sequence: bool = False
    element_type: str | None = None


@dataclass
class IdlStruct:
    """A parsed IDL struct definition."""

    name: str
    module_path: tuple[str, ...]
    fields: list[IdlField] = field(default_factory=list)
    keys: list[str] = field(default_factory=list)

    @property
    def fully_qualified(self) -> str:
        """Return the C++-style fully-qualified type name (e.g. ``lingtu::dds::Time``)."""
        return "::".join((*self.module_path, self.name))


# ---------------------------------------------------------------------------
# IDL parser
# ---------------------------------------------------------------------------


class IdlParser:
    """A lightweight regex/state-machine IDL parser.

    Supports:
      * ``module`` nesting (tracked as namespace path)
      * ``struct`` definitions with basic types, sequences, fixed arrays,
        and nested-struct references
      * ``//`` and ``/* */`` comments
      * ``#pragma keylist`` directives (key fields)
    """

    _PRAGMA_KEYLIST_RE = re.compile(
        r"#pragma\s+keylist\s+(\w+)\s+(.*?)(?:\n|$)",
        re.IGNORECASE,
    )

    # Map of IDL primitive types -> Python type annotation
    PRIMITIVE_MAP: dict[str, str] = {
        # Signed integers
        "short": "int",
        "long": "int",
        "long long": "int",
        "int8": "int",
        "int16": "int",
        "int32": "int",
        "int64": "int",
        # Unsigned integers
        "unsigned short": "int",
        "unsigned long": "int",
        "unsigned long long": "int",
        "uint8": "int",
        "uint16": "int",
        "uint32": "int",
        "uint64": "int",
        "octet": "int",
        # Floating point
        "float": "float",
        "double": "float",
        "long double": "float",
        # Boolean
        "boolean": "bool",
        # String / character
        "string": "str",
        "char": "str",
        "wchar": "str",
        "wstring": "str",
    }

    def parse(self, text: str) -> list[IdlStruct]:
        """Parse IDL source text and return a list of :class:`IdlStruct`."""
        text = self._strip_comments(text)
        keylist_map = self._extract_keylists(text)
        text = re.sub(r"#pragma[^\n]*", "", text)
        return self._scan(text, keylist_map)

    # -- public helpers --------------------------------------------------

    @classmethod
    def is_primitive(cls, idl_type: str) -> bool:
        """Return ``True`` if *idl_type* is a known IDL primitive."""
        return idl_type.strip() in cls.PRIMITIVE_MAP

    @classmethod
    def map_primitive(cls, idl_type: str) -> str:
        """Map an IDL primitive type to a Python annotation string."""
        key = idl_type.strip()
        if key not in cls.PRIMITIVE_MAP:
            raise KeyError(f"Unknown IDL primitive type: {idl_type!r}")
        return cls.PRIMITIVE_MAP[key]

    # -- internal parsing ------------------------------------------------

    @staticmethod
    def _strip_comments(text: str) -> str:
        """Remove ``//`` line comments and ``/* */`` block comments."""
        text = re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)
        text = re.sub(r"//[^\n]*", "", text)
        return text

    def _extract_keylists(self, text: str) -> dict[str, list[str]]:
        """Extract ``#pragma keylist`` directives into a ``{struct: [keys]}`` map."""
        result: dict[str, list[str]] = {}
        for m in self._PRAGMA_KEYLIST_RE.finditer(text):
            struct_name = m.group(1)
            keys = m.group(2).split()
            result[struct_name] = keys
        return result

    def _scan(
        self,
        text: str,
        keylist_map: dict[str, list[str]],
    ) -> list[IdlStruct]:
        """Walk the IDL text tracking ``module`` nesting and ``struct`` blocks."""
        structs: list[IdlStruct] = []
        module_stack: list[str] = []
        pos = 0
        n = len(text)

        while pos < n:
            # Skip whitespace
            while pos < n and text[pos].isspace():
                pos += 1
            if pos >= n:
                break

            remaining = text[pos:]

            # module NAME {
            m = re.match(r"module\s+(\w+)\s*\{", remaining)
            if m:
                module_stack.append(m.group(1))
                pos += m.end()
                continue

            # struct NAME {
            m = re.match(r"struct\s+(\w+)\s*\{", remaining)
            if m:
                struct_name = m.group(1)
                body_start = pos + m.end()
                # Struct bodies contain no nested braces, so find next '}'
                body_end = text.index("}", body_start)
                body = text[body_start:body_end]
                fields = self._parse_fields(body)
                keys = keylist_map.get(struct_name, [])
                structs.append(
                    IdlStruct(
                        name=struct_name,
                        module_path=tuple(module_stack),
                        fields=fields,
                        keys=keys,
                    )
                )
                pos = body_end + 1
                # Consume optional trailing semicolons / whitespace
                while pos < n and text[pos] in " \t\n\r;":
                    if text[pos] == ";":
                        pos += 1
                        break
                    pos += 1
                continue

            # '}' closes a module
            if text[pos] == "}":
                if module_stack:
                    module_stack.pop()
                pos += 1
                # Consume optional trailing semicolon
                while pos < n and text[pos] in " \t\n\r;":
                    if text[pos] == ";":
                        pos += 1
                        break
                    pos += 1
                continue

            # Skip anything unrecognised
            pos += 1

        return structs

    def _parse_fields(self, body: str) -> list[IdlField]:
        """Parse the body of a struct into a list of :class:`IdlField`."""
        fields: list[IdlField] = []
        declarations = [d.strip() for d in body.split(";") if d.strip()]
        for decl in declarations:
            parsed = self._parse_single_field(decl)
            if parsed is not None:
                fields.append(parsed)
        return fields

    def _parse_single_field(self, decl: str) -> IdlField | None:
        """Parse a single field declaration string."""
        decl = decl.strip()
        if not decl:
            return None

        # Fixed-size array: TYPE NAME[N]
        array_match = re.match(r"(.+)\s+(\w+)\s*\[(\d+)\]\s*$", decl)
        if array_match:
            type_str = array_match.group(1).strip()
            name = array_match.group(2)
            size = int(array_match.group(3))
            return IdlField(
                name=name,
                raw_type=type_str,
                array_size=size,
                is_sequence=False,
                element_type=None,
            )

        # Sequence: sequence<ELEM> NAME
        seq_match = re.match(
            r"sequence\s*<\s*(.+?)\s*>\s+(\w+)\s*$",
            decl,
        )
        if seq_match:
            elem_type = seq_match.group(1).strip()
            name = seq_match.group(2)
            return IdlField(
                name=name,
                raw_type="sequence",
                array_size=None,
                is_sequence=True,
                element_type=elem_type,
            )

        # Plain field: TYPE NAME  (type may be multi-word, e.g. "unsigned long")
        parts = decl.rsplit(None, 1)
        if len(parts) == 2:
            type_str, name = parts
            return IdlField(
                name=name,
                raw_type=type_str,
                array_size=None,
                is_sequence=False,
                element_type=None,
            )

        return None


# ---------------------------------------------------------------------------
# Type mapper
# ---------------------------------------------------------------------------


class TypeMapper:
    """Map parsed IDL fields to Python type-annotation strings."""

    def __init__(self, known_struct_names: set[str]):
        self._struct_names = known_struct_names

    def field_annotation(self, fld: IdlField) -> str:
        """Return the Python type annotation for *fld*."""
        # Sequence<T> -> list[T]
        if fld.is_sequence:
            elem = fld.element_type or "octet"
            return f"list[{self._base_type(elem)}]"

        # Fixed array T[N] -> list[T]
        if fld.array_size is not None:
            return f"list[{self._base_type(fld.raw_type)}]"

        return self._base_type(fld.raw_type)

    def field_default(self, fld: IdlField) -> str | None:
        """Return a default-value expression for *fld*, or ``None`` for required fields."""
        if fld.is_sequence or fld.array_size is not None:
            return "field(default_factory=list)"
        return None

    def _base_type(self, idl_type: str) -> str:
        """Map an IDL type string to a Python annotation.

        Primitives are mapped via :data:`IdlParser.PRIMITIVE_MAP`; anything
        else is treated as a nested-struct reference.
        """
        idl_type = idl_type.strip()
        if IdlParser.is_primitive(idl_type):
            return IdlParser.map_primitive(idl_type)
        # Otherwise it's a struct reference — use the name as-is
        return idl_type


# ---------------------------------------------------------------------------
# Code generator
# ---------------------------------------------------------------------------


class CodeGenerator:
    """Render parsed IDL structs into Python source files."""

    def __init__(self, source_idl: str):
        self._source_idl = source_idl

    def generate(self, structs: list[IdlStruct]) -> dict[str, str]:
        """Return a ``{relative_path: source_code}`` mapping of generated files."""
        ordered = self._topological_sort(structs)
        mapper = TypeMapper({s.name for s in structs})
        all_names = [s.name for s in ordered]

        types_code = self._render_types(ordered, mapper)
        init_code = self._render_init(all_names)

        return {
            "types.py": types_code,
            "__init__.py": init_code,
        }

    # -- sorting ---------------------------------------------------------

    @staticmethod
    def _topological_sort(structs: list[IdlStruct]) -> list[IdlStruct]:
        """Order structs so that dependencies appear before dependants."""
        name_to_struct = {s.name: s for s in structs}
        visited: set[str] = set()
        result: list[IdlStruct] = []

        def visit(name: str, path: set[str]) -> None:
            if name in visited or name not in name_to_struct:
                return
            if name in path:
                # Circular dependency — skip to avoid infinite recursion
                return
            path.add(name)
            struct = name_to_struct[name]
            for fld in struct.fields:
                dep = _field_struct_ref(fld)
                if dep and dep in name_to_struct:
                    visit(dep, path)
            path.discard(name)
            visited.add(name)
            result.append(struct)

        for s in structs:
            visit(s.name, set())

        return result

    # -- rendering -------------------------------------------------------

    def _render_types(
        self,
        structs: list[IdlStruct],
        mapper: TypeMapper,
    ) -> str:
        """Render the ``types.py`` module body."""
        now = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
        lines: list[str] = []
        lines.append('r"""Auto-generated DDS dataclass definitions.')
        lines.append("")
        lines.append(f"Source IDL: {self._source_idl}")
        lines.append(f"Generated:  {now}")
        lines.append("")
        lines.append("DO NOT EDIT BY HAND -- regenerate with::")
        lines.append("")
        lines.append(f"    python scripts/codegen/idl_to_python.py {self._source_idl} --output <output_dir>/")
        lines.append('"""')
        lines.append("")
        lines.append("from __future__ import annotations")
        lines.append("")
        lines.append("from dataclasses import dataclass, field")
        lines.append("")
        lines.append("")

        for idx, struct in enumerate(structs):
            self._render_struct(struct, mapper, lines)
            if idx < len(structs) - 1:
                lines.append("")
                lines.append("")

        return "\n".join(lines) + "\n"

    def _render_struct(
        self,
        struct: IdlStruct,
        mapper: TypeMapper,
        lines: list[str],
    ) -> None:
        """Render a single ``@dataclass`` definition."""
        fq = struct.fully_qualified
        lines.append("@dataclass(kw_only=True)")
        lines.append(f"class {struct.name}:")

        # Docstring
        doc_lines = [f'    r"""IDL struct: {fq}']
        if struct.keys:
            doc_lines.append("")
            doc_lines.append(f"    Keys: {', '.join(struct.keys)}")
        doc_lines.append('    """')
        lines.extend(doc_lines)

        if not struct.fields:
            lines.append("    pass")
            return

        for fld in struct.fields:
            annotation = mapper.field_annotation(fld)
            default = mapper.field_default(fld)
            if default is not None:
                lines.append(f"    {fld.name}: {annotation} = {default}")
            else:
                lines.append(f"    {fld.name}: {annotation}")

    def _render_init(self, all_names: list[str]) -> str:
        """Render the ``__init__.py`` that re-exports all generated types."""
        now = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
        lines: list[str] = []
        lines.append('r"""Auto-generated DDS type package.')
        lines.append("")
        lines.append(f"Source IDL: {self._source_idl}")
        lines.append(f"Generated:  {now}")
        lines.append("")
        lines.append("DO NOT EDIT BY HAND -- run ``make codegen-idl`` to regenerate.")
        lines.append('"""')
        lines.append("")
        lines.append("from __future__ import annotations")
        lines.append("")
        lines.append("from .types import (  # noqa: F401")
        for name in all_names:
            lines.append(f"    {name},")
        lines.append(")")
        lines.append("")
        lines.append("__all__ = [")
        for name in all_names:
            lines.append(f'    "{name}",')
        lines.append("]")
        return "\n".join(lines) + "\n"


def _field_struct_ref(fld: IdlField) -> str | None:
    """Return the struct name referenced by *fld*, or ``None`` for primitives."""
    if fld.is_sequence:
        elem = fld.element_type or ""
        if not IdlParser.is_primitive(elem):
            return elem
        return None
    if not IdlParser.is_primitive(fld.raw_type):
        return fld.raw_type
    return None


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------


def generate_from_idl(idl_path: str, output_dir: str) -> list[str]:
    """Parse *idl_path* and write generated files into *output_dir*.

    Returns the list of written file paths.
    """
    idl_file = Path(idl_path)
    if not idl_file.is_file():
        raise FileNotFoundError(f"IDL file not found: {idl_path}")

    text = idl_file.read_text(encoding="utf-8")
    parser = IdlParser()
    structs = parser.parse(text)

    if not structs:
        raise ValueError(f"No structs found in IDL file: {idl_path}")

    # Use the source path as displayed in generated comments
    source_display = str(idl_path)

    gen = CodeGenerator(source_display)
    files = gen.generate(structs)

    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    written: list[str] = []
    for rel_path, content in files.items():
        target = out / rel_path
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(content, encoding="utf-8")
        written.append(str(target))

    return written


def main(argv: list[str] | None = None) -> int:
    """CLI entry point."""
    p = argparse.ArgumentParser(
        description="Generate Python dataclasses from an IDL file.",
    )
    p.add_argument(
        "idl_file",
        help="Path to the source .idl file",
    )
    p.add_argument(
        "--output",
        "-o",
        required=True,
        help="Output directory for generated Python files",
    )
    args = p.parse_args(argv)

    try:
        written = generate_from_idl(args.idl_file, args.output)
    except (FileNotFoundError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    print(f"Generated {len(written)} file(s):")
    for path in written:
        print(f"  {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
