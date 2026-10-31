"""Inventory SCAN C++ functions using an installed libclang C API.

No parser package or compiler output binary is installed/built. Every source and
header is parsed independently; locations and function identities come from the
full Clang translation unit, without namespace or regex function filtering.
"""
from __future__ import annotations

import argparse
import bisect
import concurrent.futures
import ctypes as ct
import json
import os
import re
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SOURCE = Path("src/nav/cpp/planning/local/scan")
SUFFIXES = {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx"}
FUNCTION_KINDS = {8, 21, 24, 25, 26, 30}
NAMESPACE_KIND = 22
LAMBDA_KIND = 144
BODY_KINDS = {202, 224}


class CXString(ct.Structure):
    _fields_ = [("data", ct.c_void_p), ("flags", ct.c_uint)]


class Cursor(ct.Structure):
    _fields_ = [("kind", ct.c_int), ("xdata", ct.c_int), ("data", ct.c_void_p * 3)]


class Location(ct.Structure):
    _fields_ = [("ptr_data", ct.c_void_p * 2), ("int_data", ct.c_uint)]


class Range(ct.Structure):
    _fields_ = [("ptr_data", ct.c_void_p * 2), ("begin", ct.c_uint), ("end", ct.c_uint)]


class Type(ct.Structure):
    _fields_ = [("kind", ct.c_int), ("data", ct.c_void_p * 2)]


Visitor = ct.CFUNCTYPE(ct.c_uint, Cursor, Cursor, ct.c_void_p)


class Clang:
    def __init__(self, path: Path):
        self.dll_directory = os.add_dll_directory(str(path.parent)) if os.name == "nt" else None
        self.lib = ct.CDLL(str(path))
        self.bind("clang_getCString", ct.c_char_p, CXString)
        self.bind("clang_disposeString", None, CXString)
        self.bind("clang_getClangVersion", CXString)
        self.bind("clang_createIndex", ct.c_void_p, ct.c_int, ct.c_int)
        self.bind("clang_disposeIndex", None, ct.c_void_p)
        self.bind("clang_parseTranslationUnit2", ct.c_int, ct.c_void_p, ct.c_char_p,
                  ct.POINTER(ct.c_char_p), ct.c_int, ct.c_void_p, ct.c_uint,
                  ct.c_uint, ct.POINTER(ct.c_void_p))
        self.bind("clang_disposeTranslationUnit", None, ct.c_void_p)
        self.bind("clang_getTranslationUnitCursor", Cursor, ct.c_void_p)
        self.bind("clang_visitChildren", ct.c_uint, Cursor, Visitor, ct.c_void_p)
        self.bind("clang_getCursorLocation", Location, Cursor)
        self.bind("clang_getCursorExtent", Range, Cursor)
        self.bind("clang_getRangeStart", Location, Range)
        self.bind("clang_getRangeEnd", Location, Range)
        self.bind("clang_getExpansionLocation", None, Location, ct.POINTER(ct.c_void_p),
                  ct.POINTER(ct.c_uint), ct.POINTER(ct.c_uint), ct.POINTER(ct.c_uint))
        self.bind("clang_getSpellingLocation", None, Location, ct.POINTER(ct.c_void_p),
                  ct.POINTER(ct.c_uint), ct.POINTER(ct.c_uint), ct.POINTER(ct.c_uint))
        self.bind("clang_getFileName", CXString, ct.c_void_p)
        self.bind("clang_getCursorSpelling", CXString, Cursor)
        self.bind("clang_getCursorDisplayName", CXString, Cursor)
        self.bind("clang_getCursorUSR", CXString, Cursor)
        self.bind("clang_getCursorSemanticParent", Cursor, Cursor)
        self.bind("clang_Cursor_isNull", ct.c_int, Cursor)
        self.bind("clang_getCursorKindSpelling", CXString, ct.c_int)
        self.bind("clang_getCursorType", Type, Cursor)
        self.bind("clang_getTypeSpelling", CXString, Type)
        self.bind("clang_isCursorDefinition", ct.c_uint, Cursor)
        self.bind("clang_Cursor_isFunctionInlined", ct.c_uint, Cursor)
        self.bind("clang_CXXMethod_isDefaulted", ct.c_uint, Cursor)
        self.bind("clang_CXXMethod_isDeleted", ct.c_uint, Cursor)
        self.bind("clang_getNumDiagnostics", ct.c_uint, ct.c_void_p)
        self.bind("clang_getDiagnostic", ct.c_void_p, ct.c_void_p, ct.c_uint)
        self.bind("clang_getDiagnosticSeverity", ct.c_uint, ct.c_void_p)
        self.bind("clang_formatDiagnostic", CXString, ct.c_void_p, ct.c_uint)
        self.bind("clang_defaultDiagnosticDisplayOptions", ct.c_uint)
        self.bind("clang_disposeDiagnostic", None, ct.c_void_p)

    def bind(self, name, result, *args):
        function = getattr(self.lib, name)
        function.restype, function.argtypes = result, args

    def string(self, value: CXString) -> str:
        try:
            data = self.lib.clang_getCString(value)
            return data.decode("utf-8", errors="replace") if data else ""
        finally:
            self.lib.clang_disposeString(value)

    def location(self, value: Location, spelling: bool = False) -> tuple[str, int, int]:
        file, line, column, offset = ct.c_void_p(), ct.c_uint(), ct.c_uint(), ct.c_uint()
        method = self.lib.clang_getSpellingLocation if spelling else self.lib.clang_getExpansionLocation
        method(value, ct.byref(file), ct.byref(line), ct.byref(column), ct.byref(offset))
        name = self.string(self.lib.clang_getFileName(file)) if file else ""
        return (str(Path(name).resolve()) if name else "", line.value, offset.value)

    def qualified(self, cursor: Cursor) -> str:
        names = [self.string(self.lib.clang_getCursorSpelling(cursor))]
        parent = self.lib.clang_getCursorSemanticParent(cursor)
        while not self.lib.clang_Cursor_isNull(parent) and parent.kind != 350:
            name = self.string(self.lib.clang_getCursorSpelling(parent))
            if not name and parent.kind == NAMESPACE_KIND:
                name = "(anonymous namespace)"
            if name:
                names.append(name)
            parent = self.lib.clang_getCursorSemanticParent(parent)
        return "::".join(reversed(names))

    def parse(self, path: Path, arguments: list[str]) -> tuple[list[dict], dict]:
        lib = self.lib
        source = path.read_bytes()
        line_offsets = [0] + [i+1 for i, value in enumerate(source) if value == 10]
        canonical = os.path.normcase(str(path.resolve()))
        index = lib.clang_createIndex(0, 0)
        tu = ct.c_void_p()
        args = (ct.c_char_p * len(arguments))(*(a.encode() for a in arguments))
        entries, errors = [], []
        visited, lambdas = 0, 0
        try:
            code = lib.clang_parseTranslationUnit2(index, str(path).encode(), args,
                      len(arguments), None, 0, 0, ct.byref(tu))
            if code or not tu:
                raise RuntimeError(f"Clang parse failed ({code}): {path}")
            diagnostics = []
            for i in range(lib.clang_getNumDiagnostics(tu)):
                diagnostic = lib.clang_getDiagnostic(tu, i)
                severity = lib.clang_getDiagnosticSeverity(diagnostic)
                message = self.string(lib.clang_formatDiagnostic(
                    diagnostic, lib.clang_defaultDiagnosticDisplayOptions()))
                diagnostics.append({"severity": severity, "message": message})
                lib.clang_disposeDiagnostic(diagnostic)
            if any(d["severity"] >= 3 for d in diagnostics):
                raise RuntimeError("\n".join(d["message"] for d in diagnostics))

            @Visitor
            def visitor(cursor, parent, data):
                nonlocal visited, lambdas
                try:
                    visited += 1
                    location = lib.clang_getCursorLocation(cursor)
                    file, line, offset = self.location(location)
                    belongs = os.path.normcase(file) == canonical
                    if cursor.kind == LAMBDA_KIND and belongs:
                        lambdas += 1
                        return 1
                    if belongs and cursor.kind in FUNCTION_KINDS:
                        extent = lib.clang_getCursorExtent(cursor)
                        begin_file, _, begin = self.location(lib.clang_getRangeStart(extent))
                        end_file, _, end = self.location(lib.clang_getRangeEnd(extent))
                        body_start = []

                        @Visitor
                        def body_visitor(child, enclosing, unused):
                            if child.kind in BODY_KINDS:
                                child_file, _, child_offset = self.location(
                                    lib.clang_getRangeStart(lib.clang_getCursorExtent(child)))
                                if os.path.normcase(child_file) == canonical:
                                    body_start.append(child_offset)
                            return 1

                        lib.clang_visitChildren(cursor, body_visitor, None)
                        deleted = bool(lib.clang_CXXMethod_isDeleted(cursor))
                        defaulted = bool(lib.clang_CXXMethod_isDefaulted(cursor))
                        definition = bool(lib.clang_isCursorDefinition(cursor))
                        definition_kind = "body" if body_start else "deleted" if deleted else "defaulted" if defaulted else "other" if definition else None
                        definition = definition or deleted or defaulted or bool(body_start)
                        spelling = self.location(location, spelling=True)
                        macro = spelling != (file, line, offset)
                        signature_end = min(body_start) if body_start else end
                        if os.path.normcase(begin_file) == canonical and os.path.normcase(end_file) == canonical:
                            signature = source[begin:signature_end].decode("utf-8", errors="replace").strip().rstrip(";")
                        else:
                            signature = self.string(lib.clang_getCursorDisplayName(cursor))
                        semantic_parent = lib.clang_getCursorSemanticParent(cursor)
                        entries.append({
                            "file": path.relative_to(ROOT).as_posix(),
                            "line": line, "end_line": bisect.bisect_right(line_offsets, max(begin, end-1)),
                            "offset": offset,
                            "qualified_name": self.qualified(cursor),
                            "owner_class": self.qualified(semantic_parent) if semantic_parent.kind in {2,3,4,31,32} else None,
                            "signature": re.sub(r"\s+", " ", signature),
                            "clang_type": self.string(lib.clang_getTypeSpelling(lib.clang_getCursorType(cursor))),
                            "cursor_kind": self.string(lib.clang_getCursorKindSpelling(cursor.kind)),
                            "usr": self.string(lib.clang_getCursorUSR(cursor)),
                            "is_definition": definition,
                            "definition_kind": definition_kind,
                            "inline": bool(lib.clang_Cursor_isFunctionInlined(cursor)),
                            "macro_generated": macro,
                            "classification": "upstream" if "upstream" in path.relative_to(ROOT / SOURCE).parts else "native",
                        })
                    # Imported records and function bodies cannot own lexical
                    # definitions in this independently parsed source file.
                    # Keep namespace traversal, including anonymous namespaces.
                    return 2 if belongs or cursor.kind == NAMESPACE_KIND else 1
                except BaseException as error:
                    errors.append(str(error))
                    return 0

            lib.clang_visitChildren(lib.clang_getTranslationUnitCursor(tu), visitor, None)
            if errors:
                raise RuntimeError("; ".join(errors))
            return entries, {"file": path.relative_to(ROOT).as_posix(), "parsed": True,
                             "diagnostics": diagnostics, "ast_cursors_visited": visited,
                             "local_lambdas_excluded": lambdas,
                             "lexical_function_records": len(entries)}
        finally:
            if tu:
                lib.clang_disposeTranslationUnit(tu)
            lib.clang_disposeIndex(index)


def run(args) -> None:
    compiler = Clang(args.libclang.resolve())
    toolchain = args.libclang.resolve().parent.parent
    version = compiler.string(compiler.lib.clang_getClangVersion())
    major = re.search(r"version (\d+)", version).group(1)
    arguments = ["-std=c++17", "-x", "c++", "-Wno-everything", "-include", "iostream",
                 "--target=x86_64-w64-windows-gnu",
                 "-resource-dir="+str(toolchain / "lib/clang" / major),
                 "-isystem", str(toolchain / "include/c++/v1"),
                 "-isystem", str(toolchain / "include"),
                 "-I"+str(ROOT / "src/nav/cpp"),
                 "-I"+str(ROOT / "src/nav/cpp/include"), "-I"+str(args.eigen.resolve())]
    files = sorted(p for p in (ROOT / SOURCE).rglob("*") if p.suffix.lower() in SUFFIXES)
    entries, coverage = [], []
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
        futures = {pool.submit(compiler.parse, p, arguments): p for p in files}
        for future in concurrent.futures.as_completed(futures):
            rows, record = future.result()
            entries.extend(rows)
            coverage.append(record)
            print(record["file"], len(rows), flush=True)
    definitions, declaration_groups = {}, {}
    for entry in sorted(entries, key=lambda r: (r["file"], r["line"], r["offset"])):
        identity = entry["usr"] or entry["qualified_name"]+entry["clang_type"]
        if entry["is_definition"]:
            definitions.setdefault((entry["file"], entry["offset"], identity), entry)
        else:
            if identity in declaration_groups:
                declaration_groups[identity]["additional_declarations"].append(
                    {"file": entry["file"], "line": entry["line"]})
            else:
                declaration_groups[identity] = {**entry, "additional_declarations": []}
    definitions = list(definitions.values())
    declarations = list(declaration_groups.values())
    for declaration in declarations:
        declaration["definitions_in_scope"] = [
            {"file": d["file"], "line": d["line"], "definition_kind": d["definition_kind"]}
            for d in definitions if d["usr"] == declaration["usr"]]
        declaration["has_definition_in_scope"] = bool(declaration["definitions_in_scope"])
    declarations_without_definition = [d for d in declarations if not d["has_definition_in_scope"]]
    for row in coverage:
        row["definitions"] = sum(d["file"] == row["file"] for d in definitions)
        row["declarations"] = sum(d["file"] == row["file"] for d in declarations)
    assert {r["file"] for r in coverage} == {p.relative_to(ROOT).as_posix() for p in files}
    result = {"scope": SOURCE.as_posix(), "parser": version,
              "method": "Installed libclang C API via Python ctypes; complete translation-unit cursor traversal and exact expansion-file location ownership; no regex or namespace function filtering",
              "arguments": arguments,
              "limits": ["C++17 host preprocessing configuration; inactive preprocessor branches are not parsed.",
                         "Analysis TUs pre-include <iostream>, providing the include context needed by polynomial_traj.h; this is declaration extraction, not a production build check.",
                         "Local lambda expression bodies and generated closure call operators are excluded.",
                         "Compiler-generated implicit special members and template instantiations are not source-level function declarations exposed by this cursor traversal.",
                         "Function pointer typedefs, call expressions and macros without emitted function declarations are not functions.",
                         "This inventories functions, not runtime reachability or proof of algorithm correctness."],
              "file_count": len(files),
              "definition_count": len(definitions),
              "definition_kinds": dict(Counter(d["definition_kind"] for d in definitions)),
              "declaration_count": len(declarations),
              "declarations_without_definition_count": len(declarations_without_definition),
              "coverage": sorted(coverage, key=lambda r: r["file"]),
              "definitions": definitions, "declarations": declarations,
              "declarations_without_definition": declarations_without_definition}
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "function-inventory.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    lines = ["# SCAN function inventory", "", f"Parser: {result['parser']}", "", result["method"], "",
             f"Files: {len(files)}; source definitions: {len(definitions)}; unique non-definition declarations: {len(declarations)}.",
             "Definitions include explicitly defaulted/deleted definitions, identified separately below. A header declaration and its implementation are not summed into a unique API count.", "",
             "## Scope and limits", "", *["- "+s for s in result["limits"]], "", "## File coverage", "",
             "| File | Definitions | Declarations | Parse errors |", "|---|---:|---:|---:|"]
    for row in result["coverage"]:
        lines.append(f"| {row['file']} | {row['definitions']} | {row['declarations']} | 0 |")
    for title, rows in [("Definitions", definitions), ("Declarations without a body at this location", declarations)]:
        lines.extend(["", "## "+title, ""])
        for file in sorted({r["file"] for r in rows}):
            lines.extend(["", "### "+file, "", "| Line | Origin | Qualified function | Signature | Definition kind |", "|---:|---|---|---|---|"])
            for row in (r for r in rows if r["file"] == file):
                signature = row["signature"].replace("|", "&#124;").replace("`", "&#96;")
                name = row["qualified_name"].replace("|", "&#124;")
                source_link = (ROOT/file).as_posix()
                lines.append(f"| [{row['line']}]({source_link}:{row['line']}) | {row['classification']} | `{name}` | `{signature}` | {row['definition_kind'] or 'declaration'} |")
    lines.extend(["", "## Declarations without a definition in this scope", "",
                  "This is a subset of the declarations above, not additional functions or proof of a runtime failure.", ""])
    for row in declarations_without_definition:
        lines.append(f"- `{row['qualified_name']}` — {row['file']}:{row['line']}")
    (args.out / "functions.md").write_text("\n".join(lines)+"\n", encoding="utf-8")
    print(json.dumps({k: result[k] for k in ["file_count", "definition_count", "definition_kinds", "declaration_count", "declarations_without_definition_count"]}), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--libclang", type=Path, default=Path("D:/msys64/clang64/bin/libclang.dll"))
    parser.add_argument("--eigen", type=Path, default=ROOT / "third_party/install/slam-windows/x64-windows/include/eigen3")
    parser.add_argument("--out", type=Path, default=ROOT / "build/scan-review-20261025")
    parser.add_argument("--jobs", type=int, default=4)
    run(parser.parse_args())
