"""Command-line surface for SimCatalog and deterministic session composition."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Sequence, TextIO

from sim.catalog.composer import SessionComposer
from sim.catalog.diagnostics import CatalogDiagnostic, DiagnosticCode
from sim.catalog.management import SimCatalog
from sim.catalog.resolver import CatalogError, CatalogResolver


def _add_repo_root(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=None,
        help="repository root; defaults to the current working directory",
    )
    parser.add_argument(
        "--artifact-root",
        type=Path,
        default=None,
        help="owned output root; defaults to <repo-root>/build/simstudio",
    )


def _add_package_identity(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("reference", help="exact package reference in id@version form")
    parser.add_argument("--kind", help="optional package kind for unambiguous lookup")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Query the LingTu SimCatalog or compose a deterministic SessionBundle."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    list_parser = subparsers.add_parser("list", help="list validated catalog packages")
    _add_repo_root(list_parser)
    list_parser.add_argument("--kind")

    for command in ("inspect", "validate", "dependencies", "qualification"):
        command_parser = subparsers.add_parser(command)
        _add_repo_root(command_parser)
        _add_package_identity(command_parser)

    compose_parser = subparsers.add_parser("compose", help="compose SessionIntent through the canonical resolver")
    _add_repo_root(compose_parser)
    compose_parser.add_argument("intent", type=Path)
    compose_parser.add_argument("--output-dir", type=Path, required=True)

    resolve_parser = subparsers.add_parser("resolve", help="resolve an authored SessionSpec")
    _add_repo_root(resolve_parser)
    resolve_parser.add_argument("session", type=Path)
    resolve_parser.add_argument("--output-dir", type=Path)
    return parser


def _json_line(value: object) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))


def main(argv: Sequence[str] | None = None, *, stdout: TextIO | None = None) -> int:
    """Run one catalog command and emit a stable JSON response envelope."""

    output = stdout or sys.stdout
    args = _parser().parse_args(sys.argv[1:] if argv is None else argv)
    repo_root = (args.repo_root or Path.cwd()).resolve()
    try:
        resolver = CatalogResolver.from_repository(repo_root)
        catalog = SimCatalog(
            resolver,
            qualification_roots=(repo_root / "sim" / "evaluation" / "package_qualifications",),
        )
        composer = SessionComposer(
            resolver,
            artifact_root=args.artifact_root or repo_root / "build" / "simstudio",
        )
        if args.command == "list":
            result = catalog.list_packages(kind=args.kind)
        elif args.command in {"inspect", "validate", "dependencies", "qualification"}:
            operation = {
                "inspect": catalog.inspect_package,
                "validate": catalog.validate_package,
                "dependencies": catalog.dependencies,
                "qualification": catalog.qualification,
            }[args.command]
            result = operation(args.reference, kind=args.kind)
        elif args.command == "compose":
            result = composer.compose(args.intent, output_dir=args.output_dir).to_dict()
        else:
            resolved, bundle_dir = composer.resolve_session(args.session, output_dir=args.output_dir)
            result = {
                "schema": "lingtu.sim.resolved-session-result.v1",
                "session_id": resolved.session_id,
                "bundle_dir": str(bundle_dir) if bundle_dir is not None else None,
            }
        payload = {"ok": True, "result": result}
    except CatalogError as exc:
        payload = {"ok": False, "diagnostics": [exc.to_diagnostic().to_dict()]}
    except (OSError, ValueError) as exc:
        diagnostic = CatalogDiagnostic(
            code=DiagnosticCode.CATALOG_INVALID,
            message=str(exc),
            context=args.command,
        )
        payload = {"ok": False, "diagnostics": [diagnostic.to_dict()]}
    output.write(_json_line(payload) + "\n")
    return 0 if payload.get("ok") else 2


if __name__ == "__main__":
    raise SystemExit(main())
