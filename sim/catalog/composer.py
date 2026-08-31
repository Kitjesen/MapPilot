"""Deterministic SessionIntent-to-SessionSpec composition."""

from __future__ import annotations

import copy
import json
import shutil
import tempfile
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from .diagnostics import DiagnosticCode
from .resolver import CatalogError, CatalogResolver, ResolvedSession

_INTENT_SESSION_SCHEMAS = {
    "lingtu.sim.session-intent.v1": "lingtu.sim.session.v1",
    "lingtu.sim.session-intent.v2": "lingtu.sim.session.v2",
}
_RUNTIME_BINDINGS = frozenset({"physics", "visual", "sensors", "control"})
_GENERATED_ARTIFACTS = (
    "physics.plan.json",
    "visual.plan.json",
    "sensor.plan.json",
    "control.plan.json",
    "transport.intent.json",
    "scenario.plan.json",
)


def _canonical_json(value: Any, *, pretty: bool = False) -> str:
    if pretty:
        return json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2) + "\n"
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))


def _mapping(value: Any, context: str, *, code: DiagnosticCode) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise CatalogError(f"{context} must be an object", code=code, context=context)
    return value


def _strict_keys(
    value: Mapping[str, Any],
    *,
    required: set[str],
    optional: set[str],
    context: str,
    code: DiagnosticCode,
) -> None:
    missing = sorted(required - set(value))
    unknown = sorted(set(value) - required - optional)
    if missing or unknown:
        raise CatalogError(
            f"{context} has invalid fields",
            code=code,
            context=context,
            details={"missing": missing, "unknown": unknown},
        )


@dataclass(frozen=True)
class SessionIntent:
    """Validated authoring intent that deterministically emits one SessionSpec."""

    document: Mapping[str, Any]

    @classmethod
    def load(cls, value: Path | Mapping[str, Any]) -> SessionIntent:
        """Load and strictly normalize an intent mapping or YAML file."""

        if isinstance(value, Path):
            path = value.resolve()
            try:
                document = yaml.safe_load(path.read_text(encoding="utf-8"))
            except (OSError, yaml.YAMLError) as exc:
                raise CatalogError(
                    f"cannot read SessionIntent {path}: {exc}",
                    code=DiagnosticCode.INTENT_INVALID,
                    context=str(path),
                ) from exc
        else:
            document = value
        normalized = cls._validate_document(document)
        return cls(document=normalized)

    @staticmethod
    def _validate_document(value: Any) -> dict[str, Any]:
        intent = _mapping(value, "SessionIntent", code=DiagnosticCode.INTENT_INVALID)
        _strict_keys(
            intent,
            required={"schema", "session"},
            optional={"overrides"},
            context="SessionIntent",
            code=DiagnosticCode.INTENT_INVALID,
        )
        intent_schema = intent["schema"]
        session_schema = _INTENT_SESSION_SCHEMAS.get(intent_schema)
        if session_schema is None:
            raise CatalogError(
                "SessionIntent.schema is unsupported",
                code=DiagnosticCode.INTENT_INVALID,
                context="SessionIntent.schema",
                details={"allowed": sorted(_INTENT_SESSION_SCHEMAS)},
            )
        session = _mapping(
            intent["session"],
            "SessionIntent.session",
            code=DiagnosticCode.INTENT_INVALID,
        )
        allowed_session_fields = {
            "schema",
            "session_id",
            "mujoco_version",
            "seed",
            "world",
            "scenario",
            "robots",
            "runtime",
        }
        required_session_fields = allowed_session_fields - {"schema", "scenario"}
        _strict_keys(
            session,
            required=required_session_fields,
            optional={"schema", "scenario"},
            context="SessionIntent.session",
            code=DiagnosticCode.INTENT_INVALID,
        )
        if "schema" in session and session["schema"] != session_schema:
            raise CatalogError(
                f"SessionIntent.session.schema must be {session_schema}",
                code=DiagnosticCode.INTENT_INVALID,
                context="SessionIntent.session.schema",
            )

        normalized: dict[str, Any] = {
            "schema": intent_schema,
            "session": copy.deepcopy(dict(session)),
        }
        if "overrides" in intent:
            overrides = _mapping(
                intent["overrides"],
                "SessionIntent.overrides",
                code=DiagnosticCode.OVERRIDE_INVALID,
            )
            _strict_keys(
                overrides,
                required=set(),
                optional={"seed", "runtime"},
                context="SessionIntent.overrides",
                code=DiagnosticCode.OVERRIDE_INVALID,
            )
            normalized_overrides: dict[str, Any] = {}
            if "seed" in overrides:
                seed = overrides["seed"]
                if isinstance(seed, bool) or not isinstance(seed, int):
                    raise CatalogError(
                        "SessionIntent.overrides.seed must be an integer",
                        code=DiagnosticCode.OVERRIDE_INVALID,
                        context="SessionIntent.overrides.seed",
                    )
                normalized_overrides["seed"] = seed
            if "runtime" in overrides:
                runtime = _mapping(
                    overrides["runtime"],
                    "SessionIntent.overrides.runtime",
                    code=DiagnosticCode.OVERRIDE_INVALID,
                )
                _strict_keys(
                    runtime,
                    required=set(),
                    optional={"mode", "required_bindings"},
                    context="SessionIntent.overrides.runtime",
                    code=DiagnosticCode.OVERRIDE_INVALID,
                )
                normalized_runtime: dict[str, Any] = {}
                if "mode" in runtime:
                    mode = runtime["mode"]
                    if not isinstance(mode, str) or not mode:
                        raise CatalogError(
                            "SessionIntent.overrides.runtime.mode must be a non-empty string",
                            code=DiagnosticCode.OVERRIDE_INVALID,
                            context="SessionIntent.overrides.runtime.mode",
                        )
                    normalized_runtime["mode"] = mode
                if "required_bindings" in runtime:
                    bindings = runtime["required_bindings"]
                    if not isinstance(bindings, list) or any(
                        not isinstance(binding, str) or not binding for binding in bindings
                    ):
                        raise CatalogError(
                            "SessionIntent.overrides.runtime.required_bindings must be an array of strings",
                            code=DiagnosticCode.OVERRIDE_INVALID,
                            context="SessionIntent.overrides.runtime.required_bindings",
                        )
                    unsupported = sorted(set(bindings) - _RUNTIME_BINDINGS)
                    if unsupported:
                        raise CatalogError(
                            "SessionIntent.overrides.runtime.required_bindings contains unsupported bindings",
                            code=DiagnosticCode.OVERRIDE_INVALID,
                            context="SessionIntent.overrides.runtime.required_bindings",
                            details={"unsupported": unsupported, "allowed": sorted(_RUNTIME_BINDINGS)},
                        )
                    if len(bindings) != len(set(bindings)):
                        raise CatalogError(
                            "SessionIntent.overrides.runtime.required_bindings contains duplicates",
                            code=DiagnosticCode.OVERRIDE_INVALID,
                            context="SessionIntent.overrides.runtime.required_bindings",
                        )
                    if "physics" not in bindings:
                        raise CatalogError(
                            "SessionIntent.overrides.runtime.required_bindings must include physics",
                            code=DiagnosticCode.OVERRIDE_INVALID,
                            context="SessionIntent.overrides.runtime.required_bindings",
                        )
                    normalized_runtime["required_bindings"] = list(bindings)
                normalized_overrides["runtime"] = normalized_runtime
            normalized["overrides"] = normalized_overrides
        return normalized

    def to_session_spec(self) -> dict[str, Any]:
        """Apply allowed overrides and emit one canonical SessionSpec mapping."""

        session = copy.deepcopy(dict(self.document["session"]))
        session["schema"] = _INTENT_SESSION_SCHEMAS[self.document["schema"]]
        overrides = self.document.get("overrides", {})
        if "seed" in overrides:
            session["seed"] = overrides["seed"]
        runtime_overrides = overrides.get("runtime", {})
        if runtime_overrides:
            runtime = copy.deepcopy(dict(session["runtime"]))
            if "mode" in runtime_overrides:
                runtime["mode"] = runtime_overrides["mode"]
            if "required_bindings" in runtime_overrides:
                runtime["required_bindings"] = list(runtime_overrides["required_bindings"])
            session["runtime"] = runtime
        ordered_fields = (
            "schema",
            "session_id",
            "mujoco_version",
            "seed",
            "world",
            "scenario",
            "robots",
            "runtime",
        )
        return {field: session[field] for field in ordered_fields if field in session}


@dataclass(frozen=True)
class ComposedSession:
    """The emitted SessionSpec and existing resolver/compiler result."""

    session_spec: dict[str, Any]
    session_spec_path: Path
    resolved: ResolvedSession
    bundle_dir: Path

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-serializable composition result."""

        artifacts = ["session.yaml", *_GENERATED_ARTIFACTS[:-1]]
        if self.resolved.scenario_json is not None:
            artifacts.append("scenario.plan.json")
        return {
            "schema": "lingtu.sim.composed-session.v1",
            "session_id": self.session_spec["session_id"],
            "session_spec": self.session_spec,
            "session_spec_path": str(self.session_spec_path),
            "bundle_dir": str(self.bundle_dir),
            "artifacts": [str(self.bundle_dir / artifact) for artifact in artifacts],
        }


class SessionComposer:
    """Compose authoring intent and delegate compilation to one resolver instance."""

    def __init__(self, resolver: CatalogResolver, *, artifact_root: Path) -> None:
        self.resolver = resolver
        self.artifact_root = Path(artifact_root).resolve()

    @classmethod
    def from_repository(
        cls,
        repo_root: Path,
        *,
        artifact_root: Path | None = None,
    ) -> SessionComposer:
        """Create a composer over the repository's canonical resolver."""

        resolved_root = Path(repo_root).resolve()
        return cls(
            CatalogResolver.from_repository(resolved_root),
            artifact_root=artifact_root or resolved_root / "build" / "simstudio",
        )

    def resolve_output_dir(self, output_dir: Path) -> Path:
        """Resolve one caller name inside the service-owned artifact root."""

        requested = Path(output_dir)
        target = (self.artifact_root / requested).resolve() if not requested.is_absolute() else requested.resolve()
        try:
            target.relative_to(self.artifact_root)
        except ValueError as exc:
            raise CatalogError(
                "session output directory escapes the SimStudio artifact root",
                code=DiagnosticCode.PATH_TRAVERSAL,
                context=str(requested),
                details={"artifact_root": str(self.artifact_root)},
            ) from exc
        if target == self.artifact_root:
            raise CatalogError(
                "session output directory must name a child of the SimStudio artifact root",
                code=DiagnosticCode.PATH_TRAVERSAL,
                context=str(requested),
                details={"artifact_root": str(self.artifact_root)},
            )
        return target

    def _stage_output(self, output_dir: Path) -> tuple[Path, Path]:
        target = self.resolve_output_dir(output_dir)
        if target.exists():
            raise CatalogError(
                "session output directory already exists",
                code=DiagnosticCode.ARTIFACT_CONFLICT,
                context=str(target),
            )
        self.artifact_root.mkdir(parents=True, exist_ok=True)
        target.parent.mkdir(parents=True, exist_ok=True)
        staging = Path(tempfile.mkdtemp(prefix=f".{target.name}.staging-", dir=target.parent)).resolve()
        return target, staging

    @staticmethod
    def _publish(staging: Path, target: Path) -> None:
        staging.replace(target)

    def resolve_session(self, session: Path, *, output_dir: Path | None = None) -> tuple[ResolvedSession, Path | None]:
        """Resolve one SessionSpec and optionally publish its bundle transactionally."""

        resolved = self.resolver.resolve(Path(session).resolve())
        if output_dir is None:
            return resolved, None
        target, staging = self._stage_output(output_dir)
        try:
            resolved.write_bundle(staging)
            self._publish(staging, target)
        except Exception:
            shutil.rmtree(staging, ignore_errors=True)
            raise
        return resolved, target

    def compose(
        self,
        intent: SessionIntent | Path | Mapping[str, Any],
        *,
        output_dir: Path,
    ) -> ComposedSession:
        """Emit SessionSpec, invoke the existing resolver, and write its bundle."""

        resolved_intent = intent if isinstance(intent, SessionIntent) else SessionIntent.load(intent)
        session_spec = resolved_intent.to_session_spec()
        session_text = _canonical_json(session_spec, pretty=True)
        bundle_dir, staging = self._stage_output(output_dir)
        staging_session_path = staging / "session.yaml"
        try:
            staging_session_path.write_text(session_text, encoding="utf-8", newline="\n")

            # This is the sole compiler boundary. Do not duplicate resolver logic here.
            resolved = self.resolver.resolve(staging_session_path)
            resolved.write_bundle(staging)
            if resolved.scenario_json is None:
                (staging / "scenario.plan.json").unlink(missing_ok=True)
            self._publish(staging, bundle_dir)
        except Exception:
            shutil.rmtree(staging, ignore_errors=True)
            raise
        session_spec_path = bundle_dir / "session.yaml"
        return ComposedSession(
            session_spec=session_spec,
            session_spec_path=session_spec_path,
            resolved=resolved,
            bundle_dir=bundle_dir,
        )


__all__ = ["ComposedSession", "SessionComposer", "SessionIntent"]
