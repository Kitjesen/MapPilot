"""Typed FastAPI adapter for the isolated local SimStudio API."""

from __future__ import annotations

from collections.abc import Mapping
from enum import Enum
from pathlib import Path
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field

from .artifact_service import ArtifactNotFound, ArtifactSecurityError
from .models import (
    IdempotencyConflict,
    RecordNotFound,
    RevisionConflict,
    StoreError,
    StoreValidationError,
)
from .package_service import PackageServiceError
from .run_service import ActiveRunConflict, RunNotFound, RunServiceError, RunStateError
from .scene_tools import FactoryParkSceneTool
from .source_inbox import SourceInboxError

API_PREFIX = "/api/sim/v1"


class LaunchProfile(str, Enum):
    """The only launch modes a caller may select."""

    HEADLESS = "headless"
    VISUAL = "visual"


class SourceDescriptor(BaseModel):
    """An entry below the Studio-managed inbox."""

    model_config = ConfigDict(extra="forbid", strict=True)

    entry: str = Field(min_length=1, max_length=512)


class ImportCreateRequest(BaseModel):
    """Request to stage a robot or world import from the Studio inbox."""

    model_config = ConfigDict(extra="forbid", strict=True)

    kind: Literal["robot", "world"]
    source: SourceDescriptor
    request: dict[str, Any] = Field(default_factory=dict)


class SessionDraftCreateRequest(BaseModel):
    """Request to create one validated session draft."""

    model_config = ConfigDict(extra="forbid", strict=True)

    intent: dict[str, Any]


class SessionDraftUpdateRequest(BaseModel):
    """Request to CAS-update one session draft."""

    model_config = ConfigDict(extra="forbid", strict=True)

    revision: int = Field(ge=1)
    intent: dict[str, Any]


class ComposeRequest(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)

    revision: int | None = Field(default=None, ge=1)


class RunCreateRequest(BaseModel):
    """Request to create a run from a compiled bundle."""

    model_config = ConfigDict(extra="forbid")

    bundle_id: str = Field(min_length=1, max_length=128)
    launch_profile: LaunchProfile


class RunOperationRequest(BaseModel):
    """Optional optimistic revision guard for run lifecycle operations."""

    model_config = ConfigDict(extra="forbid", strict=True)

    revision: int | None = Field(default=None, ge=1)


class FactoryParkElementRequest(BaseModel):
    """One declarative element placement; it contains no process controls."""

    model_config = ConfigDict(extra="forbid", strict=True)

    instance_key: str = Field(pattern=r"^[a-z][a-z0-9_]{0,63}$")
    element_type: str = Field(min_length=1)
    surface_id: str = Field(min_length=1)
    position_xy_m: list[float] = Field(min_length=2, max_length=2)
    yaw_deg: float = 0.0


class FactoryParkElementBatchRequest(BaseModel):
    """Strict JSON input for read-only FactoryPark_HF batch validation."""

    model_config = ConfigDict(extra="forbid", strict=True)

    schema_name: Literal["lingtu.sim.factory-park-element-batch.v1"] = Field(
        alias="schema"
    )
    batch_id: str = Field(pattern=r"^[a-z][a-z0-9_]{0,63}$")
    description: str = ""
    elements: list[FactoryParkElementRequest] = Field(min_length=1)


class SceneDraftCreateRequest(BaseModel):
    """Create one draft through a fixed, server-owned SceneTool."""

    model_config = ConfigDict(extra="forbid", strict=True)

    scene_tool: Literal["factory-park-hf"]
    batch: FactoryParkElementBatchRequest


class SceneDraftUpdateRequest(BaseModel):
    """CAS-update one SceneDraft after compiler validation."""

    model_config = ConfigDict(extra="forbid", strict=True)

    revision: int = Field(ge=1)
    batch: FactoryParkElementBatchRequest


class WorldPackagePublicationRequest(BaseModel):
    """Caller-selected immutable identity; all filesystem locations stay server-owned."""

    model_config = ConfigDict(extra="forbid", strict=True)

    id: str = Field(pattern=r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
    version: str = Field(pattern=r"^[A-Za-z0-9][A-Za-z0-9+_.-]*$")
    description: str = Field(min_length=1, max_length=512)


class SceneDraftPublishRequest(BaseModel):
    """Publish one exact SceneDraft revision into a new WorldPackage version."""

    model_config = ConfigDict(extra="forbid", strict=True)

    revision: int = Field(ge=1)
    package: WorldPackagePublicationRequest


_FORBIDDEN_INPUT_NAMES = {
    "absolute_path",
    "bind_host",
    "command",
    "command_line",
    "cwd",
    "dds_domain",
    "domain",
    "env",
    "environment",
    "executable",
    "executable_path",
    "filesystem_path",
    "host",
    "output_dir",
    "port",
    "ports",
    "robot_host",
    "shm_name",
    "source_path",
    "working_dir",
}


def _reject_process_and_path_controls(value: Any, *, context: str) -> None:
    """Keep future dictionary extensions inside the simulation contract."""

    if isinstance(value, Mapping):
        for key, child in value.items():
            if not isinstance(key, str):
                raise ValueError(f"{context} contains a non-string field name")
            lowered = key.lower().replace("-", "_")
            if lowered in _FORBIDDEN_INPUT_NAMES or lowered.endswith("_path") or "output" in lowered:
                raise ValueError(f"{context}.{key} is not a caller-controlled Studio field")
            _reject_process_and_path_controls(child, context=f"{context}.{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            _reject_process_and_path_controls(child, context=f"{context}[{index}]")


def create_app(service: Any) -> Any:
    """Create the versioned SimStudio app without starting any runtime."""

    from fastapi import Body, FastAPI, Header, Query, Request
    from fastapi.exceptions import RequestValidationError
    from fastapi.responses import JSONResponse

    app = FastAPI(
        title="LingTu SimStudio API",
        version="1.0",
        description="Loopback-only simulation engineering API; isolated from field Gateway ownership.",
    )

    def success(result: Any, *, status_code: int = 200):
        return JSONResponse({"ok": True, "result": result}, status_code=status_code)

    def failure(code: str, message: str, *, status_code: int, details: Any | None = None):
        error: dict[str, Any] = {"code": code, "message": message}
        if details is not None:
            error["details"] = details
        return JSONResponse(
            {"ok": False, "error": error, "diagnostics": [error]},
            status_code=status_code,
        )

    def invoke(method: str, *args: Any, **kwargs: Any):
        try:
            handler = getattr(service, method)
            return success(handler(*args, **kwargs))
        except Exception as exc:  # Route adapters must share one stable envelope.
            return _exception_response(exc, failure)

    def factory_park_scene_tool() -> Any:
        configured = getattr(service, "scene_tools", None)
        if configured is not None:
            return configured
        package_service = getattr(service, "package_service", None)
        repo_root = getattr(package_service, "repo_root", None)
        if repo_root is None:
            repo_root = Path(__file__).resolve().parents[3]
        return FactoryParkSceneTool(Path(repo_root))

    def _exception_response(exc: Exception, renderer: Any):
        if isinstance(exc, (RecordNotFound, RunNotFound, ArtifactNotFound)):
            return renderer("SIMSTUDIO_NOT_FOUND", str(exc), status_code=404)
        if isinstance(exc, (RevisionConflict, IdempotencyConflict, ActiveRunConflict, RunStateError)):
            details: dict[str, Any] = {}
            if isinstance(exc, RevisionConflict):
                details = {"expected": exc.expected, "actual": exc.actual}
            return renderer("SIMSTUDIO_CONFLICT", str(exc), status_code=409, details=details or None)
        if isinstance(
            exc,
            (StoreValidationError, PackageServiceError, SourceInboxError, ArtifactSecurityError, ValueError),
        ):
            code = getattr(exc, "code", "SIMSTUDIO_INVALID_REQUEST")
            error_details = getattr(exc, "details", None)
            return renderer(str(code), str(exc), status_code=422, details=error_details)
        if isinstance(exc, RunServiceError):
            return renderer("SIMSTUDIO_RUN_ERROR", str(exc), status_code=409)
        if isinstance(exc, (CatalogErrorCompat, StoreError)):
            return renderer("SIMSTUDIO_INVALID_REQUEST", str(exc), status_code=422)
        return renderer("SIMSTUDIO_INTERNAL_ERROR", str(exc) or type(exc).__name__, status_code=500)

    @app.exception_handler(RequestValidationError)
    async def request_validation_handler(request: Request, exc: RequestValidationError):
        del request
        return failure("SIMSTUDIO_INVALID_REQUEST", "request validation failed", status_code=422, details=exc.errors())

    @app.get(f"{API_PREFIX}/health")
    def health():
        if hasattr(service, "health"):
            return invoke("health")
        return success({"service": "simstudio", "api_version": "v1", "status": "ok", "field_isolated": True})

    @app.get(f"{API_PREFIX}/packages")
    def list_packages(kind: str | None = Query(default=None)):
        return invoke("list_packages", kind=kind)

    @app.get(f"{API_PREFIX}/inbox/sources")
    def list_inbox_sources():
        return invoke("list_inbox_sources")

    @app.get(f"{API_PREFIX}/inbox/sources/{{source_id}}/inspection")
    def inspect_inbox_source(source_id: str):
        return invoke("inspect_inbox_source", source_id)

    @app.get(f"{API_PREFIX}/import-contracts/{{kind}}")
    def import_contract(kind: str):
        return invoke("import_contract", kind)

    async def upload_inbox_source(
        request,
        filename: str = Header(..., alias="X-SimStudio-Filename"),
    ):
        try:
            result = await service.upload_inbox_source(filename, request.stream())
            return success(result, status_code=201)
        except Exception as exc:
            return _exception_response(exc, failure)

    # ``Request`` is imported lazily so importing the service layer does not
    # require FastAPI.  Attach the concrete annotation before route
    # registration; with postponed annotations a local ``Request`` name would
    # otherwise be interpreted as a caller-supplied query parameter.
    upload_inbox_source.__annotations__["request"] = Request
    app.post(f"{API_PREFIX}/inbox/uploads", status_code=201)(upload_inbox_source)

    @app.get(f"{API_PREFIX}/packages/{{kind}}/{{reference}}")
    def inspect_package(kind: str, reference: str):
        return invoke("inspect_package", reference, kind=kind)

    @app.get(f"{API_PREFIX}/scene-tools/factory-park-hf/catalog")
    def factory_park_element_catalog():
        try:
            return success(factory_park_scene_tool().catalog())
        except Exception as exc:
            return _exception_response(exc, failure)

    @app.post(
        f"{API_PREFIX}/scene-tools/factory-park-hf/element-batches/validate"
    )
    def validate_factory_park_element_batch(
        request: FactoryParkElementBatchRequest = Body(...),
    ):
        try:
            document = request.model_dump(by_alias=True)
            return success(
                factory_park_scene_tool().validate_element_batch(document)
            )
        except Exception as exc:
            return _exception_response(exc, failure)

    @app.get(f"{API_PREFIX}/scene-drafts")
    def list_scene_drafts():
        return invoke("list_scene_drafts")

    @app.post(f"{API_PREFIX}/scene-drafts", status_code=201)
    def create_scene_draft(
        request: SceneDraftCreateRequest = Body(...),
        idempotency_key: str | None = Header(default=None, alias="Idempotency-Key"),
    ):
        try:
            batch = request.batch.model_dump(by_alias=True)
            _reject_process_and_path_controls(batch, context="batch")
            return success(
                service.create_scene_draft(
                    batch,
                    idempotency_key=idempotency_key,
                ),
                status_code=201,
            )
        except Exception as exc:
            return _exception_response(exc, failure)

    @app.get(f"{API_PREFIX}/scene-drafts/{{scene_draft_id}}")
    def get_scene_draft(scene_draft_id: str):
        return invoke("get_scene_draft", scene_draft_id)

    @app.put(f"{API_PREFIX}/scene-drafts/{{scene_draft_id}}")
    def update_scene_draft(
        scene_draft_id: str,
        request: SceneDraftUpdateRequest = Body(...),
        idempotency_key: str | None = Header(default=None, alias="Idempotency-Key"),
    ):
        try:
            batch = request.batch.model_dump(by_alias=True)
            _reject_process_and_path_controls(batch, context="batch")
            return invoke(
                "update_scene_draft",
                scene_draft_id,
                revision=request.revision,
                batch=batch,
                idempotency_key=idempotency_key,
            )
        except Exception as exc:
            return _exception_response(exc, failure)

    @app.post(f"{API_PREFIX}/scene-drafts/{{scene_draft_id}}/publish", status_code=201)
    def publish_scene_draft(
        scene_draft_id: str,
        request: SceneDraftPublishRequest = Body(...),
        idempotency_key: str | None = Header(default=None, alias="Idempotency-Key"),
    ):
        try:
            package = request.package.model_dump()
            _reject_process_and_path_controls(package, context="package")
            return success(
                service.publish_scene_draft(
                    scene_draft_id,
                    revision=request.revision,
                    package=package,
                    idempotency_key=idempotency_key,
                ),
                status_code=201,
            )
        except Exception as exc:
            return _exception_response(exc, failure)

    @app.post(f"{API_PREFIX}/imports")
    def create_import(
        request: ImportCreateRequest = Body(...),
        idempotency_key: str | None = Header(default=None, alias="Idempotency-Key"),
    ):
        try:
            _reject_process_and_path_controls(request.request, context="request.request")
            return invoke(
                "create_import",
                kind=request.kind,
                request=request.request,
                source_entry=request.source.entry,
                idempotency_key=idempotency_key,
            )
        except Exception as exc:
            return _exception_response(exc, failure)

    @app.get(f"{API_PREFIX}/imports/{{import_id}}")
    def get_import(import_id: str):
        return invoke("get_import", import_id)

    @app.post(f"{API_PREFIX}/imports/{{import_id}}/promote")
    def promote_import(
        import_id: str,
        idempotency_key: str | None = Header(default=None, alias="Idempotency-Key"),
    ):
        return invoke("promote_import", import_id, idempotency_key=idempotency_key)

    @app.post(f"{API_PREFIX}/session-drafts")
    def create_draft(
        request: SessionDraftCreateRequest = Body(...),
        idempotency_key: str | None = Header(default=None, alias="Idempotency-Key"),
    ):
        try:
            _reject_process_and_path_controls(request.intent, context="intent")
            return invoke("create_draft", request.intent, idempotency_key=idempotency_key)
        except Exception as exc:
            return _exception_response(exc, failure)

    @app.get(f"{API_PREFIX}/session-drafts/{{draft_id}}")
    def get_draft(draft_id: str):
        return invoke("get_draft", draft_id)

    @app.put(f"{API_PREFIX}/session-drafts/{{draft_id}}")
    def update_draft(
        draft_id: str,
        request: SessionDraftUpdateRequest = Body(...),
        idempotency_key: str | None = Header(default=None, alias="Idempotency-Key"),
    ):
        try:
            _reject_process_and_path_controls(request.intent, context="intent")
            return invoke(
                "update_draft",
                draft_id,
                revision=request.revision,
                intent=request.intent,
                idempotency_key=idempotency_key,
            )
        except Exception as exc:
            return _exception_response(exc, failure)

    @app.post(f"{API_PREFIX}/session-drafts/{{draft_id}}/compose")
    def compose_draft(
        draft_id: str,
        request: ComposeRequest | None = Body(default=None),
        idempotency_key: str | None = Header(default=None, alias="Idempotency-Key"),
    ):
        return invoke(
            "compose_draft",
            draft_id,
            revision=None if request is None else request.revision,
            idempotency_key=idempotency_key,
        )

    @app.get(f"{API_PREFIX}/bundles/{{bundle_id}}")
    def get_bundle(bundle_id: str):
        return invoke("get_bundle", bundle_id)

    @app.get(f"{API_PREFIX}/runs")
    def list_runs():
        return invoke("list_runs")

    @app.post(f"{API_PREFIX}/runs")
    def create_run(
        request: RunCreateRequest = Body(...),
        idempotency_key: str | None = Header(default=None, alias="Idempotency-Key"),
    ):
        return invoke(
            "create_run",
            bundle_id=request.bundle_id,
            launch_profile=request.launch_profile.value,
            idempotency_key=idempotency_key,
        )

    @app.get(f"{API_PREFIX}/runs/{{run_id}}")
    def get_run(run_id: str):
        return invoke("get_run", run_id)

    def lifecycle(operation: str, run_id: str, request: RunOperationRequest | None, key: str | None):
        return invoke(
            "run_operation",
            operation,
            run_id,
            revision=None if request is None else request.revision,
            idempotency_key=key,
        )

    @app.post(f"{API_PREFIX}/runs/{{run_id}}/prepare")
    def prepare_run(
        run_id: str,
        request: RunOperationRequest | None = Body(default=None),
        idempotency_key: str | None = Header(default=None, alias="Idempotency-Key"),
    ):
        return lifecycle("prepare", run_id, request, idempotency_key)

    @app.post(f"{API_PREFIX}/runs/{{run_id}}/start")
    def start_run(
        run_id: str,
        request: RunOperationRequest | None = Body(default=None),
        idempotency_key: str | None = Header(default=None, alias="Idempotency-Key"),
    ):
        return lifecycle("start", run_id, request, idempotency_key)

    @app.post(f"{API_PREFIX}/runs/{{run_id}}/pause")
    def pause_run(
        run_id: str,
        request: RunOperationRequest | None = Body(default=None),
        idempotency_key: str | None = Header(default=None, alias="Idempotency-Key"),
    ):
        return lifecycle("pause", run_id, request, idempotency_key)

    @app.post(f"{API_PREFIX}/runs/{{run_id}}/reset")
    def reset_run(
        run_id: str,
        request: RunOperationRequest | None = Body(default=None),
        idempotency_key: str | None = Header(default=None, alias="Idempotency-Key"),
    ):
        return lifecycle("reset", run_id, request, idempotency_key)

    @app.post(f"{API_PREFIX}/runs/{{run_id}}/stop")
    def stop_run(
        run_id: str,
        request: RunOperationRequest | None = Body(default=None),
        idempotency_key: str | None = Header(default=None, alias="Idempotency-Key"),
    ):
        return lifecycle("stop", run_id, request, idempotency_key)

    @app.get(f"{API_PREFIX}/runs/{{run_id}}/readiness")
    def run_readiness(run_id: str):
        return invoke("run_readiness", run_id)

    @app.get(f"{API_PREFIX}/runs/{{run_id}}/artifacts")
    def list_artifacts(run_id: str):
        return invoke("list_artifacts", run_id)

    @app.get(f"{API_PREFIX}/recordings")
    def list_recordings():
        return invoke("list_recordings")

    @app.get(f"{API_PREFIX}/runs/{{run_id}}/recording")
    def inspect_recording(run_id: str):
        return invoke("inspect_recording", run_id)

    @app.get(f"{API_PREFIX}/runs/{{run_id}}/recording/timeline")
    def recording_timeline(
        run_id: str,
        offset: int = Query(default=0, ge=0),
        limit: int = Query(default=100, ge=1, le=200),
    ):
        return invoke(
            "recording_timeline",
            run_id,
            offset=offset,
            limit=limit,
        )

    @app.get(f"{API_PREFIX}/runs/{{run_id}}/recording/frames/{{frame_index}}")
    def recording_frame(run_id: str, frame_index: int):
        return invoke("recording_frame", run_id, frame_index)

    @app.post(f"{API_PREFIX}/runs/{{run_id}}/recording/start")
    def start_recording(
        run_id: str,
        request: RunOperationRequest | None = Body(default=None),
        idempotency_key: str | None = Header(default=None, alias="Idempotency-Key"),
    ):
        return lifecycle("start_recording", run_id, request, idempotency_key)

    @app.post(f"{API_PREFIX}/runs/{{run_id}}/recording/stop")
    def stop_recording(
        run_id: str,
        request: RunOperationRequest | None = Body(default=None),
        idempotency_key: str | None = Header(default=None, alias="Idempotency-Key"),
    ):
        return lifecycle("stop_recording", run_id, request, idempotency_key)

    @app.get(f"{API_PREFIX}/artifacts/{{artifact_id}}")
    def get_artifact(artifact_id: str):
        return invoke("get_artifact", artifact_id)

    return app


class CatalogErrorCompat(Exception):
    """Marker used to keep import-time dependencies out of FastAPI setup."""


__all__ = [
    "API_PREFIX",
    "FactoryParkElementBatchRequest",
    "FactoryParkElementRequest",
    "ImportCreateRequest",
    "LaunchProfile",
    "RunCreateRequest",
    "RunOperationRequest",
    "SceneDraftCreateRequest",
    "SceneDraftPublishRequest",
    "SceneDraftUpdateRequest",
    "SessionDraftCreateRequest",
    "SessionDraftUpdateRequest",
    "SourceDescriptor",
    "WorldPackagePublicationRequest",
    "create_app",
]
