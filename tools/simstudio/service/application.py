"""Application boundary for the local, field-isolated SimStudio service."""

from __future__ import annotations

import hashlib
import threading
from collections.abc import Callable, Mapping
from pathlib import Path
from typing import Any, cast

from sim.catalog.composer import SessionComposer
from sim.catalog.management import SimCatalog
from sim.catalog.resolver import CatalogResolver

from .artifact_service import ArtifactService
from .models import RecordNotFound
from .package_service import PackageImportService
from .recording_service import RecordingService
from .run_service import RunService
from .runtime_factory import RuntimeFactory
from .scene_draft_service import SceneDraftService
from .scene_publication import ScenePublicationService
from .scene_tools import FactoryParkSceneTool
from .session_service import SessionAuthoringService
from .source_inbox import SourceInboxService
from .store import StudioStore


class SimulationStudioService:
    """Compose the simulation-local application services behind one boundary.

    ``from_repository`` creates the complete local service graph. Tests may
    inject individual services, including fakes.
    """

    def __init__(
        self,
        catalog: SimCatalog | None = None,
        composer: SessionComposer | None = None,
        *,
        store: StudioStore | None = None,
        package_service: PackageImportService | Any | None = None,
        session_service: SessionAuthoringService | Any | None = None,
        run_service: RunService | Any | None = None,
        artifact_service: ArtifactService | Any | None = None,
        source_inbox_service: SourceInboxService | Any | None = None,
        recording_service: RecordingService | Any | None = None,
        scene_draft_service: SceneDraftService | Any | None = None,
        scene_publication_service: ScenePublicationService | Any | None = None,
        repo_root: Path | None = None,
    ) -> None:
        if catalog is not None and composer is not None and catalog.resolver is not composer.resolver:
            raise ValueError("SimCatalog and SessionComposer must share one CatalogResolver")
        self.catalog = catalog
        self.composer = composer
        self.resolver = catalog.resolver if catalog is not None else getattr(composer, "resolver", None)
        self.store = store
        self.package_service = package_service
        self.session_service = session_service
        self.run_service = run_service
        self.artifact_service = artifact_service
        self.source_inbox_service = source_inbox_service
        self.recording_service = recording_service
        self.scene_draft_service = scene_draft_service
        self.scene_publication_service = scene_publication_service
        self.repo_root = None if repo_root is None else Path(repo_root).resolve()
        self._catalog_refresh_lock = threading.RLock()

    @classmethod
    def from_repository(
        cls,
        repo_root: Path,
        *,
        artifact_root: Path | None = None,
        session_factory: Callable[..., Any] | None = None,
    ) -> SimulationStudioService:
        """Construct the complete service graph below a private local root."""

        resolved_root = Path(repo_root).resolve()
        resolver = CatalogResolver.from_repository(resolved_root)
        catalog = SimCatalog(
            resolver,
            qualification_roots=(resolved_root / "sim" / "evaluation" / "package_qualifications",),
        )
        composer = SessionComposer(
            resolver,
            artifact_root=artifact_root or resolved_root / "build" / "simstudio",
        )
        store = StudioStore(Path(artifact_root or resolved_root / "build" / "simstudio-state"))
        package_service = PackageImportService(
            catalog=catalog,
            store=store,
            repo_root=resolved_root,
        )
        source_inbox_service = SourceInboxService(store.root / "inbox")
        session_service = SessionAuthoringService(
            store,
            composer,
            catalog,
        )
        factory = (
            session_factory
            if session_factory is not None
            else RuntimeFactory.from_repository(resolved_root, studio_root=store.root).create_session
        )
        run_service = RunService(
            store,
            factory,
            artifact_root=store.root / "artifacts" / "runs",
        )
        artifact_service = ArtifactService(store)
        scene_tool = FactoryParkSceneTool(resolved_root)
        return cls(
            catalog,
            composer,
            store=store,
            package_service=package_service,
            session_service=session_service,
            run_service=run_service,
            artifact_service=artifact_service,
            source_inbox_service=source_inbox_service,
            recording_service=RecordingService(store, artifact_service),
            scene_draft_service=SceneDraftService(store, scene_tool),
            scene_publication_service=ScenePublicationService(
                store=store,
                scene_tool=scene_tool,
                repo_root=resolved_root,
            ),
            repo_root=resolved_root,
        )

    def health(self) -> dict[str, Any]:
        """Return a stable local-service health document."""

        return {
            "service": "simstudio",
            "api_version": "v1",
            "status": "ok",
            "field_isolated": True,
            "runtime_bound": self.run_service is not None,
        }

    def list_packages(self, *, kind: str | None = None) -> dict[str, Any]:
        """List catalog packages, optionally filtered by package kind."""

        service = self.package_service
        if service is not None and hasattr(service, "list_packages"):
            return cast(dict[str, Any], service.list_packages(kind=kind))
        if self.catalog is None:
            raise RuntimeError("package service is not configured")
        return cast(dict[str, Any], self.catalog.list_packages(kind=kind))

    def list_inbox_sources(self) -> dict[str, Any]:
        """List verified archives in the Studio-managed source inbox."""

        service = self._require(self.source_inbox_service, "source inbox service")
        return cast(dict[str, Any], service.list_sources())

    def inspect_inbox_source(self, source_id: str) -> dict[str, Any]:
        """Safely inspect one source archive by its Studio-owned ID."""

        service = self._require(self.source_inbox_service, "source inbox service")
        return cast(dict[str, Any], service.inspect(source_id))

    async def upload_inbox_source(self, filename: str, chunks: Any) -> dict[str, Any]:
        """Stream an archive into the managed source inbox."""

        service = self._require(self.source_inbox_service, "source inbox service")
        return cast(dict[str, Any], await service.upload(filename, chunks))

    def import_contract(self, kind: str) -> dict[str, Any]:
        """Return the Studio-owned request template for one package importer."""

        service = self._require(self.package_service, "package service")
        return cast(dict[str, Any], service.import_contract(kind))

    def inspect_package(self, reference: str, *, kind: str) -> dict[str, Any]:
        """Return details for one referenced catalog package."""

        service = self.package_service
        if service is not None and hasattr(service, "detail_package"):
            return cast(dict[str, Any], service.detail_package(reference, kind=kind))
        if self.catalog is None:
            raise RuntimeError("package service is not configured")
        return cast(dict[str, Any], self.catalog.inspect_package(reference, kind=kind))

    def validate_package(self, reference: str, *, kind: str) -> dict[str, Any]:
        """Validate one referenced catalog package."""

        service = self.package_service
        if service is not None and hasattr(service, "validate_package"):
            return cast(dict[str, Any], service.validate_package(reference, kind=kind))
        if self.catalog is None:
            raise RuntimeError("package service is not configured")
        return cast(dict[str, Any], self.catalog.validate_package(reference, kind=kind))

    def dependencies(self, reference: str, *, kind: str | None = None) -> dict[str, Any]:
        """Return the transitive dependency graph for one catalog package."""

        if self.catalog is None:
            raise RuntimeError("package service is not configured")
        return cast(dict[str, Any], self.catalog.dependencies(reference, kind=kind))

    def qualification(self, reference: str, *, kind: str | None = None) -> dict[str, Any]:
        """Return the qualification status view for one catalog package."""

        if self.catalog is None:
            raise RuntimeError("package service is not configured")
        return cast(dict[str, Any], self.catalog.qualification(reference, kind=kind))

    def create_import(
        self,
        *,
        kind: str,
        request: Mapping[str, Any],
        source_entry: str,
        idempotency_key: str | None = None,
    ) -> dict[str, Any]:
        """Create a package import job from one source entry."""

        return cast(
            dict[str, Any],
            self._require(self.package_service, "package service").create_import_job(
                kind=kind,
                request=request,
                source_entry=source_entry,
                idempotency_key=idempotency_key,
            ),
        )

    def get_import(self, import_id: str) -> dict[str, Any]:
        """Return one package import job."""

        return cast(
            dict[str, Any],
            self._require(self.package_service, "package service").get_import_job(import_id),
        )

    def promote_import(self, import_id: str, *, idempotency_key: str | None = None) -> dict[str, Any]:
        """Promote a completed import into the package catalog."""

        result = cast(
            dict[str, Any],
            self._require(self.package_service, "package service").promote_import_job(
                import_id,
                idempotency_key=idempotency_key,
            ),
        )
        self._refresh_catalog_views()
        return result

    def create_draft(self, intent: Mapping[str, Any], *, idempotency_key: str | None = None) -> dict[str, Any]:
        """Create a session draft from an intent document."""

        return cast(
            dict[str, Any],
            self._require(self.session_service, "session service").create_session_draft(
                intent,
                idempotency_key=idempotency_key,
            ),
        )

    def get_draft(self, draft_id: str) -> dict[str, Any]:
        """Return one session draft."""

        return cast(
            dict[str, Any],
            self._require(self.session_service, "session service").get_session_draft(draft_id),
        )

    def update_draft(
        self,
        draft_id: str,
        *,
        revision: int,
        intent: Mapping[str, Any],
        idempotency_key: str | None = None,
    ) -> dict[str, Any]:
        """Update a session draft at the expected revision."""

        return cast(
            dict[str, Any],
            self._require(self.session_service, "session service").update_session_draft(
                draft_id,
                revision=revision,
                intent=intent,
                idempotency_key=idempotency_key,
            ),
        )

    def compose_draft(
        self,
        draft_id: str,
        *,
        revision: int | None = None,
        idempotency_key: str | None = None,
    ) -> dict[str, Any]:
        """Compose a session bundle from a draft revision."""

        return cast(
            dict[str, Any],
            self._require(self.session_service, "session service").compose_session(
                draft_id,
                revision=revision,
                idempotency_key=idempotency_key,
            ),
        )

    def get_bundle(self, bundle_id: str) -> dict[str, Any]:
        """Return one composed session bundle."""

        return cast(
            dict[str, Any],
            self._require(self.session_service, "session service").get_bundle(bundle_id),
        )

    def list_runs(self) -> list[dict[str, Any]]:
        """List simulation run records."""

        return cast(list[dict[str, Any]], self._require(self.run_service, "run service").list_runs())

    def create_run(self, *, bundle_id: str, launch_profile: str, idempotency_key: str | None = None) -> dict[str, Any]:
        """Create a simulation run for a bundle and launch profile."""

        return cast(
            dict[str, Any],
            self._require(self.run_service, "run service").create_run(
                bundle_id,
                launch_profile,
                idempotency_key=idempotency_key,
            ),
        )

    def get_run(self, run_id: str) -> dict[str, Any]:
        """Return one simulation run record."""

        return cast(dict[str, Any], self._require(self.run_service, "run service").get_run(run_id))

    def run_operation(
        self,
        operation: str,
        run_id: str,
        *,
        revision: int | None = None,
        idempotency_key: str | None = None,
    ) -> dict[str, Any]:
        """Apply a named lifecycle operation to a simulation run."""

        service = self._require(self.run_service, "run service")
        method = getattr(service, operation)
        return cast(
            dict[str, Any],
            method(run_id, expected_revision=revision, idempotency_key=idempotency_key),
        )

    def run_readiness(self, run_id: str) -> dict[str, Any]:
        """Return the readiness and sensor summary for a run."""

        service = self._require(self.run_service, "run service")
        record = service.get_run(run_id)
        payload = record.get("payload", {}) if isinstance(record, Mapping) else {}
        readiness = payload.get("readiness", {}) if isinstance(payload, Mapping) else {}
        sensors = payload.get("sensor_summary", {}) if isinstance(payload, Mapping) else {}
        return {
            "run_id": run_id,
            "status": record.get("status"),
            "revision": record.get("revision"),
            "ready": record.get("status") in {"READY", "RUNNING", "PAUSED"},
            "readiness": readiness,
            "sensors": sensors,
        }

    def list_artifacts(self, run_id: str) -> list[dict[str, Any]]:
        """List public artifact records for one simulation run."""

        service = self._require(self.artifact_service, "artifact service")
        return [self._artifact_public(run_id, item) for item in service.list_artifacts(run_id)]

    def list_recordings(self) -> dict[str, Any]:
        """List committed recording artifacts and their validation state."""

        service = self._require(self.recording_service, "recording service")
        return cast(dict[str, Any], service.list_recordings())

    def inspect_recording(self, run_id: str) -> dict[str, Any]:
        """Inspect the recording commit marker owned by one run."""

        service = self._require(self.recording_service, "recording service")
        return cast(dict[str, Any], service.inspect(run_id))

    def recording_timeline(
        self,
        run_id: str,
        *,
        offset: int = 0,
        limit: int = 100,
    ) -> dict[str, Any]:
        """Return one bounded page of validated recording-frame summaries."""

        service = self._require(self.recording_service, "recording service")
        return cast(
            dict[str, Any],
            service.timeline(run_id, offset=offset, limit=limit),
        )

    def recording_frame(self, run_id: str, frame_index: int) -> dict[str, Any]:
        """Return one validated recording frame by dense frame index."""

        service = self._require(self.recording_service, "recording service")
        return cast(
            dict[str, Any],
            service.frame(run_id, frame_index=frame_index),
        )

    def create_scene_draft(
        self,
        batch: Mapping[str, Any],
        *,
        idempotency_key: str | None = None,
    ) -> dict[str, Any]:
        """Create one compiler-validated scene authoring draft."""

        service = self._require(self.scene_draft_service, "scene draft service")
        return cast(
            dict[str, Any],
            service.create_scene_draft(batch, idempotency_key=idempotency_key),
        )

    def get_scene_draft(self, scene_draft_id: str) -> dict[str, Any]:
        """Return one scene authoring draft."""

        service = self._require(self.scene_draft_service, "scene draft service")
        return cast(dict[str, Any], service.get_scene_draft(scene_draft_id))

    def list_scene_drafts(self) -> list[dict[str, Any]]:
        """List compiler-validated scene authoring drafts."""

        service = self._require(self.scene_draft_service, "scene draft service")
        return cast(list[dict[str, Any]], service.list_scene_drafts())

    def update_scene_draft(
        self,
        scene_draft_id: str,
        *,
        revision: int,
        batch: Mapping[str, Any],
        idempotency_key: str | None = None,
    ) -> dict[str, Any]:
        """CAS-update one scene authoring draft after compiler validation."""

        service = self._require(self.scene_draft_service, "scene draft service")
        return cast(
            dict[str, Any],
            service.update_scene_draft(
                scene_draft_id,
                revision=revision,
                batch=batch,
                idempotency_key=idempotency_key,
            ),
        )

    def publish_scene_draft(
        self,
        scene_draft_id: str,
        *,
        revision: int,
        package: Mapping[str, Any],
        idempotency_key: str | None = None,
    ) -> dict[str, Any]:
        """Publish one exact SceneDraft revision as an immutable WorldPackage."""

        service = self._require(self.scene_publication_service, "scene publication service")
        result = cast(
            dict[str, Any],
            service.publish(
                scene_draft_id,
                revision=revision,
                package=package,
                idempotency_key=idempotency_key,
            ),
        )
        self._refresh_catalog_views()
        return result

    def _refresh_catalog_views(self) -> None:
        """Atomically rebind every authoring view to one freshly scanned resolver."""

        if self.repo_root is None:
            return
        with self._catalog_refresh_lock:
            resolver = CatalogResolver.from_repository(self.repo_root)
            catalog = SimCatalog(
                resolver,
                qualification_roots=(self.repo_root / "sim" / "evaluation" / "package_qualifications",),
            )
            self.resolver = resolver
            self.catalog = catalog
            if self.composer is not None:
                self.composer = SessionComposer(
                    resolver,
                    artifact_root=self.composer.artifact_root,
                )
            if self.package_service is not None and hasattr(self.package_service, "catalog"):
                self.package_service.catalog = catalog
            if self.session_service is not None:
                self.session_service.catalog = catalog
                self.session_service.composer = SessionComposer(
                    resolver,
                    artifact_root=self.session_service.bundles_root,
                )

    def get_artifact(self, artifact_id: str) -> dict[str, Any]:
        """Return an artifact record by its public identifier."""

        service = self._require(self.artifact_service, "artifact service")
        for run in self.list_runs():
            run_id = run.get("id")
            if not isinstance(run_id, str):
                continue
            for item in service.list_artifacts(run_id):
                if self._artifact_id(run_id, item.get("path")) == artifact_id:
                    return self._artifact_public(run_id, service.get_artifact(run_id, item["path"]))
        raise RecordNotFound(f"artifact {artifact_id} was not found")

    @staticmethod
    def _artifact_id(run_id: str, path: Any) -> str:
        if not isinstance(path, str):
            raise ValueError("artifact path is invalid")
        return hashlib.sha256(f"{run_id}\0{path}".encode()).hexdigest()[:32]

    def _artifact_public(self, run_id: str, item: Mapping[str, Any]) -> dict[str, Any]:
        result = dict(item)
        result["artifact_id"] = self._artifact_id(run_id, result.get("path"))
        result["run_id"] = run_id
        return result

    @staticmethod
    def _require(value: Any, label: str) -> Any:
        if value is None:
            raise RuntimeError(f"{label} is not configured")
        return value

__all__ = ["SimulationStudioService"]
