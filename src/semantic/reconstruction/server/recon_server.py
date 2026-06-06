"""recon_server.py — LingTu reconstruction server (FastAPI).

Receives RGB-D keyframe batches from robots running
ReconKeyframeExporterModule and dispatches reconstruction jobs to
pluggable backends (TSDF, Open3D, Nerfstudio, GSFusion).

API:
    POST /api/v1/keyframes          — upload a batch of keyframes
    POST /api/v1/jobs/{session_id}/trigger   — trigger reconstruction now
    GET  /api/v1/jobs/{session_id}  — job status + result path
    GET  /api/v1/jobs               — list all sessions
    GET  /api/v1/backends           — list available backends
    GET  /health                    — liveness probe

Quick start:
    # CPU-only TSDF (default, no GPU needed)
    python -m semantic.reconstruction.server.recon_server

    # Open3D pipeline
    python -m semantic.reconstruction.server.recon_server --backend open3d

    # NeRF via nerfstudio (needs GPU + nerfstudio installed)
    python -m semantic.reconstruction.server.recon_server --backend nerfstudio

    # GSFusion (needs CUDA GPU + GSFUSION_BIN env var)
    python -m semantic.reconstruction.server.recon_server --backend gsfusion

    # Custom port / output dir
    python -m semantic.reconstruction.server.recon_server \\
        --host 0.0.0.0 --port 7890 --output-dir /data/reconstructions

Environment variables:
    RECON_BACKEND       default backend name (overridden by --backend)
    RECON_OUTPUT_DIR    base output directory (overridden by --output-dir)
    GSFUSION_BIN        path to GSFusion binary

Requirements (server-side):
    pip install fastapi uvicorn python-multipart
    pip install open3d          # for tsdf / open3d backends
    pip install nerfstudio      # for nerfstudio backend (+ CUDA)
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any

from core.backend_status import BackendStatus
from core.msgs.numpy_compat import np

logger = logging.getLogger(__name__)


# ── Job state machine ────────────────────────────────────────────────────────

class JobStatus(str, Enum):
    COLLECTING = "collecting"
    QUEUED     = "queued"
    RUNNING    = "running"
    DONE       = "done"
    FAILED     = "failed"


@dataclass
class ReconJob:
    session_id: str
    backend_name: str
    output_dir: Path
    status: JobStatus = JobStatus.COLLECTING
    created_at: float = field(default_factory=time.time)
    started_at: float | None = None
    finished_at: float | None = None
    num_keyframes: int = 0
    result: dict | None = None
    error: str | None = None

    def to_dict(self) -> dict:
        return {
            "session_id":    self.session_id,
            "backend":       self.backend_name,
            "status":        self.status.value,
            "created_at":    self.created_at,
            "started_at":    self.started_at,
            "finished_at":   self.finished_at,
            "num_keyframes": self.num_keyframes,
            "result":        self.result,
            "error":         self.error,
            "output_dir":    str(self.output_dir),
        }


# ── Server core ──────────────────────────────────────────────────────────────

class ReconServer:
    """Manages keyframe ingestion and reconstruction job lifecycle.

    Can be used standalone (without FastAPI) for testing:
        server = ReconServer(backend="tsdf", output_dir=Path("/tmp/recon"))
        server.ingest_keyframes(session_id, keyframe_list)
        server.trigger_reconstruction(session_id)
    """

    def __init__(
        self,
        backend: str = "tsdf",
        output_dir: Path = Path("outputs/reconstruction"),
        max_workers: int = 2,
        auto_trigger_frames: int = 0,  # 0 = manual trigger only
    ) -> None:
        from .backends.registry import list_backends

        self._default_backend  = backend
        self._backend_status   = BackendStatus.configured_as(backend)
        self._output_dir       = Path(output_dir)
        self._output_dir.mkdir(parents=True, exist_ok=True)
        self._auto_trigger     = int(auto_trigger_frames)

        # jobs: session_id → ReconJob
        self._jobs: dict[str, ReconJob] = {}
        # keyframes: session_id → list of Keyframe
        self._keyframes: dict[str, list] = {}
        self._lock = asyncio.Lock() if _in_async_context() else None

        self._executor = ThreadPoolExecutor(max_workers=max_workers)
        self._loop: asyncio.AbstractEventLoop | None = None

        # Verify backend is known
        try:
            available = list_backends()
            if backend not in available:
                logger.warning("Backend '%s' not in registry %s", backend, available)
                self._backend_status.mark_degraded("backend not registered")
        except Exception:
            self._backend_status.mark_degraded("backend registry unavailable")

    def set_event_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        self._loop = loop

    # ── Keyframe ingestion ───────────────────────────────────────────────────

    def ingest_keyframe_files(
        self,
        session_id: str,
        frame_idx: int,
        timestamp: float,
        pose_json: str,
        intrinsics_json: str,
        color_path: Path,
        depth_path: Path,
    ) -> None:
        """Register one keyframe from already-saved image files."""
        from .backends.base import Keyframe

        pose_data = json.loads(pose_json)
        intr_data = json.loads(intrinsics_json)

        # Reconstruct 4×4 camera-to-world matrix
        if "r00" in pose_data:
            R = np.array([
                [pose_data["r00"], pose_data["r01"], pose_data["r02"]],
                [pose_data["r10"], pose_data["r11"], pose_data["r12"]],
                [pose_data["r20"], pose_data["r21"], pose_data["r22"]],
            ], dtype=np.float64)
            t = np.array([pose_data["tx"], pose_data["ty"], pose_data["tz"]],
                         dtype=np.float64)
            T = np.eye(4, dtype=np.float64)
            T[:3, :3] = R
            T[:3, 3]  = t
        else:
            # Fallback: identity
            T = np.eye(4, dtype=np.float64)
            T[0, 3] = pose_data.get("tx", 0)
            T[1, 3] = pose_data.get("ty", 0)
            T[2, 3] = pose_data.get("tz", 0)

        kf = Keyframe(
            frame_idx=frame_idx,
            timestamp=timestamp,
            pose=T,
            fx=float(intr_data.get("fx", 615.0)),
            fy=float(intr_data.get("fy", 615.0)),
            cx=float(intr_data.get("cx", 320.0)),
            cy=float(intr_data.get("cy", 240.0)),
            width=int(intr_data.get("w", 640)),
            height=int(intr_data.get("h", 480)),
            color_path=color_path,
            depth_path=depth_path,
        )

        if session_id not in self._keyframes:
            self._keyframes[session_id] = []
            self._jobs[session_id] = ReconJob(
                session_id=session_id,
                backend_name=self._default_backend,
                output_dir=self._output_dir / session_id,
            )
        self._keyframes[session_id].append(kf)
        self._jobs[session_id].num_keyframes = len(self._keyframes[session_id])

        # Auto-trigger if enough frames accumulated
        if self._auto_trigger > 0:
            n = len(self._keyframes[session_id])
            if n >= self._auto_trigger and n % self._auto_trigger == 0:
                self._submit_job(session_id)

    # ── Job management ───────────────────────────────────────────────────────

    def trigger_reconstruction(
        self,
        session_id: str,
        backend: str | None = None,
        options: dict | None = None,
    ) -> ReconJob:
        """Submit a reconstruction job for the given session."""
        if session_id not in self._jobs:
            raise KeyError(f"Unknown session: {session_id}")
        job = self._jobs[session_id]
        if job.status in (JobStatus.RUNNING, JobStatus.QUEUED):
            return job
        if backend:
            job.backend_name = backend
        return self._submit_job(session_id, options=options or {})

    def _submit_job(
        self, session_id: str, options: dict | None = None
    ) -> ReconJob:
        job = self._jobs[session_id]
        job.status = JobStatus.QUEUED
        keyframes = list(self._keyframes.get(session_id, []))
        opts = options or {}

        if self._loop is not None:
            # Inside asyncio event loop — run in thread pool
            asyncio.run_coroutine_threadsafe(
                self._run_job_async(job, keyframes, opts),
                self._loop,
            )
        else:
            # Sync mode (testing / CLI)
            self._executor.submit(self._run_job_sync, job, keyframes, opts)

        return job

    async def _run_job_async(
        self, job: ReconJob, keyframes: list, options: dict
    ) -> None:
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(
            self._executor, self._run_job_sync, job, keyframes, options
        )

    def _run_job_sync(
        self, job: ReconJob, keyframes: list, options: dict
    ) -> None:
        from .backends.registry import get_backend

        job.status     = JobStatus.RUNNING
        job.started_at = time.time()

        try:
            backend_cls = get_backend(job.backend_name)
            backend     = backend_cls()
            job.output_dir.mkdir(parents=True, exist_ok=True)
            result = backend.reconstruct(keyframes, job.output_dir, **options)
            job.result      = result.to_dict()
            job.status      = JobStatus.DONE if result.success else JobStatus.FAILED
            job.error       = None if result.success else result.message
        except Exception as exc:
            logger.exception("Reconstruction job failed for session %s", job.session_id)
            job.status = JobStatus.FAILED
            job.error  = str(exc)
        finally:
            job.finished_at = time.time()

    def get_job(self, session_id: str) -> ReconJob | None:
        return self._jobs.get(session_id)

    def list_jobs(self) -> list[dict]:
        return [j.to_dict() for j in self._jobs.values()]

    def health(self) -> dict[str, Any]:
        return {
            "status": "ok",
            **self._backend_status.as_health_fields(),
        }


# ── FastAPI application ──────────────────────────────────────────────────────

def create_app(
    backend: str = "tsdf",
    output_dir: str = "outputs/reconstruction",
    auto_trigger_frames: int = 0,
) -> Any:
    """Create and return the FastAPI application.

    This function avoids importing FastAPI at module level so the
    server package can be imported without fastapi installed.
    """
    try:
        from fastapi import FastAPI, File, Form, HTTPException, UploadFile
        from fastapi.responses import FileResponse, JSONResponse  # noqa: F401
    except ImportError as exc:
        raise ImportError(
            "FastAPI is required for the reconstruction server. "
            "Run: pip install fastapi uvicorn python-multipart"
        ) from exc

    app = FastAPI(
        title="LingTu Reconstruction Server",
        description=(
            "Receives RGB-D keyframes from LingTu robots and runs "
            "3D reconstruction (TSDF, NeRF, Gaussian Splatting)."
        ),
        version="1.0.0",
    )

    server = ReconServer(
        backend=backend,
        output_dir=Path(output_dir),
        auto_trigger_frames=auto_trigger_frames,
    )

    @app.on_event("startup")
    async def _startup():
        server.set_event_loop(asyncio.get_event_loop())

    @app.get("/health")
    async def health():
        return server.health()

    @app.get("/api/v1/backends")
    async def get_backends():
        from .backends.registry import list_backends, get_backend
        result = []
        for name in list_backends():
            cls  = get_backend(name)
            inst = cls()
            ok, reason = inst.check_dependencies()
            result.append({"name": name, "available": ok, "reason": reason})
        return {"backends": result}

    @app.post("/api/v1/keyframes")
    async def upload_keyframes(
        session_id:  str = Form(...),
        frame_idx:   int = Form(...),
        timestamp:   float = Form(...),
        pose_json:   str = Form(...),
        intrinsics:  str = Form(...),
        color_jpg:   UploadFile = File(...),
        depth_png:   UploadFile = File(...),
    ):
        """Receive one keyframe from the robot exporter."""
        # Save images to disk
        session_dir = Path(output_dir) / session_id / "frames"
        session_dir.mkdir(parents=True, exist_ok=True)

        color_path = session_dir / f"color_{frame_idx:06d}.jpg"
        depth_path = session_dir / f"depth_{frame_idx:06d}.png"

        color_path.write_bytes(await color_jpg.read())
        depth_path.write_bytes(await depth_png.read())

        server.ingest_keyframe_files(
            session_id=session_id,
            frame_idx=frame_idx,
            timestamp=timestamp,
            pose_json=pose_json,
            intrinsics_json=intrinsics,
            color_path=color_path,
            depth_path=depth_path,
        )

        job = server.get_job(session_id)
        return {
            "session_id":    session_id,
            "frame_idx":     frame_idx,
            "total_frames":  job.num_keyframes if job else 1,
            "status":        job.status.value if job else "collecting",
        }

    @app.post("/api/v1/jobs/{session_id}/trigger")
    async def trigger_job(
        session_id: str,
        backend_override: str | None = None,
    ):
        """Trigger reconstruction for the given session."""
        try:
            job = server.trigger_reconstruction(
                session_id, backend=backend_override
            )
            return job.to_dict()
        except KeyError as exc:
            raise HTTPException(status_code=404, detail=str(exc))

    @app.get("/api/v1/jobs/{session_id}")
    async def get_job(session_id: str):
        job = server.get_job(session_id)
        if job is None:
            raise HTTPException(status_code=404, detail=f"Session {session_id} not found")
        return job.to_dict()

    @app.get("/api/v1/jobs/{session_id}/download")
    async def download_result(session_id: str):
        """Download the reconstruction output file (PLY / mesh / etc.)."""
        job = server.get_job(session_id)
        if job is None:
            raise HTTPException(status_code=404, detail="Session not found")
        if job.status != JobStatus.DONE:
            raise HTTPException(status_code=409,
                                detail=f"Job not done (status={job.status.value})")
        result = job.result or {}
        out_path = result.get("output_path")
        if not out_path or not Path(out_path).exists():
            raise HTTPException(status_code=404, detail="Output file not found")
        return FileResponse(out_path, filename=Path(out_path).name)

    @app.get("/api/v1/jobs")
    async def list_jobs():
        return {"jobs": server.list_jobs()}

    return app


# ── CLI entry point ──────────────────────────────────────────────────────────

def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(
        description="LingTu 3D reconstruction server",
        formatter_class=argparse.RawTextHelpFormatter,
        epilog="""
Available backends:
  tsdf         TSDF volumetric fusion (Open3D, CPU only) — default
  open3d       Open3D multi-step pipeline (CPU)
  nerfstudio   NeRF or 3DGS via nerfstudio CLI (requires GPU)
  gsfusion     GSFusion online Gaussian Splatting + TSDF (requires CUDA + binary)

Examples:
  python -m semantic.reconstruction.server.recon_server
  python -m semantic.reconstruction.server.recon_server --backend nerfstudio --port 7891
  GSFUSION_BIN=/opt/gsfusion/build/GSFusion python -m ... --backend gsfusion
""",
    )
    parser.add_argument("--backend",     default=os.environ.get("RECON_BACKEND", "tsdf"))
    parser.add_argument("--output-dir",  default=os.environ.get("RECON_OUTPUT_DIR", "outputs/reconstruction"))
    parser.add_argument("--host",        default="0.0.0.0")
    parser.add_argument("--port",        type=int, default=7890)
    parser.add_argument("--auto-trigger", type=int, default=0,
                        help="Automatically trigger reconstruction every N keyframes (0=manual)")
    parser.add_argument("--workers",     type=int, default=1)
    parser.add_argument("--log-level",   default="INFO")
    parser.add_argument("--list-backends", action="store_true",
                        help="Print available backends and exit")
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
    )

    if args.list_backends:
        from .backends.registry import list_backends, get_backend
        print("Available reconstruction backends:")
        for name in list_backends():
            cls  = get_backend(name)
            inst = cls()
            ok, reason = inst.check_dependencies()
            mark = "✓" if ok else "✗"
            print(f"  {mark} {name:20s}  {reason}")
        return

    try:
        import uvicorn
    except ImportError:
        raise SystemExit(
            "uvicorn not installed — run: pip install uvicorn"
        )

    app = create_app(
        backend=args.backend,
        output_dir=args.output_dir,
        auto_trigger_frames=args.auto_trigger,
    )

    logger.info("Starting reconstruction server: backend=%s port=%d output=%s",
                args.backend, args.port, args.output_dir)
    uvicorn.run(app, host=args.host, port=args.port, workers=args.workers)


def _in_async_context() -> bool:
    try:
        asyncio.get_running_loop()
        return True
    except RuntimeError:
        return False


if __name__ == "__main__":
    main()
