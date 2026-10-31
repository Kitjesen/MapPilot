"""Local synthetic SHM and Gateway fixtures for video-check.html; no robot control."""

import argparse
import asyncio
import subprocess
import sys
import time
from pathlib import Path
from types import SimpleNamespace


def source(args: argparse.Namespace) -> None:
    """Drive the existing native 16x16 color fixture until its control file says quit."""
    fixture = subprocess.Popen(
        [str(args.fixture.resolve()), args.shm],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        text=True,
        bufsize=1,
    )
    try:
        ready = fixture.stdout.readline().strip()
        if ready != f"ready {args.shm}":
            raise RuntimeError(f"video_fixture did not start: {ready!r}")
        print(ready, flush=True)
        while True:
            command = args.control.read_text().strip() if args.control.exists() else "501"
            if command == "quit":
                break
            if command not in {"", "pause"}:
                generation = int(command)
                if generation <= 0:
                    raise ValueError("control must contain a positive generation, pause, or quit")
                fixture.stdin.write(f"publish {generation} 1 0 rgb8\n")
                fixture.stdin.flush()
                response = fixture.stdout.readline().strip()
                if not response.startswith(f"published {generation} "):
                    raise RuntimeError(f"video_fixture publish failed: {response!r}")
            time.sleep(0.05)
    finally:
        if fixture.poll() is None:
            fixture.communicate("quit\n")
        else:
            fixture.communicate()


def gateway(args: argparse.Namespace) -> None:
    """Mount real signalling routes and an independent blue JPEG fallback on localhost."""
    sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "src"))
    import uvicorn
    from fastapi import FastAPI, WebSocket, WebSocketDisconnect

    from gateway.routes.video import register_video_routes

    image = Path(__file__).with_name("video-fallback.jpg").read_bytes()
    app = FastAPI()
    register_video_routes(
        app,
        SimpleNamespace(
            _go2rtc_upstream=args.go2rtc_url,
            _webrtc_upstream=args.native_endpoint,
            _webrtc_ice_servers=[],
            _webrtc_rotation=0,
            _webrtc_status_file=args.native_status_file,
        ),
    )

    @app.websocket("/ws/camera")
    async def fallback(websocket: WebSocket):
        await websocket.accept()
        try:
            while True:
                await websocket.send_bytes(image)
                await asyncio.sleep(0.1)
        except WebSocketDisconnect:
            pass

    uvicorn.run(app, host="127.0.0.1", port=args.port)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    source_parser = commands.add_parser("source", help="Linux: continuously publish synthetic color SHM")
    source_parser.add_argument("--fixture", type=Path, required=True, help="existing CMake video_fixture binary")
    source_parser.add_argument("--control", type=Path, required=True, help="positive generation, pause, or quit")
    source_parser.add_argument("--shm", default="/lingtu_video_browser_check", help="dedicated POSIX SHM name")
    source_parser.set_defaults(run=source)
    gateway_parser = commands.add_parser("gateway", help="serve real signalling and synthetic JPEG on localhost")
    gateway_parser.add_argument("--port", type=int, default=5094)
    gateway_parser.add_argument("--native-endpoint", default="", help="existing native WHEP endpoint; empty uses go2rtc")
    gateway_parser.add_argument("--native-status-file", default="", help="native --status-file snapshot, when configured")
    gateway_parser.add_argument("--go2rtc-url", default="http://127.0.0.1:1984")
    gateway_parser.set_defaults(run=gateway)
    args = parser.parse_args()
    if args.command == "source" and sys.platform != "linux":
        parser.error("source requires Linux POSIX SHM; run it in WSL on Windows")
    try:
        args.run(args)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
