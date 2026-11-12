# LingTu Python SDK distribution

This package provides the synchronous and asynchronous `lingtu.sdk` clients and
the `lingtu-sdk` command. It supports Python 3.10 or newer. HTTP, downloads and
SSE use the Python standard library. Live WebSocket sessions use the optional
`websocket` extra.

Build the wheel and source archive from the LingTu repository root with the
installed `setuptools>=77` development tool.

```sh
python tools/sdk/package_python.py
```

The build stages the current authoritative `src/lingtu/sdk` implementation and
the minimal `lingtu` package initializer into `build/sdk/python`. It reads the
version from the repository's main `pyproject.toml`. Distribution metadata lives
in the `sdk/python/pyproject.toml.in` template. Use the packaging script to
prepare the complete installable project.

The resulting wheel and source archive are in `build/sdk/dist`. Both contain the
SDK sources and can be installed outside the repository.

```sh
python -m pip install --force-reinstall --no-deps build/sdk/dist/lingtu_sdk-2.3.0-py3-none-any.whl
lingtu-sdk --help
```

Install the extra when using live point clouds, camera frames or teleoperation.

```sh
python -m pip install --force-reinstall "build/sdk/dist/lingtu_sdk-2.3.0-py3-none-any.whl[websocket]"
```

```python
from lingtu.sdk import AsyncLingTuClient, LingTuClient

robot = LingTuClient(host="127.0.0.1", port=5050, api_key="your-key")
print(robot.system.health())
```

Use `base_url="https://robot-host:7443"` for an HTTPS origin with either client.
The CLI accepts the same origin through `--base-url`. HTTPS and WSS retain normal
certificate verification. Python's standard trust configuration, including
process-local `SSL_CERT_FILE`, applies to both transports.

Session construction is lazy. Opening a context connects to the declared
Gateway endpoint. Each received preview is the original binary message.

```python
with robot.watch_cloud() as frames:
    frame = next(frames)

with robot.watch_camera(viewer_token="viewer-token") as frames:
    jpeg = next(frames)
```

`AsyncLingTuClient` exposes matching factories used with `async with` and
`async for`. Camera HTTP methods also accept `viewer_token`. The media Bearer
credential is separate from the Gateway API key.

`robot.teleop()` creates an explicit operator session. `move()` requests a
fresh input window and sends one velocity sample. It returns the original
Gateway `ingress_ack` or rejection. `hold()` releases the deadman and returns
the hold receipt. Closing or cancelling the session closes the WebSocket and
lets the Gateway perform its existing disconnect hold and authority release.
Observe native state for executed motion and physical stopping evidence.

The package shares the `lingtu` import namespace with the full runtime
distribution. Install the standalone SDK in the external application's own
environment.

The five domain views also cover ProductControl status/switch/operation lookup,
saved recording lists, files and historical playback queries, and inspection
route editing, evidence review and downloads. Synchronous and asynchronous
clients expose the same operations. Product switching requires the expected
session identity and retains the owning transaction's complete response.

Local builds currently share the main project's 2.3.0 version. The install
commands force replacement so that a newly built archive updates an existing
local build with the same version.
