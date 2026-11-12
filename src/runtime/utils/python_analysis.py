"""Run operator-approved Python against captured observations in a temporary directory."""

import asyncio
import json
import os
import signal
import subprocess
import sys
import tempfile
from pathlib import Path


async def stop(process):
    # Windows venv launchers have a child interpreter; stopping only the launcher leaves code running.
    if os.name == "nt":
        kill = await asyncio.create_subprocess_exec("taskkill", "/PID", str(process.pid), "/T", "/F",
            stdout=asyncio.subprocess.DEVNULL, stderr=asyncio.subprocess.DEVNULL,
            creationflags=subprocess.CREATE_NO_WINDOW)
        await kill.wait()
    else:
        os.killpg(process.pid, signal.SIGKILL)
    await process.wait()


async def run(code, observations):
    # This is trusted Host code execution, not a security sandbox.
    with tempfile.TemporaryDirectory(prefix="lingtu-analysis-") as directory:
        script = "import json, sys\ndata = json.load(sys.stdin)\n" + code
        path = Path(directory) / "analysis.py"
        path.write_text(script, encoding="utf-8")
        with tempfile.TemporaryFile() as output, tempfile.TemporaryFile() as errors:
            process = await asyncio.create_subprocess_exec(
                sys.executable, "-X", "utf8", "-I", "-u", str(path), cwd=directory,
                stdin=asyncio.subprocess.PIPE, stdout=output, stderr=errors,
                **({"creationflags": subprocess.CREATE_NO_WINDOW} if os.name == "nt" else {"start_new_session": True}),
            )
            timed_out = False
            try:
                await asyncio.wait_for(process.communicate(json.dumps(observations, allow_nan=False).encode()), timeout=10)
            except asyncio.TimeoutError:
                timed_out = True
                await stop(process)
            except BaseException:
                if process.returncode is None:
                    await stop(process)
                raise
            output.seek(0)
            errors.seek(0)
            stdout, stderr = output.read(65537), errors.read(65537)
    return {"success": not timed_out and process.returncode == 0, "command": code,
            "stdout": stdout[:65536].decode("utf-8", errors="replace"),
            "stderr": stderr[:65536].decode("utf-8", errors="replace"), "exit_code": process.returncode,
            "output_truncated": len(stdout) > 65536 or len(stderr) > 65536,
            **({"error": "Analysis exceeded 10 seconds"} if timed_out else {})}
