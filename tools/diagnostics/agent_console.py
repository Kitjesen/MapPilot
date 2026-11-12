"""Run the production Gateway and Agent locally for account/chat verification.

This is a no-robot diagnostic Host, not a real/sim Product. It serves web/dist
and the production conversation routes without replacing their responses.
"""

import argparse
import logging
from pathlib import Path

from fastapi.responses import JSONResponse

from decision.modules.agent_planner import AgentPlannerModule
from gateway.gateway_module import GatewayModule
from lingtu.assembly.config.product import _model_config
from runtime.host import HostBuilder


def start_console(port: int = 4326):
    settings = _model_config(Path(__file__).resolve().parents[2] / "config" / "system", "kimi")
    host = (HostBuilder()
            .add(AgentPlannerModule, llm_backend="kimi", **settings)
            # This HTTP diagnostic must not compete with an existing Host's RPC port.
            .add(GatewayModule, host="127.0.0.1", port=port, grpc_port=0)
            .wire("AgentPlannerModule", "agent_message", "GatewayModule", "agent_message")
            .require_modules("AgentPlannerModule", "GatewayModule")
            .build())
    gateway = host.get_module("GatewayModule")
    gateway._defer_server = True
    try:
        host.start()
    except Exception:
        host.stop()
        raise

    @gateway._app.middleware("http")
    async def local_writes(request, call_next):
        # This Host has no Product or robot; only conversation/account writes are meaningful.
        if request.method not in {"GET", "HEAD", "OPTIONS"} and not request.url.path.startswith("/api/v1/agent/"):
            return JSONResponse({"error": "local_agent_console", "message": "当前本机服务仅支持模型连接与对话操作。"}, status_code=409)
        return await call_next(request)

    return host


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=4326)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO)
    host = start_console(args.port)
    try:
        print(f"LingTu real account/chat verification: http://127.0.0.1:{args.port}/ (no robot)", flush=True)
        host.get_module("GatewayModule")._run_server()
    finally:
        host.stop()


if __name__ == "__main__":
    main()
