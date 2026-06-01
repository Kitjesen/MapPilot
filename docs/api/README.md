# LingTu API 文档

> 自动提取的 API 文档，与代码同源。

## 文档索引

| 文档 | 描述 | 来源 |
|------|------|------|
| [MCP 工具](mcp_tools.md) | 所有 `@skill` 装饰器暴露的 MCP 工具（JSON-RPC） | 扫描 `src/` 下所有 Module 文件 |
| [Gateway REST API](gateway_rest.md) | GatewayModule REST 端点（FastAPI, port 5050） | 扫描 `src/gateway/routes/` 路由注册 |

## 生成命令

```bash
# 重新生成两份文档
python scripts/extract_api_docs.py
```

## 架构分层

```
L0  Safety    REST: /api/v1/stop, /api/v1/mode
L1  Hardware  MCP tools via driver backends (thunder/stub/sim)
L2  Maps      REST: /api/v1/maps/*, /api/v1/slam/*
L3  Perception REST: /api/v1/scene_graph, /api/v1/camera/*
    Memory    REST: /api/v1/locations, /api/v1/memory/*
L4  Decision  REST: /api/v1/instruction, /api/v1/goal
L5  Planning  REST: /api/v1/navigation/*
L6  Interface REST: /api/v1/app/*, /api/v1/health
               MCP: http://<robot>:8090/mcp
```

## 协议

- **REST**: JSON over HTTP, Pydantic v2 request/response models, 422 on validation error
- **SSE**: `GET /api/v1/events` — `text/event-stream`
- **WebSocket**: `ws://<robot>:5050/ws/teleop` — teleop joystick + camera
- **MCP**: `http://<robot>:8090/mcp` — JSON-RPC 2.0
