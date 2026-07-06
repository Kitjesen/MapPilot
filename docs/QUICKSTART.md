# LingTu Quick Start

## Install

```bash
cd ~/data/inovxio/lingtu
uv sync --locked
```

`uv.lock` pins the resolved Python packages. `.python-version` pins the
default interpreter to Python 3.10.12, matching the S100P runtime.

Run commands through `uv run --locked` so deployment fails fast if
`pyproject.toml` and `uv.lock` drift:

```bash
uv run --locked python lingtu.py --list
```

Install extras only for the profile you are running:

```bash
uv sync --locked --extra vision --extra ml --extra llm --extra nlp
uv sync --locked --extra perception --extra vector   # heavy semantic deps
uv sync --locked --extra dev                         # test/lint tooling
```

After sync the `lingtu` console script (`lingtu_cli:main`) is available in the
uv-managed environment and is equivalent to `uv run --locked python lingtu.py`.

## Robot environment (Thunder/S100P)

```bash
# Native product path
cd ~/data/SLAM/navigation
bash scripts/lingtu status
```

The field robot runs the product chain through native systemd services:
`lingtu-livox-dds`, `lingtu-slam-dds`, `lingtu-nav-dds`, and `lingtu`.
Do not source ROS 2 or a colcon overlay for the normal navigation path.
ROS 2 setup is only for explicit compatibility checks.

## Profiles

The canonical profile source is `src/runtime/profiles/catalog/`.
`cli/profiles_data.py` is a compatibility export for CLI imports.
`uv run --locked lingtu --list` lists product profiles.
`uv run --locked lingtu --list --all` lists the full registered catalog.

| Command | Purpose | Hardware |
|---------|---------|----------|
| `lingtu stub` | framework only | none |
| `lingtu dev` | semantic pipeline, mock LLM | none |
| `lingtu sim_nav` | pure-Python navigation sim | none |
| `lingtu sim` | MuJoCo full stack | none (CPU MuJoCo) |
| `lingtu map` | build a map with native SLAM save + map optimization | LiDAR + IMU |
| `lingtu nav` | navigate using a saved map (OctoPlanner3D planner) | LiDAR + IMU + camera |
| `lingtu explore` | wavefront frontier exploration | LiDAR + IMU + camera |
| `lingtu tare_explore` | ROS-free traversable frontier exploration with OctoPlanner3D | LiDAR + IMU + camera |

## Typical session

### 1. Build a map

```bash
lingtu map
```

Drive the robot manually around the area, then save through the operations CLI
or the Gateway:

```
lingtu map save building_a
```

The map package must be treated as a directory, not one file. A complete
navigation-ready map may contain `map.pcd`, `map.raw.pcd`, `patches/*.pcd`,
`poses.txt`, `map_optimization.json`, `metadata.json`, `octomap.ot`, and
`occupancy.npz`. `map.pcd` is the optimized navigation map. `map.raw.pcd`
keeps the raw SLAM/builder output. `patches/*.pcd` and `poses.txt` are the
keyframe bundle used by native save-time optimization and occupancy raycasting.
`octomap.ot` plus `metadata.json` is the OctoPlanner3D artifact gate.
`tomogram.pickle` is optional legacy/PCT data.

### 2. Navigate

```bash
lingtu nav
```

Then in the REPL:

```
> map use building_a       # set the active map package
> navigate 5 3             # x, y in map frame
> go charging station      # natural-language instruction (semantic planner)
> stop                     # zero cmd_vel + cancel mission
> status                   # mission state, ports, hz
> teleop status            # teleop client count + lease
```

### 3. Stop the daemon

```bash
lingtu stop
```

## REPL command summary

```
navigate / nav x y [z]    Pose goal in map frame
go <text>                 Natural-language instruction
agent <text>              Multi-turn AgentLoop (7 LLM tools, 10 step cap)
stop                      Emergency stop (publishes 2 to all stop_signal ports)
cancel                    Cancel current mission
status / s                Module list + mission state
health / h                System health report
map list|save|use|build|delete       Map lifecycle
smap status|rooms|save|load|query    Semantic map (RoomObjectKG + topology)
vmem query|stats          Vector memory (CLIP + ChromaDB or numpy fallback)
teleop status|release     Teleop lease and clients
rerun on|off|status       Rerun visualization bridge (port 9090 by default)
watch / w [interval]      Auto-refresh status (default 2 s)
live                      Full-screen dashboard with hotkeys
module / m <name>         Inspect a single module
connections / c           List all wires
log debug|info|warning|error    Set root log level
config                    Print resolved profile
quit / q / exit           Leave the REPL
```

The REPL is `cli/repl.py`. It only runs when `stdin` is a TTY and
`--no-repl` was not passed.

## Lifecycle commands (run without entering the REPL)

```bash
lingtu stop                Stop the running daemon (SIGTERM)
lingtu restart             Stop and relaunch with the same argv
lingtu status              External run state (add --json)
lingtu show-config nav     Resolved config (add --json)
lingtu log -f              Follow the run log
lingtu doctor              scripts/diagnostics/doctor.py
lingtu rerun               scripts/visualization/rerun_gateway_live.py
lingtu --list              List profiles
lingtu --version           Print version
```

`lingtu --daemon` forks via `setsid`; the PID and run state live in
`.lingtu/run.pid` and `.lingtu/run.json` (see `cli/run_state.py`).

## Overrides

Common command-line overrides:

```bash
lingtu nav --llm mock              # bypass real LLM
lingtu nav --planner pct           # explicit legacy/PCT experiment only
lingtu nav --detector yoloe        # alternative detector
lingtu sim --no-native             # disable C++ autonomy stack
lingtu nav --no-semantic           # geometric only
```

## Network ports

| Port | Service | Module |
|------|---------|--------|
| 5050 | REST + SSE + `/ws/teleop` + `/ws/camera` | `GatewayModule` (and `TeleopModule` shares it) |
| 8090 | MCP JSON-RPC | `MCPServerModule` |
| 9090 | Rerun web UI (when `--rerun`) | `RerunBridgeModule` |

## MCP

```bash
codex mcp add --transport http lingtu http://192.168.66.13:8090/mcp
```

`MCPServerModule` auto-discovers `@skill` methods from every module in
`SystemHandle`. The current API reference is `docs/api/mcp_tools.md`.

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `ros2: command not found` | Only relevant for explicit compatibility checks; native DDS navigation does not need ROS 2 sourced |
| Port 5050 / 8090 already in use | `cli.runtime_extra.kill_residual_ports` runs `fuser -k` on startup; otherwise kill manually |
| `No active map` | `lingtu map` first, then `map save <name>` and `map use <name>` |
| Slow Path / LLM unavailable | Set `MOONSHOT_API_KEY` (Kimi), `DASHSCOPE_API_KEY` (Qwen), `OPENAI_API_KEY` or `ANTHROPIC_API_KEY`, or run with `--llm mock` |
| SSH session drops | `lingtu nav --daemon`, then `lingtu log -f` and `lingtu stop` |
