# LingTu Quick Start

## Install

```bash
cd ~/data/inovxio/lingtu
pip install -e .
```

After install the `lingtu` console script (`lingtu_cli:main`) is on `$PATH`
and is equivalent to `python lingtu.py`.

## Robot environment (S100P)

```bash
# Once per shell — usually added to ~/.bashrc
source /opt/ros/humble/setup.bash
source ~/data/inovxio/lingtu/install/setup.bash   # only after `make build`
```

Only ROS2 + the colcon overlay are sourced; LingTu does not require any
extra `setup.bash` or env script.

## Profiles

The full table is in [`archive/AGENTS.md`](./archive/AGENTS.md) (section 3 "Profiles").
The canonical source is `src/core/runtime_profiles.py`; `cli/profiles_data.py`
keeps compatibility exports for CLI callers.

| Command | Purpose | Hardware |
|---------|---------|----------|
| `lingtu stub` | framework only | none |
| `lingtu dev` | semantic pipeline, mock LLM | none |
| `lingtu sim_nav` | pure-Python navigation sim | none |
| `lingtu sim` | MuJoCo full stack | none (CPU MuJoCo) |
| `lingtu map` | build a map with Fast-LIO2 + PGO | LiDAR + IMU |
| `lingtu nav` | navigate using a saved map (PCT planner) | LiDAR + IMU + camera |
| `lingtu explore` | wavefront frontier exploration | LiDAR + IMU + camera |
| `lingtu tare_explore` | CMU TARE hierarchical explorer | LiDAR + IMU + camera + built TARE binary |

`lingtu --list` lists what is currently registered.

## Typical session

### 1. Build a map

```bash
lingtu map
```

Drive the robot manually around the area, then in the REPL:

```
> map save building_a
```

`MapManagerModule` writes `map.pcd`, an offline DUFOMap pass cleans dynamic
obstacles, and the gateway publishes a `tomogram.pickle` and an
`occupancy.npz` for the planner. Default location is `~/data/nova/maps/<name>/`
(override with `NAV_MAP_DIR`).

### 2. Navigate

```bash
lingtu nav
```

Then in the REPL:

```
> map use building_a       # set the active tomogram
> navigate 5 3             # x, y in map frame
> go 体育馆                  # natural-language instruction (semantic planner)
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
lingtu doctor              scripts/doctor.py
lingtu rerun               scripts/rerun_live.py
lingtu --list              List profiles
lingtu --version           Print version
```

`lingtu --daemon` forks via `setsid`; the PID and run state live in
`.lingtu/run.pid` and `.lingtu/run.json` (see `cli/run_state.py`).

## Overrides

Any profile field can be overridden on the command line. The full set is
listed in `archive/AGENTS.md` section 20. Common ones:

```bash
lingtu nav --llm mock              # bypass real LLM
lingtu nav --planner astar         # force pure-Python A*
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
claude mcp add --transport http lingtu http://192.168.66.190:8090/mcp
```

`MCPServerModule` auto-discovers `@skill` methods from every module in
`SystemHandle`. The current tool catalogue is in `archive/AGENTS.md` section 18.

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `ros2: command not found` | `source /opt/ros/humble/setup.bash` |
| Port 5050 / 8090 already in use | `cli.runtime_extra.kill_residual_ports` runs `fuser -k` on startup; otherwise kill manually |
| `No active map` | `lingtu map` first, then `map save <name>` and `map use <name>` |
| Slow Path / LLM unavailable | Set `MOONSHOT_API_KEY` (Kimi), `DASHSCOPE_API_KEY` (Qwen), `OPENAI_API_KEY` or `ANTHROPIC_API_KEY`, or run with `--llm mock` |
| SSH session drops | `lingtu nav --daemon`, then `lingtu log -f` and `lingtu stop` |
| TARE binary missing | `scripts/build/fetch_ortools.sh && scripts/build/build_tare.sh` |
