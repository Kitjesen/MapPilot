# LingTu MCP Tools

LingTu exposes robot capabilities through `MCPServerModule` as JSON-RPC over
HTTP:

```text
POST http://<robot>:8090/mcp
```

The server follows the MCP tool flow:

| Method | Purpose |
| --- | --- |
| `initialize` | Return protocol and server capabilities |
| `notifications/initialized` | Client initialization acknowledgement |
| `tools/list` | Return the current tool descriptors |
| `tools/call` | Call one tool by name with JSON arguments |

Tools are not maintained as a static hand-written list. At runtime,
`MCPServerModule.on_system_modules()` scans every loaded Module for `@skill`
methods and builds the `tools/list` response from method signatures and
docstrings.

## Tool Ownership

Common tool groups:

| Owner | Example Tools | Notes |
| --- | --- | --- |
| `MCPServerModule` | `get_health`, `list_modules`, `get_config`, `get_robot_position`, `get_scene_graph`, `detect_objects`, `query_memory`, `list_tagged_locations`, `tag_location`, `navigate_to_object`, `send_instruction`, `emergency_stop`, `set_mode`, `switch_backend` | Built-in system, perception, memory, and backend-control surface |
| `NavigationModule` | `navigate_to`, `stop_navigation`, `cancel_mission`, `get_navigation_status`, `start_patrol` | Goal and mission control; goals are not direct motor commands |
| `MapService` | `list_maps`, `list_map_types`, `save_map`, `use_map`, `build_tomogram` | Saved-map lifecycle and artifacts |
| `SafetyRingModule` | `get_safety_status`, `emergency_stop` | Safety status and stop command |
| `TeleopModule` | `get_teleop_status`, `force_release` | Teleop lease/status helpers |
| `VisualServoModule` | `find_object`, `follow_person`, `stop_servo`, `get_servo_status`, `tune_bbox_gains` | Visual-servo hot-switch target/mode tools when the module is loaded |
| `SlamModule` / `SlamBridgeModule` | `relocalize`, `slam_status`, map/load helpers where available | SLAM/localization control and status |
| Memory modules | query, tag, temporal, episodic, vector, and semantic-map helpers | Optional; depends on profile |
| Exploration modules | frontier/TARE start, stop, status, and fallback helpers | Only present in exploration profiles |
| Rerun bridge | `start_rerun`, stop/status helpers | Optional visualization |

The exact set depends on the active profile and loaded Modules. Always call
`tools/list` against the running robot or simulation before assuming a tool is
available.

## Motion Boundary

MCP tools submit intent. They must not bypass navigation or safety:

```text
MCP tool
  -> Gateway / Module port / GoalService
  -> Navigation or native endpoint boundary
  -> CmdVelMux / Safety / native DDS endpoint
```

Important rules:

- Goal tools publish goals or instructions, not raw motor commands.
- `emergency_stop` publishes stop and zero velocity through the Module ports.
- Backend switches for motion surfaces require navigation to be IDLE.
- Visual-servo tools are hot-switch entries only when `VisualServoModule` is
  already loaded in the active profile.
- The default `thunder_field` product path has one field `/nav/cmd_vel` writer:
  C++ `lingtu-nav-dds`.

## Example Calls

List tools:

```json
{
  "jsonrpc": "2.0",
  "id": 1,
  "method": "tools/list",
  "params": {}
}
```

Call a tool:

```json
{
  "jsonrpc": "2.0",
  "id": 2,
  "method": "tools/call",
  "params": {
    "name": "navigate_to",
    "arguments": {"x": 5.0, "y": 3.0, "yaw": 0.0}
  }
}
```

Emergency stop:

```json
{
  "jsonrpc": "2.0",
  "id": 3,
  "method": "tools/call",
  "params": {
    "name": "emergency_stop",
    "arguments": {}
  }
}
```

## Registration Pattern

Add a tool by adding `@skill` to an owning Module method:

```python
from runtime.module import Module, skill

class ExampleModule(Module):
    @skill
    def do_thing(self, name: str) -> str:
        """Do one bounded thing."""
        return f"done: {name}"
```

Do not add a duplicate static tool table. The descriptor is generated from the
method signature and docstring.
