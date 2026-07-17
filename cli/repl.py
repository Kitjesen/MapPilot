"""Interactive cmd-loop REPL for LingTu."""

from __future__ import annotations

import cmd
import logging
import os
import sys
import time

from . import term as T
from .term import IS_TTY, c


class LingTuREPL(cmd.Cmd):
    """Runtime control REPL with navigation commands."""

    prompt = c("1;34", "lingtu> ") if IS_TTY else "lingtu> "

    def __init__(self, system, profile_cfg):
        super().__init__()
        self._system = system
        self._cfg = profile_cfg

    def _get_module(self, name: str):
        try:
            return self._system.get_module(name)
        except KeyError:
            return None

    def _get_slam_module(self):
        return self._get_module("SlamAdapterModule") or self._get_module("SlamModule")

    def _nav_commands(self):
        return self._get_module("nav.skills") or self._get_module("nav.mission")

    def do_navigate(self, arg):
        """Send navigation goal: navigate <x> <y> [z]"""
        parts = arg.split()
        if len(parts) < 2:
            print("  Usage: navigate <x> <y> [z]")
            return
        try:
            x, y = float(parts[0]), float(parts[1])
            z = float(parts[2]) if len(parts) > 2 else 0.0
        except ValueError:
            print("  Error: coordinates must be numbers")
            return

        nav = self._nav_commands()
        if not nav or not hasattr(nav, "navigate_to"):
            print("  Navigation not available")
            return
        print(f"  {nav.navigate_to(x, y, z=z)}")

    do_nav = do_navigate

    def do_go(self, arg):
        """Semantic navigation: go <natural language instruction>"""
        if not arg:
            print("  Usage: go <instruction>  (e.g. go find the table)")
            return
        planner = self._get_module("SemanticPlannerModule")
        if not planner:
            print("  Semantic planner not available")
            return
        if hasattr(planner, "send_instruction"):
            print(f"  {planner.send_instruction(arg)}")
        else:
            planner.instruction._deliver(arg)
            print(f"  Instruction -> {arg}")

    def do_stop(self, arg):
        """Emergency stop -> halt all motion immediately."""
        safety = self._get_module("nav.safety")
        if safety and hasattr(safety, "emergency_stop"):
            print(f"  {safety.emergency_stop()}")
            return
        count = 0
        for _name, mod in self._system.modules.items():
            if hasattr(mod, "stop_signal") and hasattr(mod.stop_signal, "_deliver"):
                mod.stop_signal._deliver(2)
                count += 1
        print(f"  {T.red('STOP')} sent to {count} module(s)")

    def do_cancel(self, arg):
        """Cancel current navigation mission."""
        nav = self._nav_commands()
        if not nav or not hasattr(nav, "cancel_mission"):
            print("  Navigation not available")
            return
        print(f"  {nav.cancel_mission('user')}")

    def do_map(self, arg):
        """Map management: map list | save <name> | use <name> | build <name> | delete <name> | rename <old> <new>"""
        parts = arg.split()
        if not parts:
            print("  Usage: map list | save <name> | use <name> | build <name> | delete <name> | rename <old> <new>")
            return

        subcmd = parts[0]
        name = parts[1] if len(parts) > 1 else ""
        name2 = parts[2] if len(parts) > 2 else ""

        if subcmd == "list":
            self._map_cmd({"action": "list"})
        elif subcmd == "save" and name:
            print(f"  Saving map '{name}'... (calls /pgo/save_maps -> may take 10-> 0s)")
            self._map_cmd({"action": "save", "name": name})
        elif subcmd == "use" and name:
            self._map_cmd({"action": "set_active", "name": name})
        elif subcmd == "build" and name:
            print(f"  Building OctoPlanner3D octomap for '{name}'...")
            self._map_cmd({"action": "build_octomap", "name": name})
        elif subcmd == "delete" and name:
            self._map_cmd({"action": "delete", "name": name})
        elif subcmd == "rename" and name and name2:
            self._map_cmd({"action": "rename", "name": name, "new_name": name2})
        else:
            print("  Usage: map list | save <name> | use <name> | build <name> | delete <name> | rename <old> <new>")

    def _map_cmd(self, cmd: dict) -> None:
        """Send a command to MapsModule and print the result.

        Persistent map reads and mutations always go through maps.service.
        """
        from runtime.msgs.map import MapControlRequest

        mod = self._get_module("maps.service")
        if mod is not None:
            execute = getattr(mod, "execute", None)
            if not callable(execute):
                print(f"  {T.red('Error')}: maps.service has no typed control endpoint")
                return
            resp = execute(MapControlRequest.from_mapping(cmd))
            self._map_print_response(cmd.get("action", ""), resp)
            return

        print(f"  {T.red('Error')}: maps.service is unavailable; start a map-enabled profile")

    def _map_print_response(self, action: str, resp: dict) -> None:
        """Render a MapsModule response to the terminal."""
        if not resp.get("success"):
            print(f"  {T.red('Error')}: {resp.get('message', '?')}")
            return

        if action == "list":
            maps = resp.get("maps", [])
            active = resp.get("active", "")
            if not maps:
                print("  No maps found")
                return
            for m in maps:
                name = m["name"]
                marker = T.green(" *") if name == active else ""
                parts = []
                if m.get("has_pcd"):
                    parts.append("pcd")
                if m.get("has_octomap"):
                    parts.append("octomap")
                if m.get("has_occupancy"):
                    parts.append("occupancy")
                print(f"  {name}{marker}  [{', '.join(parts) or 'empty'}]")

        elif action == "save":
            pcd = resp.get("pcd", "?")
            print(f"  Saved: {pcd}")
            if resp.get("octomap_ok"):
                print(f"  Octomap: {resp.get('octomap', '?')}")
            else:
                print(f"  {T.yellow('Octomap build failed')} -> run: map build <name>")
            # Offer to set as active
            name = os.path.basename(os.path.dirname(pcd))
            print(f"  Tip: run  {T.bold(f'map use {name}')}  to activate this map")

        elif action == "set_active":
            name = resp.get("active", "?")
            octomap = resp.get("octomap", "")
            nav = self._get_module("nav.mission")
            if nav and octomap and os.path.exists(octomap) and hasattr(nav, "reload_planner_map"):
                self._reload_planner_map(nav, octomap, name)
            else:
                print(f"  Active map: {T.green(name)}")
                if octomap and not os.path.exists(octomap):
                    print(f"  {T.yellow('No octomap')} -> run: map build {name}")

        elif action in {"build_octomap", "build_octomap_artifact"}:
            print(f"  Octomap: {resp.get('octomap', '?')}")

        elif action == "delete":
            print(f"  {resp.get('message', 'deleted')}")

        elif action == "rename":
            print(f"  {resp.get('message', 'renamed')}")

        else:
            print(f"  {resp}")

    def _reload_planner_map(self, nav, map_path: str, name: str) -> None:
        """Hot-reload the planner's saved-map artifact after map switch."""
        reload_planner_map = getattr(nav, "reload_planner_map", None)
        if callable(reload_planner_map):
            result = reload_planner_map(map_path)
            if result.get("ok"):
                mode = result.get("mode", "map")
                suffix = "costmap reloaded" if mode == "costmap" else "map hot-reloaded"
                print(f"  Active map: {T.green(name)} ({suffix})")
            else:
                print(f"  Active map: {T.green(name)} (restart required to apply new map artifact)")
        else:
            print(f"  Active map: {T.green(name)} (restart required to apply new map artifact)")

    # -鈧?鈧?SLAM hot-switch -鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?
    def do_slam(self, arg):
        """SLAM control: slam status | fastlio2 | localizer | super_lio | super_lio_relocation | stop"""
        parts = arg.split()
        subcmd = parts[0] if parts else "status"

        if subcmd == "status":
            self._slam_status()
        elif subcmd in ("fastlio2", "mapping"):
            self._slam_switch("fastlio2")
        elif subcmd in ("localizer", "nav"):
            self._slam_switch("localizer")
        elif subcmd in ("super_lio", "super-lio", "superlio"):
            self._slam_switch("super_lio")
        elif subcmd in (
            "super_lio_relocation",
            "super_lio_reloc",
            "super-lio-relocation",
            "super-lio-reloc",
            "relocation",
        ):
            self._slam_switch("super_lio_relocation")
        elif subcmd == "stop":
            self._slam_stop()
        else:
            print("  Usage: slam status | fastlio2 | localizer | super_lio | super_lio_relocation | stop")

    def complete_slam(self, text, line, begidx, endidx):
        options = [
            "status",
            "fastlio2",
            "localizer",
            "super_lio",
            "super_lio_relocation",
            "stop",
            "mapping",
            "nav",
        ]
        return [o for o in options if o.startswith(text)]

    def _slam_status(self):
        try:
            from runtime.service_manager import get_service_manager

            svc = get_service_manager()
            st = svc.status(
                "lidar",
                "slam",
                "slam_pgo",
                "localizer",
                "super_lio",
                "super_lio_relocation",
            )
        except Exception:
            st = {
                "lidar": "?",
                "slam": "?",
                "slam_pgo": "?",
                "localizer": "?",
                "super_lio": "?",
                "super_lio_relocation": "?",
            }

        # Determine current mode
        if st.get("super_lio_relocation") == "running":
            mode = "super_lio_relocation (experimental saved-map)"
        elif st.get("super_lio") == "running":
            mode = "super_lio (experimental mapping/LIO)"
        elif st.get("slam_pgo") == "running":
            mode = "fastlio2 (mapping)"
        elif st.get("localizer") == "running":
            mode = "localizer (navigation)"
        elif st.get("slam") == "running":
            mode = "slam only (no PGO/localizer)"
        else:
            mode = "stopped"

        print(f"  SLAM mode: {mode}")
        for name, state in st.items():
            icon = "*" if state == "running" else "*"
            print(f"    {icon} {name}: {state}")

        # Show SLAM health if available
        slam = self._get_slam_module()
        if slam:
            try:
                h = slam.health()
                print(f"  SLAM module: alive={h.get('slam', {}).get('alive', '?')}")
            except Exception:
                pass

    def _slam_switch(self, profile: str):
        try:
            from runtime.service_manager import get_service_manager

            svc = get_service_manager()
        except Exception as e:
            print(f"  ServiceManager not available: {e}")
            return

        # Check current state
        pgo_running = svc.is_running("slam_pgo")
        loc_running = svc.is_running("localizer")
        super_lio_running = svc.is_running("super_lio")
        super_lio_reloc_running = svc.is_running("super_lio_relocation")

        if profile == "fastlio2" and pgo_running:
            print("  Already in fastlio2 (mapping) mode")
            return
        if profile == "localizer" and loc_running:
            print("  Already in localizer (navigation) mode")
            return
        if profile == "super_lio" and super_lio_running:
            print("  Already in super_lio mode")
            return
        if profile == "super_lio_relocation" and super_lio_reloc_running:
            print("  Already in super_lio_relocation mode")
            return

        print(f"  Switching to {profile}...")

        # Stop conflicting services
        if profile == "fastlio2":
            svc.stop("localizer", "super_lio", "super_lio_relocation")
            svc.ensure("slam", "slam_pgo")
            wait_services = ("slam", "slam_pgo")
        elif profile == "localizer":
            svc.stop("slam_pgo", "super_lio", "super_lio_relocation")
            svc.ensure("slam", "localizer")
            wait_services = ("slam", "localizer")
        elif profile == "super_lio":
            svc.stop("slam", "slam_pgo", "localizer", "super_lio_relocation")
            svc.ensure("lidar", "super_lio")
            wait_services = ("lidar", "super_lio")
        elif profile == "super_lio_relocation":
            svc.stop("slam", "slam_pgo", "localizer", "super_lio")
            svc.ensure("lidar", "super_lio_relocation")
            wait_services = ("lidar", "super_lio_relocation")
        else:
            print(f"  Unknown SLAM profile: {profile}")
            return

        # Wait for readiness
        if svc.wait_ready(*wait_services, timeout=10.0):
            print(f"  Switched to {profile} -> bridge reconnects in ~3s")
        else:
            print(f"  Warning: services not ready after 10s: {', '.join(wait_services)}")

        # Update config
        if hasattr(self, "_cfg") and self._cfg:
            self._cfg["slam_profile"] = profile

    def _slam_stop(self):
        try:
            from runtime.service_manager import get_service_manager

            svc = get_service_manager()
            svc.stop("super_lio_relocation", "super_lio", "slam_pgo", "localizer", "slam")
            print("  All SLAM services stopped")
        except Exception as e:
            print(f"  Failed: {e}")

    def do_gnss(self, arg):
        """GNSS-SLAM fusion: gnss status | relock | enable | disable"""
        parts = arg.split()
        subcmd = parts[0] if parts else "status"

        if subcmd == "status":
            self._gnss_status()
        elif subcmd == "relock":
            self._gnss_relock()
        elif subcmd in ("enable", "on"):
            self._gnss_set_fusion(True)
        elif subcmd in ("disable", "off"):
            self._gnss_set_fusion(False)
        else:
            print("  Usage: gnss status | relock | enable | disable")

    def complete_gnss(self, text, line, begidx, endidx):
        options = ["status", "relock", "enable", "disable", "on", "off"]
        return [o for o in options if o.startswith(text)]

    def _gnss_status(self):
        bridge = self._get_slam_module()
        if bridge is None:
            print("  SLAM module not available")
            return
        try:
            snap = bridge._gnss_health_snapshot()
        except Exception as e:
            print(f"  Failed to read GNSS fusion status: {e}")
            return
        enabled = snap.get("enabled")
        locked = snap.get("alignment_locked")
        print(f"  Fusion enabled: {'*' if enabled else '*'}")
        print(f"  Alignment locked: {'*' if locked else '*'}")
        if snap.get("map_offset"):
            mx, my = snap["map_offset"]
            print(f"  Map->ENNU offset: ({mx:+.3f}, {my:+.3f}) m")
        ant = snap.get("antenna_offset_body") or [0.0, 0.0, 0.0]
        print(f"  Antenna body offset: ({ant[0]:+.3f}, {ant[1]:+.3f}, {ant[2]:+.3f}) m")
        fix = snap.get("last_fix_type", "NONE")
        age = snap.get("last_gnss_age_s", float("inf"))
        age_str = f"{age:.1f}s" if age != float("inf") else "inf"
        print(f"  Last fix: {fix}  age: {age_str}")
        print(f"  Residual: {snap.get('last_residual_m', 0.0):.2f} m")
        print(f"  Fused frames: {snap.get('fused_count', 0)}")
        print(f"  Relock count: {snap.get('relock_count', 0)}")

        gnss = self._get_module("GnssModule")
        if gnss is not None:
            try:
                h = gnss.health()
                origin = h.get("origin")
                if origin:
                    print(f"  Origin: ({origin['lat']:.7f}, {origin['lon']:.7f}, {origin['alt']:.2f})")
            except Exception:
                pass

    def _gnss_relock(self):
        bridge = self._get_slam_module()
        if bridge is None:
            print("  SLAM module not available")
            return
        try:
            result = bridge.relock_gnss_alignment()
            print(f"  {result}")
        except Exception as e:
            print(f"  Relock failed: {e}")

    def _gnss_set_fusion(self, enabled: bool):
        bridge = self._get_slam_module()
        if bridge is None:
            print("  SLAM module not available")
            return
        try:
            result = bridge.set_gnss_fusion(enabled)
            print(f"  {result}")
        except Exception as e:
            print(f"  Failed: {e}")

    def do_smap(self, arg):
        """Semantic map: smap status | rooms | save | load <dir> | query <text>"""
        parts = arg.split(None, 1)
        subcmd = parts[0] if parts else ""
        rest = parts[1] if len(parts) > 1 else ""

        if subcmd == "status":
            self._smap_status()
        elif subcmd == "rooms":
            self._smap_rooms()
        elif subcmd == "save":
            self._smap_save()
        elif subcmd == "load":
            self._smap_load(rest.strip())
        elif subcmd == "query" and rest:
            self._smap_query(rest.strip())
        else:
            print("  Usage: smap status | rooms | save | load <dir> | query <text>")

    def _smap_get(self):
        try:
            from memory.modules.semantic_mapper_module import SemanticMapperModule

            return self._get_module(SemanticMapperModule.__name__)
        except ImportError:
            return None

    def _smap_status(self):
        mod = self._smap_get()
        if mod is None:
            print("  SemanticMapperModule not running")
            return
        s = mod.get_semantic_status()
        kg = s.get("kg", {})
        tsg = s.get("tsg", {})
        print(f"  Save dir : {s['save_dir']}")
        print(
            f"  KG       : {kg.get('room_types', 0)} room types, "
            f"{kg.get('unique_objects', 0)} object types, "
            f"{kg.get('total_observations', 0)} observations"
        )
        print(
            f"  TSG      : {tsg.get('rooms', 0)} rooms, "
            f"{tsg.get('frontiers', 0)} frontiers, "
            f"current={tsg.get('current_room', 'unknown')}"
        )
        print(f"  Updates  : {s['sg_updates']} scene-graph callbacks")

    def _smap_rooms(self):
        mod = self._smap_get()
        if mod is None:
            print("  SemanticMapperModule not running")
            return
        summary = mod.get_room_summary()
        for line in summary.splitlines():
            print(f"  {line}")

    def _smap_save(self):
        mod = self._smap_get()
        if mod is None:
            print("  SemanticMapperModule not running")
            return
        mod._save_now()
        print(f"  Saved to {mod._save_dir}")

    def _smap_load(self, directory):
        if not directory:
            print("  Usage: smap load <directory>")
            return
        import os

        directory = os.path.expanduser(directory)
        mod = self._smap_get()
        if mod is None:
            print("  SemanticMapperModule not running")
            return
        ok = mod.load_from_dir(directory)
        if ok:
            print(f"  Loaded from {directory}")
        else:
            print(f"  Load failed (check path: {directory})")

    def _smap_query(self, text):
        mod = self._smap_get()
        if mod is None:
            print("  SemanticMapperModule not running")
            return
        result = mod.query_room_for_object(text)
        rooms = result.get("rooms", [])
        if not rooms:
            print(f"  No data for '{text}' (source: {result.get('source', '?')})")
            return
        print(f"  '{text}' most likely in:")
        for r in rooms:
            bar = "#" * int(r["probability"] * 20)
            print(f"    {r['room']:20s} {bar} {r['probability']:.2f}")

    def do_agent(self, arg):
        """Multi-turn LLM agent with live step output: agent <instruction>

        The LLM observes the robot state, decides actions, and executes them
        step by step. Each tool call and result is printed in real time.

        Example:
          agent find the chair and tag it as 'chair'
          agent go to the kitchen and describe what you see
        """
        if not arg.strip():
            print("  Usage: agent <instruction>")
            print("  Example: agent find the red chair")
            return

        llm_mod = self._get_module("LLMModule")
        if llm_mod is None:
            print("  LLMModule not running -> start with a semantic-enabled profile")
            return

        # Get LLM client from the module
        llm_client = getattr(llm_mod, "_client", None)
        if llm_client is None:
            print("  LLM client not initialized")
            return

        # Check if LLM is available
        backend = getattr(llm_mod, "_backend", "?")
        if backend == "mock":
            print(f"  {T.yellow('!')} LLM backend is 'mock' -> responses will be simulated")
            print("    Set a real API key and restart with --llm kimi/qwen/openai")
            print()

        # Build context function from live modules
        nav = self._get_module("nav.mission")
        self._get_module("SemanticMapperModule")
        vmem = self._get_module("VectorMemoryModule")
        sem = self._get_module("SemanticPlannerModule")

        def _context():
            ctx = {
                "robot_x": 0.0,
                "robot_y": 0.0,
                "visible_objects": "none",
                "nav_status": "IDLE",
                "memory_context": "none",
                "camera_image": None,
                "scene_graph": None,
                "camera_available": False,
            }
            if nav and hasattr(nav, "_robot_pos"):
                ctx["robot_x"] = float(nav._robot_pos[0])
                ctx["robot_y"] = float(nav._robot_pos[1])
                ctx["nav_status"] = getattr(nav, "_state", "IDLE")
            if sem and hasattr(sem, "_current_scene_graph") and sem._current_scene_graph:
                sg = sem._current_scene_graph
                try:
                    labels = [o.label for o in sg.objects[:8]] if hasattr(sg, "objects") else []
                    ctx["visible_objects"] = ", ".join(labels) if labels else "none"
                    ctx["scene_graph"] = {"objects": [{"label": l} for l in labels]}
                except Exception:
                    pass
            if sem and hasattr(sem, "_latest_rgb") and sem._latest_rgb is not None:
                ctx["camera_image"] = sem._latest_rgb
                ctx["camera_available"] = True
            return ctx

        # Build tool handlers that publish to module ports
        def _navigate_to(x, y, z=None, **_):
            if nav and hasattr(nav, "goal_pose"):
                from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3

                if z is None:
                    try:
                        z = float(nav._robot_pos[2])
                    except Exception:
                        z = 0.0
                goal = PoseStamped(
                    pose=Pose(
                        position=Vector3(x=float(x), y=float(y), z=float(z)),
                        orientation=Quaternion(x=0, y=0, z=0, w=1),
                    )
                )
                nav.goal_pose._deliver(goal)
                return f"Navigating to ({float(x):.1f}, {float(y):.1f}, {float(z):.1f})"
            return "Navigation not available"

        def _navigate_to_object(label, **_):
            if sem and hasattr(sem, "instruction"):
                sem.instruction._deliver(f"go to {label}")
                return f"Searching for '{label}' and navigating"
            return "SemanticPlannerModule not available"

        def _detect_object(label, **_):
            if sem and hasattr(sem, "_current_scene_graph") and sem._current_scene_graph:
                try:
                    labels = [o.label.lower() for o in sem._current_scene_graph.objects]
                    found = [l for l in labels if label.lower() in l]
                    return f"Found: {found}" if found else f"'{label}' not visible"
                except Exception:
                    pass
            return "Scene graph not available"

        def _query_memory(text, **_):
            if vmem and hasattr(vmem, "query_location"):
                try:
                    r = vmem.query_location(text)
                    if r.get("found"):
                        if not (
                            r.get("navigable") is True
                            and r.get("semantic_encoder_ready") is True
                            and r.get("degraded") is False
                        ):
                            return (
                                "Memory match is query-only; navigation "
                                f"coordinates withheld (encoder={r.get('encoder_type', 'unknown')})"
                            )
                        results = r.get("results", [])[:3]
                        return "; ".join(f"({x['x']:.1f},{x['y']:.1f}) score={x['score']:.2f}" for x in results)
                    return "No matching memory"
                except Exception as e:
                    return f"Memory query error: {e}"
            return "VectorMemoryModule not available"

        def _tag_location(name, **_):
            if nav and hasattr(nav, "_robot_pos"):
                x, y = nav._robot_pos[0], nav._robot_pos[1]
                if vmem and hasattr(vmem, "store_observation"):
                    try:
                        vmem.store_observation(x, y, [name])
                        return f"Tagged ({x:.1f}, {y:.1f}) as '{name}'"
                    except Exception:
                        pass
            return f"Tagged current position as '{name}'"

        def _say(text, **_):
            print(f"\n  {T.cyan('Robot:')} {text}")
            return "said"

        tool_handlers = {
            "navigate_to": _navigate_to,
            "navigate_to_object": _navigate_to_object,
            "detect_object": _detect_object,
            "query_memory": _query_memory,
            "tag_location": _tag_location,
            "say": _say,
        }

        # Run the agent loop with live output
        import asyncio
        import threading

        instruction = arg.strip()
        print(f"\n  {T.bold('Agent')} {T.dim('starting:')} {instruction}")
        print(f"  {T.dim('LLM backend:')} {backend}  {T.dim('(Ctrl+C to abort)')}\n")

        # Patch AgentLoop to print each step live
        try:
            from decision.tasks.agent import AgentLoop
        except ImportError:
            print("  AgentLoop not available -> check semantic stack is enabled")
            return

        class _LiveAgentLoop(AgentLoop):
            async def _execute_tool(self_inner, tool_call, state):
                fn = tool_call.get("function", {})
                name = fn.get("name", "")
                try:
                    import json as _json

                    args = _json.loads(fn.get("arguments", "{}"))
                except Exception:
                    args = {}
                args_str = ", ".join(f"{k}={v!r}" for k, v in args.items())
                step_color = T.cyan if name != "done" else T.green
                print(f"  {T.dim(f'step {state.step}:')} {step_color(name)}({args_str})")
                result = await super()._execute_tool(tool_call, state)
                print(f"  {T.dim('result:')} {result[:120]}")
                if state.completed and name == "done":
                    print(f"\n  {T.green('Done:')} {state.summary}")
                return result

        loop_obj = _LiveAgentLoop(
            llm_client=llm_client,
            tool_handlers=tool_handlers,
            context_fn=_context,
            max_steps=10,
            timeout=120.0,
        )

        result_holder = {}

        def _run():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                state = loop.run_until_complete(loop_obj.run(instruction))
                result_holder["state"] = state
            except Exception as e:
                result_holder["error"] = str(e)
            finally:
                loop.close()

        t = threading.Thread(target=_run, daemon=True)
        t.start()
        try:
            t.join(timeout=130.0)
        except KeyboardInterrupt:
            print(f"\n  {T.yellow('Aborted by user')}")
            return

        if "error" in result_holder:
            print(f"\n  {T.red('Agent error:')} {result_holder['error']}")
        elif "state" not in result_holder:
            print(f"\n  {T.yellow('Agent timed out')}")
        print()

    def do_chat(self, arg):
        """Chat directly with the LLM about the robot's current state.

        The LLM sees the robot's position, visible objects, and navigation status.
        It can answer questions and suggest actions, but does not execute them
        automatically (use 'agent' for autonomous execution).

        Example:
          chat what do you see around you?
          chat where should I go to find the kitchen?
          chat explain the current navigation status
        """
        if not arg.strip():
            print("  Usage: chat <question>")
            print("  Example: chat what can you see?")
            return

        llm_mod = self._get_module("LLMModule")
        if llm_mod is None:
            print("  LLMModule not running")
            return

        llm_client = getattr(llm_mod, "_client", None)
        if llm_client is None:
            print("  LLM client not initialized")
            return

        backend = getattr(llm_mod, "_backend", "?")
        if backend == "mock":
            print(f"  {T.yellow('!')} Using mock LLM -> set a real API key for actual responses")

        # Build system prompt with current robot state
        nav = self._get_module("nav.mission")
        sem = self._get_module("SemanticPlannerModule")

        x, y, z = 0.0, 0.0, 0.0
        nav_state = "IDLE"
        visible = "none"

        if nav and hasattr(nav, "_robot_pos"):
            x, y, z = nav._robot_pos
            nav_state = getattr(nav, "_state", "IDLE")

        if sem and hasattr(sem, "_current_scene_graph") and sem._current_scene_graph:
            try:
                labels = [o.label for o in sem._current_scene_graph.objects[:10]]
                visible = ", ".join(labels) if labels else "none"
            except Exception:
                pass

        system_prompt = (
            f"You are the AI assistant for a quadruped navigation robot (LingTu).\n"
            f"Current robot state:\n"
            f"  Position: x={x:.2f}, y={y:.2f}, z={z:.2f}\n"
            f"  Navigation status: {nav_state}\n"
            f"  Visible objects: {visible}\n\n"
            f"Answer questions about the robot's environment and status. "
            f"If the user wants the robot to do something, suggest using the 'agent' command."
        )

        import asyncio

        print(f"  {T.dim(f'[{backend}]')} thinking...", end="", flush=True)

        result_holder = {}

        def _run():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                resp = loop.run_until_complete(llm_client.chat(arg.strip(), system_prompt=system_prompt))
                result_holder["response"] = resp
            except Exception as e:
                result_holder["error"] = str(e)
            finally:
                loop.close()

        import threading

        t = threading.Thread(target=_run, daemon=True)
        t.start()
        try:
            t.join(timeout=30.0)
        except KeyboardInterrupt:
            print(f"\r  {T.yellow('Cancelled')}")
            return

        print(f"\r  {T.dim(f'[{backend}]')}          ")  # clear "thinking..."

        if "error" in result_holder:
            print(f"  {T.red('Error:')} {result_holder['error']}")
        elif "response" in result_holder:
            resp = result_holder["response"]
            print()
            for line in resp.splitlines():
                print(f"  {line}")
            print()
        else:
            print(f"  {T.yellow('No response (timeout)')}")

    def do_llm(self, arg):
        """LLM backend status and connectivity test: llm status | llm test | llm backends

        Examples:
          llm status    -> show current backend, model, API key status
          llm test      -> send a test message and measure latency
          llm backends  -> list all available backends and how to enable them
        """
        subcmd = arg.strip().lower()

        llm_mod = self._get_module("LLMModule")
        if llm_mod is None:
            print("  LLMModule not running -> start with a semantic-enabled profile")
            return

        if subcmd in ("", "status"):
            backend = getattr(llm_mod, "_backend", "?")
            model = getattr(llm_mod, "_model", "?")
            client = getattr(llm_mod, "_client", None)
            req_cnt = llm_mod.request.msg_count if hasattr(llm_mod, "request") else 0
            resp_cnt = llm_mod.response.msg_count if hasattr(llm_mod, "response") else 0

            key_env = getattr(getattr(client, "config", None), "api_key_env", "?")
            import os

            key_set = bool(os.environ.get(key_env, ""))
            key_str = T.green("set") if key_set else T.red("not set")

            print(f"\n  LLM backend:  {T.bold(backend)}")
            print(f"  Model:        {model or T.dim('(default)')}")
            print(f"  API key ({key_env}): {key_str}")
            print(f"  Requests:     {req_cnt} sent, {resp_cnt} responses")

            if not key_set and backend != "mock":
                print("\n  To set the key:")
                print(f"    export {key_env}=your_key_here")
                print("    then restart lingtu")

            print()

        elif subcmd == "test":
            client = getattr(llm_mod, "_client", None)
            if client is None:
                print("  LLM client not initialized")
                return

            backend = getattr(llm_mod, "_backend", "?")
            print(f"  Testing {backend}...", end="", flush=True)

            import asyncio
            import threading
            import time as _time

            result_holder = {}
            t0 = _time.time()

            def _run():
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                try:
                    resp = loop.run_until_complete(
                        client.chat(
                            "Reply with exactly: OK", system_prompt="You are a test assistant. Reply only with 'OK'."
                        )
                    )
                    result_holder["response"] = resp
                except Exception as e:
                    result_holder["error"] = str(e)
                finally:
                    loop.close()

            t = threading.Thread(target=_run, daemon=True)
            t.start()
            t.join(timeout=15.0)
            elapsed = _time.time() - t0

            if "error" in result_holder:
                print(f"\r  {T.red('Failed')} ({elapsed:.1f}s): {result_holder['error']}")
            elif "response" in result_holder:
                print(f"\r  {T.green('OK')} ({elapsed:.1f}s): {result_holder['response'][:60]}")
            else:
                print(f"\r  {T.yellow('Timeout')} after {elapsed:.1f}s")
            print()

        elif subcmd == "backends":
            print("\n  Available LLM backends:\n")
            backends = [
                ("kimi", "MOONSHOT_API_KEY", "Kimi K2 (Moonshot AI, China-direct, recommended)"),
                ("qwen", "DASHSCOPE_API_KEY", "Qwen (Alibaba, China-direct fallback)"),
                ("openai", "OPENAI_API_KEY", "GPT-4o (OpenAI, needs VPN in China)"),
                ("claude", "ANTHROPIC_API_KEY", "Claude (Anthropic, needs VPN in China)"),
                ("mock", "", "Mock (offline testing, no real LLM)"),
            ]
            import os

            current = getattr(llm_mod, "_backend", "?")
            for name, env, desc in backends:
                key_ok = bool(os.environ.get(env)) if env else True
                marker = T.green("*") if name == current else " "
                key_str = T.green("key set") if key_ok else T.dim("no key")
                print(f"  {marker} {T.bold(name):<10} {key_str:<20} {T.dim(desc)}")
            print()
            print(f"  Switch backend: restart with  {T.bold('python3 lingtu.py nav --llm kimi')}")
            print()

    def do_vmem(self, arg):
        """Vector memory: vmem query <text> | vmem stats"""
        parts = arg.split(None, 1)
        subcmd = parts[0] if parts else ""
        rest = parts[1] if len(parts) > 1 else ""

        mod = self._get_module("VectorMemoryModule")
        if mod is None:
            print("  VectorMemoryModule not running")
            return

        if subcmd == "query" and rest:
            result = mod.query_location(rest.strip())
            if not result.get("found"):
                print(f"  No match for '{rest.strip()}'")
                return
            for r in result.get("results", []):
                bar = "#" * int(r["score"] * 20)
                print(f"    ({r['x']:6.1f}, {r['y']:6.1f})  {bar} {r['score']:.2f}  {r.get('labels', '')[:40]}")
        elif subcmd == "stats":
            s = mod.get_memory_stats()
            print(f"  Backend:  {s['backend']}")
            print(f"  Encoder:  {s.get('encoder_type', 'unknown')}")
            print(f"  Semantic: {bool(s.get('semantic_encoder_ready', False))}")
            print(f"  Degraded: {bool(s.get('degraded', False))}")
            print(f"  Entries:  {s['entries']}")
            print(f"  Stored:   {s['store_count']} snapshots")
            print(f"  Dir:      {s['persist_dir']}")
        else:
            print("  Usage: vmem query <text> | vmem stats")

    def do_teleop(self, arg):
        """Teleop control: teleop status | teleop release"""
        mod = self._get_module("TeleopModule")
        if mod is None:
            print("  TeleopModule not running")
            return

        subcmd = arg.strip()
        if subcmd == "status":
            s = mod.get_teleop_status()
            active = "ACTIVE" if s["active"] else "idle"
            print(f"  Status:  {active}")
            print(f"  Clients: {s['clients']}")
            print(f"  URL:     ws://0.0.0.0:{s['port']}/ws/teleop")
        elif subcmd == "release":
            print(f"  {mod.force_release()}")
        else:
            print("  Usage: teleop status | teleop release")

    def do_rerun(self, arg):
        """Rerun visualization: rerun on | rerun off | rerun status"""
        subcmd = arg.strip().lower()
        mod = self._get_module("RerunBridgeModule")

        if subcmd == "on":
            if mod is None:
                print("  RerunBridgeModule not in blueprint (start with --rerun)")
                return
            url = mod.start_rerun()
            print(f"  Rerun: {url}")
        elif subcmd == "off":
            if mod is None:
                print("  RerunBridgeModule not running")
                return
            result = mod.stop_rerun()
            print(f"  Rerun: {result}")
        elif subcmd == "status":
            if mod is None:
                print("  RerunBridgeModule not in blueprint")
                return
            s = mod.rerun_status()
            active = T.green("ACTIVE") if s["active"] else T.dim("inactive")
            print(f"  Status:  {active}")
            if s["url"]:
                print(f"  URL:     {s['url']}")
            print(f"  Counts:  odom={s['counts'].get('odom', 0)} cloud={s['counts'].get('cloud', 0)}")
        else:
            print("  Usage: rerun on | off | status")

    def do_info(self, arg):
        """System summary -> startup status, sensor data flow, position, map progress."""
        import time as _time

        s = self._system
        if not s.started:
            print("  System not started")
            return

        now = _time.strftime("%H:%M:%S")
        profile = self._cfg.get("slam_profile", "?")
        print(f"\n  [{now}]  profile: {self._cfg.get('_desc', profile)}\n")

        # -鈧?鈧?Startup -鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?        n_mods = len(s.modules)
        n_mods = len(s.modules)
        n_conn = len(s.connections)
        active = sum(
            1
            for mod in s.modules.values()
            if sum(p.msg_count for p in list(mod.ports_in.values()) + list(mod.ports_out.values())) > 0
        )
        print(f"  Startup    {n_mods} modules loaded, {n_conn} connections")
        print(f"             {active} modules have received/sent data")

        # SLAM / localization
        slam = self._get_slam_module()
        if slam:
            odom_n = slam.odometry.msg_count if hasattr(slam, "odometry") else 0
            cloud_n = slam.map_cloud.msg_count if hasattr(slam, "map_cloud") else 0
            odom_ok = T.green("flowing") if odom_n > 0 else T.yellow("no data yet")
            cloud_ok = T.green("flowing") if cloud_n > 0 else T.yellow("no data yet")
            print("\n  SLAM")
            print(f"    odometry   {odom_ok}  ({odom_n} msgs)")
            print(f"    point cloud {cloud_ok}  ({cloud_n} msgs)")
        else:
            print(f"\n  SLAM       {T.dim('not in this profile')}")

        # -鈧?鈧?Robot position -鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?        nav = self._get_module("nav.mission")
        nav = self._get_module("nav.mission")
        if nav and hasattr(nav, "_robot_pos"):
            x, y, z = nav._robot_pos
            state = getattr(nav, "_state", "?")
            state_color = T.green if state in ("IDLE", "SUCCESS") else T.yellow if state == "EXECUTING" else T.red
            print("\n  Navigation")
            print(f"    position   x={x:.2f}  y={y:.2f}  z={z:.2f}")
            print(f"    state      {state_color(state)}")

        # -鈧?鈧?Map modules -鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?        occ = self._get_module("OccupancyGridModule")
        occ = self._get_module("OccupancyGridModule")
        if occ:
            n = sum(p.msg_count for p in occ.ports_in.values())
            ok = T.green("building") if n > 0 else T.yellow("waiting for point cloud")
            print("\n  Map building")
            print(f"    occupancy grid  {ok}  ({n} cloud updates)")

        elev = self._get_module("ElevationMapModule")
        if elev:
            n = sum(p.msg_count for p in elev.ports_in.values())
            ok = T.green("building") if n > 0 else T.yellow("waiting")
            print(f"    elevation map   {ok}  ({n} updates)")

        # -鈧?鈧?C++ backends -鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?        terrain = self._get_module("nav.terrain")
        terrain = self._get_module("nav.terrain")
        lp = self._get_module("nav.local_planner")
        pf = self._get_module("nav.path_follower")
        if terrain or lp or pf:
            print("\n  C++ backends")
            if terrain:
                be = getattr(terrain, "_backend", "?")
                ok = T.green("nanobind") if be == "nanobind" else T.yellow(be)
                print(f"    terrain        {ok}")
            if lp:
                be = getattr(lp, "_backend", "?")
                ok = T.green("nanobind") if be == "nanobind" else T.yellow(be)
                print(f"    local planner  {ok}")
            if pf:
                be = getattr(pf, "_backend", "?")
                ok = T.green("nav_kernel") if be == "nav_kernel" else T.yellow(be)
                print(f"    path follower  {ok}")

        # -鈧?鈧?Gateway -鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?        gw = self._get_module("GatewayModule")
        gw = self._get_module("GatewayModule")
        if gw:
            port = getattr(gw, "_port", 5050)
            print(f"\n  Gateway    http://localhost:{port}  (accessible from LAN)")

        # -鈧?鈧?Teleop -鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?鈧?        tp = self._get_module("TeleopModule")
        tp = self._get_module("TeleopModule")
        if tp:
            try:
                st = tp.get_teleop_status()
                tp_port = st.get("port", "?")
                clients = st.get("clients", 0)
                active_str = T.green("ACTIVE") if st.get("active") else T.dim("idle")
                print(f"  Teleop     ws://0.0.0.0:{tp_port}/ws/teleop  {active_str}  ({clients} clients)")
            except Exception:
                pass

        print()

    do_i = do_info

    def do_status(self, arg):
        """Module overview with layer tags and message counts."""
        s = self._system
        if not s.started:
            print("  System not started")
            return
        mods = s.modules
        print(f"\n  {len(mods)} modules, {len(s.connections)} connections\n")
        for name, mod in sorted(mods.items(), key=lambda x: x[1].layer or 0):
            layer = f"L{mod.layer}" if mod.layer is not None else "L?"
            n_in = sum(p.msg_count for p in mod.ports_in.values())
            n_out = sum(p.msg_count for p in mod.ports_out.values())
            traffic = f"in:{n_in} out:{n_out}" if (n_in + n_out) > 0 else T.dim("idle")
            print(f"  [{layer}] {name:30s} {traffic}")
        nav = self._get_module("nav.mission")
        if nav and hasattr(nav, "_state"):
            print(f"\n  Mission: {T.bold(nav._state)}")
        print()

    do_s = do_status

    def do_health(self, arg):
        """Detailed health -> totals, startup order, violations."""
        s = self._system
        if not s.started:
            print("  System not started")
            return
        h = s.health()
        print(f"\n  Modules: {h['module_count']}  Connections: {h['connection_count']}")
        print(f"  Total in: {h['total_messages_in']}  Total out: {h['total_messages_out']}")
        if h["layer_violations"]:
            for v in h["layer_violations"]:
                print(f"  {T.red('!')} {v}")
        print("\n  Startup order:")
        for i, name in enumerate(h["startup_order"], 1):
            print(f"    {i}. {name}")
        print()

    do_h = do_health

    def do_watch(self, arg):
        """Live status refresh: watch [interval_sec]. Ctrl+C to stop."""
        interval = float(arg) if arg else 2.0
        try:
            while True:
                os.system("cls" if os.name == "nt" else "clear")
                ts = time.strftime("%H:%M:%S")
                print(f"  {T.dim(f'[{ts}]  refresh every {interval:.0f}s  Ctrl+C to stop')}\n")
                self.do_status("")
                time.sleep(interval)
        except KeyboardInterrupt:
            print("\n  Watch stopped")

    do_w = do_watch

    def do_live(self, arg):
        """Real-time dashboard with hotkeys."""
        interval = float(arg) if arg else 1.0
        start_time = time.time()
        import select

        if os.name != "nt":
            import termios
            import tty

            old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        try:
            while True:
                os.system("cls" if os.name == "nt" else "clear")
                elapsed = int(time.time() - start_time)
                ts = time.strftime("%H:%M:%S")

                mode = self._cfg.get("slam_profile", "?")
                if mode in ("fastlio2", "pointlio"):
                    mode_str = T.green("MAP")
                elif mode == "localizer":
                    mode_str = T.green("NAV")
                else:
                    mode_str = T.green(mode.upper())

                print(f"  {T.bold('LingTu')} [{mode_str}]  {ts}  {elapsed}s  Ctrl+C to stop\n")

                nav = self._get_module("nav.mission")
                x, y, z, yaw_deg = 0.0, 0.0, 0.0, 0.0
                if nav:
                    x, y, z = nav._robot_pos[0], nav._robot_pos[1], nav._robot_pos[2]
                    try:
                        import math

                        q = nav._latest_quat if hasattr(nav, "_latest_quat") else None
                        if q:
                            yaw_deg = math.degrees(
                                math.atan2(
                                    2 * (q[3] * q[2] + q[0] * q[1]),
                                    1 - 2 * (q[1] * q[1] + q[2] * q[2]),
                                )
                            )
                    except Exception:
                        pass

                print(f"  {T.bold('POSE')}:  x={x:7.2f}  y={y:7.2f}  z={z:5.2f}  yaw={yaw_deg:6.1f} deg")
                print()

                slam = self._get_slam_module()
                if slam:
                    odom = slam.odometry.msg_count if hasattr(slam, "odometry") else 0
                    cloud = slam.map_cloud.msg_count if hasattr(slam, "map_cloud") else 0
                    hz = odom / max(elapsed, 1)
                    print(f"  SLAM:  odom {T.bold(str(odom)):>6s} ({hz:.0f}Hz)  cloud {T.bold(str(cloud)):>6s}")

                if nav:
                    state = nav._state
                    wp = f"{nav._tracker.wp_index}/{nav._tracker.path_length}" if hasattr(nav, "_tracker") else "-"
                    color = T.green if state in ("IDLE", "SUCCESS") else T.yellow if state == "EXECUTING" else T.red
                    print(f"  NAV:   {color(state)}  waypoint {wp}")

                print()
                print(f"  {T.dim('[s]ave map  [g]o target  [x] stop  [n]av x,y  [q]uit')}")

                key = None
                if os.name != "nt":
                    deadline = time.time() + interval
                    while time.time() < deadline:
                        if select.select([sys.stdin], [], [], 0.1)[0]:
                            key = sys.stdin.read(1)
                            break
                else:
                    time.sleep(interval)

                if key == "q":
                    break
                elif key == "s":
                    print("\n  Enter map name: ", end="", flush=True)
                    if os.name != "nt":
                        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                    name = input().strip()
                    if os.name != "nt":
                        tty.setcbreak(sys.stdin.fileno())
                    if name:
                        self._map_cmd({"action": "save", "name": name})
                        time.sleep(2)
                elif key == "g":
                    print("\n  Go to: ", end="", flush=True)
                    if os.name != "nt":
                        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                    target = input().strip()
                    if os.name != "nt":
                        tty.setcbreak(sys.stdin.fileno())
                    if target:
                        self.do_go(target)
                        time.sleep(1)
                elif key == "x":
                    self.do_stop("")
                elif key == "n":
                    print("\n  Navigate x y: ", end="", flush=True)
                    if os.name != "nt":
                        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                    coords = input().strip()
                    if os.name != "nt":
                        tty.setcbreak(sys.stdin.fileno())
                    if coords:
                        self.do_navigate(coords)
                        time.sleep(1)

        except KeyboardInterrupt:
            pass
        finally:
            if os.name != "nt":
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            print("\n  Live stopped")

    def do_module(self, arg):
        """Inspect a module: module <name>"""
        if not arg:
            print("  Usage: module <name>")
            return
        try:
            mod = self._system.get_module(arg)
        except KeyError:
            matches = [n for n in self._system.modules if arg.lower() in n.lower()]
            if matches:
                print(f"  Unknown '{arg}'. Did you mean: {', '.join(matches)}")
            else:
                print(f"  Unknown module: {arg}")
            return
        layer = f"L{mod.layer}" if mod.layer is not None else "L?"
        print(f"\n  {T.bold(arg)} [{layer}]  ({type(mod).__name__})")
        if mod.ports_in:
            print("  Inputs:")
            for pname, port in mod.ports_in.items():
                policy = getattr(port, "_policy_name", "all")
                print(f"    {pname}: {port.msg_type.__name__}  count={port.msg_count}  policy={policy}")
        if mod.ports_out:
            print("  Outputs:")
            for pname, port in mod.ports_out.items():
                n_subs = len(getattr(port, "_callbacks", []))
                print(f"    {pname}: {port.msg_type.__name__}  count={port.msg_count}  subs={n_subs}")
        print()

    do_m = do_module

    def complete_module(self, text, line, begidx, endidx):
        return [n for n in self._system.modules if n.lower().startswith(text.lower())]

    complete_m = complete_module

    def do_connections(self, arg):
        """Show all port wiring."""
        conns = self._system.connections
        if not conns:
            print("  No connections")
            return
        print(f"\n  {len(conns)} connections:\n")
        for out_mod, out_port, in_mod, in_port in conns:
            print(f"  {out_mod}.{out_port} {T.dim('->')} {in_mod}.{in_port}")
        print()

    do_c = do_connections

    def do_log(self, arg):
        """Change log level: log debug|info|warning|error"""
        if not arg:
            cur = logging.getLevelName(logging.getLogger().level)
            print(f"  Current: {cur}")
            print("  Usage: log debug|info|warning|error")
            return
        lvl = getattr(logging, arg.upper(), None)
        if lvl is None:
            print(f"  Invalid level: {arg}")
            return
        logging.getLogger().setLevel(lvl)
        print(f"  Log level -> {arg.upper()}")

    def do_config(self, arg):
        """Show current configuration."""
        print()
        for k, v in sorted(self._cfg.items()):
            if not k.startswith("_"):
                print(f"  {k:20s} {v}")
        print()

    def do_history(self, arg):
        """Mission history: history [list [N]] | history detail <id> | history stats

        Examples:
          history            -> list 10 most recent missions
          history list 20    -> list 20 most recent missions
          history stats      -> aggregate statistics (total, success rate, distance)
          history detail <id>  -> full detail including trajectory waypoints
        """
        parts = arg.split(None, 1)
        subcmd = parts[0].strip().lower() if parts else "list"
        rest = parts[1].strip() if len(parts) > 1 else ""

        mod = self._get_module("MissionLoggerModule")
        if mod is None:
            print("  MissionLoggerModule not running -> start with a semantic-enabled profile")
            return

        if subcmd in ("", "list"):
            count = 10
            if rest and rest.isdigit():
                count = int(rest)
            missions = mod._list_missions_raw(count)
            if not missions:
                print("  No mission records found")
                return
            print(f"\n  {T.bold('Mission history')} (most recent {len(missions)}):\n")
            for m in missions:
                start = m.get("start_time", "?")[:19].replace("T", " ")
                dur = f"{m['duration_sec']:.0f}s" if m.get("duration_sec") else "?"
                dist = f"{m['distance_m']:.1f}m" if m.get("distance_m") else "?"
                result = m.get("result", "?")
                if result in ("COMPLETE", "SUCCESS"):
                    result_str = T.green(result)
                elif result == "FAILED":
                    result_str = T.red(result)
                elif result == "IN_PROGRESS":
                    result_str = T.yellow(result)
                else:
                    result_str = T.dim(result)
                goal = m.get("goal", "")
                goal_str = f"  {T.dim(goal[:40])}" if goal else ""
                replans = f" ({m['replan_count']}x replan)" if m.get("replan_count") else ""
                print(f"  {start}  {result_str:<20} {dur:>6}  {dist:>7}{replans}{goal_str}")
            print()

        elif subcmd == "stats":
            s = mod._stats_raw()
            total = s["total"]
            if total == 0:
                print("  No mission records yet")
                return
            srate = f"{s['success'] / total * 100:.0f}%" if total > 0 else "?"
            print(f"\n  {T.bold('Mission statistics')}")
            print(f"  Total missions : {total}")
            print(f"  Successful     : {s['success']}  ({srate})")
            print(f"  Failed         : {s['failed']}")
            if s["in_progress"]:
                print(f"  In progress    : {T.yellow('1')}")
            print(f"  Total distance : {s['total_distance_m']:.0f} m")
            print(f"  Total time     : {s['total_duration_sec'] / 60:.1f} min")
            print(f"  Log dir        : {T.dim(s['log_dir'])}")
            print()

        elif subcmd == "detail":
            if not rest:
                print("  Usage: history detail <mission-id>")
                return
            data = mod.get_mission(rest)
            if data is None:
                print(f"  Mission not found: {rest}")
                return
            result = data.get("result", "?")
            if result in ("COMPLETE", "SUCCESS"):
                result_str = T.green(result)
            elif result == "FAILED":
                result_str = T.red(result)
            else:
                result_str = T.yellow(result)
            print(f"\n  {T.bold('Mission detail')}")
            print(f"  ID         : {data.get('id', '?')}")
            print(f"  Goal       : {data.get('goal', '?')}")
            print(f"  Result     : {result_str}")
            print(f"  Start      : {data.get('start_time', '?')[:19].replace('T', ' ')}")
            print(f"  End        : {(data.get('end_time') or '?')[:19].replace('T', ' ')}")
            print(f"  Duration   : {data.get('duration_sec', '?')} s")
            print(f"  Distance   : {data.get('distance_m', '?')} m")
            print(f"  Replans    : {data.get('replan_count', 0)}")
            traj = data.get("trajectory", [])
            if traj:
                print(f"  Trajectory : {len(traj)} points")
                for pt in traj[:5]:
                    print(f"    ({pt[0]:.2f}, {pt[1]:.2f})  t={pt[2]:.1f}")
                if len(traj) > 5:
                    print(f"    ... {len(traj) - 5} more points")
            print()

        else:
            print("  Usage: history [list [N]] | history stats | history detail <id>")

    do_hist = do_history

    def do_quit(self, arg):
        """Graceful shutdown."""
        return True

    do_q = do_quit
    do_exit = do_quit

    def do_EOF(self, arg):
        print()
        return True

    def default(self, line):
        if line.strip():
            print(f"  Unknown: {line}. Type {T.bold('help')} for commands.")

    def emptyline(self):
        pass
