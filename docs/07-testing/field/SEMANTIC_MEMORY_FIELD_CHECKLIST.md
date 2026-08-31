# Semantic Memory Field-Run Checklist

Status: current reusable field checklist; completed runs belong in `field-runs/`

Use this checklist before claiming on-robot semantic world-memory quality.
Record the completed run in `docs/07-testing/field-runs/YYYY-MM-DD.md`.

## Preconditions

- Active profile includes `SemanticMapperModule`, `SemanticPlannerModule`, and the memory stack.
- `smap status` reports a writable semantic save directory.
- Gateway health is stable before any semantic-memory claim.
- The test area is safe for two repeated loops through the same rooms or zones.

## Session Flow

1. Start a fresh semantic-memory session and note the save directory.
2. Walk loop A through the test area while recording video or bag evidence.
3. Run `smap status` and `smap rooms`; record room count, object count, and current room.
4. Save the semantic map with `smap save`; confirm `room_object_kg.json` and `topology_graph.json` exist.
5. Walk loop B through the same area.
6. Issue at least 10 semantic instructions that refer to objects or rooms seen in loop A.
7. Record whether each instruction resolves through known memory, live scene graph, frontier exploration, or visual servo fallback.

## Required Evidence

| Metric | Pass target |
| --- | --- |
| Semantic map growth | `smap status` shows increasing KG/TSG counts during loop A |
| Save/load | `smap save` succeeds and files are reloadable in the same session |
| Second-loop hit rate | At least 7/10 repeated semantic instructions resolve without random exploration |
| Duplicate exploration | Revisited known rooms are not repeatedly selected as unexplored targets |
| Planner memory consumption | Logs show `SemanticMapperModule.topo_summary/room_graph` reaching `SemanticPlannerModule` |

## Failure Notes

Mark the run `BLOCKED` instead of `FAIL` when upstream room segmentation is unstable enough that `smap rooms` never becomes meaningful.
Mark it `FAIL` when the semantic map grows but planner decisions do not use it.
