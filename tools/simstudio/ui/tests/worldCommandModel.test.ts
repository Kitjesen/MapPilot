import assert from "node:assert/strict";
import test from "node:test";

import {
  buildWorldCommandState,
  createWorldForgeEditorState,
  selectWorldForgeAsset,
  updateWorldForgeRoadInspector,
  SIMSTUDIO_DEFAULT_VIEW,
  WORLD_FORGE_ASSETS,
  WORLD_FORGE_REGIONS,
  WORLD_COMMAND_NAVIGATION,
} from "../src/worldCommandModel.ts";
import type { HealthStatus } from "../src/api.ts";

test("simstudio opens on runtime management instead of the web authoring preview", () => {
  assert.equal(SIMSTUDIO_DEFAULT_VIEW, "runs");
});

test("world command navigation exposes exactly the five product targets", () => {
  assert.deepEqual(
    WORLD_COMMAND_NAVIGATION.map((item) => item.id),
    ["world", "models", "create", "runs", "evidence"],
  );
});

test("world navigation identifies the web authoring preview without claiming a UE5 viewport", () => {
  const world = WORLD_COMMAND_NAVIGATION.find((item) => item.id === "world");

  assert.deepEqual(world, {
    id: "world",
    label: "World Preview",
    detail: "Web authoring preview · not UE5 runtime",
  });
});

test("world forge exposes the screenshot-aligned editor regions", () => {
  assert.deepEqual(WORLD_FORGE_REGIONS, [
    "topbar",
    "prompt",
    "toolRail",
    "worldCanvas",
    "routeOverlay",
    "collaborationOverlay",
    "inspector",
    "assetTray",
    "warningToast",
  ]);
});

test("world forge asset tray follows the factory authoring order", () => {
  assert.deepEqual(
    WORLD_FORGE_ASSETS.map((asset) => asset.id),
    [
      "terrain",
      "road",
      "warehouse",
      "fence",
      "pedestrian",
      "thunderv4",
      "sensor",
      "mission",
    ],
  );
});

test("world forge starts in selection mode with the reference road inspector", () => {
  const editor = createWorldForgeEditorState();

  assert.equal(editor.selectedTool, "select");
  assert.equal(editor.selectedAsset, "road");
  assert.deepEqual(editor.roadInspector, {
    name: "Road Segment 12",
    widthMeters: 10,
    material: "Asphalt",
    navigationEnabled: true,
  });
});

test("selecting an asset changes authoring selection without mutating runtime truth", () => {
  const original = createWorldForgeEditorState();
  const selected = selectWorldForgeAsset(original, "thunderv4");

  assert.equal(original.selectedAsset, "road");
  assert.equal(selected.selectedAsset, "thunderv4");
  assert.deepEqual(selected.runtimeTruth, original.runtimeTruth);
});

test("road inspector updates immutably", () => {
  const original = createWorldForgeEditorState();
  const updated = updateWorldForgeRoadInspector(original, {
    widthMeters: 12,
    material: "Concrete",
    navigationEnabled: false,
  });

  assert.equal(original.roadInspector.widthMeters, 10);
  assert.deepEqual(updated.roadInspector, {
    name: "Road Segment 12",
    widthMeters: 12,
    material: "Concrete",
    navigationEnabled: false,
  });
});

test("world command disables live UE viewport when runtime is not bound", () => {
  const state = buildWorldCommandState({
    health: health({ status: "ok", runtime_bound: false }),
  });

  assert.equal(state.stage.canShowLiveViewport, false);
  assert.equal(state.stage.status, "runtime-missing");
  assert.equal(state.stage.label, "Run service not bound");
});

test("world command reports loading before the health request settles", () => {
  const state = buildWorldCommandState({
    health: null,
    healthLoading: true,
  });

  assert.equal(state.stage.canShowLiveViewport, false);
  assert.equal(state.stage.status, "loading");
});

test("world command reports service error after the health request fails", () => {
  const state = buildWorldCommandState({
    health: null,
    healthError: "无法连接本地 SimStudio 服务",
  });

  assert.equal(state.stage.canShowLiveViewport, false);
  assert.equal(state.stage.status, "service-error");
});

test("world command keeps a bound runtime separate from the Web authoring preview", () => {
  const state = buildWorldCommandState({
    health: health({ status: "ok", runtime_bound: true }),
  });

  assert.equal(state.stage.canShowLiveViewport, false);
  assert.equal(state.stage.status, "authoring-preview");
  assert.equal(state.stage.label, "Web authoring preview · UE5 runtime is separate");
});

test("verified UE evidence does not turn the Web authoring preview into a runtime viewport", () => {
  const state = buildWorldCommandState({
    health: health({ status: "ok", runtime_bound: true }),
    verifiedVisualFrame: true,
  });

  assert.equal(state.stage.canShowLiveViewport, false);
  assert.equal(state.stage.status, "authoring-preview");
  assert.equal(state.stage.label, "Web authoring preview · UE5 runtime is separate");
});

test("world command disables live UE viewport when the service reports an error", () => {
  const state = buildWorldCommandState({
    health: health({ status: "error", runtime_bound: true }),
    verifiedVisualFrame: true,
  });

  assert.equal(state.stage.canShowLiveViewport, false);
  assert.equal(state.stage.status, "service-error");
});

function health(
  overrides: Pick<HealthStatus, "status" | "runtime_bound">,
): HealthStatus {
  return {
    service: "simstudio",
    api_version: "v1",
    field_isolated: true,
    ...overrides,
  };
}
