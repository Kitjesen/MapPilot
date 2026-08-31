import assert from "node:assert/strict";
import test from "node:test";

import type { SceneToolCatalog } from "../src/api.ts";
import { resolveWorldRef } from "../src/sessionIntent.ts";
import {
  buildSceneElementBatch,
  createScenePlacement,
  eligibleSceneSurfaces,
  sceneBounds,
} from "../src/sceneDraft.ts";

const catalog: SceneToolCatalog = {
  schema: "lingtu.sim.studio.scene-tool-catalog.v1",
  scene_tool: "factory-park-hf",
  world_package: "factory_park_hf@1.0.0",
  element_batch_schema: "lingtu.sim.factory-park-element-batch.v1",
  read_only: true,
  executes_runtime: false,
  layout_digest: "a".repeat(64),
  element_types: {
    lane_marker: {
      semantic_class: "lane_marker",
      shape: "box",
      material: "road_marking",
      authority: "VisualOnly",
      allowed_surface_classes: ["parking_area", "road"],
      size_m: [2.5, 0.15, 0.012],
    },
    traffic_cone: {
      semantic_class: "traffic_cone",
      shape: "cylinder",
      material: "orange",
      authority: "PhysicsShared",
      allowed_surface_classes: ["parking_area"],
      radius_m: 0.2,
      half_height_m: 0.35,
    },
  },
  surfaces: [
    {
      surface_id: "parking_apron",
      semantic_class: "parking_area",
      position_xy_m: [25, -34],
      size_xy_m: [38, 34],
      yaw_deg: 0,
    },
    {
      surface_id: "north_road",
      semantic_class: "road",
      position_xy_m: [0, 62],
      size_xy_m: [186, 10],
      yaw_deg: 0,
    },
  ],
  spawn: { position_xy_m: [0, -76], clearance_radius_m: 4 },
};

test("scene placement starts on an eligible surface with deterministic identity", () => {
  assert.deepEqual(
    eligibleSceneSurfaces(catalog, "traffic_cone").map((item) => item.surface_id),
    ["parking_apron"],
  );
  assert.deepEqual(createScenePlacement(catalog, 3), {
    instance_key: "element_03",
    element_type: "lane_marker",
    surface_id: "parking_apron",
    position_xy_m: [25, -34],
    yaw_deg: 0,
  });
});

test("scene batch validation rejects duplicate identities before HTTP", () => {
  const placement = createScenePlacement(catalog, 1);
  assert.throws(
    () => buildSceneElementBatch(catalog, "studio_layout", "", [placement, placement]),
    /unique/,
  );
  const batch = buildSceneElementBatch(catalog, "studio_layout", "preview", [placement]);
  assert.equal(batch.schema, "lingtu.sim.factory-park-element-batch.v1");
  assert.equal(batch.elements[0]?.surface_id, "parking_apron");
});

test("scene bounds include every support surface and spawn clearance", () => {
  const bounds = sceneBounds(catalog);
  assert.ok(bounds.minX < -93);
  assert.ok(bounds.maxX > 93);
  assert.ok(bounds.minY < -80);
  assert.ok(bounds.maxY > 67);
});

test("published world preference wins once it is present in the refreshed catalog", () => {
  const worlds = ["factory_park_hf@1.0.0", "factory_park_smoke@1.0.0"];

  assert.equal(
    resolveWorldRef(worlds, "factory_park_hf@1.0.0", "factory_park_smoke@1.0.0"),
    "factory_park_smoke@1.0.0",
  );
  assert.equal(
    resolveWorldRef(worlds, "factory_park_hf@1.0.0", "not-yet-refreshed@1.0.0"),
    "factory_park_hf@1.0.0",
  );
  assert.equal(resolveWorldRef([], "", "factory_park_smoke@1.0.0"), "");
});
