import assert from "node:assert/strict";
import test from "node:test";

import type { SourceInspection } from "../src/api.ts";
import {
  applySourceRecommendations,
  setImportRequestField,
} from "../src/importDraft.ts";

function inspection(): SourceInspection {
  return {
    schema: "lingtu.sim.studio.source-inspection.v1",
    source: {
      source_id: "source01",
      entry: "objects/source01.zip",
      bytes: 128,
      archive_format: "zip",
    },
    summary: { files: 4, total_bytes: 96 },
    candidates: {
      robot_models: [
        { path: "model/robot.xml", size: 40, sha256: "b".repeat(64), format: "mjcf" },
      ],
      licenses: [
        { path: "legal/LICENSE.txt", size: 20, sha256: "c".repeat(64) },
      ],
      heightmaps: [
        { path: "terrain/height.r16", size: 16, sha256: "d".repeat(64) },
      ],
      meshes: [
        { path: "terrain/collision.obj", size: 20, sha256: "e".repeat(64) },
      ],
      textures: [],
    },
    recommendations: {
      robot: {
        source_format: "mjcf",
        source_model: "model/robot.xml",
        license_file: "legal/LICENSE.txt",
      },
      world: {
        heightmap: "terrain/height.r16",
        mesh: "terrain/collision.obj",
        license_file: "legal/LICENSE.txt",
      },
    },
    files: [],
  };
}

test("robot source recommendations update only importer-owned file selections", () => {
  const template = {
    schema: "lingtu.sim.robot-import-request.v1",
    id: "my_robot",
    source_format: "urdf",
    source_model: "old.urdf",
    provenance: {
      owner: "Asset owner",
      license: "LicenseRef-Test",
      license_file: "OLD-LICENSE",
      source_uri: "https://example.invalid/robot",
    },
    physics: { attach_root: "base_link", root_joint: "floating_base_joint" },
  };

  const updated = applySourceRecommendations("robot", template, inspection());

  assert.deepEqual(updated, {
    ...template,
    source_format: "mjcf",
    source_model: "model/robot.xml",
    provenance: {
      ...template.provenance,
      license_file: "legal/LICENSE.txt",
    },
  });
  assert.deepEqual(template, {
    schema: "lingtu.sim.robot-import-request.v1",
    id: "my_robot",
    source_format: "urdf",
    source_model: "old.urdf",
    provenance: {
      owner: "Asset owner",
      license: "LicenseRef-Test",
      license_file: "OLD-LICENSE",
      source_uri: "https://example.invalid/robot",
    },
    physics: { attach_root: "base_link", root_joint: "floating_base_joint" },
  });
});

test("world source recommendations select heightmap mesh and license without replacing dimensions", () => {
  const template = {
    schema: "lingtu.sim.world-import-request.v1",
    package: { id: "my_world", version: "1.0.0" },
    source: {
      provenance: {
        owner: "Asset owner",
        license: "LicenseRef-Test",
        license_file: "OLD-LICENSE",
        source_uri: "https://example.invalid/world",
      },
    },
    heightmap: {
      path: "old.r16",
      width: 2048,
      height: 1024,
      extent_m: [200, 100],
      elevation_min_m: -4,
      elevation_max_m: 12,
    },
  };

  const updated = applySourceRecommendations("world", template, inspection());

  assert.deepEqual(updated, {
    ...template,
    source: {
      provenance: {
        ...template.source.provenance,
        license_file: "legal/LICENSE.txt",
      },
    },
    heightmap: {
      ...template.heightmap,
      path: "terrain/height.r16",
    },
    mesh: { path: "terrain/collision.obj", collision: true },
  });
  assert.equal("mesh" in template, false);
});

test("guided import fields update nested values immutably", () => {
  const request = {
    schema: "lingtu.sim.world-import-request.v1",
    heightmap: {
      width: 1024,
      extent_m: [100, 80],
    },
  };

  const updated = setImportRequestField(request, ["heightmap", "extent_m", 0], 250);

  assert.deepEqual(updated, {
    schema: "lingtu.sim.world-import-request.v1",
    heightmap: {
      width: 1024,
      extent_m: [250, 80],
    },
  });
  assert.deepEqual(request.heightmap.extent_m, [100, 80]);
  assert.throws(
    () => setImportRequestField(request, ["heightmap", "missing", 0], 1),
    /missing/,
  );
});
