import type {
  ImportKind,
  JsonObject,
  SourceInspection,
} from "./api.ts";

function cloneJson(value: JsonObject): JsonObject {
  return JSON.parse(JSON.stringify(value)) as JsonObject;
}

function objectField(value: unknown, context: string): JsonObject {
  if (value === null || typeof value !== "object" || Array.isArray(value)) {
    throw new Error(`${context} must be an object before source recommendations can be applied`);
  }
  return value as JsonObject;
}

export function setImportRequestField(
  request: JsonObject,
  path: readonly (string | number)[],
  value: unknown,
): JsonObject {
  if (path.length === 0) {
    throw new Error("import request field path must not be empty");
  }
  const updated = cloneJson(request);
  let cursor: unknown = updated;
  for (const [index, segment] of path.entries()) {
    const context = path.slice(0, index + 1).join(".");
    const last = index === path.length - 1;
    if (typeof segment === "number") {
      if (!Array.isArray(cursor) || segment < 0 || segment >= cursor.length) {
        throw new Error(`import request field ${context} does not exist`);
      }
      if (last) {
        cursor[segment] = value;
        return updated;
      }
      cursor = cursor[segment];
      continue;
    }
    if (
      cursor === null
      || typeof cursor !== "object"
      || Array.isArray(cursor)
      || !Object.prototype.hasOwnProperty.call(cursor, segment)
    ) {
      throw new Error(`import request field ${context} does not exist`);
    }
    const object = cursor as JsonObject;
    if (last) {
      object[segment] = value;
      return updated;
    }
    cursor = object[segment];
  }
  return updated;
}

export function applySourceRecommendations(
  kind: ImportKind,
  request: JsonObject,
  inspection: SourceInspection,
): JsonObject {
  const updated = cloneJson(request);
  if (kind === "robot") {
    const recommendation = inspection.recommendations.robot;
    if (!recommendation) return updated;
    updated.source_format = recommendation.source_format;
    updated.source_model = recommendation.source_model;
    if (recommendation.license_file !== null) {
      objectField(updated.provenance, "robot import provenance").license_file = recommendation.license_file;
    }
    return updated;
  }
  const recommendation = inspection.recommendations.world;
  if (!recommendation) return updated;
  objectField(updated.heightmap, "world import heightmap").path = recommendation.heightmap;
  const source = objectField(updated.source, "world import source");
  if (recommendation.license_file !== null) {
    objectField(source.provenance, "world import provenance").license_file = recommendation.license_file;
  }
  if (recommendation.mesh !== null) {
    updated.mesh = { path: recommendation.mesh, collision: true };
  }
  return updated;
}
