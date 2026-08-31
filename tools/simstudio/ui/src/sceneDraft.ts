import type {
  SceneElementBatch,
  SceneElementPlacement,
  SceneSurface,
  SceneToolCatalog,
} from "./api.ts";

const SAFE_ID = /^[a-z][a-z0-9_]{0,63}$/;

export interface SceneBounds {
  minX: number;
  minY: number;
  maxX: number;
  maxY: number;
  width: number;
  height: number;
}

export function eligibleSceneSurfaces(
  catalog: SceneToolCatalog,
  elementType: string,
): SceneSurface[] {
  const template = catalog.element_types[elementType];
  if (!template) return [];
  const allowed = new Set(template.allowed_surface_classes);
  return catalog.surfaces
    .filter((surface) => allowed.has(surface.semantic_class))
    .sort((left, right) => left.surface_id.localeCompare(right.surface_id));
}

export function createScenePlacement(
  catalog: SceneToolCatalog,
  ordinal: number,
): SceneElementPlacement {
  if (!Number.isInteger(ordinal) || ordinal < 1 || ordinal > 9999) {
    throw new Error("scene element ordinal must be an integer in [1, 9999]");
  }
  const elementType = Object.keys(catalog.element_types).sort()[0];
  if (!elementType) throw new Error("scene catalog has no element types");
  const surfaces = eligibleSceneSurfaces(catalog, elementType);
  const surface = surfaces.find((candidate) => candidate.semantic_class === "parking_area")
    ?? surfaces[0];
  if (!surface) throw new Error(`scene element ${elementType} has no eligible surface`);
  return {
    instance_key: `element_${String(ordinal).padStart(2, "0")}`,
    element_type: elementType,
    surface_id: surface.surface_id,
    position_xy_m: [...surface.position_xy_m],
    yaw_deg: 0,
  };
}

export function buildSceneElementBatch(
  catalog: SceneToolCatalog,
  batchId: string,
  description: string,
  elements: SceneElementPlacement[],
): SceneElementBatch {
  const normalizedId = batchId.trim();
  if (!SAFE_ID.test(normalizedId)) {
    throw new Error("batch id must match [a-z][a-z0-9_]{0,63}");
  }
  if (!Array.isArray(elements) || elements.length === 0) {
    throw new Error("scene batch must contain at least one element");
  }
  const identities = new Set<string>();
  const normalized = elements.map((element, index): SceneElementPlacement => {
    const identity = element.instance_key.trim();
    if (!SAFE_ID.test(identity)) {
      throw new Error(`elements[${index}] instance key is invalid`);
    }
    if (identities.has(identity)) {
      throw new Error("scene element instance keys must be unique");
    }
    identities.add(identity);
    const template = catalog.element_types[element.element_type];
    const surface = catalog.surfaces.find(
      (candidate) => candidate.surface_id === element.surface_id,
    );
    if (!template || !surface) {
      throw new Error(`elements[${index}] references unknown catalog content`);
    }
    if (!template.allowed_surface_classes.includes(surface.semantic_class)) {
      throw new Error(`elements[${index}] is not allowed on its selected surface`);
    }
    if (
      element.position_xy_m.length !== 2
      || element.position_xy_m.some((value) => !Number.isFinite(value))
      || !Number.isFinite(element.yaw_deg)
    ) {
      throw new Error(`elements[${index}] placement must be finite`);
    }
    return {
      instance_key: identity,
      element_type: element.element_type,
      surface_id: element.surface_id,
      position_xy_m: [...element.position_xy_m],
      yaw_deg: element.yaw_deg,
    };
  });
  return {
    schema: "lingtu.sim.factory-park-element-batch.v1",
    batch_id: normalizedId,
    description: description.trim(),
    elements: normalized,
  };
}

export function sceneBounds(catalog: SceneToolCatalog): SceneBounds {
  if (catalog.surfaces.length === 0) {
    throw new Error("scene catalog has no support surfaces");
  }
  let minX = Number.POSITIVE_INFINITY;
  let minY = Number.POSITIVE_INFINITY;
  let maxX = Number.NEGATIVE_INFINITY;
  let maxY = Number.NEGATIVE_INFINITY;
  for (const surface of catalog.surfaces) {
    const radians = (surface.yaw_deg * Math.PI) / 180;
    const halfX = (
      Math.abs(Math.cos(radians)) * surface.size_xy_m[0]
      + Math.abs(Math.sin(radians)) * surface.size_xy_m[1]
    ) / 2;
    const halfY = (
      Math.abs(Math.sin(radians)) * surface.size_xy_m[0]
      + Math.abs(Math.cos(radians)) * surface.size_xy_m[1]
    ) / 2;
    minX = Math.min(minX, surface.position_xy_m[0] - halfX);
    maxX = Math.max(maxX, surface.position_xy_m[0] + halfX);
    minY = Math.min(minY, surface.position_xy_m[1] - halfY);
    maxY = Math.max(maxY, surface.position_xy_m[1] + halfY);
  }
  const [spawnX, spawnY] = catalog.spawn.position_xy_m;
  const clearance = catalog.spawn.clearance_radius_m;
  minX = Math.min(minX, spawnX - clearance);
  maxX = Math.max(maxX, spawnX + clearance);
  minY = Math.min(minY, spawnY - clearance);
  maxY = Math.max(maxY, spawnY + clearance);
  const margin = Math.max(5, Math.max(maxX - minX, maxY - minY) * 0.04);
  minX -= margin;
  minY -= margin;
  maxX += margin;
  maxY += margin;
  return {
    minX,
    minY,
    maxX,
    maxY,
    width: maxX - minX,
    height: maxY - minY,
  };
}
