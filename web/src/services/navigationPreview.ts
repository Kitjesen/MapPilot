import type { PlanPreviewResponse, OdometryEvent } from '../types/index.ts'

export function navigationPreviewIsCurrent(preview: PlanPreviewResponse | null, pose: Pick<OdometryEvent, 'x' | 'y' | 'z'> | null): boolean {
  const start = preview?.start
  return preview?.feasible === true && preview.frame_id === 'map' && !!start && !!pose
    && Math.hypot(start.x - pose.x, start.y - pose.y) <= 0.25
    && typeof pose.z === 'number' && Math.abs(start.z - pose.z) <= 0.15
}

/** A planar click targets the current body height; an explicit 3D target keeps its height. */
export function sceneGoalHeight(bodyZ: number | null | undefined, selectedZ?: number): number | null {
  const z = selectedZ ?? bodyZ
  return typeof z === 'number' && Number.isFinite(z) ? z : null
}
