/** Static, single-height eligibility from the active native global planner. */
import type { PlanPreviewResponse, OdometryEvent } from '../types/index.ts'

export function navigationPreviewIsCurrent(preview: PlanPreviewResponse | null, pose: Pick<OdometryEvent, 'x' | 'y' | 'z'> | null): boolean {
  const start = preview?.start
  return preview?.feasible === true && preview.frame_id === 'map' && !!start && !!pose
    && Math.hypot(start.x - pose.x, start.y - pose.y) <= 0.25
    && typeof pose.z === 'number' && Math.abs(start.z - pose.z) <= 0.15
}

export interface PlanningMap {
  available: boolean
  reason: string
  schema_version?: number
  frame_id?: string
  product_session_id?: string
  map_id?: string
  map_content_epoch?: number
  stamp_s?: number
  resolution?: number
  rows: number
  cols: number
  origin?: [number, number, number]
  reference_z?: number
  cells: number[]
}

export type ReadyPlanningMap = PlanningMap & {
  available: true; resolution: number; origin: [number, number, number]; reference_z: number
}

export function currentPlanningMap(
  map: PlanningMap | null, connected: boolean, mapId: string | null | undefined,
  sessionId: string | null | undefined,
): ReadyPlanningMap | null {
  if (!connected || !map?.available || !mapId || !sessionId
    || map.map_id !== mapId || map.product_session_id !== sessionId
    || map.frame_id !== 'map' || map.schema_version !== 1
    || !map.origin || map.origin.length !== 3 || !map.origin.every(Number.isFinite)
    || !Number.isFinite(map.reference_z)
    || !Number.isFinite(map.resolution) || map.resolution! <= 0
    || !Number.isInteger(map.rows) || !Number.isInteger(map.cols) || map.rows <= 0 || map.cols <= 0
    || map.cells.length !== map.rows * map.cols || map.cells.some(v => v !== 0 && v !== 1 && v !== 2)) return null
  return map as ReadyPlanningMap
}

export function planningCellAt(map: ReadyPlanningMap | null, x: number, y: number): number | null {
  if (!map || !Number.isFinite(x) || !Number.isFinite(y)) return null
  const col = Math.floor((x - map.origin[0]) / map.resolution)
  const row = Math.floor((y - map.origin[1]) / map.resolution)
  if (row < 0 || col < 0 || row >= map.rows || col >= map.cols) return null
  return map.cells[row * map.cols + col]
}

export function planningCellLabel(cell: number | null): string {
  return cell === 1 ? '此处满足静态通行条件，仍需路径预检'
    : cell === 2 ? '此高度层受阻：占据或机身净空不足'
      : cell === 0 ? '此高度层缺少有效地面支撑'
        : '此处没有可通行性数据'
}

export function planningMapUnavailableLabel(reason: string | undefined): string {
  if (reason === 'navigation_status_stale') return '导航数据未更新'
  if (reason?.includes('mismatch')) return '正在同步当前地图'
  if (reason?.includes('invalid') || reason?.includes('failed')) return '通行图生成失败'
  return '等待规划器通行图'
}

// Alpha and nearest-neighbour sampling preserve discrete cell boundaries.
export const PLANNING_CELL_COLORS = [
  [113, 118, 127, 105], [99, 174, 152, 195], [213, 113, 100, 225],
] as const
