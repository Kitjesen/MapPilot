import type { Bounds, XYZ } from '../types/stairs.ts'
import type { MapEditOperation } from '../types/mapEdit.ts'

export class MapEditSaveUnconfirmed extends Error {
  constructor() { super('未收到保存结果，请点击“确认保存结果”继续。'); this.name = 'MapEditSaveUnconfirmed' }
}

export function validMapEditBounds(bounds: Bounds): boolean {
  return (['x', 'y', 'z'] as const).every(axis => Number.isFinite(bounds.min[axis])
    && Number.isFinite(bounds.max[axis]) && bounds.min[axis] < bounds.max[axis])
}

export function pointInMapEditBounds(x: number, y: number, z: number, bounds: Bounds): boolean {
  return x >= bounds.min.x && x <= bounds.max.x
    && y >= bounds.min.y && y <= bounds.max.y
    && z >= bounds.min.z && z <= bounds.max.z
}

export function mapEditKeepsPoint(x: number, y: number, z: number, operations: readonly MapEditOperation[]): boolean {
  if (![x, y, z].every(Number.isFinite)) return false
  return operations.every(op => pointInMapEditBounds(x, y, z, op.bounds) === (op.operation === 'keep_inside'))
}

/** Preview operates on the supplied display cloud; mapd applies edits to the complete source. */
export function applyMapEditOperations(points: readonly number[], operations: readonly MapEditOperation[]): number[] {
  const kept: number[] = []
  for (let i = 0; i + 2 < points.length; i += 3) {
    if (mapEditKeepsPoint(points[i], points[i + 1], points[i + 2], operations)) {
      kept.push(points[i], points[i + 1], points[i + 2])
    }
  }
  return kept
}

/** Count the selected finite points in the supplied current preview, not the source history. */
export function mapEditSelectionImpact(points: readonly number[], bounds: Bounds): { selected: number; total: number } {
  let selected = 0
  let total = 0
  for (let i = 0; i + 2 < points.length; i += 3) {
    const x = points[i], y = points[i + 1], z = points[i + 2]
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) continue
    total++
    if (pointInMapEditBounds(x, y, z, bounds)) selected++
  }
  return { selected, total }
}

export function mapEditCloudBounds(points: readonly number[]): Bounds | null {
  const min: XYZ = { x: Infinity, y: Infinity, z: Infinity }
  const max: XYZ = { x: -Infinity, y: -Infinity, z: -Infinity }
  for (let i = 0; i + 2 < points.length; i += 3) {
    const xyz = [points[i], points[i + 1], points[i + 2]]
    if (!xyz.every(Number.isFinite)) continue
    for (const [j, axis] of (['x', 'y', 'z'] as const).entries()) {
      min[axis] = Math.min(min[axis], xyz[j]); max[axis] = Math.max(max[axis], xyz[j])
    }
  }
  if (!Number.isFinite(min.x)) return null
  for (const axis of ['x', 'y', 'z'] as const) {
    if (min[axis] === max[axis]) { min[axis] -= 0.005; max[axis] += 0.005 }
  }
  return { min, max }
}

export function mapEditWireOperations(operations: readonly MapEditOperation[]) {
  return operations.map(({ operation, bounds }) => ({ operation, bounds: {
    min_x: bounds.min.x, min_y: bounds.min.y, min_z: bounds.min.z,
    max_x: bounds.max.x, max_y: bounds.max.y, max_z: bounds.max.z,
  } }))
}

export function suggestedEditedMapName(source: string): string {
  const suggested = `${source.slice(0, 93)}-edited`
  return suggested === source ? `${source.slice(0, 92)}-edited2` : suggested
}

export function mapEditNameError(name: string, source: string): string | null {
  const target = name.trim()
  if (!target) return '填写新地图名称。'
  if (target === source) return '使用新名称，原地图会保留。'
  if (target.length > 100) return '地图名称最多 100 个字符。'
  if (target === 'operations') return '这个名称已由地图服务使用，请换一个名称。'
  if (target.includes('..')) return '名称不能包含连续的两个点。'
  if (!/^[A-Za-z0-9][A-Za-z0-9_.-]*$/.test(target)) return '名称以字母或数字开头，可包含字母、数字、点、横线和下划线。'
  return null
}
