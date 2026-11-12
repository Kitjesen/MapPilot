import type { Bounds, XYZ } from '../types/stairs.ts'
import type { MapCropOperation, MapEditOperation, MapPlaneOperation, MapEditSource } from '../types/mapEdit.ts'

export function readMapEditRecipe(result: Record<string, unknown>): { source: MapEditSource; operations: MapEditOperation[] } {
  const recipe = result.preview_request as Record<string, unknown> | undefined
  if (!recipe || typeof recipe.map_id !== 'string' || !recipe.map_id
    || !Number.isSafeInteger(recipe.expected_source_content_epoch) || (recipe.expected_source_content_epoch as number) < 1
    || !Array.isArray(recipe.operations) || typeof result.frame_id !== 'string' || !result.frame_id) {
    throw new Error('预览缺少地图版本或坐标系，请重新生成。')
  }
  const operations: unknown[] = recipe.operations.map(value => {
    if (!value || typeof value !== 'object') return value
    const edit = value as Record<string, unknown>
    if (!edit.bounds || typeof edit.bounds !== 'object') return edit
    const bounds = edit.bounds as Record<string, number>
    return { ...edit, bounds: { min: { x: bounds.min_x, y: bounds.min_y, z: bounds.min_z },
      max: { x: bounds.max_x, y: bounds.max_y, z: bounds.max_z } } }
  })
  if (!operations.every(validMapEditOperation)) throw new Error('预览参数无法显示，请重新生成。')
  return { source: { name: recipe.map_id, contentEpoch: recipe.expected_source_content_epoch as number, frameId: result.frame_id }, operations }
}

export class MapEditSaveUnconfirmed extends Error {
  constructor() { super('未收到保存结果，请点击“确认保存结果”继续。'); this.name = 'MapEditSaveUnconfirmed' }
}

export function validMapEditBounds(bounds: Bounds): boolean {
  return (['x', 'y', 'z'] as const).every(axis => Number.isFinite(bounds.min[axis])
    && Number.isFinite(bounds.max[axis]) && bounds.min[axis] < bounds.max[axis])
}

export function isMapCrop(edit: MapEditOperation): edit is MapCropOperation {
  return edit.operation === 'delete_inside' || edit.operation === 'keep_inside'
}

export function isLocalMapEdit(edit: MapEditOperation): edit is MapCropOperation | MapPlaneOperation {
  return isMapCrop(edit) || edit.operation === 'add_plane'
}

export function planePoints(edit: MapPlaneOperation): number[] {
  const axes = (['x', 'y', 'z'] as const).filter(axis => axis !== edit.axis)
  const [u, v] = axes
  const { min, max } = edit.bounds
  const nu = Math.ceil((max[u] - min[u]) / edit.spacing), nv = Math.ceil((max[v] - min[v]) / edit.spacing)
  const points: number[] = []
  const point = { x: 0, y: 0, z: 0, [edit.axis]: edit.offset }
  for (let i = 0; i <= nu; i++) for (let j = 0; j <= nv; j++) {
    point[u] = min[u] + (max[u] - min[u]) * i / nu
    point[v] = min[v] + (max[v] - min[v]) * j / nv
    points.push(point.x, point.y, point.z)
  }
  return points
}

export function validMapEditOperation(value: unknown): value is MapEditOperation {
  if (value === null || typeof value !== 'object') return false
  const edit = value as Record<string, unknown>
  const positive = (key: string) => typeof edit[key] === 'number' && Number.isFinite(edit[key]) && edit[key] > 0
  const count = (key: string) => positive(key) && Number.isInteger(edit[key])
  switch (edit.operation) {
    case 'add_plane': {
      const plane = value as MapPlaneOperation
      if (!['x', 'y', 'z'].includes(plane.axis) || !plane.bounds?.min || !plane.bounds.max
        || !validMapEditBounds(plane.bounds) || !positive('spacing') || !Number.isFinite(plane.offset)) return false
      const { min, max } = plane.bounds
      const count = (['x', 'y', 'z'] as const).filter(axis => axis !== plane.axis)
        .reduce((count, axis) => count * (Math.ceil((max[axis] - min[axis]) / plane.spacing) + 1), 1)
      return plane.offset >= min[plane.axis] && plane.offset <= max[plane.axis] && count <= 1_000_000
    }
    case 'delete_inside': case 'keep_inside': {
      const b = edit.bounds as MapCropOperation['bounds'] | undefined
      return !!b?.min && !!b.max && validMapEditBounds(b)
    }
    case 'voxel': return positive('voxel_size')
    case 'statistical': return count('neighbors') && positive('std_ratio')
    case 'radius': return positive('radius') && count('min_points')
    case 'clusters': return positive('radius') && count('neighbors') && count('min_cluster_points')
    case 'plane_crop': return positive('distance_threshold') && typeof edit.margin === 'number' && Number.isFinite(edit.margin) && edit.margin >= 0
    default: return false
  }
}

export function mapEditLabel(edit: MapEditOperation): string {
  return { delete_inside: '删除框内', keep_inside: '裁掉框外', voxel: '体素降采样',
    statistical: '统计离群过滤', radius: '半径离群过滤', clusters: '删除小簇', plane_crop: '平面范围裁剪', add_plane: '人工补面' }[edit.operation]
}

export function pointInMapEditBounds(x: number, y: number, z: number, bounds: Bounds): boolean {
  return x >= bounds.min.x && x <= bounds.max.x
    && y >= bounds.min.y && y <= bounds.max.y
    && z >= bounds.min.z && z <= bounds.max.z
}

export function mapEditKeepsPoint(x: number, y: number, z: number, operations: readonly MapEditOperation[]): boolean {
  if (![x, y, z].every(Number.isFinite)) return false
  return operations.every(op => {
    if (op.operation === 'add_plane') return true
    if (!isMapCrop(op)) throw new Error('清洗操作需要原生地图预览。')
    return pointInMapEditBounds(x, y, z, op.bounds) === (op.operation === 'keep_inside')
  })
}

/** Preview operates on the supplied display cloud; mapd applies edits to the complete source. */
export function applyMapEditOperations(points: ArrayLike<number>, operations: readonly MapEditOperation[]): number[] {
  if (operations.some(edit => edit.operation === 'add_plane')) {
    let current = Array.from(points)
    for (const edit of operations) {
      if (edit.operation === 'add_plane') {
        for (const value of planePoints(edit)) current.push(value)
      } else current = applyMapEditOperations(current, [edit])
    }
    return current
  }
  const kept: number[] = []
  for (let i = 0; i + 2 < points.length; i += 3) {
    if (mapEditKeepsPoint(points[i], points[i + 1], points[i + 2], operations)) {
      kept.push(points[i], points[i + 1], points[i + 2])
    }
  }
  return kept
}

/** Count the selected finite points in the supplied current preview, not the source history. */
export function mapEditSelectionImpact(points: ArrayLike<number>, bounds: Bounds): { selected: number; total: number } {
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

export function mapEditCloudBounds(points: ArrayLike<number>): Bounds | null {
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
  return operations.map(edit => {
    if (!isLocalMapEdit(edit)) return { ...edit }
    const { operation, bounds } = edit
    return { ...edit, operation, bounds: {
      min_x: bounds.min.x, min_y: bounds.min.y, min_z: bounds.min.z,
      max_x: bounds.max.x, max_y: bounds.max.y, max_z: bounds.max.z,
    } }
  })
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
