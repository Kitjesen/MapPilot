import { MapSaveError, type MapSaveOperationStatus, type MapSaveProcessingStep, type SaveMapResult } from './api.ts'
import type { MapInfo } from '../types/index.ts'
import { mapIsActivationReady } from './mapReadiness.ts'

export interface MapLibraryEntry {
  name: string
  map?: MapInfo
  operations: MapSaveOperationStatus[]
}

export function mapLibraryEntries(maps: MapInfo[], operations: MapSaveOperationStatus[]): MapLibraryEntry[] {
  const entries = new Map<string, MapLibraryEntry>(maps.map(map => [map.name, { name: map.name, map, operations: [] }]))
  for (const operation of [...operations].sort((a, b) => (b.created_at_ns ?? 0) - (a.created_at_ns ?? 0))) {
    const name = operation.map_id || operation.name || '未命名地图'
    const entry = entries.get(name) ?? { name, operations: [] }
    entry.operations.push(operation)
    entries.set(name, entry)
  }
  return [...entries.values()]
}

export function mapLibraryStatus(map?: MapInfo, operation?: MapSaveOperationStatus, unavailable = false) {
  if (unavailable) return { kind: 'attention', label: '保存状态待确认', detail: '暂时无法更新保存记录，请刷新后确认。' } as const
  if (operation && mapSaveStateLabel(operation.state) === '进行中') return {
    kind: 'busy', label: operation.cancel_requested ? '取消待确认'
      : operation.state === 'QUEUED' ? '等待处理'
      : operation.phase ? mapSavePhaseLabel(operation.phase) : '正在保存',
    detail: operation.source_saved ? '原始点云已保存，导航地图仍在处理。' : '正在保存建图数据。',
  } as const
  if (operation && operation.state !== 'SUCCEEDED') return {
    kind: 'attention', label: mapSaveStateLabel(operation.state),
    detail: map && mapIsActivationReady(map) ? '本次更新尚未完成，已有导航地图仍保留。'
      : operation.source_saved ? '原始点云已保存，导航地图尚未就绪。' : '地图尚未保存完成。',
  } as const
  if (map && mapIsActivationReady(map)) return {
    kind: 'ready', label: '地图就绪', detail: '导航数据齐全，使用前仍需完成定位。',
  } as const
  return {
    kind: 'attention', label: map ? '地图未就绪' : '仅有保存记录',
    detail: map ? '地图信息不完整，暂不能用于导航。' : '当前地图库中没有这张地图，可在记录中查看存档。',
  } as const
}

interface MapSaveStatusDetails {
  name: string
  detail: string
  location?: string
  summary?: string
}
export type MapSaveStatus = MapSaveStatusDetails & (
  | { state: 'saving' | 'saved' }
  | { state: 'failed'; operation?: SaveMapResult & { operation_id: string } }
  | { state: 'archived'; operation: SaveMapResult & { operation_id: string } }
  | { state: 'pending'; operation: SaveMapResult & { operation_id: string } }
)

const PHASES: Record<string, string> = {
  CAPTURE: '保存点云', VALIDATE: '检查建图数据', OPTIMIZE_SOURCE: '优化建图轨迹',
  PROCESS_SOURCE: '清理动态点', BUILD_ARTIFACTS: '生成导航地图', VERIFY: '检查地图产物',
  COMMIT: '写入机载地图', DONE: '处理完成',
}
const REASONS: Record<string, string> = {
  sequential_chain_incomplete: '连续轨迹约束不完整',
  insufficient_keyframes: '关键帧不足',
  no_verified_loops: '没有已验证的回环',
  optimization_disabled: '未启用优化',
  disabled: '未启用',
  optimized: '优化完成',
  snapshot_failed: '地图快照生成失败',
  artifact_failed: '导航地图生成失败',
  source_failed: '地图源数据处理失败',
  dynamic_filter_failed: '动态点清理失败',
  snapshot_session_expired: '原建图会话已结束，请重新保存地图。',
  source_not_saved: '点云尚未存档，请先完成保存。',
  saved_source_missing: '点云存档文件不可用，请核对保存记录。',
  job_not_ready: '请等待当前保存任务结束后再重新优化。',
  patch_bundle_incomplete: '缺少完整的分帧扫描或位姿，无法重新优化。',
  active_map_conflict: '该地图正在使用，暂时不能重新处理。',
  pgo_timeout: '几何优化超时，原始点云仍保留。',
  cancelled: '已取消',
  canceled: '已取消',
}

export function mapSaveStateLabel(state?: string | null): string {
  if (state === 'CAPTURED') return '等待处理'
  if (state === 'SUCCEEDED') return '成功'
  if (state === 'FAILED') return '失败'
  if (state === 'CANCELLED' || state === 'CANCELED') return '已取消'
  if (state === 'WAITING_SNAPSHOT' || state === 'QUEUED' || state === 'RUNNING') return '进行中'
  return state || '状态待确认'
}

export function mapSavePhaseLabel(phase?: string | null): string {
  return phase ? PHASES[phase.toUpperCase()] ?? phase : '阶段未提供'
}

export function mapSaveReason(operation: MapSaveOperationStatus): string {
  const code = operation.reason_code
  if (code) return REASONS[code] ? `${REASONS[code]}（${code}）` : code
  return operation.message || '未提供原因'
}

export function failedMapSaveStatus(name: string, error: unknown): MapSaveStatus {
  const result = error instanceof MapSaveError ? error.result : undefined
  const id = result?.operation_id ?? result?.operation?.operation_id
  const operation = result && id ? { ...result, name: result.name || name, operation_id: id } : undefined
  return {
    name, state: 'failed', operation,
    detail: `${result?.operation?.source_saved ? 'PCD 已保存，处理失败。' : ''}${error instanceof Error ? error.message : String(error)}`,
    summary: result?.operation
      ? `${mapSavePhaseLabel(result.operation.phase)} · ${mapSaveReason(result.operation)}` : undefined,
  }
}

export function archivedMapStatus(result: SaveMapResult & { operation_id: string }): MapSaveStatus {
  const operation = result.operation
  if (operation?.state === 'FAILED') return failedMapSaveStatus(
    result.name, new MapSaveError('请点击地图状态图标查看失败原因。', result),
  )
  return {
    name: result.name, state: 'archived', operation: result,
    detail: operation?.state === 'CAPTURED' ? 'PCD 已保存，可点击地图状态图标开始处理。' : 'PCD 已保存，可点击地图状态图标查看处理进度。',
    summary: '此处保存的是建图点云存档。导航产物完成后会更新地图库。',
  }
}

export function mappingCapacityNotice(
  product: string | null | undefined,
  diagnostics: Record<string, unknown> | null | undefined,
): string | null {
  const mapping = diagnostics?.global_mapping
  return product === 'map' && typeof mapping === 'object' && mapping !== null
    && 'state' in mapping && mapping.state === 'mapping_keyframe_capacity_reached'
    ? '建图容量已满，已停止记录新区域；定位仍会处理新数据。保存仅包含满额前的记录。'
    : null
}

export function mappingFailureNotice(
  product: string | null | undefined,
  diagnostics: Record<string, unknown> | null | undefined,
): string | null {
  const mapping = diagnostics?.global_mapping
  if (product !== 'map' || typeof mapping !== 'object' || mapping === null
    || !('state' in mapping) || typeof mapping.state !== 'string'
    || !mapping.state.startsWith('online_mapping_failed:')) return null
  return 'points' in mapping && typeof mapping.points === 'number' && mapping.points > 0
    ? '在线建图处理失败；保留上次生成的地图预览。'
    : '在线建图处理失败；还没有可用的地图预览。'
}

export function mapSaveProgressValue(result?: SaveMapResult): number | undefined {
  const value = result?.operation?.progress
  return typeof value === 'number' && Number.isFinite(value)
    ? Math.max(0, Math.min(1, value)) : undefined
}

export function mapSaveElapsedMs(result: SaveMapResult | undefined, fallbackMs: number): number {
  const start = result?.operation?.created_at_ns
  const end = result?.operation?.completed_at_ns
  const serverNow = result?.ts
  if (typeof start === 'number' && start > 0) {
    const endMs = typeof end === 'number' && end > 0 ? end / 1e6
      : typeof serverNow === 'number' && serverNow > 0 ? serverNow * 1000 : undefined
    if (endMs !== undefined) return Math.max(0, endMs - start / 1e6)
  }
  return Math.max(0, fallbackMs)
}

export function formatMapSaveElapsed(ms: number): string {
  const seconds = Math.floor(Math.max(0, ms) / 1000)
  return seconds < 60 ? `${seconds} 秒` : `${Math.floor(seconds / 60)} 分 ${seconds % 60} 秒`
}

function processingReason(step?: MapSaveProcessingStep): string {
  const code = step?.reason_code
  return code ? `（${REASONS[code] ?? `处理原因：${code}`}）` : ''
}

export function formatMapSaveProgress(result: SaveMapResult): string {
  const operation = result.operation
  const phase = operation?.phase === 'OPTIMIZE_SOURCE' && operation.source_operation_id
    ? '重新对齐扫描' : operation?.phase ? PHASES[operation.phase.toUpperCase()] : undefined
  const label = operation?.state === 'QUEUED'
    ? operation.source_operation_id ? '排队等待重新优化' : '排队等待保存' : phase ?? '等待保存结果'
  const progress = operation?.progress
  return typeof progress === 'number' && Number.isFinite(progress)
    ? `${label} · ${Math.round(Math.max(0, Math.min(1, progress)) * 100)}%`
    : label
}

export function pendingMapSaveStatus(
  result: SaveMapResult & { operation_id: string },
  reason: 'timeout' | 'status_unavailable',
): MapSaveStatus {
  return {
    name: result.name, state: 'pending', operation: result,
    detail: `${reason === 'timeout' ? '本次等待已结束，尚未确认保存完成' : '暂时无法查询保存状态'}。机载任务可能仍在继续，请继续查询同一次保存。`,
    summary: `最近进度：${formatMapSaveProgress(result)}`,
  }
}

export function savedMapStatus(result: SaveMapResult): MapSaveStatus {
  const { optimization, cleanup } = result.operation?.processing ?? {}
  const optimized = optimization?.performed === true && optimization.success === true
  const cleaned = cleanup?.success === true && cleanup.performed !== false
  const summary = [
    optimized ? '优化已完成' : `优化未完成${processingReason(optimization) || '（未收到完成确认）'}`,
    cleaned ? '动态点清理完成' : `动态点清理未确认${processingReason(cleanup)}`,
  ].join('；')
  return {
    name: result.name, state: 'saved', location: `机载地图 / ${result.name}`,
    detail: '地图已保存。保存成功不等于可走，仍需检查地图质量、定位和路径。',
    summary,
  }
}
