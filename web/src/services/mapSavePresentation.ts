import type { MapSaveProcessingStep, SaveMapResult } from './api.ts'

interface MapSaveStatusDetails {
  name: string
  detail: string
  location?: string
  summary?: string
}
export type MapSaveStatus = MapSaveStatusDetails & (
  | { state: 'saving' | 'saved' | 'failed' }
  | { state: 'pending'; operation: SaveMapResult & { operation_id: string } }
)

const PHASES: Record<string, string> = {
  CAPTURE: '等待地图快照', VALIDATE: '检查建图数据', OPTIMIZE_SOURCE: '优化建图轨迹',
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
  const phase = operation?.phase ? PHASES[operation.phase.toUpperCase()] : undefined
  const label = operation?.state === 'QUEUED' ? '排队等待保存' : phase ?? '等待保存结果'
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
