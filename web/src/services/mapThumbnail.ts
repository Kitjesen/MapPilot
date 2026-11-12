import { cloudColorRange, writeCloudHeightColor } from './cloudHeightColor.ts'

export interface MapThumbnailData {
  name: string
  content_epoch: number
  frame_id: string | null
  count: number
  total_points: number
  points: Array<[number, number, number]>
  built_at_ms: number | null
  cleanup_state: 'cleaned' | 'not_cleaned' | 'unknown'
}

export async function fetchMapThumbnail(name: string, epoch?: number | null, signal?: AbortSignal): Promise<MapThumbnailData> {
  const query = epoch == null ? '' : `?content_epoch=${epoch}`
  const response = await fetch(`/api/v1/maps/${encodeURIComponent(name)}/preview${query}`, { signal, cache: 'no-store' })
  if (!response.ok) throw new Error(`地图预览读取失败（HTTP ${response.status}）`)
  const result = await response.json() as MapThumbnailData
  if (result.name !== name || !Number.isSafeInteger(result.content_epoch) || result.content_epoch <= 0
    || (epoch != null && result.content_epoch !== epoch)) throw new Error('地图版本已变化，请刷新地图库')
  if (!Array.isArray(result.points) || result.points.some(point => !Array.isArray(point)
    || point.length !== 3 || !point.every(Number.isFinite))) throw new Error('地图预览点格式无效')
  if (result.points.length > 24000 || !Number.isSafeInteger(result.count) || result.count !== result.points.length
    || !Number.isSafeInteger(result.total_points) || result.total_points < result.count
    || (result.frame_id != null && typeof result.frame_id !== 'string')) throw new Error('地图预览元数据无效')
  return result
}

export function thumbnailProjection(points: MapThumbnailData['points'], size: number) {
  const projected = points.map(([x, y, z]) => [(x - y) * .8660254, (x + y) * .7 - z * .75])
  let left = Infinity, right = -Infinity, top = Infinity, bottom = -Infinity
  for (const [x, y] of projected) {
    left = Math.min(left, x); right = Math.max(right, x)
    top = Math.min(top, y); bottom = Math.max(bottom, y)
  }
  const scale = size * .84 / Math.max(right - left, bottom - top, .001)
  return projected.map(([x, y]) => [size / 2 + (x - (left + right) / 2) * scale,
    size / 2 + (y - (top + bottom) / 2) * scale])
}

export function drawMapThumbnail(points: MapThumbnailData['points']): string {
  const canvas = document.createElement('canvas')
  canvas.width = canvas.height = 640
  const context = canvas.getContext('2d')
  if (!context) throw new Error('浏览器无法绘制地图预览')
  const projected = thumbnailProjection(points, canvas.width)
  const range = cloudColorRange(points.flat())
  const rgb = new Float32Array(3)
  context.globalAlpha = .85
  for (let index = 0; index < points.length; index++) {
    writeCloudHeightColor(points[index][2], rgb, 0, range?.min, range?.max)
    context.fillStyle = `rgb(${rgb[0] * 255} ${rgb[1] * 255} ${rgb[2] * 255})`
    context.fillRect(projected[index][0], projected[index][1], 1.7, 1.7)
  }
  return canvas.toDataURL('image/png')
}

export function mapCleanupLabel(state?: MapThumbnailData['cleanup_state']): string {
  return state === 'cleaned' ? '已清洗' : state === 'not_cleaned' ? '未清洗' : '清洗状态未知'
}

export function mapUpdateTime(milliseconds?: number | null): string {
  return milliseconds ? new Date(milliseconds).toLocaleString('zh-CN', {
    year: 'numeric', month: '2-digit', day: '2-digit', hour: '2-digit', minute: '2-digit', hour12: false,
  }) : '未记录'
}
