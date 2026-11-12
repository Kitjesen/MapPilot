import { dashboardFetch } from './observationMode.ts'

export interface SurfaceOptions { headroom_m: number; radius_m: number; max_hole_area_m2: number; max_slope: number }
export const defaultSurfaceOptions: SurfaceOptions = { headroom_m: .6, radius_m: .2, max_hole_area_m2: .16, max_slope: .7 }
export interface SurfaceMap {
  map_id: string; content_epoch: number; frame_id: string; resolution: number
  options: SurfaceOptions; observed_count: number; inferred_count: number; inferred_area_m2: number
  observed_xyz: number[]; inferred_xyz: number[]; sampled: boolean
}

export async function fetchSurfaceMap(name: string, options: SurfaceOptions, signal?: AbortSignal): Promise<SurfaceMap> {
  const query = new URLSearchParams(Object.entries(options).map(([key, value]) => [key, String(value)]))
  const response = await dashboardFetch(`/api/v1/maps/${encodeURIComponent(name)}/surface?${query}`, { signal, cache: 'no-store' })
  const data = await response.json()
  if (!response.ok || data.success === false) throw new Error(data.message || data.detail || data.reason_code || '补面预览失败')
  const xyz = (value: unknown): value is number[] => Array.isArray(value) && value.length % 3 === 0 && value.every(Number.isFinite)
  if (data.map_id !== name || !Number.isSafeInteger(data.content_epoch) || data.content_epoch <= 0
    || typeof data.frame_id !== 'string' || !data.frame_id || !(data.resolution > 0)
    || !xyz(data.observed_xyz) || !xyz(data.inferred_xyz)
    || !Number.isSafeInteger(data.observed_count) || data.observed_count < data.observed_xyz.length / 3
    || !Number.isSafeInteger(data.inferred_count) || data.inferred_count < data.inferred_xyz.length / 3) {
    throw new Error('补面结果与当前地图不匹配，请刷新地图。')
  }
  return data
}
