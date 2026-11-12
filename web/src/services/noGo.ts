import { dashboardFetch } from './observationMode.ts'

export interface NoGoRegion { id: string; min: [number, number, number]; max: [number, number, number]; label?: string; kind?: 'no_go' | 'glass' }
export interface NoGoMap { map_id: string; content_epoch: number; frame_id: string; regions: NoGoRegion[] }

function endpoint(name: string) { return `/api/v1/maps/${encodeURIComponent(name)}/no_go` }

async function responseData(response: Response) {
  const data = await response.json()
  if (!response.ok || data.success === false) {
    throw new Error(data.message || data.detail?.message || data.reason_code || `禁行区操作失败（HTTP ${response.status}）`)
  }
  return data
}

export async function fetchNoGo(name: string, signal?: AbortSignal): Promise<NoGoMap> {
  const data = await responseData(await dashboardFetch(endpoint(name), { signal, cache: 'no-store' }))
  const vector = (value: unknown): value is [number, number, number] =>
    Array.isArray(value) && value.length === 3 && value.every(Number.isFinite)
  if (data.map_id !== name || !Number.isSafeInteger(data.content_epoch) || data.content_epoch <= 0
    || typeof data.frame_id !== 'string' || !data.frame_id || !Array.isArray(data.regions)
    || !data.regions.every((region: NoGoRegion) => typeof region.id === 'string' && region.id
      && vector(region.min) && vector(region.max) && region.min.every((v, i) => v < region.max[i]))) {
    throw new Error('禁行区数据与当前地图不匹配，请刷新地图。')
  }
  return data
}

export async function putNoGo(name: string, epoch: number, region: NoGoRegion): Promise<void> {
  await responseData(await dashboardFetch(endpoint(name), {
    method: 'PUT', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ expected_content_epoch: epoch, region }),
  }))
}

export async function deleteNoGo(name: string, epoch: number, id: string): Promise<void> {
  await responseData(await dashboardFetch(`${endpoint(name)}/${encodeURIComponent(id)}?expected_content_epoch=${epoch}`, { method: 'DELETE' }))
}
