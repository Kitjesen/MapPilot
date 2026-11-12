import { dashboardFetch } from './observationMode.ts'

export interface ImuSample { seq: number; t: number; received_at_s: number; gap_before?: boolean; values: Record<string, Record<string, number>> }
export interface ImuRecord { id: string; state: string; started_at_s: number; ended_at_s?: number; sample_count: number; duration_s?: number; robot?: string; error?: string; source?: { provider: string } }
export interface ImuLive { state: 'live' | 'connecting' | 'disconnected'; error?: string; provider: string; robot?: string; source_clock: string; sequence: number; samples: ImuSample[]; truncated: boolean; recording: ImuRecord | null }
export interface ImuHistory { record: ImuRecord; samples: ImuSample[]; sampled: boolean }
const base = '/api/v1/robot/imu'

async function request<T>(path: string, init?: RequestInit): Promise<T> {
  const response = await dashboardFetch(base + path, init)
  const value = await response.json()
  if (!response.ok) throw new Error(typeof value.detail === 'string' ? value.detail : value.error ?? `HTTP ${response.status}`)
  return value as T
}
function samples(value: ImuLive | ImuHistory) {
  if (!Array.isArray(value.samples) || value.samples.some(row => !Number.isSafeInteger(row.seq) || !Number.isFinite(row.t)
    || !Number.isFinite(row.received_at_s) || !row.values || typeof row.values !== 'object'
    || Object.values(row.values).some(vector => !vector || typeof vector !== 'object'
      || Object.values(vector).some(number => typeof number !== 'number' || !Number.isFinite(number))))) throw new Error('IMU 数据格式无效')
  return value
}
export async function readImu(after: number, signal?: AbortSignal) { return samples(await request<ImuLive>(`/live?after=${after}`, { signal })) as ImuLive }
export function startImuRecord() { return request<ImuRecord>('/records', { method: 'POST' }) }
export function stopImuRecord(id: string) { return request<ImuRecord>(`/records/${encodeURIComponent(id)}/stop`, { method: 'POST' }) }
export function listImuRecords() { return request<{ records: ImuRecord[] }>('/records') }
export async function readImuRecord(id: string) { return samples(await request<ImuHistory>(`/records/${encodeURIComponent(id)}`)) as ImuHistory }
export function imuDownload(id: string, format: 'csv' | 'jsonl') { return `${base}/records/${encodeURIComponent(id)}/download?format=${format}` }

export function mergeImu(previous: ImuSample[], incoming: ImuSample[], reset = false): ImuSample[] {
  const last = reset ? undefined : previous.at(-1)
  const next = incoming.filter(row => !last || row.seq > last.seq)
  if (last && next[0] && next[0].seq > last.seq + 1) next[0] = { ...next[0], gap_before: true }
  const end = next.at(-1)?.t ?? last?.t ?? 0
  return [...(reset ? [] : previous), ...next].filter(row => end - row.t <= 300).slice(-30000)
}

