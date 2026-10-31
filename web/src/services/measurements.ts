import { dashboardFetch } from './observationMode.ts'

export interface SourceIdentity {
  product_session_id: string
  session_id: string
  model_generation: number
  reset_generation: number
  sequence?: number
  frame_id?: string
}
export interface MeasurementNoise {
  enabled: boolean
  unit: string
  stddev: number
  seed: number
}
export interface SensorSettings {
  enabled?: boolean
  rate_hz?: number
  recording?: boolean | null
  noise_enabled?: boolean | null
  noise?: MeasurementNoise | null
  measurement_settings?: {
    noise?: MeasurementNoise | null
    camera?: { gimbal?: { pan_deg: number; tilt_deg: number } }
  } | null
}
export interface MeasurementStatus {
  sensor_id: string
  status: string
  fresh: boolean
  stale?: boolean | null
  receipt_age_s: number | null
  stale_timeout_s: number
  source_identity: SourceIdentity | null
  modality: string | null
  raw_url: string
  controls: SensorSettings | null
  captured_at_s?: number | null
  clock_mapped?: boolean
  validity?: { valid_fraction: number | null } | null
}
interface ImageGeometry {
  width: number
  height: number
  source_width: number
  source_height: number
  row_indices: number[]
  column_indices: number[]
  valid_mask: boolean[][]
}
export interface ThermalDisplay extends ImageGeometry {
  kind: 'thermal'
  temperature_k: (number | null)[][]
}
export interface SemanticDisplay extends ImageGeometry {
  kind: 'semantic'
  ground_truth: true
  class_ids: (number | null)[][]
  instance_ids: (number | null)[][]
}
export interface AcousticDisplay {
  kind: 'acoustic'
  sample_rate_hz: number
  source_sample_count: number
  bin_start_samples: number[]
  bin_end_samples: number[]
  pressure_min_pa: (number | null)[][]
  pressure_max_pa: (number | null)[][]
  valid_fraction: number[][]
}
export interface MeasurementPreview {
  measurement: MeasurementStatus
  display: ThermalDisplay | SemanticDisplay | AcousticDisplay
}
export type ControlOperation = 'query' | 'enabled' | 'rate' | 'noise' | 'noise_enabled' | 'recording' | 'gimbal'
export interface ControlAck extends SourceIdentity {
  request_id: string
  sensor_id: string
  applied: boolean
  reason: string
  settings: SensorSettings
}

export function measurementNoise(settings: SensorSettings | null): MeasurementNoise | null {
  return settings?.measurement_settings?.noise ?? settings?.noise ?? null
}

export interface MeasurementControlState {
  sensors: MeasurementStatus[]
  controlRevision: number
}
export type MeasurementControlUpdate =
  | { type: 'list'; sensors: MeasurementStatus[]; startedAtRevision: number }
  | { type: 'ack'; ack: ControlAck }

/** GETs begun before a confirmed ACK cannot overwrite it; later GETs are authoritative. */
export function measurementControlReducer(state: MeasurementControlState, update: MeasurementControlUpdate): MeasurementControlState {
  if (update.type === 'list') {
    return update.startedAtRevision === state.controlRevision ? { ...state, sensors: update.sensors } : state
  }
  const { ack } = update
  if (!ack.applied) return state
  return {
    controlRevision: state.controlRevision + 1,
    sensors: state.sensors.map(sensor => sensor.sensor_id === ack.sensor_id
      && bindingKey(sensor) === bindingKey({ ...sensor, source_identity: ack })
      ? { ...sensor, controls: ack.settings } : sensor),
  }
}

export class MeasurementApiError extends Error {
  status: number
  constructor(status: number, message: string) {
    super(message)
    this.status = status
  }
}

async function request<T>(url: string, init: RequestInit, acceptRejection = false): Promise<T> {
  const controller = new AbortController()
  const abort = () => controller.abort()
  init.signal?.addEventListener('abort', abort, { once: true })
  if (init.signal?.aborted) controller.abort()
  const timeout = setTimeout(() => controller.abort(), 5000)
  try {
    const response = await dashboardFetch(url, { ...init, signal: controller.signal, cache: 'no-store' })
    const body = await response.json()
    if (!response.ok && !(acceptRejection && response.status === 409 && body.applied === false)) {
      throw new MeasurementApiError(response.status, body.message || body.error || `HTTP ${response.status}`)
    }
    return body as T
  } finally {
    clearTimeout(timeout)
    init.signal?.removeEventListener('abort', abort)
  }
}

export function listMeasurements(signal?: AbortSignal): Promise<{ sensors: MeasurementStatus[] }> {
  return request('/api/v1/measurements', { signal })
}
export async function getMeasurementPreview(sensorId: string, signal?: AbortSignal): Promise<MeasurementPreview> {
  const started = performance.now()
  const result = await request<MeasurementPreview>(`/api/v1/measurements/${encodeURIComponent(sensorId)}/preview`, { signal })
  // Include the entire request duration conservatively: server receipt age
  // alone cannot establish freshness once a delayed response reaches the UI.
  const age = result.measurement.receipt_age_s
  if (age !== null) result.measurement = {
    ...result.measurement,
    receipt_age_s: age + Math.max(0, performance.now() - started) / 1000,
  }
  return result
}
export function hasFreshMeasurement(status: MeasurementStatus): boolean {
  return status.fresh === true && status.status === 'live' && status.stale !== true
    && status.receipt_age_s !== null && Number.isFinite(status.receipt_age_s)
    && Number.isFinite(status.stale_timeout_s) && status.receipt_age_s < status.stale_timeout_s
}
export function bindingKey(status: MeasurementStatus): string {
  const identity = status.source_identity
  return JSON.stringify([status.sensor_id, identity?.product_session_id, identity?.session_id,
    identity?.model_generation, identity?.reset_generation])
}
export function controlDisabledReason(status: MeasurementStatus, online = true): string {
  if (!online) return 'Offline: controls unavailable / 离线，控制不可用'
  if (!status.source_identity?.product_session_id) return 'No source session binding / 无来源会话绑定'
  return ''
}
export function validateControlParams(operation: ControlOperation, params: Record<string, unknown>): void {
  const finite = (value: unknown): value is number => typeof value === 'number' && Number.isFinite(value)
  if (['enabled', 'noise_enabled', 'recording', 'noise'].includes(operation) && typeof params.enabled !== 'boolean') {
    throw new Error('enabled must be boolean')
  }
  if (operation === 'rate' && (!finite(params.rate_hz) || params.rate_hz <= 0)) throw new Error('Rate must be positive Hz')
  if (operation === 'noise') {
    if (!finite(params.stddev) || params.stddev < 0) throw new Error('Noise stddev must be nonnegative')
    if (!Number.isSafeInteger(params.seed) || (params.seed as number) < 0) throw new Error('Seed must be a nonnegative safe integer (≤ 9007199254740991)')
    if (typeof params.unit !== 'string' || !params.unit.trim()) throw new Error('Noise unit is required')
  }
  if (operation === 'gimbal' && (!finite(params.pan_deg) || !finite(params.tilt_deg))) throw new Error('Gimbal angles must be finite degrees')
}
export async function controlMeasurement(status: MeasurementStatus, operation: ControlOperation,
  params: Record<string, unknown>, signal?: AbortSignal): Promise<ControlAck> {
  const reason = controlDisabledReason(status)
  if (reason) throw new Error(reason)
  validateControlParams(operation, params)
  const requestId = crypto.randomUUID()
  const ack = await request<ControlAck>(`/api/v1/sensors/${encodeURIComponent(status.sensor_id)}/control`, {
    method: 'POST', signal, headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ request_id: requestId,
      expected_product_session_id: status.source_identity!.product_session_id, operation, params }),
  }, true)
  if (ack.request_id !== requestId || ack.sensor_id !== status.sensor_id
    || bindingKey({ ...status, source_identity: ack }) !== bindingKey(status)) {
    throw new Error('ACK source identity mismatch; applied state unconfirmed')
  }
  return ack
}

export function thermalValue(display: ThermalDisplay, row: number, column: number, unit: 'K' | 'C'): number | null {
  const value = display.temperature_k[row]?.[column]
  return display.valid_mask[row]?.[column] === true && typeof value === 'number' && Number.isFinite(value)
    ? value - (unit === 'C' ? 273.15 : 0) : null
}
export function semanticValue(display: SemanticDisplay, row: number, column: number): { classId: number; instanceId: number } | null {
  const classId = display.class_ids[row]?.[column]
  const instanceId = display.instance_ids[row]?.[column]
  return display.ground_truth === true && display.valid_mask[row]?.[column] === true
    && Number.isInteger(classId) && Number.isInteger(instanceId)
    ? { classId: classId as number, instanceId: instanceId as number } : null
}
export function acousticBin(display: AcousticDisplay, channel: number, bin: number) {
  const min = display.pressure_min_pa[channel]?.[bin]
  const max = display.pressure_max_pa[channel]?.[bin]
  const fraction = display.valid_fraction[channel]?.[bin]
  if (typeof min !== 'number' || typeof max !== 'number' || !Number.isFinite(min) || !Number.isFinite(max)
    || !(fraction > 0) || !(display.sample_rate_hz > 0)) return null
  return { min, max, fraction, start: display.bin_start_samples[bin] / display.sample_rate_hz,
    end: display.bin_end_samples[bin] / display.sample_rate_hz }
}

/** Serial, bounded reads only. The caller owns rendering and freshness expiry. */
export function pollMeasurements(read: (signal: AbortSignal) => Promise<void>, onError: (error: unknown) => void,
  intervalMs = 1000): () => void {
  const controller = new AbortController()
  let timer: ReturnType<typeof setTimeout> | undefined
  const tick = async () => {
    try { await read(controller.signal) } catch (error) {
      if (!controller.signal.aborted) onError(error)
    }
    if (!controller.signal.aborted) timer = setTimeout(() => { void tick() }, intervalMs)
  }
  void tick()
  return () => { controller.abort(); clearTimeout(timer) }
}
