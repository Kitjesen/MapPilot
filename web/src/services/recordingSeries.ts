import type { RecordingLiveTopic, RecordingStreamsResponse } from '../types/index.ts'
import type { RecordingNumericSample } from '../types/recordingSeries.ts'

export function numericValues(value: unknown, prefix = ''): Record<string, number | null> {
  if (typeof value === 'number') return prefix ? { [prefix]: Number.isFinite(value) ? value : null } : {}
  if (Array.isArray(value)) return Object.assign({}, ...value.map((item, index) => numericValues(item, `${prefix}[${index}]`)))
  if (value && typeof value === 'object') return Object.assign({}, ...Object.entries(value).map(([key, item]) => numericValues(item, prefix ? `${prefix}.${key}` : key)))
  return {}
}

export function preferredNumericField(paths: string[]): string {
  return paths.find(path => !/(^|\.|\[)(?:header|stamp|timestamp|time|count|sequence|seq|version|frame|size|width|height|valid|mode|status|id)(?:[._]|\[|$)/i.test(path)
    && !/(?:_count|_time|_ns|_sec|_nsec)$/.test(path)) ?? paths[0] ?? ''
}

export interface LiveNumericTrace {
  count: number
  origin: number
  samples: Array<{ t: number; values: Record<string, number | null>; gap_before?: boolean }>
}

export function appendLiveTrace(previous: LiveNumericTrace | undefined, item: RecordingLiveTopic, snapshot: RecordingStreamsResponse): LiveNumericTrace | undefined {
  if (!snapshot.available || snapshot.state !== 'observing' || (snapshot.snapshot_age_ms ?? 0) > 3000
    || item.message_count <= 0 || !item.preview || item.last_sample_age_ms === null || item.last_sample_age_ms > 3000) return previous
  if (previous?.count === item.message_count) return previous
  const time = (snapshot.observed_at_unix_ns ?? 0) / 1e9 - item.last_sample_age_ms / 1000
  if (!Number.isFinite(time) || time <= 0) return previous
  const retained = previous && item.message_count > previous.count ? previous : undefined
  const origin = retained?.origin ?? time
  const t = time - origin
  const last = retained?.samples.at(-1)
  if (last && t < last.t) return previous
  return { count: item.message_count, origin, samples: [...(retained?.samples ?? []), { t, values: numericValues(item.preview), ...(last && t - last.t > 3 ? { gap_before: true } : {}) }].slice(-120) }
}

export function liveFieldSamples(trace: LiveNumericTrace | undefined, field: string): RecordingNumericSample[] {
  return trace?.samples.map(sample => ({ t: sample.t, value: sample.values[field] ?? null, gap_before: sample.gap_before })) ?? []
}

export function shiftSamples(samples: RecordingNumericSample[], offset: number): RecordingNumericSample[] {
  return samples.map(sample => ({ ...sample, t: sample.t + offset }))
}

export function sampleAt(samples: RecordingNumericSample[], time: number): RecordingNumericSample | undefined {
  if (!samples.length || time < samples[0].t || time > samples.at(-1)!.t) return undefined
  let low = 0
  let high = samples.length
  while (low < high) { const middle = (low + high) >>> 1; if (samples[middle].t <= time) low = middle + 1; else high = middle }
  const sample = samples[low - 1]
  const next = samples[low]
  if (sample?.value === null || (next?.gap_before && time > sample.t)) return undefined
  return sample
}

export function advancePlayhead(time: number, elapsed: number, rate: number, start: number, end: number): number {
  return Math.max(start, Math.min(end, time + Math.max(0, elapsed) * rate))
}

export function canPlayRecordingWindow(a: RecordingNumericSample[], b: RecordingNumericSample[], compatible: boolean, start = -Infinity, end = Infinity): boolean {
  const hasValues = (samples: RecordingNumericSample[]) => samples.some(sample => sample.t >= start && sample.t <= end && sample.value !== null && Number.isFinite(sample.value))
  return hasValues(a) || (compatible && hasValues(b))
}

export function seriesSegments(samples: RecordingNumericSample[], start: number, end: number): RecordingNumericSample[][] {
  const segments: RecordingNumericSample[][] = []
  let segment: RecordingNumericSample[] = []
  for (const sample of samples) {
    if (sample.value === null || !Number.isFinite(sample.value) || sample.gap_before || sample.t < start || sample.t > end) {
      if (segment.length) segments.push(segment)
      segment = []
    }
    if (sample.value !== null && Number.isFinite(sample.value) && sample.t >= start && sample.t <= end) segment.push(sample)
  }
  if (segment.length) segments.push(segment)
  return segments
}

export function formatSeriesValue(value: number | null | undefined): string {
  if (value === null || value === undefined || !Number.isFinite(value)) return '—'
  return Math.abs(value) >= 10000 || (Math.abs(value) > 0 && Math.abs(value) < .001) ? value.toExponential(3) : Number(value.toFixed(4)).toString()
}

export function formatSeriesAxis(value: number): string {
  if (!Number.isFinite(value)) return '—'
  if (value === 0) return '0'
  const magnitude = Math.abs(value)
  if (magnitude >= 1000 || magnitude < .1) return Number(value.toPrecision(3)).toExponential().replace('e+', 'e')
  return Number(value.toPrecision(magnitude < 1 ? 2 : 3)).toString()
}
