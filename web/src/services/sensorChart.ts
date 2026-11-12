export interface TraceSample { t: number; receivedAt?: number; gap_before?: boolean; values: Record<string, number | undefined> }
export interface TraceChannel { id: string; label: string; group: string; unit: string; color: string }
export const traceColors = ['#5b83cc','#bd814c','#8b72bc','#459b8b','#c36777','#6e9a49','#b59535','#4a9cb7','#ab74a2','#85775e','#6477ad','#b27765','#5a9380','#9d81bc','#849555','#bd688f']
export const traceSpan = (seconds: number) => Math.max(5, Math.min(300, Math.round(seconds)))
export function traceEnd(rows: readonly TraceSample[], end: number | null): number {
  const latest = rows.at(-1)?.t ?? 0
  return end === null ? latest : Math.max(rows[0]?.t ?? 0, Math.min(latest, end))
}
export function traceTime(t: number, epoch: number | undefined, precise = false): string {
  if (epoch === undefined) return `${t.toFixed(precise ? 3 : 1)} s`
  const date = new Date((epoch + t) * 1000)
  return date.toLocaleTimeString('zh-CN', { hour12: false, hour: '2-digit', minute: '2-digit', second: '2-digit' }) + (precise ? `.${String(date.getMilliseconds()).padStart(3,'0')}` : '')
}
export function traceAt(rows: readonly TraceSample[], t: number): TraceSample | undefined {
  if (!rows.length || t < rows[0].t || t > rows[rows.length - 1].t) return undefined
  let lo = 0, hi = rows.length - 1
  while (lo < hi) { const mid = Math.floor((lo + hi) / 2); if (rows[mid].t < t) lo = mid + 1; else hi = mid }
  if (rows[lo].t === t || lo === 0) return rows[lo]
  if (rows[lo].gap_before) return undefined
  return t - rows[lo - 1].t < rows[lo].t - t ? rows[lo - 1] : rows[lo]
}
export function traceGroups(channels: readonly TraceChannel[]) {
  const groups = new Map<string, TraceChannel[]>()
  for (const channel of channels) {
    const key = `${channel.group} · ${channel.unit}`
    groups.set(key, [...(groups.get(key) ?? []), channel])
  }
  return [...groups]
}
export function numericChannels(fields: readonly string[]): TraceChannel[] {
  return fields.map((id,index) => {
    const split = id.lastIndexOf(' · '), label = split < 0 ? id : id.slice(0, split)
    const metric = id.slice(split + 3), unit = metric.match(/\(([^)]+)\)$/)?.[1] ?? 'V'
    const group = metric.replace(/\s*\([^)]+\)$/, '') === 'V' ? '电压' : metric.replace(/\s*\([^)]+\)$/, '')
    return { id, label, group, unit, color: traceColors[index % traceColors.length] }
  })
}

export function tracePath(rows: readonly TraceSample[], channel: string, start: number, end: number, low: number, high: number) {
  let path = '', drawing = false, bucket = -1
  let pending: Array<{ x: number; y: number; index: number }> = []
  function flush() {
    if (!pending.length) return
    const minimum = pending.reduce((a,b) => a.y < b.y ? a : b), maximum = pending.reduce((a,b) => a.y > b.y ? a : b)
    const selected = [...new Set([pending[0], minimum, maximum, pending.at(-1)!])].sort((a,b) => a.index - b.index)
    for (const point of selected) { path += `${drawing ? 'L' : 'M'}${point.x.toFixed(1)},${point.y.toFixed(1)}`; drawing = true }
    pending = []
  }
  rows.forEach((row, index) => {
    const value = row.values[channel]
    if (row.gap_before || !Number.isFinite(value)) { flush(); drawing = false }
    if (row.t < start || row.t > end || typeof value !== 'number' || !Number.isFinite(value)) return
    const x = (row.t - start) / Math.max(.001, end - start) * 320
    const y = 86 - (value - low) / Math.max(.001, high - low) * 76
    const column = Math.floor(x)
    if (column !== bucket) { flush(); bucket = column }
    pending.push({ x, y, index })
  })
  flush()
  return path
}
