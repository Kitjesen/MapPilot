import { text, type Locale } from '../i18n.ts'
import type { RecordingLiveTopic, RecordingTopic } from '../types/index.ts'

export function summarizeStreamUsage(topics: RecordingTopic[], observations: RecordingLiveTopic[]) {
  const observed = new Map(observations.map(item => [item.topic, item]))
  const groups = new Map<string, { group: string; bytesPerSecond: number | null; observedCount: number; topicCount: number }>()
  let bytesPerSecond: number | null = null
  let observedCount = 0
  for (const topic of topics) {
    let usage = groups.get(topic.group)
    if (!usage) {
      usage = { group: topic.group, bytesPerSecond: null, observedCount: 0, topicCount: 0 }
      groups.set(topic.group, usage)
    }
    usage.topicCount += 1
    const rate = observed.get(topic.topic)?.bytes_per_second
    if (rate === undefined || !Number.isFinite(rate) || rate < 0) continue
    usage.bytesPerSecond = (usage.bytesPerSecond ?? 0) + rate
    usage.observedCount += 1
    bytesPerSecond = (bytesPerSecond ?? 0) + rate
    observedCount += 1
  }
  return { groups: [...groups.values()], bytesPerSecond, observedCount, topicCount: topics.length }
}

export function formatStreamRate(value: number | undefined): string {
  return value === undefined || !Number.isFinite(value) ? '—' : `${value.toFixed(value < 10 ? 1 : 0)} Hz`
}

export function formatStreamBytes(value: number | undefined): string {
  if (value === undefined || !Number.isFinite(value)) return '—'
  if (value >= 1024 * 1024) return `${(value / (1024 * 1024)).toFixed(1)} MiB/s`
  if (value >= 1024) return `${(value / 1024).toFixed(1)} KiB/s`
  return `${Math.round(value)} B/s`
}

export function formatSampleAge(value: number | null | undefined, locale: Locale): string {
  if (value == null || !Number.isFinite(value)) return text(locale, 'No samples', '未收到')
  if (value < 1000) return `${Math.round(value)} ms`
  if (value < 60000) return `${(value / 1000).toFixed(1)} s`
  return `${Math.floor(value / 60000)} min`
}

export function filterRecordingTopics(topics: RecordingTopic[], query: string, group: string): RecordingTopic[] {
  const search = query.trim().toLocaleLowerCase()
  return topics.filter(topic => (group === 'all' || topic.group === group)
    && (!search || `${topic.label} ${topic.topic} ${topic.type_name}`.toLocaleLowerCase().includes(search)))
}

export function streamFields(preview: Record<string, unknown>): Array<[string, string]> {
  const fields: Array<[string, string]> = []
  const append = (key: string, value: unknown, depth: number) => {
    if (fields.length >= 64) return
    if (value !== null && typeof value === 'object' && !Array.isArray(value) && depth < 2) {
      for (const [child, nested] of Object.entries(value)) append(key ? `${key}.${child}` : child, nested, depth + 1)
    } else {
      const display = Array.isArray(value)
        ? `[${value.slice(0, 16).map(item => typeof item === 'object' ? JSON.stringify(item) : String(item)).join(', ')}${value.length > 16 ? ', …' : ''}]`
        : typeof value === 'object' ? JSON.stringify(value) : String(value)
      fields.push([key, display])
    }
  }
  for (const [key, value] of Object.entries(preview)) append(key, value, 0)
  return fields
}
