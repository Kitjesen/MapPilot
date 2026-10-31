import { useEffect, useState } from 'react'
import { Activity, ArrowDownToLine, ChevronRight, Radio, Search } from 'lucide-react'
import { text, type Locale } from '../i18n'
import { fetchRecordingStreams } from '../services/api'
import type { RecordingLiveTopic, RecordingStreamsResponse, RecordingTopic } from '../types'
import styles from './RecordingStreams.module.css'
import { filterRecordingTopics, formatSampleAge, formatStreamBytes, formatStreamRate, streamFields } from '../services/recordingStreams'
import { appendLiveTrace, liveFieldSamples, numericValues, preferredNumericField, type LiveNumericTrace } from '../services/recordingSeries'
import { RecordingPlot } from './RecordingPlot'

const GROUP_LABELS: Record<string, [string, string]> = {
  robot: ['Robot', '机器人'], sensors: ['Sensors', '传感器'],
  localization: ['Localization', '定位'], navigation: ['Navigation', '导航'], inspection: ['Inspection', '巡检'],
}

export function RecordingStreamInspector({ topic, observation, rates, stale, locale, trace }: {
  topic: RecordingTopic
  observation: RecordingLiveTopic | undefined
  rates: number[]
  stale: boolean
  locale: Locale
  trace?: LiveNumericTrace
}) {
  const [chosenField, setChosenField] = useState('')
  const fields = observation?.preview ? streamFields(observation.preview) : []
  const numeric = Object.keys(numericValues(observation?.preview))
  const field = numeric.includes(chosenField) ? chosenField : preferredNumericField(numeric)
  const samples = liveFieldSamples(trace, field)
  const traceEnd = samples.at(-1)?.t ?? 0
  const sampleStale = stale || observation?.last_sample_age_ms === null || (observation?.last_sample_age_ms ?? 0) > 3000
  const maximum = Math.max(1, ...rates)
  const points = rates.map((value, index) => `${(index / Math.max(1, rates.length - 1)) * 280},${42 - (value / maximum) * 34}`).join(' ')
  return <section className={styles.inspector} aria-label={text(locale, 'Stream fields', '数据流字段')}>
    <header className={styles.inspectorHeading}>
      <div><strong>{topic.label}</strong><code>{topic.topic}</code></div>
      <span className={styles.liveLabel}><i data-live={!stale && !!observation?.rate_hz} />{stale ? text(locale, 'Unavailable', '未更新') : text(locale, 'Read only', '只读')}</span>
    </header>
    <dl className={styles.facts}>
      <div><dt>{text(locale, 'Rate', '接收频率')}</dt><dd>{stale ? '—' : formatStreamRate(observation?.rate_hz)}</dd></div>
      <div><dt>{text(locale, 'Throughput', '数据流量')}</dt><dd>{stale ? '—' : formatStreamBytes(observation?.bytes_per_second)}</dd></div>
      <div><dt>{text(locale, 'Samples', '累计样本')}</dt><dd>{observation?.message_count.toLocaleString() ?? '—'}</dd></div>
      <div><dt>{text(locale, 'Latest sample', '最近样本距今')}</dt><dd>{stale ? '—' : formatSampleAge(observation?.last_sample_age_ms, locale)}</dd></div>
    </dl>
    {!!numeric.length && <div className={styles.numericCurve}>
      <label>{text(locale, 'Numeric field', '数值字段')}<select aria-label={text(locale, 'Live numeric field', '实时数值字段')} value={field} onChange={event => setChosenField(event.target.value)}>{numeric.map(path => <option key={path} value={path}>{path}</option>)}</select></label>
      <RecordingPlot traces={[{ label: field, samples }]} start={Math.max(0, traceEnd - 120)} end={Math.max(1, traceEnd)} locale={locale} unavailable={sampleStale} />
      <p className={styles.caption}>{text(locale, 'Web trend · latest distinct samples, up to 120 updates. Full-rate messages remain in the recording.', '网页趋势 · 最近 120 次不同消息的摘要采样，完整频率数据保存在录制中。')}</p>
    </div>}
    {!numeric.length && rates.length > 1 && <figure className={styles.sparkline}>
      <svg viewBox="0 0 280 48" role="img" aria-label={text(locale, 'Recent observed reception rate', '最近观测的接收频率')}>
        <line x1="0" x2="280" y1="43" y2="43" />
        <polyline points={points} />
      </svg>
      <figcaption>{text(locale, 'Reception rate · latest 30 updates', '接收频率 · 最近 30 次更新')}</figcaption>
    </figure>}
    <div className={styles.fieldHeading}><span>{text(locale, 'Latest sample', '最近一条消息')}</span><small>{topic.type_name}</small></div>
    {fields.length ? <dl className={styles.fields}>{fields.map(([key, value]) => <div key={key}><dt>{key}</dt><dd>{value}</dd></div>)}</dl>
      : <div className={styles.noSample}><Radio size={22} strokeWidth={1.4} /><span>{text(locale, 'Waiting for a sample', '等待这条数据流')}</span><small>{text(locale, 'A supported topic may have no publisher in the current mode.', '当前模式可能未发布这条数据。')}</small></div>}
    <p className={styles.caption}>{text(locale, 'This view summarizes large arrays; recording keeps the full messages. Field values retain native units.', '这里显示摘要，录制保留完整消息。字段保留原生单位。')}</p>
  </section>
}

export function RecordingStreams({ topics, selected, disabled, locale, onChange }: {
  topics: RecordingTopic[]
  selected: string[]
  disabled: boolean
  locale: Locale
  onChange: (topics: string[]) => void
}) {
  const [snapshot, setSnapshot] = useState<RecordingStreamsResponse | null>(null)
  const [error, setError] = useState<string | null>(null)
  const [query, setQuery] = useState('')
  const [group, setGroup] = useState('all')
  const [focused, setFocused] = useState('')
  const [history, setHistory] = useState<Record<string, number[]>>({})
  const [traces, setTraces] = useState<Record<string, LiveNumericTrace | undefined>>({})
  useEffect(() => {
    let current = true
    let timer: ReturnType<typeof setTimeout>
    const poll = async () => {
      try {
        const next = await fetchRecordingStreams()
        if (!current) return
        setSnapshot(next)
        setError(null)
        if (next.available && next.state === 'observing') {
          setTraces(previous => {
            const updated = { ...previous }
            for (const item of next.topics) updated[item.topic] = appendLiveTrace(previous[item.topic], item, next)
            return updated
          })
          setHistory(previous => {
            const updated = { ...previous }
            for (const item of next.topics) {
              if (item.message_count > 0) updated[item.topic] = [...(previous[item.topic] ?? []), item.rate_hz].slice(-30)
            }
            return updated
          })
        }
      } catch (cause: unknown) {
        if (current) setError(cause instanceof Error ? cause.message : String(cause))
      } finally {
        if (current) timer = setTimeout(() => void poll(), 1000)
      }
    }
    void poll()
    return () => { current = false; clearTimeout(timer) }
  }, [])
  const stale = !!error || !snapshot?.available || snapshot.state !== 'observing'
    || (snapshot.snapshot_age_ms !== null && snapshot.snapshot_age_ms > 3000)
  const filtered = filterRecordingTopics(topics, query, group)
  const focusedTopic = topics.find(topic => topic.topic === focused) ?? filtered[0] ?? topics[0]
  const activeCount = stale ? null : snapshot?.topics.filter(item => item.rate_hz > 0).length
  const groups = [...new Set(topics.map(topic => topic.group))]
  const toggle = (name: string, checked: boolean) => {
    const next = new Set(selected)
    if (checked) next.add(name)
    else next.delete(name)
    onChange(topics.filter(topic => next.has(topic.topic)).map(topic => topic.topic))
  }
  return <div className={styles.workspace}>
    <div className={styles.toolbar}>
      <label className={styles.search}><Search size={15} /><input aria-label={text(locale, 'Search data streams', '搜索数据流')} value={query} onChange={event => setQuery(event.target.value)} placeholder={text(locale, 'Search streams or fields…', '搜索数据流…')} /></label>
      <select aria-label={text(locale, 'Stream category', '数据流分类')} value={group} onChange={event => setGroup(event.target.value)}>
        <option value="all">{text(locale, 'All categories', '全部分类')}</option>
        {groups.map(value => <option key={value} value={value}>{GROUP_LABELS[value] ? text(locale, ...GROUP_LABELS[value]) : value}</option>)}
      </select>
    </div>
    <div className={styles.observerStatus} role="status">
      <span><Activity size={13} />{error || snapshot?.state === 'unavailable' ? text(locale, 'Live observer unavailable', '实时查看暂不可用') : stale ? text(locale, 'Connecting to data streams…', '正在连接数据流…') : `${activeCount} ${text(locale, 'receiving', '路正在接收')}`}</span>
      <span><ArrowDownToLine size={12} />{selected.length} {text(locale, 'selected for capture', '项待录制')}</span>
    </div>
    {snapshot?.state === 'stale' && <p className={styles.caption}>{text(locale, 'The observer snapshot is stale. These values are not current.', '查看器快照已过期，下面的值不代表当前数据。')}</p>}
    <div className={styles.split}>
      <div className={styles.streamList} role="list" aria-label={text(locale, 'Live data streams', '实时数据流')}>
        <div className={styles.listHeading}><span>{text(locale, 'Record / stream', '录制 / 数据流')}</span><span>Hz</span><span>{text(locale, 'Data rate', '流量')}</span></div>
        {filtered.map(topic => {
          const observation = snapshot?.topics.find(item => item.topic === topic.topic)
          return <div key={topic.topic} className={styles.streamRow} data-selected={focusedTopic?.topic === topic.topic} role="listitem">
            <input type="checkbox" aria-label={`${text(locale, 'Record', '录制')} ${topic.label}`} checked={selected.includes(topic.topic)} disabled={disabled} onChange={event => toggle(topic.topic, event.target.checked)} />
            <button onClick={() => setFocused(topic.topic)} aria-pressed={focusedTopic?.topic === topic.topic}>
              <span className={styles.streamName}><strong><i data-live={!stale && !!observation?.rate_hz} />{topic.label}</strong><code>{topic.topic}</code></span>
              <span className={styles.rate}>{stale || !observation ? '—' : observation.rate_hz.toFixed(observation.rate_hz < 10 ? 1 : 0)}</span>
              <span className={styles.throughput}>{stale ? '—' : formatStreamBytes(observation?.bytes_per_second)}</span>
              <ChevronRight size={13} />
            </button>
          </div>
        })}
        {!filtered.length && <p className={styles.empty}>{text(locale, 'No matching streams', '没有匹配的数据流')}</p>}
      </div>
      {focusedTopic && <RecordingStreamInspector key={focusedTopic.topic} topic={focusedTopic} observation={snapshot?.topics.find(item => item.topic === focusedTopic.topic)} rates={history[focusedTopic.topic] ?? []} trace={traces[focusedTopic.topic]} stale={stale} locale={locale} />}
    </div>
    <p className={styles.caption}>{text(locale, 'Checkboxes choose what to record. Viewing streams does not start recording.', '勾选决定录制内容，查看数据不会开始录制。')}</p>
  </div>
}
