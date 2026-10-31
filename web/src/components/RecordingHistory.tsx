import { useEffect, useMemo, useRef, useState } from 'react'
import { AlignStartVertical, ChevronLeft, ChevronRight, Pause, Play, RotateCcw, ZoomIn, ZoomOut } from 'lucide-react'
import { text, type Locale } from '../i18n'
import { fetchRecordingSeries } from '../services/api'
import { advancePlayhead, canPlayRecordingWindow, formatSeriesValue, sampleAt, shiftSamples } from '../services/recordingSeries'
import type { RecordingSessionSummary, RecordingSummaryResponse, RecordingTopic } from '../types'
import type { RecordingSeries } from '../types/recordingSeries'
import { RecordingPlot } from './RecordingPlot'
import { RecordingVisualSource } from './RecordingVisuals'
import { useRecordingSceneInfo } from '../hooks/useRecordingScene'
import { cameraCursorTime } from '../services/recordingScene'
import styles from './RecordingHistory.module.css'

interface HistoryResult { key: string; bId?: string; a?: RecordingSeries; b?: RecordingSeries; error?: string; bError?: string }

export function RecordingHistory({ sessionId, sessions, summary, topics, locale }: {
  sessionId: string
  sessions: RecordingSessionSummary[]
  summary?: RecordingSummaryResponse['summary']
  topics: RecordingTopic[]
  locale: Locale
}) {
  const available = summary?.topics.filter(topic => topic.message_count > 0) ?? []
  const defaultTopic = ['/slam/odometry', '/robot/joint_states', '/imu/raw'].find(topic => available.some(item => item.topic === topic)) ?? available[0]?.topic ?? ''
  const [chosenTopic, setChosenTopic] = useState(defaultTopic)
  const topic = available.some(item => item.topic === chosenTopic) ? chosenTopic : defaultTopic
  const [field, setField] = useState('')
  const [compareId, setCompareId] = useState('')
  const [offset, setOffset] = useState('0')
  const offsetS = Number.isFinite(Number(offset)) ? Number(offset) : 0
  const [windowRange, setWindowRange] = useState<[number, number] | null>(null)
  const [cursor, setCursor] = useState(0)
  const [playing, setPlaying] = useState(false)
  const [rate, setRate] = useState(1)
  const [result, setResult] = useState<HistoryResult>({ key: '' })
  const [visualSource, setVisualSource] = useState('A')
  const [narrow, setNarrow] = useState(false)
  const rootRef = useRef<HTMLElement>(null)
  const infoA = useRecordingSceneInfo(sessionId)
  const infoB = useRecordingSceneInfo(compareId)
  const hasVisualA = !!infoA.info?.camera.available || !!infoA.info?.scene?.topics.length
  const hasVisualB = !!infoB.info?.camera.available || !!infoB.info?.scene?.topics.length
  useEffect(() => {
    const element = rootRef.current
    if (!element) return
    const observer = new ResizeObserver(entries => setNarrow(entries[0].contentRect.width <= 500))
    observer.observe(element)
    return () => observer.disconnect()
  }, [])
  const discoveredFields = useRef<Record<string, string>>({})
  const queryKey = JSON.stringify([sessionId, topic, field, compareId, offsetS, windowRange])
  const loading = !!topic && result.key !== queryKey

  useEffect(() => {
    if (!topic) return
    let current = true
    const controller = new AbortController()
    const selectedField = field || discoveredFields.current[topic]
    const query = { topic, field: selectedField || undefined, max_points: 2000,
      ...(windowRange ? { start: Math.max(0, windowRange[0]), end: Math.max(0, windowRange[1]) } : {}) }
    const read = async () => {
      const next: HistoryResult = { key: queryKey, bId: compareId }
      const errorMessage = (cause: unknown) => cause instanceof Error ? cause.message : String(cause)
      try { next.a = (await fetchRecordingSeries(sessionId, query, controller.signal)).series }
      catch (cause: unknown) { next.error = errorMessage(cause) }
      if (!current) return
      if (next.a?.field) discoveredFields.current[topic] = next.a.field
      else if (next.a && selectedField) next.a = { ...next.a, field: selectedField }
      if (compareId && next.a?.field) {
        try { next.b = (await fetchRecordingSeries(compareId, { ...query, field: next.a.field,
          ...(windowRange ? { start: Math.max(0, windowRange[0] - offsetS), end: Math.max(0, windowRange[1] - offsetS) } : {}) }, controller.signal)).series }
        catch (cause: unknown) { next.bError = errorMessage(cause) }
      }
      if (current) setResult(previous => {
        const a = next.a && !next.a.fields.length && previous.a?.topic === topic
          ? { ...next.a, fields: previous.a.fields, field: next.a.field ?? previous.a.field, frame_id: next.a.frame_id ?? previous.a.frame_id } : next.a
        const b = next.b && !next.b.fields.length && previous.b?.topic === topic && previous.bId === compareId
          ? { ...next.b, fields: previous.b.fields, field: next.b.field ?? previous.b.field, frame_id: next.b.frame_id ?? previous.b.frame_id } : next.b
        return { ...next, a, b }
      })
    }
    void read()
    return () => { current = false; controller.abort() }
  }, [sessionId, topic, field, compareId, offsetS, windowRange, queryKey])

  const a = result.a?.topic === topic ? result.a : undefined
  const b = result.b?.topic === topic && result.bId === compareId ? result.b : undefined
  const mediaStartA = Math.min(0, ...infoA.info?.camera.segments.map(segment => segment.start_s) ?? [])
  const mediaStartB = Math.min(0, ...infoB.info?.camera.segments.map(segment => segment.start_s) ?? [])
  const durationA = Math.max(a?.duration_s ?? (summary?.duration_ns ?? 0) / 1e9, infoA.info?.duration_s ?? 0)
  const durationB = Math.max(b?.duration_s ?? 0, infoB.info?.duration_s ?? 0)
  const fullStart = Math.min(mediaStartA, compareId ? mediaStartB + offsetS : 0)
  const fullEnd = Math.max(durationA, compareId ? durationB + offsetS : 0, .001)
  const start = windowRange?.[0] ?? fullStart
  const end = windowRange?.[1] ?? fullEnd
  const playhead = Math.max(start, Math.min(end, cursor))
  const plottedField = field || a?.field || ''
  const unitA = a?.fields.find(item => item.path === plottedField)?.unit
  const unitB = b?.fields.find(item => item.path === plottedField)?.unit
  const compatible = !!b && a?.field === b.field && unitA === unitB
  const shiftedB = useMemo(() => shiftSamples(b?.samples ?? [], offsetS), [b, offsetS])
  const visualCanPlay = (hasVisualA && end >= mediaStartA && start <= durationA) || (hasVisualB && end >= mediaStartB + offsetS && start <= durationB + offsetS)
  const canPlay = visualCanPlay || canPlayRecordingWindow(a?.samples ?? [], shiftedB, compatible, start, end)
  const playbackLoading = loading && !visualCanPlay
  const isPlaying = playing && cursor < end && !playbackLoading && canPlay && (!result.error || visualCanPlay)
  const cameraTime = cameraCursorTime(playhead, 0, isPlaying)
  const cursorA = sampleAt(a?.samples ?? [], playhead)
  const cursorB = sampleAt(shiftedB, playhead)
  const difference = compatible && cursorA?.value !== null && cursorA?.value !== undefined && cursorB?.value !== null && cursorB?.value !== undefined ? cursorB.value - cursorA.value : undefined
  const sourceRecords: Array<[string, RecordingSeries]> = a ? [['A', a]] : []
  if (compareId && b) sourceRecords.push(['B', b])

  useEffect(() => {
    if (!isPlaying) return
    let last = performance.now()
    const timer = window.setInterval(() => {
      const now = performance.now()
      const elapsed = (now - last) / 1000
      last = now
      setCursor(previous => advancePlayhead(previous, elapsed, rate, start, end))
    }, 50)
    return () => window.clearInterval(timer)
  }, [isPlaying, rate, start, end])

  const seek = (time: number) => { setPlaying(false); setCursor(Math.max(start, Math.min(end, time))) }
  const resetView = () => { setPlaying(false); setWindowRange(null); setCursor(0) }
  const zoom = (scale: number) => {
    const span = Math.min(fullEnd - fullStart, Math.max(.05, (end - start) * scale))
    const left = Math.max(fullStart, Math.min(fullEnd - span, playhead - span / 2))
    setPlaying(false)
    setWindowRange([left, left + span])
  }

  return <section ref={rootRef} className={styles.history} aria-label={text(locale, 'Historical numeric inspector', '历史数值查看器')}>
    <header className={styles.heading}><div><h3>{text(locale, 'Recording history', '历史回看')}</h3><span>{text(locale, 'Inspect saved recordings or compare two runs', '查看已保存的录制，也可以对齐比较两次运行')}</span></div><span className={styles.readOnly}>{text(locale, 'Recorded · not live', '录制回看 · 非实时')}</span></header>
    <div className={styles.comparison}>
      <label><AlignStartVertical size={14} /><span>{text(locale, 'Compare B', '对比 B')}</span><select aria-label={text(locale, 'Comparison recording B', '对比记录 B')} value={compareId} onChange={event => { setCompareId(event.target.value); setOffset('0'); resetView() }}><option value="">{text(locale, 'No comparison', '不比较')}</option>{sessions.filter(item => item.session_id !== sessionId).map(item => <option key={item.session_id} value={item.session_id}>{item.session_id}</option>)}</select></label>
      {compareId && <label className={styles.offset}>{text(locale, 'B offset', 'B 偏移')}<input aria-label={text(locale, 'B time offset in seconds; positive delays B', 'B 时间偏移秒数，正数使 B 延后')} type="number" step="0.1" value={offset} onChange={event => { setOffset(event.target.value); setPlaying(false) }} /><span>s</span><button type="button" onClick={() => { setOffset('0'); resetView() }} aria-label={text(locale, 'Reset alignment', '重置对齐')} title={text(locale, 'Reset alignment', '重置对齐')}><RotateCcw size={13} /></button></label>}
    </div>
    {compareId && <p className={styles.hint}>{text(locale, 'Both recordings start at 0 s. Positive offset moves B later; negative offset extends the shared timeline before 0. Values are unchanged; differences do not establish accuracy.', '默认以各自起点对齐到 0 秒；正偏移使 B 延后，负偏移可将共同时间轴延伸到 0 秒之前。仅移动时间，数值差异不代表精度。')}</p>}
    <div className={styles.timeline}>
      <div className={styles.coverage} aria-label={text(locale, 'Recording time coverage', '记录时间覆盖')}>
        {[[mediaStartA, durationA, 'A'], ...(compareId ? [[mediaStartB + offsetS, durationB + offsetS, 'B']] : [])].map(([left, right, label]) => {
          const leftTime = Math.max(start, Number(left)); const rightTime = Math.min(end, Number(right))
          return <div key={label} data-secondary={label === 'B'}><span>{label}</span>{rightTime >= leftTime && <i style={{ left: `${Math.max(0, (leftTime - start) / (end - start) * 100)}%`, width: `${Math.max(0, (rightTime - leftTime) / (end - start) * 100)}%` }} />}</div>
        })}
      </div>
      <input className={styles.scrubber} aria-label={text(locale, 'Historical playhead in seconds', '历史播放位置秒数')} type="range" min={start} max={end} step={Math.max(.001, (end - start) / 10000)} value={playhead} onChange={event => seek(Number(event.target.value))} />
      <div className={styles.transport}>
        <div className={styles.playControls}>
          <button type="button" onClick={() => seek(playhead - 1)} title={text(locale, 'Back one second', '后退一秒')} aria-label={text(locale, 'Back one second', '后退一秒')}><ChevronLeft size={16} /></button>
          <button type="button" className={styles.play} disabled={playbackLoading || !canPlay} onClick={() => { if (isPlaying) setPlaying(false); else { if (cursor >= end) setCursor(start); setPlaying(true) } }} aria-label={isPlaying ? text(locale, 'Pause historical playback', '暂停历史回看') : text(locale, 'Play historical recording', '播放历史记录')}>{isPlaying ? <Pause size={15} /> : <Play size={15} />}</button>
          <button type="button" onClick={() => seek(playhead + 1)} title={text(locale, 'Forward one second', '前进一秒')} aria-label={text(locale, 'Forward one second', '前进一秒')}><ChevronRight size={16} /></button>
          <select aria-label={text(locale, 'Historical playback speed', '历史回看倍速')} value={rate} onChange={event => setRate(Number(event.target.value))}>{[.25, .5, 1, 2, 4].map(value => <option key={value} value={value}>{value}×</option>)}</select>
        </div>
        <span className={styles.time}><input aria-label={text(locale, 'Seek to second', '跳转到秒')} type="number" min={start} max={end} step="0.1" value={Number(playhead.toFixed(2))} onChange={event => seek(Number(event.target.value))} /> / {end.toFixed(2)} s</span>
        <div className={styles.zoom}><button onClick={() => zoom(.5)} aria-label={text(locale, 'Zoom into timeline', '放大时间窗')} title={text(locale, 'Zoom in around playhead', '以播放位置放大')}><ZoomIn size={15} /></button><button onClick={() => zoom(2)} aria-label={text(locale, 'Zoom out of timeline', '缩小时间窗')}><ZoomOut size={15} /></button><button onClick={resetView} title={text(locale, 'Show full recording', '显示完整时间')} aria-label={text(locale, 'Show full recording', '显示完整时间')}><RotateCcw size={13} /></button></div>
      </div>
    </div>
    {(hasVisualA || hasVisualB) && <>
      {compareId && narrow && <div className={styles.visualSourceTabs} role="group" aria-label={text(locale, 'Visible comparison source', '当前比较画面来源')}><button aria-pressed={visualSource === 'A'} onClick={() => setVisualSource('A')}>A</button><button aria-pressed={visualSource === 'B'} onClick={() => setVisualSource('B')}>B</button><span>{text(locale, 'Same shared playhead', '共用同一播放位置')}</span></div>}
      <div className={styles.visualGrid} data-compare={!!compareId && !narrow}>
        {(!narrow || visualSource === 'A' || !compareId) && <RecordingVisualSource key={`A:${sessionId}`} label="A" sessionId={sessionId} time={playhead} cameraTime={cameraTime} playing={isPlaying} info={infoA.info} error={infoA.error} locale={locale} single={!compareId} />}
        {compareId && (!narrow || visualSource === 'B') && <RecordingVisualSource key={`B:${compareId}`} label="B" sessionId={compareId} time={playhead - offsetS} cameraTime={cameraCursorTime(playhead, offsetS, isPlaying)} playing={isPlaying} info={infoB.info} error={infoB.error} locale={locale} />}
      </div>
    </>}
    {!hasVisualA && !hasVisualB && (infoA.error || infoB.error) && <p className={styles.hint}>{text(locale, 'Visual data unavailable', '场景数据暂不可用')} · {infoA.error || infoB.error}</p>}
    {!!available.length && <details className={styles.numericDetails} open={!hasVisualA && !hasVisualB}>
      <summary>{text(locale, 'Numeric curves', '数值曲线')}</summary>
    {!!available.length && <div className={styles.selectors}>
      <label>{text(locale, 'Data stream', '数据流')}<select aria-label={text(locale, 'Historical data stream', '历史数据流')} value={topic} onChange={event => { setChosenTopic(event.target.value); setField(''); resetView() }}>
        {!available.length && <option value="">{text(locale, 'No received streams', '没有收到的数据流')}</option>}
        {available.map(item => <option key={item.topic} value={item.topic}>{topics.find(value => value.topic === item.topic)?.label ?? item.topic} · {item.topic}</option>)}
      </select></label>
      <label>{text(locale, 'Numeric field', '数值字段')}<select aria-label={text(locale, 'Historical numeric field', '历史数值字段')} value={plottedField} disabled={!a?.fields.length} onChange={event => { setField(event.target.value); setPlaying(false) }}>
        {!a?.fields.length && <option value="">{loading ? text(locale, 'Reading fields…', '正在读取字段…') : text(locale, 'No numeric field', '无数值字段')}</option>}
        {a?.fields.map(item => <option key={item.path} value={item.path}>{item.path}{item.unit ? ` (${item.unit})` : ''}</option>)}
      </select></label>
    </div>}
    {!!available.length && <div className={styles.chartArea} aria-busy={loading}>
      <RecordingPlot traces={[{ label: `A · ${sessionId}`, samples: loading ? [] : a?.samples ?? [], localTimeOffset: 0 }, ...(compatible && !loading ? [{ label: `B · ${compareId}`, samples: shiftedB, secondary: true, localTimeOffset: offsetS }] : [])]} start={start} end={end} cursor={playhead} unit={unitA} locale={locale} onSeek={seek} />
      {loading && <span className={styles.loading} role="status">{text(locale, 'Reading recorded messages…', '正在读取录制消息…')}</span>}
    </div>}
    {compareId && compatible && !loading && <div className={styles.difference}><span>{text(locale, 'Δ B − A · displayed samples', 'Δ B − A · 显示采样值')}</span><strong>{formatSeriesValue(difference)}{unitA ? ` ${unitA}` : ''}</strong></div>}
    {result.key === queryKey && (result.error || result.bError) && <p className={styles.error} role="alert">{result.error ? `A · ${result.error}` : `B · ${result.bError}`}</p>}
    {compareId && b && !compatible && !loading && <p className={styles.error} role="status">{text(locale, 'B does not contain the same numeric field and unit; it is not plotted on this axis.', 'B 没有相同数值字段或单位，因此不叠加到此坐标轴。')}</p>}
    {compareId && a?.frame_id && b?.frame_id && a.frame_id !== b.frame_id && <p className={styles.hint}>{text(locale, 'Different coordinate frames', '坐标系不同')} · A: {a.frame_id} / B: {b.frame_id}</p>}
    {!loading && a && <>
      <footer className={styles.footer}>
        {sourceRecords.map(([label, source]) => <span key={label}>{label} · {source.returned_count.toLocaleString()} / {source.numeric_count.toLocaleString()} {text(locale, 'numeric samples', '数值样本')}{source.downsampling.method === 'minmax' ? text(locale, ' · min/max downsampled', ' · 极值降采样') : ''}</span>)}
        <span>{text(locale, 'Gaps stay empty. Playback never publishes DDS or motion commands.', '缺失数据留空，回看不发布 DDS 或运动指令。')}</span>
      </footer>
      <details className={styles.metadata}>
        <summary>{text(locale, 'Source time and coordinate frames', '原始时间与坐标系')}</summary>
        {sourceRecords.map(([label, source]) => <dl key={label}>
          <div><dt>{label} · {text(locale, 'File start (Unix ns)', '文件起点（Unix ns）')}</dt><dd>{source.start_time_ns}</dd></div>
          <div><dt>{label} · frame_id</dt><dd>{source.frame_id || '—'}</dd></div>
        </dl>)}
      </details>
    </>}
    </details>}
  </section>
}
