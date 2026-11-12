import { useEffect, useMemo, useRef, useState } from 'react'
import { Circle, Download, History, Pause, Play, Radio, Square } from 'lucide-react'
import { TraceChart } from './sensors/TraceChart.tsx'
import { traceEnd, type TraceChannel } from '../services/sensorChart.ts'
import { readDisplay } from '../services/sensors.ts'
import * as api from '../services/imu.ts'
import type { ImuSample, ImuHistory, ImuLive, ImuRecord } from '../services/imu.ts'
import styles from './ImuMonitor.module.css'

const states: Record<string, string> = { recording: '记录中', completed: '已保存', failed: '保存失败', interrupted: '已中断' }
const time = (value: number) => `${Math.floor(value / 60)}:${Math.floor(value % 60).toString().padStart(2, '0')}`
const imuChannels: TraceChannel[] = [
  ['angular_velocity_rad_s', '角速度', 'rad/s'], ['linear_acceleration_m_s2', '线加速度', 'm/s²'],
].flatMap(([field, group, unit]) => ['x','y','z'].map((axis,index) => ({ id: `${field}.${axis}`, label: axis.toUpperCase(), group, unit, color: ['#5b83cc','#49988a','#bd874c'][index] })))

export function ImuPanel({ onAnalyze }: { onAnalyze?: (id: string, start: number, end: number) => void }) {
  const [live, setLive] = useState<ImuLive | null>(null)
  const [rows, setRows] = useState<ImuSample[]>([])
  const [paused, setPaused] = useState<ImuSample[] | null>(null)
  const [windowS, setWindow] = useState(() => readDisplay().window)
  const [selected, setSelected] = useState(imuChannels.map(channel => channel.id))
  const [liveEnd, setLiveEnd] = useState<number | null>(null)
  const [error, setError] = useState('')
  const [streamError, setStreamError] = useState('')
  const [busy, setBusy] = useState(false)
  const [records, setRecords] = useState<ImuRecord[] | null>(null)
  const [history, setHistory] = useState<ImuHistory | null>(null)
  const [cursor, setCursor] = useState(0)
  const [playing, setPlaying] = useState(false)
  const [notice, setNotice] = useState('')
  const sequence = useRef(0)
  const pendingRecord = useRef<{ action: 'start' | 'stop'; id?: string; uncertain?: boolean } | null>(null)
  const recording = live?.recording?.state === 'recording'
  useEffect(() => {
    const abort = new AbortController()
    let timer: ReturnType<typeof setTimeout>
    async function poll() {
      try {
        const next = await api.readImu(sequence.current, abort.signal)
        if (abort.signal.aborted) return
        const reset = next.sequence < sequence.current
        sequence.current = reset ? 0 : next.sequence
        setLive(next); setStreamError(next.error ?? '')
        const pending = pendingRecord.current
        if (pending?.uncertain && ((pending.action === 'start' && next.recording?.state === 'recording')
          || (pending?.action === 'stop' && next.recording?.id === pending.id && next.recording?.state === 'completed'))) {
          pendingRecord.current = null
          setError('')
          setNotice(pending.action === 'start' ? '已确认记录正在 Host 进行。' : `已确认保存 ${next.recording!.sample_count} 条测量。`)
        }
        setRows(previous => api.mergeImu(previous, next.samples, reset))
        timer = setTimeout(() => void poll(), 200)
      } catch (cause) {
        if (abort.signal.aborted) return
        setStreamError(cause instanceof Error ? cause.message : String(cause))
        timer = setTimeout(() => void poll(), 2000)
      }
    }
    void poll()
    return () => { abort.abort(); clearTimeout(timer) }
  }, [])
  useEffect(() => {
    if (!playing || !history) return
    const timer = setInterval(() => setCursor(value => {
      const end = history.samples.at(-1)?.t ?? 0
      if (value + .1 >= end) { setPlaying(false); return end }
      return value + .1
    }), 100)
    return () => clearInterval(timer)
  }, [playing, history])

  async function record() {
    setBusy(true); setError(''); setNotice('')
    pendingRecord.current = recording ? { action: 'stop', id: live!.recording!.id } : { action: 'start' }
    try {
      const result = recording ? await api.stopImuRecord(live!.recording!.id) : await api.startImuRecord()
      pendingRecord.current = null
      setLive(value => value ? { ...value, recording: result } : value)
      setHistory(previous => previous?.record.id === result.id ? { ...previous, record: result } : previous)
      setNotice(result.state === 'completed' ? `已保存 ${result.sample_count.toLocaleString()} 条测量` : '记录已在 Host 开始；关闭此卡片会继续记录。')
      if (records) setRecords((await api.listImuRecords()).records)
    } catch (cause) { if (pendingRecord.current) pendingRecord.current.uncertain = true; setError(cause instanceof Error ? cause.message : String(cause)) }
    finally { setBusy(false) }
  }
  async function library() {
    setError('')
    try { setRecords((await api.listImuRecords()).records) }
    catch (cause) { setError(cause instanceof Error ? cause.message : String(cause)) }
  }
  async function review(id: string) {
    setBusy(true); setError('')
    try { const result = await api.readImuRecord(id); setHistory(result); setCursor(result.samples.at(-1)?.t ?? 0); setPlaying(false); setRecords(null) }
    catch (cause) { setError(cause instanceof Error ? cause.message : String(cause)) }
    finally { setBusy(false) }
  }
  const shown = history?.samples ?? paused ?? rows
  const chartRows = useMemo(() => shown.map(row => ({ t: row.t, receivedAt: row.received_at_s, gap_before: row.gap_before,
    values: Object.fromEntries(imuChannels.map(channel => { const [field,axis] = channel.id.split('.'); return [channel.id, row.values[field]?.[axis]] })) })), [shown])
  const end = traceEnd(chartRows, history ? cursor : liveEnd)
  const start = Math.max(0, end - windowS)
  const currentRecord = history?.record ?? live?.recording
  const state = history ? '回看记录' : paused ? '画面已暂停' : streamError ? '连接异常' : live?.state === 'live' ? '实时' : live?.state === 'disconnected' ? '已断开' : '连接中'
  return <div className={styles.embedded}>
    <div className={styles.body}>
      <div className={styles.source}><Radio size={12} /><span>{history?.record.source?.provider ?? live?.provider ?? '等待适配器'}{live?.robot ? ` · ${live.robot}` : ''}</span><span className={styles.liveState} data-live={state === '实时'}>{state}</span></div>
      <TraceChart rows={chartRows} channels={imuChannels} selected={selected} onSelect={setSelected} seconds={windowS} onSeconds={setWindow}
        end={history ? cursor : liveEnd} defaultLayout="stack" live={!history} onSeek={value => {
          if (history) { setCursor(value ?? history.samples.at(-1)?.t ?? 0); setPlaying(false) } else setLiveEnd(value)
        }} actions={<button type="button" onClick={() => history ? (() => { if (!playing && cursor >= (history.samples.at(-1)?.t ?? 0)) setCursor(0); setPlaying(!playing) })() : setPaused(paused ? null : [...rows])} aria-label={history ? playing ? '暂停回放' : '播放记录' : paused ? '继续实时显示' : '暂停画面'}>{(history ? playing : !paused) ? <Pause size={14} /> : <Play size={14} />}</button>} />
      {history && <div className={styles.playback}><input aria-label="记录回放位置" type="range" min={0} max={Math.max(.001, history.samples.at(-1)?.t ?? 0)} step="0.1" value={cursor} onChange={event => { setCursor(Number(event.target.value)); setPlaying(false) }} /><button type="button" onClick={() => { setHistory(null); setPlaying(false); setPaused(null); setLiveEnd(null) }}>返回实时</button></div>}
      <div className={styles.record}>{(!history || recording) && <button type="button" className={recording ? styles.stop : styles.start} disabled={busy || (!live && !recording)} onClick={() => void record()}>{recording ? <Square size={12} /> : <Circle size={12} />}{recording ? '停止并保存' : 'Record · 开始记录'}</button>}<button type="button" onClick={() => records ? setRecords(null) : void library()}><History size={14} />记录</button></div>
      {recording && <p className={styles.recording}><i /><span>记录中 · {time(live!.recording!.duration_s ?? 0)} · {live!.recording!.sample_count.toLocaleString()} 条</span><small>画面暂停不影响记录</small></p>}
      {notice && <p role="status" className={styles.note}>{notice}</p>}
      {currentRecord && currentRecord.state !== 'recording' && <div className={styles.downloads}><span>{states[currentRecord.state] ?? currentRecord.state} · {currentRecord.sample_count ?? '—'} 条</span><a href={api.imuDownload(currentRecord.id, 'csv')} download><Download size={13} />CSV</a><a href={api.imuDownload(currentRecord.id, 'jsonl')} download>JSONL</a></div>}
      {history && <p className={styles.note}>回看曲线保留各轴极值；CSV／JSONL 导出完整观测。区间统计使用所有测量。</p>}
      {history && onAnalyze && <button type="button" className={styles.analyze} onClick={() => { onAnalyze(history.record.id, start, end); setNotice('已将这段分析请求加入聊天草稿。') }}>让 Agent 分析这段 · {time(start)}–{time(end)}</button>}
      {records && <section className={styles.library} aria-label="IMU 记录列表"><h3>最近记录</h3>{!records.length && <p>还没有保存的 IMU 记录</p>}{records.map(item => <button type="button" key={item.id} disabled={busy} onClick={() => void review(item.id)}><span>{new Date(item.started_at_s * 1000).toLocaleString()}<small>{states[item.state] ?? item.state} · {item.sample_count ?? '—'} 条</small></span><Play size={13} /></button>)}</section>}
      {(error || streamError || currentRecord?.error) && <p className={styles.error} role="alert">{error || streamError || currentRecord?.error}</p>}
      <details className={styles.info}><summary>来源与记录说明</summary><p className={styles.note}>实时显示最多五分钟／3 万条；Record 保存完整收到的观测。原始数据保留来源时钟与时间，曲线使用接收后的相对时间。关闭卡片后，Host 记录仍继续。</p></details>
    </div>
  </div>
}
