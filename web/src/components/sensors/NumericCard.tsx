import { useEffect, useMemo, useRef, useState } from 'react'
import { Download, Pause, Play } from 'lucide-react'
import { appendTrace, numericFields, readDisplay, sensorRequest, traceCsv, type NumericSensor, type SensorTrace } from '../../services/sensors.ts'
import { numericChannels } from '../../services/sensorChart.ts'
import { TraceChart } from './TraceChart.tsx'
import { SensorResult } from '../agentConversation/SensorResult.tsx'
import styles from '../SensorMonitor.module.css'

export function NumericCard({ sensor }: { sensor: NumericSensor }) {
  const [settings] = useState(readDisplay)
  const [snapshot, setSnapshot] = useState<Record<string, unknown> | null>(null)
  const [rows, setRows] = useState<SensorTrace[]>([])
  const [error, setError] = useState('')
  const [paused, setPaused] = useState(false)
  const [selected, setSelected] = useState<string[] | null>(null)
  const [windowS, setWindow] = useState(settings.window)
  const [viewEnd, setViewEnd] = useState<number | null>(null)
  const origin = useRef(performance.now())
  useEffect(() => {
    if (paused) return
    const abort = new AbortController()
    let timer: ReturnType<typeof setTimeout>
    function append(values: Record<string, number>, source?: unknown) {
      const row = { t: (performance.now() - origin.current) / 1000, receivedAt: Date.now() / 1000, source, values }
      setRows(previous => appendTrace(previous, row))
    }
    async function poll() {
      try {
        const next = await sensorRequest(`/api/v1/robot/sensors/${sensor}`, abort.signal)
        if (abort.signal.aborted) return
        setSnapshot(next); setError('')
        const source = next.source_stamp_s ?? (next.values as Record<string, unknown>)?.source_elapsed
        append(next.fresh === false ? {} : numericFields(next), source)
      } catch (cause) {
        if (abort.signal.aborted) return
        setError(String(cause)); setSnapshot(null); append({})
      } finally { if (!abort.signal.aborted) timer = setTimeout(() => void poll(), settings.interval) }
    }
    void poll()
    return () => { abort.abort(); clearTimeout(timer) }
  }, [sensor, paused, settings.interval])
  const channels = useMemo(() => numericChannels([...new Set(rows.flatMap(row => Object.keys(row.values)))]).map(channel => ({ ...channel, label: sensor === 'motors' ? `M${channel.label.padStart(2,'0')}` : sensor === 'voltages' ? channel.label.replace('通道 ', 'CH ') : channel.label })), [rows, sensor])
  const chosen = selected === null ? channels.filter(channel => channel.group === channels[0]?.group && channel.unit === channels[0]?.unit).slice(0,3).map(channel => channel.id) : selected.filter(id => channels.some(channel => channel.id === id))
  const chartRows = useMemo(() => rows.map((row,index) => ({ ...row, gap_before: index > 0 && row.t - rows[index - 1].t > settings.interval / 1000 * 2.5 })), [rows, settings.interval])
  function toggleChannel(label: string) {
    const channel = channels.find(channel => channel.label === label)
    if (channel) setSelected(chosen.includes(channel.id) ? chosen.filter(id => id !== channel.id) : [...chosen, channel.id])
  }
  const data = snapshot?.values as { motors?: Array<{ id: number; online?: boolean; temperature_c?: number; errors?: unknown[] }>; volts?: number[]; joints?: unknown[] } | undefined
  const count = data?.motors?.length ?? data?.volts?.length ?? data?.joints?.length
  const offline = data?.motors?.filter(row => row.online === false).length ?? 0
  const withErrors = data?.motors?.filter(row => Array.isArray(row.errors) && row.errors.length > 0).length ?? 0
  function download() {
    const url = URL.createObjectURL(new Blob(['\ufeff', traceCsv(rows)], { type: 'text/csv;charset=utf-8' }))
    const link = document.createElement('a'); link.href = url; link.download = `${sensor}-display-${Date.now()}.csv`; link.click()
    setTimeout(() => URL.revokeObjectURL(url), 1000)
  }
  return <>
    <div className={styles.numericStatus}><span className={styles.status} data-live={!paused && !error && !!snapshot && snapshot.fresh !== false}>{paused ? '画面暂停' : error ? '连接中断' : snapshot?.fresh === false ? '观测过期' : snapshot ? '已收到观测' : '正在连接'}</span><span className={offline || withErrors ? styles.error : undefined}>{offline || withErrors ? `${offline} 离线 · ${withErrors} 有错误码` : count === undefined ? '等待来源' : `${count} ${sensor === 'joints' ? '个关节' : sensor === 'motors' ? '个电机' : '个通道'}`}</span></div>
    <TraceChart rows={chartRows} channels={channels} selected={chosen} onSelect={setSelected} seconds={windowS} onSeconds={setWindow} end={viewEnd} onSeek={setViewEnd}
      actions={<button type="button" onClick={() => setPaused(value => !value)} aria-label={paused ? '继续' : '暂停'}>{paused ? <Play size={14} /> : <Pause size={14} />}</button>} />
    <div className={styles.exportRow}><span>{settings.interval / 1000} s 刷新</span><button type="button" disabled={!channels.length} onClick={download} title="导出本卡片所有收到的通道"><Download size={13} />CSV</button></div>
    {sensor === 'motors' && data?.motors && <div className={styles.channels} aria-label="电机温度">{data.motors.map(row => <button type="button" key={row.id} onClick={() => toggleChannel(`M${String(row.id).padStart(2,'0')}`)} aria-pressed={channels.some(channel => channel.label === `M${String(row.id).padStart(2,'0')}` && chosen.includes(channel.id))}><small>M{String(row.id).padStart(2,'0')}<span data-online={row.online}>{row.online === undefined ? '—' : row.online ? '在线' : '离线'}</span></small><b>{Number.isFinite(row.temperature_c) ? row.temperature_c!.toFixed(1) : '—'} <i>°C</i></b></button>)}</div>}
    {sensor === 'voltages' && data?.volts && <div className={styles.channels} aria-label="通道电压">{data.volts.map((value,index) => <button type="button" key={index} onClick={() => toggleChannel(`CH ${index}`)} aria-pressed={channels.some(channel => channel.label === `CH ${index}` && chosen.includes(channel.id))}><small>CH {String(index).padStart(2,'0')}</small><b>{Number.isFinite(value) ? value.toFixed(2) : '—'} <i>V</i></b></button>)}</div>}
    {error && <p role="status" className={styles.error}>{error}</p>}
    {snapshot && <details className={styles.details} open={sensor === 'joints'}><summary>{sensor === 'joints' ? '全部关节' : '状态与来源'}</summary><SensorResult result={snapshot} monitorLink={false} /></details>}
    <details className={styles.details}><summary>显示采样 · 最近 {windowS} 秒</summary><p className={styles.note}>保留最近五分钟的网页采样，关闭卡片后停止。CSV 包含接收时间与来源时间，并非原始高频记录。</p></details>
  </>
}
