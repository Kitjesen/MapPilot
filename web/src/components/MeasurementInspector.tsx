import { useEffect, useReducer, useRef, useState } from 'react'
import { text, type Locale } from '../i18n'
import { isObservationMode } from '../services/observationMode.ts'
import {
  acousticBin, bindingKey, controlDisabledReason, controlMeasurement, getMeasurementPreview,
  hasFreshMeasurement, listMeasurements, measurementControlReducer, measurementNoise,
  pollMeasurements, semanticValue, thermalValue,
  type AcousticDisplay, type ControlAck, type ControlOperation, type MeasurementPreview,
  type MeasurementStatus, type SemanticDisplay, type ThermalDisplay,
} from '../services/measurements.ts'
import styles from './MeasurementInspector.module.css'

function ImagePreview({ display, locale }: { display: ThermalDisplay | SemanticDisplay; locale: Locale }) {
  const canvas = useRef<HTMLCanvasElement>(null)
  const [unit, setUnit] = useState<'K' | 'C'>('C')
  const [row, setRow] = useState(0)
  const [column, setColumn] = useState(0)
  const values = display.kind === 'thermal'
    ? display.temperature_k.flatMap((line, r) => line.flatMap((_, c) => {
      const value = thermalValue(display, r, c, unit)
      return value === null ? [] : [value]
    })) : []
  const low = values.length ? Math.min(...values) : 0
  const high = values.length ? Math.max(...values) : 0
  useEffect(() => {
    const context = canvas.current?.getContext('2d')
    if (!context) return
    const pixels = context.createImageData(display.width, display.height)
    for (let r = 0; r < display.height; r++) for (let c = 0; c < display.width; c++) {
      let color: number[] = (r + c) % 2 ? [175, 175, 175] : [75, 75, 75]
      if (display.kind === 'thermal') {
        const value = thermalValue(display, r, c, unit)
        if (value !== null) {
          const level = high > low ? (value - low) / (high - low) : .5
          color = [Math.round(255 * level), Math.round(210 * level), Math.round(210 * (1 - level))]
        }
      } else {
        const value = semanticValue(display, r, c)
        if (value) {
          const hue = (value.classId * 67) % 255
          color = [hue, (hue + 89) % 255, (hue + 173) % 255]
        }
      }
      pixels.data.set([...color, 255], (r * display.width + c) * 4)
    }
    context.putImageData(pixels, 0, 0)
  }, [display, unit, low, high])
  const r = Math.min(row, display.height - 1)
  const c = Math.min(column, display.width - 1)
  const temperature = display.kind === 'thermal' ? thermalValue(display, r, c, unit) : null
  const semantic = display.kind === 'semantic' ? semanticValue(display, r, c) : null
  return <figure className={styles.visual}>
    <figcaption>{display.kind === 'thermal' ? text(locale, 'Temperature', '温度')
      : text(locale, 'Semantic ground truth · not detection', '语义真值 · 非检测结果')}
      {' · '}{display.width} × {display.height} / {display.source_width} × {display.source_height}
      {' · '}{text(locale, 'Nearest sampled display; checkerboard = invalid', '最近邻采样展示；棋盘格 = 无效')}</figcaption>
    <canvas ref={canvas} width={display.width} height={display.height}
      aria-label={text(locale, 'Sampled measurement image; inspect pixel numbers below', '采样测量图；下方可检查像素数值')} />
    <div className={styles.fields}>
      {display.kind === 'thermal' && <label>{text(locale, 'Scale', '温标')}<select value={unit} onChange={e => setUnit(e.target.value as 'K' | 'C')}>
        <option value="C">°C</option><option value="K">K</option>
      </select></label>}
      <label>{text(locale, 'Preview row', '预览行')}<input type="number" min={0} max={display.height - 1} value={r}
        onChange={e => setRow(Math.max(0, Math.floor(Number(e.target.value))))} /></label>
      <label>{text(locale, 'Preview column', '预览列')}<input type="number" min={0} max={display.width - 1} value={c}
        onChange={e => setColumn(Math.max(0, Math.floor(Number(e.target.value))))} /></label>
    </div>
    <p className={styles.note}>{text(locale, 'Source pixel (row, column)', '原始像素（行，列）')}: ({display.row_indices[r]}, {display.column_indices[c]}) · {display.kind === 'thermal'
      ? temperature === null ? text(locale, 'Invalid', '无效') : `${temperature.toFixed(3)} ${unit === 'C' ? '°C' : 'K'}`
      : semantic ? `class_id: ${semantic.classId}, instance_id: ${semantic.instanceId}` : text(locale, 'Invalid', '无效')}
      {display.kind === 'thermal' && ` · ${text(locale, 'Display range', '显示范围')}: ${values.length ? `${low.toFixed(2)} – ${high.toFixed(2)} ${unit === 'C' ? '°C' : 'K'}` : '—'}`}</p>
  </figure>
}

function Waveform({ display, locale }: { display: AcousticDisplay; locale: Locale }) {
  return <figure className={styles.visual}>
    <figcaption>{text(locale, 'Pressure envelope, not raw waveform', '声压包络，非原始波形')} · {display.sample_rate_hz} Hz · {display.source_sample_count} {text(locale, 'samples/channel', '样本/通道')}</figcaption>
    {display.pressure_min_pa.map((bins, channel) => {
      const samples = bins.map((_, bin) => acousticBin(display, channel, bin))
      const peak = Math.max(...samples.flatMap(bin => bin ? [Math.abs(bin.min), Math.abs(bin.max)] : []), 1e-12)
      const duration = display.source_sample_count / display.sample_rate_hz
      return <div key={channel}>
        <p className={styles.note}>{text(locale, 'Channel', '通道')} {channel} · ±{peak.toPrecision(4)} Pa · {text(locale, 'Gaps = invalid; faint = partial validity', '空缺 = 无效；淡色 = 部分有效')}</p>
        <svg className={styles.wave} viewBox="0 0 600 130" role="img" aria-label={`${text(locale, 'Channel pressure envelope', '通道声压包络')} ${channel}, Pa / s`}>
          <line className={styles.axis} x1="45" y1="55" x2="585" y2="55" />
          <text x="0" y="15">Pa</text><text x="0" y="58">0</text>
          {samples.map((bin, i) => bin && <line key={i} x1={45 + 540 * bin.start / duration} x2={45 + 540 * bin.start / duration}
            y1={55 - 45 * bin.max / peak} y2={55 - 45 * bin.min / peak} strokeLinecap="round" opacity={bin.fraction < 1 ? .45 : 1}>
            <title>{`[${bin.start}, ${bin.end}) s: ${bin.min} … ${bin.max} Pa; valid ${bin.fraction}`}</title>
          </line>)}
          <text x="45" y="120">0 s</text><text x="500" y="120">{duration.toPrecision(4)} s</text>
        </svg>
      </div>
    })}
  </figure>
}

function SensorControls({ status, online, locale, onConfirmed }: {
  status: MeasurementStatus; online: boolean; locale: Locale; onConfirmed: (ack: ControlAck) => void
}) {
  const [busy, setBusy] = useState(false)
  const [message, setMessage] = useState('')
  const command = useRef<AbortController | null>(null)
  useEffect(() => () => command.current?.abort(), [])
  const settings = status.controls
  const reason = isObservationMode() ? text(locale, 'Read-only observation session', '只读监控会话') : controlDisabledReason(status, online)
  const send = async (operation: ControlOperation, params: Record<string, unknown>) => {
    if (command.current || reason) return
    const controller = new AbortController()
    command.current = controller
    setBusy(true)
    setMessage('')
    try {
      const result = await controlMeasurement(status, operation, params, controller.signal)
      if (controller.signal.aborted) return
      if (result.applied) {
        onConfirmed(result)
      }
      setMessage(`${result.applied ? text(locale, 'Source ACK confirmed', '来源 ACK 已确认') : text(locale, 'Not applied', '未应用')}: ${result.reason} (${result.request_id})`)
    } catch (error) {
      if (!controller.signal.aborted) setMessage(`${text(locale, 'Unconfirmed; no automatic retry. Query settings before resending.', '结果未确认；不会自动重试。重新发送前请查询设置。')} ${error instanceof Error ? error.message : String(error)}`)
    } finally {
      command.current = null
      if (!controller.signal.aborted) setBusy(false)
    }
  }
  const noise = measurementNoise(settings)
  const noiseEnabled = settings?.noise_enabled ?? noise?.enabled
  const gimbal = settings?.measurement_settings?.camera?.gimbal
  return <section>
    {reason && <p className={styles.note}>{reason}</p>}
    <fieldset disabled={!!reason || busy}>
      <legend>{text(locale, 'Sensor controls · source-confirmed settings only', '传感器控制 · 仅显示来源确认设置')}</legend>
      <div className={styles.controls}>
        <button type="button" onClick={() => void send('query', {})}>{busy ? text(locale, 'Waiting for ACK…', '等待 ACK…') : text(locale, 'Query settings', '查询设置')}</button>
        {(['enabled', 'noise_enabled', 'recording'] as const).map(operation => {
          const enabled = operation === 'noise_enabled' ? noiseEnabled : settings?.[operation]
          return typeof enabled === 'boolean' && <label key={operation}>
          <input type="checkbox" checked={enabled} onChange={e => void send(operation, { enabled: e.target.checked })} />
          {operation === 'enabled' ? text(locale, 'Enabled', '启用') : operation === 'recording' ? text(locale, 'Recording', '录制') : text(locale, 'Noise enabled', '噪声开关')}
        </label>})}
      </div>
      {typeof settings?.rate_hz === 'number' && <form className={styles.fields} key={`rate:${settings.rate_hz}`} onSubmit={e => {
        e.preventDefault(); const data = new FormData(e.currentTarget); void send('rate', { rate_hz: Number(data.get('rate')) })
      }}><label>{text(locale, 'Rate (Hz)', '频率（Hz）')}<input name="rate" type="number" required min="0.000001" step="any" defaultValue={settings.rate_hz} /></label><button>{text(locale, 'Set rate', '设置频率')}</button></form>}
      {noise && <form className={styles.fields} key={`noise:${JSON.stringify(noise)}`} onSubmit={e => {
        e.preventDefault(); const data = new FormData(e.currentTarget)
        void send('noise', { enabled: noise.enabled, unit: noise.unit, stddev: Number(data.get('stddev')), seed: Number(data.get('seed')) })
      }}><label>{text(locale, 'Noise σ', '噪声 σ')} ({noise.unit})<input name="stddev" type="number" required min="0" step="any" defaultValue={noise.stddev} /></label>
        <label>{text(locale, 'Seed', '种子')}<input name="seed" type="number" required min="0" max={Number.MAX_SAFE_INTEGER} step="1" defaultValue={Number.isSafeInteger(noise.seed) ? noise.seed : ''} /></label>
        <button>{text(locale, 'Configure noise', '配置噪声')}</button></form>}
      {gimbal && <form className={styles.fields} key={`gimbal:${JSON.stringify(gimbal)}`} onSubmit={e => {
        e.preventDefault(); const data = new FormData(e.currentTarget); void send('gimbal', { pan_deg: Number(data.get('pan')), tilt_deg: Number(data.get('tilt')) })
      }}>{(['pan', 'tilt'] as const).map(axis => <label key={axis}>{axis} (°)<input name={axis} type="number" step="any" required defaultValue={gimbal[`${axis}_deg`]} /></label>)}<button>{text(locale, 'Set gimbal', '设置云台')}</button></form>}
    </fieldset>
    <p className={styles.note}>{text(locale, 'Query only reads sensor settings; it does not start motion.', '查询仅用于读取传感器设置，不触发运动。')}</p>
    {!settings && <p className={styles.note}>{text(locale, 'Query supported settings before editing.', '请先查询支持的设置，再进行编辑。')}</p>}
    <p role="status" className={styles.note}>{message}</p>
    {settings && <details><summary>{text(locale, 'Actual confirmed settings', '实际确认设置')}</summary><pre className={styles.confirmed}>{JSON.stringify(settings, null, 2)}</pre></details>}
  </section>
}

function OpenInspector({ locale }: { locale: Locale }) {
  const [{ sensors }, updateControls] = useReducer(measurementControlReducer, { sensors: [], controlRevision: 0 })
  const controlRevision = useRef(0)
  const [selected, setSelected] = useState('')
  const [preview, setPreview] = useState<MeasurementPreview | null>(null)
  const [online, setOnline] = useState(false)
  const [error, setError] = useState('')
  useEffect(() => {
    let expiry: ReturnType<typeof setTimeout> | undefined
    const stop = pollMeasurements(async signal => {
      const startedAtRevision = controlRevision.current
      const result = await listMeasurements(signal)
      if (signal.aborted || startedAtRevision !== controlRevision.current) return
      updateControls({ type: 'list', sensors: result.sensors, startedAtRevision })
      setOnline(true)
      setError('')
      const current = result.sensors.find(sensor => sensor.sensor_id === selected)
      if (!current || !hasFreshMeasurement(current)) { clearTimeout(expiry); setPreview(null); return }
      const next = await getMeasurementPreview(current.sensor_id, signal)
      if (signal.aborted || startedAtRevision !== controlRevision.current) return
      clearTimeout(expiry)
      if (!hasFreshMeasurement(next.measurement) || bindingKey(current) !== bindingKey(next.measurement)) { setPreview(null); return }
      setPreview(next)
      expiry = setTimeout(() => setPreview(null), Math.max(0,
        (next.measurement.stale_timeout_s - next.measurement.receipt_age_s!) * 1000))
    }, failure => {
      clearTimeout(expiry)
      setPreview(null)
      setOnline(false)
      setError(failure instanceof Error ? failure.message : String(failure))
    })
    return () => { stop(); clearTimeout(expiry) }
  }, [selected])
  const sensor = sensors.find(item => item.sensor_id === selected)
  return <div className={styles.body}>
    <p className={styles.note}>{text(locale, 'Display preview only; full arrays remain in raw NPZ. Receipt freshness does not prove clock alignment or inspection validity.', '仅为显示预览；完整数组保留在原始 NPZ 中。接收新鲜度不证明时钟对齐或巡检有效性。')}</p>
    <div className={styles.toolbar}><label>{text(locale, 'Sensor', '传感器')}<select value={selected} onChange={e => { setPreview(null); setSelected(e.target.value) }}>
      <option value="">{text(locale, 'Select a sensor', '选择传感器')}</option>
      {sensors.map(item => <option key={item.sensor_id} value={item.sensor_id}>{item.sensor_id} · {item.modality ?? '—'} · {item.status}</option>)}
    </select></label>{sensor && <span>{online ? sensor.status : text(locale, 'Offline', '离线')}{' · '}{text(locale, 'Receipt age', '接收年龄')}: {sensor.receipt_age_s?.toFixed(2) ?? '—'} s</span>}</div>
    {error && <p role="status" className={styles.error}>{text(locale, 'Measurement unavailable', '测量不可用')}: {error}</p>}
    {online && sensors.length === 0 && <p className={styles.note}>{text(locale, 'No declared measurement sensors', '未声明测量传感器')}</p>}
    {preview ? <>
      {preview.display.kind === 'acoustic' ? <Waveform display={preview.display} locale={locale} /> : <ImagePreview display={preview.display} locale={locale} />}
      <a href={preview.measurement.raw_url}>{text(locale, 'Download full raw NPZ (not display data)', '下载完整原始 NPZ（非显示数据）')}</a>
    </> : sensor && <p className={styles.note}>{text(locale, 'No fresh display. Stale, disabled or unavailable measurements are cleared.', '暂无新鲜显示；过期、禁用或不可用时清空预览。')}</p>}
    {sensor && <SensorControls key={bindingKey(sensor)} status={sensor} online={online} locale={locale} onConfirmed={ack => {
      controlRevision.current++
      updateControls({ type: 'ack', ack })
      if (ack.settings.enabled === false) setPreview(null)
    }} />}
  </div>
}

export default function MeasurementInspector({ locale }: { locale: Locale }) {
  const [open, setOpen] = useState(false)
  return <details className={styles.panel} onToggle={e => setOpen(e.currentTarget.open)}>
    <summary>{text(locale, 'Measurement inspector · thermal / acoustic / semantic', '测量检查器 · 热成像 / 声学 / 语义真值')}</summary>
    {open && <OpenInspector locale={locale} />}
  </details>
}
