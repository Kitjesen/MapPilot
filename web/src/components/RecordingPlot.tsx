import { useId } from 'react'
import { text, type Locale } from '../i18n'
import type { RecordingNumericSample } from '../types/recordingSeries'
import { formatSeriesAxis, formatSeriesValue, sampleAt, seriesSegments } from '../services/recordingSeries'
import styles from './RecordingPlot.module.css'

export interface RecordingPlotTrace { label: string; samples: RecordingNumericSample[]; secondary?: boolean; localTimeOffset?: number }

const PLOT_LEFT = 84
const PLOT_RIGHT = 708

export function RecordingPlot({ traces, start, end, cursor, unit, locale, onSeek, unavailable = false }: {
  traces: RecordingPlotTrace[]
  start: number
  end: number
  cursor?: number
  unit?: string
  locale: Locale
  onSeek?: (time: number) => void
  unavailable?: boolean
}) {
  const id = useId()
  const segments = traces.map(trace => seriesSegments(trace.samples, start, end))
  const values = segments.flat(2).map(sample => sample.value!)
  const minimum = values.length ? Math.min(...values) : 0
  const maximum = values.length ? Math.max(...values) : 1
  const padding = Math.max((maximum - minimum) * .12, Math.abs(maximum) * .015, .001)
  const bottom = minimum - padding
  const top = maximum + padding
  const x = (t: number) => PLOT_LEFT + ((t - start) / Math.max(.001, end - start)) * (PLOT_RIGHT - PLOT_LEFT)
  const y = (v: number) => 20 + (1 - (v - bottom) / (top - bottom)) * 174
  return <figure className={styles.plot} data-unavailable={unavailable || undefined}>
    <div className={styles.legend}>{traces.map((trace, index) => {
      const current = cursor === undefined ? trace.samples.at(-1) : sampleAt(trace.samples, cursor)
      return <span key={index} className={trace.secondary ? styles.secondary : styles.primary}><i /><span title={trace.label}>{trace.label}</span>{cursor !== undefined && trace.localTimeOffset !== undefined && <small>@ {(cursor - trace.localTimeOffset).toFixed(2)} s</small>}<strong>{unavailable ? '—' : formatSeriesValue(current?.value)}{unit && <small> {unit}</small>}</strong></span>
    })}</div>
    <svg viewBox="0 0 730 228" role="img" aria-label={text(locale, 'Numeric field over time', '数值字段随时间变化')} onClick={onSeek ? event => {
      const bounds = event.currentTarget.getBoundingClientRect()
      const time = start + Math.max(0, Math.min(1, ((event.clientX - bounds.left) / bounds.width * 730 - PLOT_LEFT) / (PLOT_RIGHT - PLOT_LEFT))) * (end - start)
      onSeek(time)
    } : undefined} className={onSeek ? styles.interactive : undefined}>
      <defs><clipPath id={id}><rect x={PLOT_LEFT} y="12" width={PLOT_RIGHT - PLOT_LEFT} height="190" /></clipPath></defs>
      {[0, .5, 1].map(fraction => <g key={fraction}>
        <line x1={PLOT_LEFT} x2={PLOT_RIGHT} y1={20 + fraction * 174} y2={20 + fraction * 174} className={styles.grid} />
        <text x={PLOT_LEFT - 9} y={24 + fraction * 174} textAnchor="end" textLength={formatSeriesAxis(top - fraction * (top - bottom)).length > 6 ? 72 : undefined} lengthAdjust="spacingAndGlyphs">{formatSeriesAxis(top - fraction * (top - bottom))}</text>
        <text x={PLOT_LEFT + fraction * (PLOT_RIGHT - PLOT_LEFT)} y="218" textAnchor={fraction === 0 ? 'start' : fraction === 1 ? 'end' : 'middle'}>{(start + fraction * (end - start)).toFixed(1)} s</text>
      </g>)}
      <g clipPath={`url(#${id})`}>{traces.map((trace, index) => <g key={index} className={trace.secondary ? styles.secondaryPath : styles.primaryPath}>{segments[index].map((segment, part) => segment.length === 1
        ? <circle key={part} cx={x(segment[0].t)} cy={y(segment[0].value!)} r="2.5" />
        : <polyline key={part} points={segment.map(sample => `${x(sample.t)},${y(sample.value!)}`).join(' ')} />)}</g>)}</g>
      {cursor !== undefined && cursor >= start && cursor <= end && <line x1={x(cursor)} x2={x(cursor)} y1="12" y2="202" className={styles.cursor} />}
      {!values.length && <text x="383" y="106" textAnchor="middle">{text(locale, 'No numeric samples in this window', '此时间窗内没有数值样本')}</text>}
    </svg>
    {unavailable && <figcaption>{text(locale, 'Updates stopped · retained history', '更新已停止 · 保留历史曲线')}</figcaption>}
  </figure>
}
