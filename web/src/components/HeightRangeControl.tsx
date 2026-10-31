import { useRef, type KeyboardEvent, type PointerEvent } from 'react'
import styles from './HeightRangeControl.module.css'

interface Props {
  min: number
  max: number
  domainMin: number
  domainMax: number
  robotZ: number | null
  goalZ?: number | null
  onChange(min: number, max: number): void
}
type Part = 'min' | 'max' | 'range'
const STEP = 0.1
export function HeightRangeControl({ min, max, domainMin, domainMax, robotZ, goalZ, onChange }: Props) {
  const track = useRef<HTMLDivElement>(null)
  const drag = useRef<{ part: Part; y: number; min: number; max: number; scale: number } | null>(null)
  const low = Math.min(domainMin, min)
  const high = Math.max(domainMax, max, low + STEP)
  const percent = (z: number) => (high - z) / (high - low) * 100
  const apply = (part: Part, delta: number, a = min, b = max) => {
    const change = Math.round(delta / STEP) * STEP
    if (part === 'min') onChange(Math.max(low, Math.min(b - STEP, a + change)), b)
    else if (part === 'max') onChange(a, Math.min(high, Math.max(a + STEP, b + change)))
    else { const shift = Math.max(low - a, Math.min(high - b, change)); onChange(a + shift, b + shift) }
  }
  const down = (event: PointerEvent<HTMLButtonElement>, part: Part) => {
    if (event.button !== 0) return
    event.preventDefault()
    event.currentTarget.setPointerCapture(event.pointerId)
    drag.current = { part, y: event.clientY, min, max, scale: (high - low) / (track.current?.clientHeight || 160) }
  }
  const move = (event: PointerEvent<HTMLButtonElement>) => {
    if (!drag.current) return
    const d = drag.current
    apply(d.part, (d.y - event.clientY) * d.scale, d.min, d.max)
  }
  const key = (event: KeyboardEvent, part: Part) => {
    if (['ArrowUp', 'ArrowRight', 'ArrowDown', 'ArrowLeft', 'Home', 'End'].includes(event.key)) {
      event.preventDefault()
      const delta = event.key === 'Home' ? low - (part === 'max' ? max : min)
        : event.key === 'End' ? high - (part === 'min' ? min : max)
          : (event.key === 'ArrowUp' || event.key === 'ArrowRight' ? 1 : -1) * STEP
      apply(part, delta)
    }
  }
  const slider = (part: Part, label: string, position: number, height?: number) => <button type="button"
    className={part === 'range' ? styles.band : styles.handle} role="slider" aria-label={label}
    aria-orientation="vertical" aria-valuemin={part === 'max' ? min + STEP : low}
    aria-valuemax={part === 'min' ? max - STEP : part === 'range' ? high - (max - min) : high}
    aria-valuenow={part === 'max' ? max : min}
    aria-valuetext={part === 'range' ? `${min.toFixed(2)} 至 ${max.toFixed(2)} 米` : `${(part === 'min' ? min : max).toFixed(2)} 米`}
    style={{ top: `${position}%`, height: height === undefined ? undefined : `${height}%` }}
    onPointerDown={event => down(event, part)} onPointerMove={move} onPointerUp={() => { drag.current = null }}
    onPointerCancel={() => { drag.current = null }} onLostPointerCapture={() => { drag.current = null }}
    onKeyDown={event => key(event, part)}><span aria-hidden="true">{part === 'range' ? '⋮' : '━'}</span></button>

  return <div className={styles.control} role="group" aria-label="固定高度范围">
    <div ref={track} className={styles.track}>
      <div className={styles.selection} style={{ top: `${percent(max)}%`, height: `${(max - min) / (high - low) * 100}%` }} />
      {slider('range', '整体移动高度范围', percent((min + max) / 2))}
      {slider('max', '高度上限', percent(max))}
      {slider('min', '高度下限', percent(min))}
      {robotZ != null && robotZ >= low && robotZ <= high && <span className={styles.marker} style={{ top: `${percent(robotZ)}%` }}>机器人</span>}
      {goalZ != null && goalZ >= low && goalZ <= high && <span className={`${styles.marker} ${styles.goal}`} style={{ top: `${percent(goalZ)}%` }}>目标</span>}
    </div>
    <div className={styles.fields}>
      {(['max', 'min'] as const).map(part => <label key={part}>{part === 'max' ? '上限' : '下限'}
        <span><input key={part === 'max' ? max : min} type="number" step="0.1" aria-label={part === 'max' ? '显示高度上限' : '显示高度下限'}
          defaultValue={(part === 'max' ? max : min).toFixed(2)} onBlur={event => {
            const n = event.currentTarget.valueAsNumber
            if (Number.isFinite(n) && (part === 'max' ? n - min >= STEP : max - n >= STEP)) {
              onChange(part === 'min' ? n : min, part === 'max' ? n : max)
            } else event.currentTarget.value = (part === 'max' ? max : min).toFixed(2)
          }} /> m</span>
      </label>)}
      <button type="button" disabled={robotZ == null} onClick={() => {
        if (robotZ != null) { const half = (max - min) / 2; onChange(robotZ - half, robotZ + half) }
      }}>回到机器人高度</button>
      <small>拖两端调整 · 拖中间平移</small>
    </div>
  </div>
}
