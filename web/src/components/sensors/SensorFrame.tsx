import { useRef, useState, type CSSProperties, type PointerEvent, type ReactNode } from 'react'
import { ChevronDown, Maximize2, Minimize2, X } from 'lucide-react'
import { loadLayouts, saveLayouts } from '../floatingWidgetLayout.ts'
import { sensorCards, type SensorId } from '../../services/sensors.ts'
import { SensorMark } from './SensorMark.tsx'
import styles from '../SensorMonitor.module.css'

const heights: Record<SensorId, number> = { imu: 640, joints: 560, motors: 560, voltages: 540, camera: 390, lidar: 430, gnss: 380, thermal: 480, acoustic: 330, semantic: 450 }
type Size = { width?: number; height: number }
type Edge = 'right' | 'bottom' | 'corner'

export function SensorFrame({ id, collapsed, expanded, onCollapse, onExpand, onClose, children }: {
  id: SensorId; collapsed: boolean; expanded: boolean; onCollapse: () => void; onExpand: () => void; onClose: () => void; children: ReactNode
}) {
  const minHeight = id === 'imu' ? 240 : 180
  const [size, setSize] = useState<Size>(() => {
    const saved = loadLayouts()?.[`sensor:${id}`]
    return saved && Number.isFinite(saved.w) && Number.isFinite(saved.h)
      ? { width: Math.max(220, saved.w), height: Math.max(minHeight, Math.min(1100, saved.h)) } : { height: heights[id] }
  })
  const frame = useRef<HTMLElement>(null)
  const drag = useRef<{ x: number; y: number; width: number; height: number; maxWidth: number; maxHeight: number; edge: Edge; next: Size } | null>(null)
  const item = sensorCards.find(item => item.id === id)!
  function commit(next: Size) {
    setSize(next)
    if (expanded) onExpand()
    const layouts = loadLayouts()
    if (next.width === undefined) { delete layouts[`sensor:${id}`]; saveLayouts(layouts) }
    else saveLayouts({ ...layouts, [`sensor:${id}`]: { x: 0, y: 0, w: next.width, h: next.height, z: 0 } })
  }
  function start(event: PointerEvent<HTMLDivElement>, edge: Edge) {
    if (event.button !== 0 || !frame.current || collapsed) return
    event.preventDefault(); event.stopPropagation()
    const box = frame.current.getBoundingClientRect(), bounds = frame.current.parentElement!.getBoundingClientRect()
    drag.current = { x: event.clientX, y: event.clientY, width: box.width, height: box.height, maxWidth: Math.max(220, bounds.right - box.left - 3), maxHeight: Math.max(240, window.innerHeight - 92), edge, next: { width: box.width, height: box.height } }
    frame.current.dataset.resizing = 'true'
    event.currentTarget.setPointerCapture(event.pointerId)
  }
  function move(event: PointerEvent<HTMLDivElement>) {
    const current = drag.current
    if (!current || !frame.current) return
    const width = current.edge === 'bottom' ? current.width : Math.max(220, Math.min(current.maxWidth, current.width + event.clientX - current.x))
    const height = current.edge === 'right' ? current.height : Math.max(minHeight, Math.min(current.maxHeight, current.height + event.clientY - current.y))
    current.next = { width, height }
    frame.current.style.setProperty('--card-width', `${width}px`)
    frame.current.style.setProperty('--card-height', `${height}px`)
  }
  function finish(event: PointerEvent<HTMLDivElement>, cancel = false) {
    const current = drag.current
    if (!current || !frame.current) return
    drag.current = null
    if (cancel) {
      frame.current.style.setProperty('--card-width', size.width === undefined ? 'var(--card-default-width)' : `${size.width}px`)
      frame.current.style.setProperty('--card-height', `${size.height}px`)
    } else commit(current.next)
    delete frame.current.dataset.resizing
    if (event.currentTarget.hasPointerCapture(event.pointerId)) event.currentTarget.releasePointerCapture(event.pointerId)
  }
  return <aside ref={frame} id={`sensor-card-${id}`} className={styles.card} data-sensor={id} data-expanded={expanded} data-collapsed={collapsed}
    style={{ '--sensor-color': item.color, '--card-width': size.width === undefined ? 'var(--card-default-width)' : `${size.width}px`, '--card-height': `${size.height}px` } as CSSProperties}
    aria-label={id === 'imu' ? 'IMU 实时监测' : `${item.name}监测`} tabIndex={-1}
    onKeyDown={event => { if (event.key === 'Escape' && event.target === event.currentTarget) { event.stopPropagation(); onClose() } }}>
    <header className={styles.cardHeader}><span className={styles.cardIcon}><SensorMark sensor={id} size={32} /></span><div><h3>{item.name}</h3><span className={styles.cardSubtitle}>{id === 'imu' ? '惯性测量' : item.detail.split(' · ')[0]}</span></div><span className={styles.compactBadge}>紧凑</span><div className={styles.cardActions}>
      <button type="button" aria-label={`${collapsed ? '展开' : '收起'}${item.name}`} aria-expanded={!collapsed} onClick={onCollapse}><ChevronDown size={15} className={collapsed ? styles.folded : undefined} /></button>
      <button type="button" aria-label={`${expanded ? '还原' : '放大'}${item.name}`} aria-pressed={expanded} onClick={onExpand}>{expanded ? <Minimize2 size={14} /> : <Maximize2 size={14} />}</button>
      <button type="button" aria-label={id === 'imu' ? '关闭 IMU 监测' : `关闭${item.name}监测`} onClick={onClose}><X size={15} /></button>
    </div></header>
    <div className={styles.cardBody} hidden={collapsed}>{children}</div>
    {!collapsed && !expanded && (['right','bottom','corner'] as Edge[]).map(edge => <div key={edge} className={styles.resizeHandle} data-edge={edge} role="separator" tabIndex={0}
      aria-label={`调整${item.name}${edge === 'right' ? '宽度' : edge === 'bottom' ? '高度' : '大小'}`} aria-orientation={edge === 'right' ? 'vertical' : 'horizontal'}
      title="拖动调整大小 · 双击恢复默认" onPointerDown={event => start(event, edge)} onPointerMove={move} onPointerUp={event => finish(event)} onPointerCancel={event => finish(event, true)}
      onDoubleClick={() => commit({ height: heights[id] })} onKeyDown={event => {
        if (event.key === 'Home') { event.preventDefault(); commit({ height: heights[id] }); return }
        if (!['ArrowLeft','ArrowRight','ArrowUp','ArrowDown'].includes(event.key) || !frame.current) return
        event.preventDefault()
        const box = frame.current.getBoundingClientRect(), step = event.shiftKey ? 40 : 16
        const maxWidth = frame.current.parentElement!.getBoundingClientRect().right - box.left - 3
        commit({ width: Math.max(220, Math.min(maxWidth, box.width + (edge !== 'bottom' ? event.key === 'ArrowRight' ? step : event.key === 'ArrowLeft' ? -step : 0 : 0))), height: Math.max(minHeight, Math.min(Math.max(240, window.innerHeight - 92), box.height + (edge !== 'right' ? event.key === 'ArrowDown' ? step : event.key === 'ArrowUp' ? -step : 0 : 0))) })
      }} />)}
  </aside>
}
