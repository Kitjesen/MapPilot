import { lazy, Suspense, useCallback, useEffect, useRef, useState, type CSSProperties, type ReactNode } from 'react'
import { Check, Plus, Search, X } from 'lucide-react'
import { SensorFrame } from './sensors/SensorFrame.tsx'
import { ImuPanel } from './ImuMonitor.tsx'
import { ImuMonitorContext } from '../hooks/useImuMonitor.ts'
import { SensorMonitorContext } from '../hooks/useSensorMonitor.ts'
import { sensorCards, type SensorId, type SensorView } from '../services/sensors.ts'
import type { SSEState } from '../types'
import type { Locale } from '../i18n'
import { NumericCard } from './sensors/NumericCard.tsx'
import { CameraCard } from './sensors/CameraCard.tsx'
import { LidarCard } from './sensors/LidarCard.tsx'
import { SensorMark } from './sensors/SensorMark.tsx'
import styles from './SensorMonitor.module.css'

const Measurements = lazy(() => import('./MeasurementInspector.tsx'))
const Localization = lazy(() => import('./LocalizationCard.tsx').then(module => ({ default: module.LocalizationCard })))
interface CardState { id: SensorId; collapsed: boolean; expanded: boolean }

function SensorPicker({ cards, anchor, opener, onOpen, onClose }: { cards: CardState[]; anchor: DOMRect | null; opener: HTMLElement | null; onOpen: (id: SensorId) => void; onClose: () => void }) {
  const [query, setQuery] = useState('')
  const menu = useRef<HTMLDivElement>(null)
  useEffect(() => {
    const previous = opener ?? document.activeElement as HTMLElement | null
    let restoreFocus = true
    menu.current?.querySelector('input')?.focus()
    const outside = (event: PointerEvent) => { if (!menu.current?.contains(event.target as Node)) { restoreFocus = false; onClose() } }
    document.addEventListener('pointerdown', outside)
    return () => { document.removeEventListener('pointerdown', outside); if (restoreFocus && previous?.isConnected) previous.focus() }
  }, [onClose, opener])
  const matches = sensorCards.filter(item => `${item.id} ${item.name} ${item.detail}`.toLowerCase().includes(query.trim().toLowerCase()))
  const position: CSSProperties = anchor ? {
    left: `max(12px, min(${anchor.left}px, calc(100vw - 348px)))`,
    ...(anchor.top > window.innerHeight / 2
      ? { bottom: window.innerHeight - anchor.top + 8, maxHeight: `min(610px, ${anchor.top - 24}px)` }
      : { top: anchor.bottom + 8, maxHeight: `min(610px, calc(100dvh - ${anchor.bottom + 24}px))` }),
  } : { right: 20, top: 76 }
  return <div ref={menu} className={styles.picker} style={position} role="dialog" aria-label="添加传感器组件" onKeyDown={event => {
    if (event.key === 'Escape') { event.stopPropagation(); onClose() }
    if (event.key === 'ArrowDown' || event.key === 'ArrowUp') {
      event.preventDefault()
      const buttons = [...event.currentTarget.querySelectorAll<HTMLButtonElement>('[data-sensor-choice]')]
      const index = buttons.indexOf(document.activeElement as HTMLButtonElement)
      const next = index < 0 ? event.key === 'ArrowDown' ? 0 : buttons.length - 1
        : (index + (event.key === 'ArrowDown' ? 1 : -1) + buttons.length) % buttons.length
      buttons[next]?.focus()
    }
  }}>
    <header><strong>添加组件</strong><span>独立查看 · 同时打开</span><button type="button" onClick={onClose} aria-label="关闭组件选择器"><X size={15} /></button></header>
    <label className={styles.search}><Search size={15} /><input aria-label="搜索传感器" placeholder="搜索传感器…" value={query} onChange={event => setQuery(event.target.value)} /></label>
    <div className={styles.choices}>{['motion','perception'].map(group => {
      const entries = matches.filter(item => ['imu','joints','motors','voltages','gnss'].includes(item.id) === (group === 'motion'))
      return entries.length > 0 && <section key={group}><h4>{group === 'motion' ? '运动与状态' : '视觉与感知'}</h4>{entries.map(item => {
        const opened = cards.some(card => card.id === item.id)
        return <button type="button" key={item.id} data-sensor-choice aria-label={`${opened ? '查看' : '添加'}${item.name}`} onClick={() => onOpen(item.id)}>
          <SensorMark sensor={item.id} size={23} style={{ color: item.color }} /><strong>{item.name}</strong><span>{opened ? '已打开' : item.id === 'imu' ? '曲线与记录' : item.id === 'camera' ? '实时画面' : item.id === 'lidar' ? '三维扫描' : item.id === 'thermal' ? '温度图' : item.id === 'acoustic' ? '声压波形' : item.id === 'semantic' ? '类别与实例' : '数据与趋势'}</span>{opened ? <Check size={14} /> : <Plus size={13} />}
        </button>
      })}</section>
    })}{!matches.length && <p className={styles.emptySearch}>没有找到这个组件</p>}</div>
    <footer>打开后查看数据来源与连接状态</footer>
  </div>
}

export function SensorMonitorProvider({ children, onAnalyze, sseState, localNowS, locale = 'zh' }: { children: ReactNode; onAnalyze?: (id: string, start: number, end: number) => void; sseState?: SSEState; localNowS?: number; locale?: Locale }) {
  const [cards, setCards] = useState<CardState[]>([])
  const [catalog, setCatalog] = useState<{ anchor: DOMRect | null; opener: HTMLElement | null } | null>(null)
  const [focus, setFocus] = useState<{ id: SensorId } | null>(null)
  const open = useCallback((view: SensorView, anchor?: HTMLElement) => {
    if (view === 'catalog') { setCatalog({ anchor: anchor?.getBoundingClientRect() ?? null, opener: anchor ?? null }); return }
    setCatalog(null)
    setCards(previous => previous.some(card => card.id === view) ? previous.map(card => card.id === view ? { ...card, collapsed: false } : card) : [...previous, { id: view, collapsed: false, expanded: false }])
    setFocus({ id: view })
  }, [])
  const closeCatalog = useCallback(() => setCatalog(null), [])
  const showImu = useCallback(() => open('imu'), [open])
  useEffect(() => {
    if (!focus) return
    const card = document.getElementById(`sensor-card-${focus.id}`)
    card?.scrollIntoView({ block: 'nearest', inline: 'nearest' }); card?.focus({ preventScroll: true })
  }, [focus])
  const close = (id: SensorId) => setCards(previous => previous.filter(card => card.id !== id))
  const toggle = (id: SensorId, field: 'collapsed' | 'expanded') => setCards(previous => previous.map(card => {
    if (card.id !== id) return card
    return field === 'expanded' ? { ...card, expanded: !card.expanded, collapsed: false } : { ...card, collapsed: !card.collapsed }
  }))
  return <SensorMonitorContext.Provider value={open}><ImuMonitorContext.Provider value={showImu}>
    {children}
    {cards.length > 0 && <section className={styles.workspace} data-single={cards.length === 1 && !cards[0].expanded} aria-label="传感器工作区">
      <header className={styles.workspaceHeader}><span>观测 <b>{cards.length}</b></span><button type="button" onClick={event => open('catalog', event.currentTarget)}><Plus size={14} />添加组件</button></header>
      <div className={styles.cardGrid}>{cards.map(card => <SensorFrame key={card.id} id={card.id} collapsed={card.collapsed} expanded={card.expanded}
        onCollapse={() => toggle(card.id, 'collapsed')} onExpand={() => toggle(card.id, 'expanded')} onClose={() => close(card.id)}>
            <Suspense fallback={<p className={styles.loading} role="status">正在连接组件…</p>}>
              {card.id === 'imu' && <ImuPanel onAnalyze={onAnalyze} />}
              {(card.id === 'joints' || card.id === 'motors' || card.id === 'voltages') && <NumericCard sensor={card.id} />}
              {card.id === 'camera' && <CameraCard />}
              {card.id === 'lidar' && <LidarCard />}
              {card.id === 'gnss' && (sseState && localNowS !== undefined ? <Localization sseState={sseState} localNowS={localNowS} locale={locale} initialTab="gnss" embedded /> : <p className={styles.note}>当前入口未提供定位状态流。</p>)}
              {(card.id === 'thermal' || card.id === 'acoustic' || card.id === 'semantic') && <Measurements locale={locale} embedded modality={card.id} />}
            </Suspense>
      </SensorFrame>)}</div>
    </section>}
    {catalog && <SensorPicker cards={cards} anchor={catalog.anchor} opener={catalog.opener} onOpen={open} onClose={closeCatalog} />}
  </ImuMonitorContext.Provider></SensorMonitorContext.Provider>
}
