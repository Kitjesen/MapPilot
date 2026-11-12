import { createPortal } from 'react-dom'
import { useEffect, useMemo, useRef, useState, type PointerEvent, type ReactNode } from 'react'
import { Check, ChevronDown, Layers2, Minus, Plus, Search, SlidersHorizontal, X } from 'lucide-react'
import { traceAt, traceEnd, traceGroups, tracePath, traceSpan, traceTime, type TraceChannel, type TraceSample } from '../../services/sensorChart.ts'
import styles from './TraceChart.module.css'

function ChannelPicker({ channels, selected, onChange }: { channels: TraceChannel[]; selected: string[]; onChange: (ids: string[]) => void }) {
  const [query, setQuery] = useState('')
  const [position, setPosition] = useState<{ left: number; top?: number; bottom?: number; maxHeight: number } | null>(null)
  const trigger = useRef<HTMLButtonElement>(null)
  const panel = useRef<HTMLDivElement>(null)
  const close = () => { setPosition(null); trigger.current?.focus() }
  useEffect(() => {
    if (!position) return
    panel.current?.querySelector('input')?.focus()
    const outside = (event: globalThis.PointerEvent) => { if (!panel.current?.contains(event.target as Node) && !trigger.current?.contains(event.target as Node)) setPosition(null) }
    const resize = () => setPosition(null)
    document.addEventListener('pointerdown', outside); window.addEventListener('resize', resize)
    return () => { document.removeEventListener('pointerdown', outside); window.removeEventListener('resize', resize) }
  }, [position])
  return <div className={styles.channelMenu}>
    <button ref={trigger} type="button" aria-label="选择数据通道" aria-expanded={!!position} onClick={() => {
      if (position) { close(); return }
      const box = trigger.current!.getBoundingClientRect()
      setPosition({ left: Math.max(12, Math.min(box.left, window.innerWidth - 332)), ...(box.top > window.innerHeight / 2 ? { bottom: window.innerHeight - box.top + 6, maxHeight: Math.min(380, box.top - 18) } : { top: box.bottom + 6, maxHeight: Math.min(380, window.innerHeight - box.bottom - 18) }) })
    }}><SlidersHorizontal size={13} /><span>通道</span><b>{selected.length}</b><ChevronDown size={12} /></button>
    {position && createPortal(<div ref={panel} style={position} className={styles.channelPanel} role="dialog" aria-label="数据通道" onKeyDown={event => { if (event.key === 'Escape') { event.stopPropagation(); close() } }}>
      <label className={styles.search}><Search size={13} /><input aria-label="搜索通道" placeholder="搜索名称或编号" value={query} onChange={event => setQuery(event.target.value)} /></label>
      <div className={styles.selectActions}><button type="button" onClick={() => onChange(channels.map(item => item.id))}>全选</button><button type="button" onClick={() => onChange([])}>清空</button><button type="button" aria-label="关闭通道选择" onClick={close}><X size={13} /></button></div>
      <div className={styles.channelOptions}>{traceGroups(channels.filter(item => `${item.label} ${item.group}`.toLowerCase().includes(query.toLowerCase()))).map(([group, items]) => <section key={group}><header><span>{group}</span><button type="button" onClick={() => onChange([...new Set([...selected, ...items.map(item => item.id)])])}>选择这组</button></header><div>{items.map(item => <label key={item.id} title={`${item.label} · ${item.group}`}>
        <input type="checkbox" checked={selected.includes(item.id)} aria-label={`${item.label} ${item.group}`} onChange={event => onChange(event.target.checked ? [...selected, item.id] : selected.filter(id => id !== item.id))} />
        <i style={{ color: item.color }}><Check size={10} /></i><span>{item.label}</span>
      </label>)}</div></section>)}</div>
    </div>, document.body)}
  </div>
}

export function TraceChart({ rows, channels, selected, onSelect, seconds, onSeconds, end: requestedEnd, onSeek, defaultLayout = 'overlay', actions, live = true }: {
  rows: readonly TraceSample[]; channels: TraceChannel[]; selected: string[]; onSelect: (ids: string[]) => void;
  seconds: number; onSeconds: (seconds: number) => void; end: number | null; onSeek: (end: number | null) => void;
  defaultLayout?: 'stack' | 'overlay'; actions?: ReactNode; live?: boolean;
}) {
  const [layout, setLayout] = useState(defaultLayout)
  const [folded, setFolded] = useState<string[]>([])
  const [hover, setHover] = useState<number | null>(null)
  const [pinned, setPinned] = useState(false)
  const area = useRef<HTMLDivElement>(null)
  const drag = useRef<{ x: number; end: number; width: number; moved: boolean } | null>(null)
  const end = traceEnd(rows, requestedEnd), start = end - seconds
  const epochRow = rows.find(row => row.receivedAt !== undefined)
  const epoch = epochRow?.receivedAt === undefined ? undefined : epochRow.receivedAt - epochRow.t
  const chosen = useMemo(() => channels.filter(item => selected.includes(item.id)), [channels, selected])
  const groups = useMemo(() => traceGroups(chosen), [chosen])
  const sample = hover !== null ? traceAt(rows, hover) : traceAt(rows, end)
  const overview = pinned || requestedEnd !== null ? sample : traceAt(rows, end)
  const wheelState = useRef({ seconds, onSeconds })
  useEffect(() => { wheelState.current = { seconds, onSeconds } }, [seconds, onSeconds])
  useEffect(() => {
    const element = area.current
    if (!element) return
    const zoom = (event: WheelEvent) => {
      if (!event.ctrlKey && !event.metaKey) return
      event.preventDefault()
      const state = wheelState.current
      state.onSeconds(traceSpan(state.seconds * (event.deltaY > 0 ? 1.2 : .8)))
    }
    element.addEventListener('wheel', zoom, { passive: false })
    return () => element.removeEventListener('wheel', zoom)
  }, [])
  function pointerTime(event: PointerEvent<SVGSVGElement>) {
    const box = event.currentTarget.getBoundingClientRect()
    return start + Math.max(0, Math.min(1, (event.clientX - box.left) / box.width)) * seconds
  }
  function move(event: PointerEvent<SVGSVGElement>) {
    const active = drag.current
    if (active) {
      if (Math.abs(event.clientX - active.x) > 3) active.moved = true
      if (active.moved) { setHover(null); setPinned(false); onSeek(traceEnd(rows, active.end - (event.clientX - active.x) / active.width * seconds)) }
    } else if (!pinned) setHover(pointerTime(event))
  }
  function plot(items: TraceChannel[]) {
    let low = Infinity, high = -Infinity
    for (const row of rows) if (row.t >= start && row.t <= end) for (const item of items) {
      const value = row.values[item.id]
      if (typeof value === 'number' && Number.isFinite(value)) { low = Math.min(low, value); high = Math.max(high, value) }
    }
    const available = Number.isFinite(low)
    if (!available) { low = 0; high = 1 }
    const margin = Math.max(.001, (high - low) * .1, Math.abs(high) * .01)
    const cursor = hover === null ? null : sample?.t ?? hover
    return <div className={styles.plotRow} key={items.map(item => item.id).join('|')}>
      <div className={styles.yAxis}><span>{available ? high.toFixed(Math.abs(high) >= 100 ? 0 : 2) : '—'}</span><span>{available ? low.toFixed(Math.abs(low) >= 100 ? 0 : 2) : '—'}</span></div>
      <svg viewBox="0 0 320 96" preserveAspectRatio="none" role="img" tabIndex={0} aria-label={`${items.map(item => item.label).join(' / ')} ${items[0].group}时间曲线`} aria-description="左右键平移时间窗口，Home 返回最新，鼠标悬停查看采样，单击固定读数"
        onKeyDown={event => {
          if (event.key === 'ArrowLeft' || event.key === 'ArrowRight') { event.preventDefault(); onSeek(traceEnd(rows, end + (event.key === 'ArrowLeft' ? -1 : 1) * seconds / 10)) }
          if (event.key === 'Home') { event.preventDefault(); onSeek(null) }
        }}
        onPointerDown={event => { if (event.button !== 0) return; event.currentTarget.setPointerCapture(event.pointerId); drag.current = { x: event.clientX, end, width: event.currentTarget.getBoundingClientRect().width, moved: false } }}
        onPointerMove={move} onPointerUp={event => {
          if (drag.current && !drag.current.moved) { setHover(pointerTime(event)); setPinned(value => !value) }
          drag.current = null
          if (event.currentTarget.hasPointerCapture(event.pointerId)) event.currentTarget.releasePointerCapture(event.pointerId)
        }} onPointerCancel={() => { drag.current = null }} onPointerLeave={() => { if (!pinned && !drag.current) setHover(null) }}>
        {[0,80,160,240,320].map(x => <line key={`x${x}`} x1={x} x2={x} y1="10" y2="86" className={styles.grid} />)}
        {[10,48,86].map(y => <line key={`y${y}`} x1="0" x2="320" y1={y} y2={y} className={styles.grid} />)}
        {available && items.map(item => <path key={item.id} data-channel={item.id} style={{ stroke: item.color }} d={tracePath(rows, item.id, start, end, low - margin, high + margin)} />)}
        {cursor !== null && cursor >= start && cursor <= end && <line className={styles.cursor} x1={(cursor-start)/seconds*320} x2={(cursor-start)/seconds*320} y1="4" y2="92" />}
        {!available && <text x="160" y="52" textAnchor="middle">暂无采样</text>}
      </svg>
    </div>
  }
  function readings(items: TraceChannel[]) {
    return <div className={styles.legend}>{items.map(item => <span key={item.id}><i style={{ background: item.color }} /><span>{item.label}</span><b>{typeof sample?.values[item.id] === 'number' ? sample.values[item.id]!.toFixed(3) : '—'}</b></span>)}</div>
  }
  return <div ref={area} data-trace-chart className={styles.chart} data-layout={layout} data-pinned={pinned} data-review={requestedEnd !== null}>
    <div className={styles.controls}><ChannelPicker channels={channels} selected={selected} onChange={onSelect} />
      <div className={styles.mode} aria-label="曲线布局"><button type="button" aria-label="叠加曲线" aria-pressed={layout === 'overlay'} onClick={() => setLayout('overlay')}><Layers2 size={13} /><span>叠加</span></button><button type="button" aria-label="堆叠曲线" aria-pressed={layout === 'stack'} onClick={() => setLayout('stack')}><svg width="13" height="13" viewBox="0 0 16 16" fill="none" aria-hidden="true"><path d="M2 3H14M2 8H14M2 13H14" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" /></svg><span>堆叠</span></button></div>{actions}
    </div>
    <div className={styles.timeControls}><button type="button" aria-label="缩短时间窗口" disabled={seconds <= 5} onClick={() => onSeconds(traceSpan(seconds/2))}><Minus size={12} /></button><label><input type="range" aria-label="时间窗口长度" min="5" max="300" step="1" value={seconds} onChange={event => onSeconds(Number(event.target.value))} /><output>{seconds} 秒</output></label><button type="button" aria-label="延长时间窗口" disabled={seconds >= 300} onClick={() => onSeconds(traceSpan(seconds*2))}><Plus size={12} /></button>
      {requestedEnd !== null && <button className={styles.follow} type="button" onClick={() => onSeek(null)}>{live ? '回到实时' : '回到末尾'}</button>}
    </div>
    {!groups.length && <p className={styles.empty}>从“通道”选择要显示的数据</p>}
    <div className={styles.compactReadings} aria-label="紧凑通道读数">{chosen.map(item => <span key={item.id} title={`${item.group} ${item.label} · ${item.unit}`}><small style={{ color: item.color }}>{item.group === '角速度' ? '角速 ' : item.group === '线加速度' ? '加速 ' : ''}{item.label}</small><b>{typeof overview?.values[item.id] === 'number' ? overview.values[item.id]!.toFixed(2) : '—'}</b><em>{item.unit}</em></span>)}</div>
    <div className={styles.groups}>{groups.map(([group, items]) => <section className={styles.group} key={group} data-folded={folded.includes(group)}>
      <button className={styles.groupToggle} type="button" aria-expanded={!folded.includes(group)} aria-label={`${folded.includes(group) ? '展开' : '折叠'}${group}`} onClick={() => setFolded(previous => previous.includes(group) ? previous.filter(value => value !== group) : [...previous, group])}><ChevronDown size={12} /><strong>{items[0].group}</strong><span>{items[0].unit}</span><small>{items.length} 通道</small></button>
      <div className={styles.groupClip} aria-hidden={folded.includes(group)} inert={folded.includes(group)}><div className={styles.groupBody}>{layout === 'overlay' ? <>{readings(items)}{plot(items)}</> : items.map(item => <div key={item.id} className={styles.strip}>{readings([item])}{plot([item])}</div>)}
        <div className={styles.xAxis}>{[0,.25,.5,.75,1].map((fraction,index) => <span key={fraction} data-intermediate={index%2===1}>{traceTime(start + seconds*fraction, epoch)}</span>)}</div>
      </div></div>
    </section>)}</div>
    <div className={styles.inspect}><span>{hover !== null ? sample ? traceTime(sample.receivedAt === undefined ? sample.t : 0, sample.receivedAt, true) : '这个时刻没有采样' : epoch === undefined ? '接收后相对时间' : '接收时间'}{pinned ? ' · 已固定' : ''}</span>{hover !== null && <button type="button" onClick={() => { setHover(null); setPinned(false) }}>清除游标</button>}<span className={styles.gesture}>拖动回看 · Ctrl＋滚轮缩放</span></div>
    <p className={styles.compactHint}>{pinned || requestedEnd !== null ? `${pinned ? '固定读数' : live ? '回看时窗' : '记录回看'} · ${overview ? traceTime(overview.receivedAt === undefined ? overview.t : 0, overview.receivedAt) : '无采样'}` : '拉高卡片以查看曲线'}{(pinned || requestedEnd !== null) && <button type="button" onClick={() => { setPinned(false); setHover(null); onSeek(null) }}>返回</button>}</p>
  </div>
}
