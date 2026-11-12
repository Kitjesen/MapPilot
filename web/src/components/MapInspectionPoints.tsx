import { useCallback, useEffect, useState } from 'react'
import { MapPin, Plus, X } from 'lucide-react'
import type { LocationEntry } from '../types/index.ts'
import type { XYZ } from '../types/stairs.ts'
import * as api from '../services/api.ts'
import { findInspectionPoints, inspectionActions, routePointFromLocation } from '../services/inspectionPoints.ts'
import { navigationSupportHeightFromRunPlan } from '../services/navigationPreview.ts'
import { useMapSettings } from '../hooks/useMapSettings.ts'
import { ConfirmModal } from './Modal.tsx'
import styles from './MapInspectionPoints.module.css'

interface Props {
  mapName: string; epoch: number; picked: XYZ | null
  onMarkers(points: LocationEntry[]): void
  onFocus(point: XYZ): void
  onClose(): void
  onExitGuardChange?: (guard: ((proceed: () => void) => void) | null) => void
}

export function MapInspectionPoints({ mapName, epoch, picked, onMarkers, onFocus, onClose, onExitGuardChange }: Props) {
  const [points, setPoints] = useState<LocationEntry[]>([])
  const [query, setQuery] = useState('')
  const [name, setName] = useState('')
  const [region, setRegion] = useState('')
  const [position, setPosition] = useState<XYZ | null>(null)
  const [supportHeight, setSupportHeight] = useState<number | null>(null)
  const [fromSurface, setFromSurface] = useState(false)
  const [yaw, setYaw] = useState(0)
  const [dwell, setDwell] = useState(3)
  const [action, setAction] = useState('capture:overview')
  const [existing, setExisting] = useState(false)
  const [busy, setBusy] = useState(false)
  const [error, setError] = useState('')
  const [message, setMessage] = useState('')
  const [deleteOpen, setDeleteOpen] = useState(false)
  const [dirty, setDirty] = useState(false)
  const [pendingLeave, setPendingLeave] = useState<(() => void) | null>(null)
  const requestLeave = useCallback((proceed: () => void) => {
    if (busy) return
    if (dirty) setPendingLeave(() => proceed)
    else proceed()
  }, [busy, dirty])
  useEffect(() => { onExitGuardChange?.(requestLeave); return () => onExitGuardChange?.(null) }, [onExitGuardChange, requestLeave])
  useEffect(() => {
    if (!dirty && !busy) return
    const guard = (event: BeforeUnloadEvent) => { event.preventDefault(); event.returnValue = '' }
    window.addEventListener('beforeunload', guard)
    return () => window.removeEventListener('beforeunload', guard)
  }, [dirty, busy])
  const setEditing = useMapSettings()?.setEditing
  useEffect(() => { setEditing?.(true); return () => setEditing?.(false) }, [setEditing])
  useEffect(() => {
    let active = true
    void api.fetchLocations(mapName).then(result => {
      if (active) { setPoints(result.locations); onMarkers(result.locations) }
    }, cause => { if (active) setError(String(cause)) })
    return () => { active = false }
  }, [mapName, epoch, onMarkers])
  useEffect(() => {
    let active = true
    void api.fetchRuntimeDataflow().then(result => {
      if (active) setSupportHeight(navigationSupportHeightFromRunPlan(result.run_plan))
    }).catch(() => {})
    return () => { active = false }
  }, [])
  useEffect(() => {
    if (picked) {
      // A picked point is a surface height; convert only when saving the body target.
      setPosition(picked); setFromSurface(true); setMessage(''); setDirty(true)
    }
  }, [picked])
  const select = (point: LocationEntry) => {
    const settings = routePointFromLocation(point)
    setExisting(true); setName(point.name); setRegion(String(point.metadata?.region ?? ''))
    setPosition(point); setFromSurface(false); setYaw((point.yaw ?? 0) * 180 / Math.PI)
    setDwell(settings.dwell); setAction(settings.action); setError(''); setMessage(''); setDirty(false)
    onFocus(point)
  }
  const save = async () => {
    if (!position || !name.trim() || fromSurface && supportHeight == null) return
    if (!existing && points.some(point => point.name === name.trim())) { setError('名称已存在，请从列表选择该点进行编辑。'); return }
    setBusy(true); setError(''); setMessage('')
    try {
      const previous = points.find(point => point.name === name.trim())
      const result = await api.saveLocation({ name: name.trim(), x: position.x, y: position.y,
        z: position.z + (fromSurface ? supportHeight! : 0), yaw: yaw * Math.PI / 180,
        map_id: mapName, expected_map_content_epoch: epoch, source: 'map_editor',
        tags: [...new Set([...(previous?.tags ?? []), 'inspection'])],
        metadata: { ...previous?.metadata, region: region.trim(), inspection: { dwell, action } } })
      if (!result.ok || !result.location) throw new Error(result.error || result.message || '巡检点保存失败')
      const next = [...points.filter(point => point.name !== result.location!.name), result.location]
      setPoints(next); onMarkers(next); select(result.location); setMessage('已保存。可在巡检页面加入路线；尚未启动任务。')
    } catch (cause) { setError(cause instanceof Error ? cause.message : String(cause)) }
    finally { setBusy(false) }
  }
  const remove = async () => {
    setDeleteOpen(false); setBusy(true); setError('')
    try {
      const result = await api.deleteLocation(name)
      if (!result.ok) throw new Error(result.error || result.message || '删除失败')
      const next = points.filter(point => point.name !== name)
      setPoints(next); onMarkers(next); setExisting(false); setName(''); setPosition(null)
      setDirty(false)
      setMessage('已删除巡检点。已经保存的巡检路线保持原有点位快照。')
    } catch (cause) { setError(cause instanceof Error ? cause.message : String(cause)) }
    finally { setBusy(false) }
  }
  return <section className={styles.panel} aria-label="地图巡检点">
    <header><strong><MapPin size={16} />巡检点</strong><button type="button" disabled={busy} aria-label="关闭巡检点编辑" onClick={() => requestLeave(onClose)}><X size={16} /></button></header>
    <input aria-label="按点名或区域查找" placeholder="搜索点名或区域…" value={query} onChange={event => setQuery(event.target.value)} />
    <div className={styles.list}>{findInspectionPoints(points, query).map(point => <button key={point.name} type="button"
      aria-pressed={existing && name === point.name} disabled={busy} onClick={() => requestLeave(() => select(point))}>
      <span>{point.name}</span><small>{String(point.metadata?.region ?? '未分区')}{point.map_content_epoch !== epoch ? ' · 地图版本已变化' : ''}</small>
    </button>)}</div>
    <button type="button" className={styles.newPoint} disabled={busy} onClick={() => requestLeave(() => {
      setExisting(false); setName(''); setRegion(''); setPosition(null); setFromSurface(false); setError(''); setMessage(''); setDirty(false)
    })}><Plus size={15} />新建巡检点</button>
    <p>点击地图上的实测点选择位置，也可输入机身坐标。拖动仍用于旋转地图。</p>
    <form onChange={() => setDirty(true)} onSubmit={event => { event.preventDefault(); void save() }}>
      <label>点位名称<input required maxLength={128} value={name} disabled={existing} placeholder="如 1 号配电柜"
        onChange={event => setName(event.target.value)} /></label>
      <label>所属区域<input maxLength={128} list="inspection-region-names" value={region} placeholder="如东区机房"
        onChange={event => setRegion(event.target.value)} /></label>
      <datalist id="inspection-region-names">{[...new Set(points.map(point => String(point.metadata?.region ?? '')).filter(Boolean))].map(value => <option key={value} value={value} />)}</datalist>
      <div className={styles.coordinates}>{(['x', 'y', 'z'] as const).map(axis => <label key={axis}>
        {axis.toUpperCase()} {axis === 'z' && fromSurface ? '表面' : ''} m<input type="number" required step="0.01" value={position?.[axis] ?? ''}
          onChange={event => setPosition({ ...(position ?? { x: 0, y: 0, z: 0 }), [axis]: event.currentTarget.valueAsNumber })} />
      </label>)}</div>
      {fromSurface && <label>机身离地高度 m<input type="number" required min="0" step="0.01" value={supportHeight ?? ''}
        placeholder="读取机器人配置或填写实测高度" onChange={event => setSupportHeight(event.currentTarget.valueAsNumber)} /></label>}
      <div className={styles.coordinates}><label>朝向 °<input type="number" required step="1" value={yaw} onChange={event => setYaw(event.currentTarget.valueAsNumber)} /></label>
        <label>停留 秒<input type="number" required min="0" max="3600" step="0.5" value={dwell} onChange={event => setDwell(event.currentTarget.valueAsNumber)} /></label></div>
      <label>到点动作<select value={action} onChange={event => setAction(event.target.value)}>{inspectionActions.map(([value, label]) => <option value={value} key={value}>{label}</option>)}</select></label>
      <p>采集动作执行时需要相机与对应检测服务就绪。</p>
      {error && <p role="alert">{error}</p>}{message && <p role="status">{message}</p>}
      <button type="submit" className={styles.save} disabled={busy || !position || fromSurface && supportHeight == null}>{busy ? '保存中…' : '保存巡检点'}</button>
      {existing && <button type="button" disabled={busy} onClick={() => setDeleteOpen(true)}>删除此巡检点</button>}
    </form>
    <ConfirmModal open={deleteOpen} title="删除巡检点" message={`删除“${name}”？已保存的路线不会自动修改。`}
      confirmLabel="删除" onCancel={() => setDeleteOpen(false)} onConfirm={() => void remove()} />
    <ConfirmModal open={pendingLeave !== null} title="放弃未保存的点位修改？" message="已保存的巡检点不会改变。"
      confirmLabel="放弃修改" onCancel={() => setPendingLeave(null)} onConfirm={() => {
        const proceed = pendingLeave; setPendingLeave(null); setDirty(false); proceed?.()
      }} />
  </section>
}
