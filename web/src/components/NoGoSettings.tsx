import { useState } from 'react'
import { ChevronDown } from 'lucide-react'
import { useNoGo } from '../hooks/useNoGo.ts'
import { deleteNoGo, putNoGo, type NoGoRegion } from '../services/noGo.ts'
import type { SettingsMap } from '../hooks/useMapSettings.ts'
import { MapProgress } from './MapProgress.tsx'
import { makeRequestId } from '../services/api.ts'
import styles from './SettingsMenu.module.css'

export function NoGoSettings({ map, editing }: { map: SettingsMap; editing: boolean }) {
  const state = useNoGo(map.name, map.contentEpoch)
  const [region, setRegion] = useState<NoGoRegion>(() => ({ id: makeRequestId('region'), min: [0, 0, 0], max: [1, 1, 2] }))
  const [busy, setBusy] = useState(false)
  const [error, setError] = useState('')
  const blocked = map.active ? '正在使用的地图须退出导航后编辑。' : editing ? '先退出点云整理，再修改禁行区。'
    : map.readOnly ? '地图编辑服务不可用。' : ''
  const apply = async (id?: string) => {
    if (!state.map || blocked) return
    setBusy(true); setError('')
    try {
      if (id) await deleteNoGo(map.name, state.map.content_epoch, id)
      else await putNoGo(map.name, state.map.content_epoch, region)
      map.onChanged()
    } catch (cause) { setError(cause instanceof Error ? cause.message : String(cause)) }
    finally { setBusy(false) }
  }
  return <details className={styles.mapSection} aria-label="独立禁行区设置" data-map-section="no-go">
    <summary className={styles.mapSectionToggle}>禁行区<ChevronDown size={16} aria-hidden="true" /></summary>
    <p>地图 {map.name} · 三维范围内禁止进入，删除标记不会擦除真实障碍。</p>
    {blocked && <p>{blocked}</p>}
    {(error || state.error) && <p role="alert" className={styles.error}>{error || state.error}</p>}
    {state.map && <ul className={styles.regionList}>{state.map.regions.map(item => <li key={item.id}>
      <button type="button" onClick={() => setRegion(item)}>{item.label || item.id}</button>
      <span>{item.min.join(', ')} → {item.max.join(', ')} m</span>
      <button type="button" disabled={!!blocked || busy} onClick={() => void apply(item.id)}>删除</button>
    </li>)}</ul>}
    <button type="button" className={styles.actionButton} disabled={!!blocked || busy}
      onClick={() => setRegion({ id: makeRequestId('region'), min: [0, 0, 0], max: [1, 1, 2], label: '', kind: 'no_go' })}>新增区域</button>
    <form onSubmit={event => { event.preventDefault(); void apply() }}>
      <label className={styles.regionName}>显示名称<input maxLength={128} value={region.label ?? ''} placeholder="如东侧玻璃门"
        onChange={event => setRegion({ ...region, label: event.currentTarget.value })} /></label>
      <label className={styles.regionName}>类型<select value={region.kind ?? 'no_go'}
        onChange={event => setRegion({ ...region, kind: event.currentTarget.value as 'no_go' | 'glass' })}>
        <option value="no_go">禁行区域</option><option value="glass">玻璃障碍</option></select></label>
      <div className={styles.regionBounds}>{(['min', 'max'] as const).flatMap(edge => ['X', 'Y', 'Z'].map((axis, i) => <label key={`${edge}${axis}`}>
        {axis} {edge === 'min' ? '下限' : '上限'} m<input type="number" required step="0.05" value={Number.isFinite(region[edge][i]) ? region[edge][i] : ''}
          onChange={event => { const values = [...region[edge]] as NoGoRegion['min']; values[i] = event.currentTarget.valueAsNumber; setRegion({ ...region, [edge]: values }) }} />
      </label>))}</div>
      <button type="submit" className={styles.actionButton} disabled={!!blocked || busy || !state.map
        || !region.min.every((value, i) => Number.isFinite(value) && value < region.max[i])}>保存禁行区</button>
    </form>
    {busy && <MapProgress stage="更新禁行区" />}
  </details>
}
