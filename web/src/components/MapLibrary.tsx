import { useId, useState } from 'react'
import { CircleAlert, CircleCheck, LoaderCircle, MoreHorizontal, Pencil, Search, Trash2 } from 'lucide-react'
import type { MapSaves } from '../hooks/useMapSaves.ts'
import { mapLibraryStatus, type MapLibraryEntry } from '../services/mapSavePresentation.ts'
import { MapSaveHistory } from './MapSaveHistory.tsx'
import styles from './MapView.module.css'

export function MapStatusIcon({ kind }: { kind: ReturnType<typeof mapLibraryStatus>['kind'] }) {
  const Icon = kind === 'ready' ? CircleCheck : kind === 'busy' ? LoaderCircle : CircleAlert
  return <Icon size={16} aria-hidden="true" data-map-status={kind} className={kind === 'busy' ? styles.statusBusy : undefined} />
}

interface MapLibraryProps {
  entries: MapLibraryEntry[]
  saves: MapSaves
  selectedMap: string | null
  readOnly: boolean
  onPreview: (name: string) => void
  onRename: (name: string) => void
  onDelete: (name: string) => void
}

function MapRow({ entry, saves, selected, readOnly, onPreview, onRename, onDelete }: Omit<MapLibraryProps, 'entries' | 'selectedMap'> & {
  entry: MapLibraryEntry
  selected: boolean
}) {
  const [detailsOpen, setDetailsOpen] = useState(false)
  const detailsId = useId()
  const { name, map, operations } = entry
  const status = mapLibraryStatus(map, operations[0], Boolean(saves.error))
  return <li className={`${styles.mapRow} ${selected ? styles.mapRowSelected : ''}`}>
    <div className={styles.mapRowMain}>
      <button className={styles.mapChoice} onClick={() => map?.has_pcd ? onPreview(name) : setDetailsOpen(true)}
        aria-pressed={selected} title={name}>
        <span className={styles.mapName}>{name}</span>
      </button>
      <button type="button" className={styles.iconButton} data-status={status.kind}
        aria-label={`${name}：${status.label}，查看详情`} title={status.label}
        aria-expanded={detailsOpen} aria-controls={detailsId} onClick={() => setDetailsOpen(value => !value)}>
        <MapStatusIcon kind={status.kind} />
      </button>
      <button type="button" className={styles.iconButton} onClick={() => setDetailsOpen(value => !value)}
        aria-expanded={detailsOpen} aria-controls={detailsId} aria-label={`${name} 详情与操作`} title="详情与操作">
        <MoreHorizontal size={18} aria-hidden="true" />
      </button>
    </div>
    <div id={detailsId} className={styles.rowDetails} hidden={!detailsOpen}>
      <p>{status.detail}</p>
      {saves.error && <button className={styles.quietButton} onClick={() => void saves.load()}>刷新保存状态</button>}
      {!!map?.activation_blockers?.length && <details>
        <summary>原因详情</summary>
        {map.activation_blockers.map(blocker => <p key={blocker}>{blocker}</p>)}
      </details>}
      {map?.is_active && <p>当前加载的地图</p>}
      {map && <div className={styles.mapMeta}>
        {map.has_pcd && <span>点云</span>}{map.has_octomap && <span>规划地图</span>}
        {map.map_evidence === 'saved_rays' && <span>射线建图</span>}
        {map.map_evidence === 'preview' && <span>点云预览</span>}
        {!!map.size_mb && <span>{map.size_mb.toFixed(1)} MB</span>}
        {!!map.patch_count && <span>{map.patch_count} 子图</span>}
      </div>}
      <MapSaveHistory operations={operations} saves={saves} readOnly={readOnly} />
      {!readOnly && map && <div className={styles.rowActions}>
        <button className={styles.quietButton} onClick={() => onRename(name)}><Pencil size={15} />重命名</button>
        <button className={styles.dangerButton} onClick={() => onDelete(name)}><Trash2 size={15} />删除</button>
      </div>}
    </div>
  </li>
}

export function MapLibrary({ entries, saves, selectedMap, ...actions }: MapLibraryProps) {
  const [search, setSearch] = useState('')
  const visible = entries.filter(entry => entry.name.toLowerCase().includes(search.trim().toLowerCase()))
  return <>
    {(entries.length > 6 || search) && <label className={styles.searchField}>
      <Search size={16} /><input aria-label="搜索地图" placeholder="搜索地图" value={search}
        onChange={event => setSearch(event.target.value)} />
    </label>}
    {saves.error && entries.length === 0 && <p className={styles.stateMsg} role="alert">
      {saves.error}<button className={styles.quietButton} onClick={() => void saves.load()}>重试</button>
    </p>}
    <ul className={styles.mapList}>
      {visible.map(entry => <MapRow key={entry.name} entry={entry} saves={saves}
        selected={selectedMap === entry.name} {...actions} />)}
    </ul>
    {entries.length > 0 && visible.length === 0 && <p className={styles.stateMsg}>没有找到匹配的地图</p>}
  </>
}
