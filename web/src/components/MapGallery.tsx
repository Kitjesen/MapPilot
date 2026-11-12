import { useMapSettings } from '../hooks/useMapSettings.ts'
import { useCallback, useEffect, useMemo, useRef, useState, type CSSProperties, type ReactNode } from 'react'
import { Brush, CircleHelp, Image, LoaderCircle, Map, Pencil, Pin, PinOff, RefreshCw, Search, SlidersHorizontal, Sparkles, Trash2 } from 'lucide-react'
import type { MapSaves } from '../hooks/useMapSaves.ts'
import { useMapThumbnails, type Thumbnail } from '../hooks/useMapThumbnails.ts'
import { useOctomap } from '../hooks/useOctomap.ts'
import { useSavedCloud } from '../hooks/useSavedCloud.ts'
import { useNoGo } from '../hooks/useNoGo.ts'
import type { MapEditScene } from '../types/mapEdit.ts'
import { mapLibraryStatus, type MapLibraryEntry } from '../services/mapSavePresentation.ts'
import { mapCleanupLabel, mapUpdateTime } from '../services/mapThumbnail.ts'
import { MapStatusIcon } from './MapLibrary.tsx'
import { MapSaveHistory } from './MapSaveHistory.tsx'
import { MapGalleryScene } from './MapGalleryScene.tsx'
import type { MapGallerySceneHandle } from './MapGalleryScene.tsx'
import { MapInspectionPoints } from './MapInspectionPoints.tsx'
import type { LocationEntry } from '../types/index.ts'
import type { XYZ } from '../types/stairs.ts'
import { MapEditor, type MapEditorHandle, type MapEditorTool } from './MapEditor.tsx'
import styles from './MapGallery.module.css'

interface Props {
  open: boolean
  entries: MapLibraryEntry[]
  selectedMap: string | null
  saves: MapSaves
  loading: boolean
  error: string
  readOnly: boolean
  onSelect: (name: string) => void
  actions?: ReactNode
  onRename: (name: string) => void
  onDelete: (name: string) => void
  onRefresh: () => void
  editing?: {
    tool: MapEditorTool
    onClose(): void
    onOpenMap(name: string): void
    onMapsChanged(): void
    onExitGuardChange(guard: ((proceed: () => void) => void) | null): void
  } | null
}

function MapChoice({ entry, thumbnail, active, unavailable, disabled, onSelect, load }: {
  entry: MapLibraryEntry
  thumbnail?: Thumbnail
  active: boolean
  unavailable: boolean
  disabled: boolean
  onSelect: () => void
  load: (entry: MapLibraryEntry) => void
}) {
  const button = useRef<HTMLButtonElement>(null)
  useEffect(() => {
    if (!button.current) return
    const observer = new IntersectionObserver(items => {
      if (items.some(item => item.isIntersecting)) { load(entry); observer.disconnect() }
    })
    observer.observe(button.current)
    return () => observer.disconnect()
  }, [entry, load])
  const data = thumbnail?.state === 'ready' ? thumbnail : null
  const status = mapLibraryStatus(entry.map, entry.operations[0], unavailable)
  const cleanup = data?.data.cleanup_state ?? 'unknown'
  const CleanupIcon = cleanup === 'cleaned' ? Sparkles : cleanup === 'not_cleaned' ? Brush : CircleHelp
  return <button ref={button} type="button" className={styles.mapChoice} aria-pressed={active} disabled={disabled}
    onPointerEnter={() => { if (!disabled) onSelect() }} onFocus={() => { if (!disabled) onSelect() }} onClick={onSelect}
    title={disabled ? '退出当前编辑后切换地图' : entry.name}>
    <span className={styles.miniature}>
      {data ? <img src={data.image} alt="" /> : <Image size={19} aria-hidden="true" />}
    </span>
    <span className={styles.choiceText}><strong>{entry.name}</strong>
      {entry.map?.size_mb != null && <small>{entry.map.size_mb.toFixed(1)} MB</small>}
      <time dateTime={data?.data.built_at_ms ? new Date(data.data.built_at_ms).toISOString() : undefined}>
        更新 {thumbnail?.state === 'ready' ? mapUpdateTime(thumbnail.data.built_at_ms)
          : thumbnail?.state === 'error' ? '暂不可用' : '读取中…'}
      </time>
    </span>
    <span className={styles.statuses}>
      <span className={styles.cleanupIcon} data-cleanup={cleanup} role="img" aria-label={mapCleanupLabel(cleanup)} title={mapCleanupLabel(cleanup)}><CleanupIcon size={14} /></span>
      <span className={styles.statusIcon} role="img" aria-label={status.label} title={status.detail}><MapStatusIcon kind={status.kind} /></span>
    </span>
  </button>
}

export function MapGallery({ open, entries, selectedMap, saves, loading, error, readOnly,
  onSelect, actions, onRename, onDelete, onRefresh, editing }: Props) {
  const mapSettings = useMapSettings()
  const [edit, setEdit] = useState<MapEditScene | null>(null)
  const [anchor, setAnchor] = useState<{ x: number; y: number } | null>(null)
  const editor = useRef<MapEditorHandle>(null)
  const scene = useRef<MapGallerySceneHandle | null>(null)
  const sceneReady = useCallback((handle: MapGallerySceneHandle | null) => { scene.current = handle }, [])
  const [pickedPoint, setPickedPoint] = useState<{ key: string; point: XYZ } | null>(null)
  const [inspectionPoints, setInspectionPoints] = useState<{ key: string; points: LocationEntry[] } | null>(null)
  const [search, setSearch] = useState('')
  const [representation, setRepresentation] = useState<'points' | 'voxels'>('points')
  const [pointSize, setPointSize] = useState(2)
  const [voxelScale, setVoxelScale] = useState(1)
  const [opacity, setOpacity] = useState(.9)
  const [grid, setGrid] = useState(true)
  const [gridStep, setGridStep] = useState(1)
  const [height, setHeight] = useState<{ key: string; value: number } | null>(null)
  const [libraryPinned, setLibraryPinned] = useState(() => {
    try { return localStorage.getItem('lingtu.maps.libraryPinned') === 'true' } catch { return false }
  })
  const [settingsPinned, setSettingsPinned] = useState(() => {
    try { return localStorage.getItem('lingtu.maps.settingsPinned') === 'true' } catch { return false }
  })
  const library = useRef<HTMLDetailsElement>(null)
  const settings = useRef<HTMLDetailsElement>(null)
  useEffect(() => {
    if (!open) return
    if (libraryPinned && library.current) library.current.open = true
    if (settingsPinned && settings.current) settings.current.open = true
    try {
      localStorage.setItem('lingtu.maps.libraryPinned', String(libraryPinned))
      localStorage.setItem('lingtu.maps.settingsPinned', String(settingsPinned))
    } catch { /* Keep the current page preferences when storage is unavailable. */ }
  }, [libraryPinned, settingsPinned, open])
  const { thumbnails, load } = useMapThumbnails()
  const visible = entries.filter(entry => entry.name.toLowerCase().includes(search.trim().toLowerCase()))
  const current = entries.find(entry => entry.name === selectedMap) ?? entries[0]
  const key = current ? `${current.name}:${current.map?.content_epoch ?? ''}` : ''
  const updateInspectionPoints = useCallback((points: LocationEntry[]) => setInspectionPoints({ key, points }), [key])
  const agentPreview = mapSettings?.agentPreview && current
    && mapSettings.agentPreview.mapId === current.name
    && mapSettings.agentPreview.contentEpoch === current.map?.content_epoch ? mapSettings.agentPreview : null
  const thumbnail = thumbnails[key]
  const ready = thumbnail?.state === 'ready' ? thumbnail : null
  const status = current ? mapLibraryStatus(current.map, current.operations[0], Boolean(saves.error)) : null
  const mode = editing || agentPreview ? 'points' : current?.map?.has_octomap ? representation : 'points'
  const fullCloud = useSavedCloud(current?.name, current?.map?.content_epoch, open && mode === 'points' && Boolean(current?.map?.has_pcd))
  const cloud = fullCloud.status === 'ready' ? fullCloud.cloud : undefined
  const noGo = useNoGo(current?.name, current?.map?.content_epoch)
  const voxels = useOctomap(current?.name, current?.map?.content_epoch, undefined, open && mode === 'voxels')
  const octomap = voxels.status === 'ready' ? voxels.map : undefined
  const previewStatus = mode === 'voxels' ? voxels.status : fullCloud.status
  const points = useMemo(() => ready?.data.points.flat(), [ready])
  const bounds = useMemo(() => {
    if (mode === 'voxels' && octomap) return { min: octomap.bounds.min[2], max: octomap.bounds.max[2] }
    if (cloud) return { min: cloud.bounds.min[2], max: cloud.bounds.max[2] }
    if (!points?.length) return null
    let min = Infinity, max = -Infinity
    for (let i = 2; i < points.length; i += 3) { min = Math.min(min, points[i]); max = Math.max(max, points[i]) }
    return { min, max }
  }, [mode, octomap, cloud, points])
  const ceiling = height?.key === key ? height.value : null
  const minHeight = bounds ? Math.floor(bounds.min * 20) / 20 : 0
  const maxHeight = bounds ? Math.ceil(bounds.max * 20) / 20 : 0
  useEffect(() => { if (open && current) load(current) }, [open, current, load])
  useEffect(() => {
    if (open && current && current.name !== selectedMap) onSelect(current.name)
  }, [open, current, selectedMap, onSelect])
  if (!open) return null

  const previewMessage = error || (!current ? loading ? '正在读取地图…' : '选择一张地图' : mode === 'voxels'
    ? voxels.status === 'error' ? voxels.message || '体素读取失败' : !octomap ? '正在读取体素地图…' : ''
    : !current.map?.has_pcd ? '点云尚未保存' : fullCloud.status === 'error' ? fullCloud.message
      : !cloud ? '正在加载完整点云…' : '')

  return <div className={styles.gallery} data-editing={Boolean(editing)}>
    <details ref={library} className={`${styles.dock} ${styles.libraryDock}`} aria-label="左侧地图库"
      data-pinned={libraryPinned} data-hover-menu={libraryPinned ? undefined : true}>
      <summary className={styles.dockHandle} aria-label="展开左侧地图库">
        <span className={styles.edgeTab}><Map size={14} /><span>地图库</span></span>
      </summary>
      <div className={`${styles.panel} ${styles.library}`}>
      <div className={styles.heading}>
        <h2>地图库 <span>{entries.length}</span></h2>
        <button type="button" className={styles.iconButton} aria-pressed={libraryPinned}
          aria-label={libraryPinned ? '取消固定地图库' : '固定地图库到左侧'} title={libraryPinned ? '取消固定，移开后自动隐藏' : '固定在左侧'}
          onClick={() => setLibraryPinned(value => !value)}>{libraryPinned ? <PinOff size={14} /> : <Pin size={14} />}</button>
        <button type="button" className={styles.iconButton} disabled={Boolean(editing)} onClick={onRefresh} title="刷新地图库" aria-label="刷新地图库"><RefreshCw size={15} /></button>
      </div>
      <label className={styles.search}><Search size={16} /><input
        aria-label="搜索地图" placeholder="搜索地图" value={search} onChange={event => setSearch(event.target.value)} /></label>
      {error && <p className={styles.message} role="alert">{error}</p>}
      {loading && !entries.length && <p className={styles.message} role="status">正在读取地图…</p>}
      {!error && !loading && !visible.length && <p className={styles.message}>{entries.length ? '没有匹配的地图' : '还没有保存的地图'}</p>}
      <div className={styles.choices}>
        {visible.map(entry => <div key={`${entry.name}:${entry.map?.content_epoch ?? ''}`} className={styles.mapRow}
          data-active={current?.name === entry.name}>
          <MapChoice entry={entry}
          thumbnail={thumbnails[`${entry.name}:${entry.map?.content_epoch ?? ''}`]} active={current?.name === entry.name}
          unavailable={Boolean(saves.error)} disabled={Boolean(editing)} onSelect={() => onSelect(entry.name)} load={load} />
          {!readOnly && entry.map && <button type="button" className={styles.rowRename}
            aria-label={`重命名 ${entry.name}`} disabled={entry.map.is_active || Boolean(editing)}
            title={entry.map.is_active ? '正在使用的地图暂不支持重命名' : '重命名地图'} onClick={event => {
              event.currentTarget.closest('details')!.querySelector('summary')!.focus()
              onRename(entry.name)
            }}><Pencil size={15} /></button>}
        </div>)}
      </div>
      {current && status && <div className={styles.libraryFooter}>
        <p className={styles.mapMetrics}>{mode === 'voxels'
          ? octomap ? `${(octomap.cells.length / 4).toLocaleString()} 个原始体素`
            : voxels.status === 'error' ? '体素读取失败' : '正在读取体素地图…'
          : !current.map?.has_pcd ? '点云尚未保存' : cloud ? `${(cloud.positions.length / 3).toLocaleString()} 个点 · 完整点云`
            : fullCloud.status === 'error' ? '完整点云读取失败' : '正在加载完整点云…'}</p>
        <details key={current.name} className={styles.details} data-hover-menu>
          <summary>详情与操作</summary>
          <p>{status.detail}</p>
          <p>更新时间：{mapUpdateTime(ready?.data.built_at_ms)}</p>
          <p>{ready && `${ready.data.total_points.toLocaleString()} 个点`}{current.map?.patch_count != null && ` · ${current.map.patch_count} 个子图`}</p>
          {current.map?.activation_blockers?.map(reason => <p key={reason}>{reason}</p>)}
          {ready?.data.cleanup_state === 'unknown' && <p>该版本未记录清洗结果。</p>}
          <MapSaveHistory operations={current.operations} saves={saves} readOnly={readOnly} />
          {actions}
          {!readOnly && current.map && <div className={styles.actions}>
            <button type="button" className={styles.textButton} onClick={() => onDelete(current.name)}><Trash2 size={14} />删除</button>
          </div>}
        </details>
      </div>}
      </div>
    </details>

    <details ref={settings} className={`${styles.dock} ${styles.settingsDock}`} aria-label="右侧地图设置"
      data-pinned={settingsPinned} data-hover-menu={settingsPinned ? undefined : true}>
      <summary className={styles.dockHandle} aria-label="展开右侧地图设置">
        <span className={styles.edgeTab}><SlidersHorizontal size={14} /><span>地图设置</span></span>
      </summary>
    <section className={`${styles.panel} ${styles.settings}`} aria-label="地图显示设置">
      <div className={styles.heading}>
        <h3>地图设置</h3>
        <button type="button" className={styles.iconButton} aria-pressed={settingsPinned}
          aria-label={settingsPinned ? '取消固定地图设置' : '固定地图设置到右侧'} title={settingsPinned ? '取消固定，移开后自动隐藏' : '固定在右侧'}
          onClick={() => setSettingsPinned(value => !value)}>{settingsPinned ? <PinOff size={14} /> : <Pin size={14} />}</button>
      </div>
      <div className={styles.settingsBody}>
      <div className={styles.modes} role="group" aria-label="地图显示类型">
        <button type="button" aria-pressed={mode === 'points'} onClick={() => setRepresentation('points')}>点云地图</button>
        <button type="button" aria-pressed={mode === 'voxels'} disabled={!current?.map?.has_octomap}
          title={!current?.map?.has_octomap ? '该地图尚无体素数据' : undefined} onClick={() => setRepresentation('voxels')}>体素地图</button>
      </div>
      {mode === 'points' ? <label className={styles.slider}>
        <span>点大小</span>
        <input aria-label="地图点大小" type="range" min=".5" max="6" step=".5" value={pointSize}
          style={{ '--fill': `${(pointSize - .5) / 5.5 * 100}%` } as CSSProperties} onChange={event => setPointSize(Number(event.target.value))} />
        <output>{pointSize.toFixed(1)} <small>px</small></output>
      </label> : <label className={styles.settingRow}><span>体素显示大小</span>
        <select aria-label="体素显示大小" value={voxelScale} onChange={event => setVoxelScale(Number(event.target.value))}>
          {[1, 2, 4, 8].map(scale => <option key={scale} value={scale}>{octomap ? `${Number((octomap.resolution * scale * 100).toFixed(2))} cm` : `${scale}×`}{scale === 1 ? ' · 原始' : ''}</option>)}
        </select>
      </label>}
      <label className={styles.slider}><span>不透明度</span>
        <input aria-label="地图不透明度" type="range" min="0" max="100" step="5" value={Math.round(opacity * 100)}
          style={{ '--fill': `${opacity * 100}%` } as CSSProperties} onChange={event => setOpacity(Number(event.target.value) / 100)} />
        <output>{Math.round(opacity * 100)}<small>%</small></output>
      </label>
      {bounds && <label className={styles.slider}><span>显示高度</span>
        <input aria-label="地图显示高度" type="range" min={minHeight} max={maxHeight} step=".05" value={ceiling ?? maxHeight}
          style={{ '--fill': `${((ceiling ?? maxHeight) - minHeight) / Math.max(.05, maxHeight - minHeight) * 100}%` } as CSSProperties}
          onChange={event => { const value = Number(event.target.value); setHeight(value >= bounds.max ? null : { key, value }) }} />
        <output>{(ceiling ?? maxHeight).toFixed(2)} <small>m</small></output>
      </label>}
      <label className={styles.settingRow}><span>参考网格</span><input type="checkbox" checked={grid} onChange={event => setGrid(event.target.checked)} /></label>
      {grid && <label className={styles.settingRow}><span>网格间距</span><select aria-label="参考网格间距" value={gridStep} onChange={event => setGridStep(Number(event.target.value))}>
        {[.5, 1, 2, 5, 10].map(step => <option key={step} value={step}>{step} m</option>)}
      </select></label>}
      <p className={styles.settingHint}>仅调整显示，不改动已保存地图。</p>
      </div>
    </section>
    </details>

    <figure className={styles.terrain} aria-label={current ? `${current.name} 地图` : '地图'}>
      <div className={styles.preview}>
        {current && <MapGalleryScene key={key} name={current.name} points={mode === 'points' && !cloud ? points : undefined}
          cloud={mode === 'points' ? cloud : undefined}
          octomap={mode === 'voxels' ? octomap : undefined} voxelScale={voxelScale} pointSize={pointSize}
          opacity={opacity} ceiling={ceiling} grid={grid} gridStep={gridStep}
          mapEdit={agentPreview?.scene ?? (editing ? edit : null)} onReady={sceneReady}
          pointPicking={editing?.tool === 'point'} onPointPick={point => setPickedPoint({ key, point })}
          inspectionPoints={editing?.tool === 'point' && inspectionPoints?.key === key ? inspectionPoints.points : undefined}
          noGoRegions={noGo.map?.regions}
          onMapEditBounds={bounds => editor.current?.setBounds(bounds)} onMapEditAnchor={setAnchor} />}
        {previewMessage && <div className={points?.length || cloud || octomap ? styles.loadingStatus : styles.placeholder}
          role={error || previewStatus === 'error' ? 'alert' : 'status'}>
          <LoaderCircle size={16} className={previewStatus === 'loading' ? styles.spinner : undefined} />
          <span>{previewMessage}</span>
          {current && previewStatus === 'error' && <button type="button" className={styles.textButton} onClick={onRefresh}>重新读取</button>}
          {editing && <button type="button" className={styles.textButton} onClick={editing.onClose}>退出编辑</button>}
        </div>}
      </div>
    </figure>
    {editing?.tool === 'point' && current?.map?.content_epoch && <MapInspectionPoints key={key}
      mapName={current.name} epoch={current.map.content_epoch} picked={pickedPoint?.key === key ? pickedPoint.point : null} onMarkers={updateInspectionPoints}
      onFocus={point => scene.current?.focusPoint(point)} onClose={editing.onClose} onExitGuardChange={editing.onExitGuardChange} />}
    {editing && editing.tool !== 'point' && current?.map?.content_epoch && fullCloud.status === 'ready' && <MapEditor
      key={`${key}:${editing.tool}`} ref={editor}
      source={{ name: current.name, contentEpoch: current.map.content_epoch, frameId: fullCloud.frameId }}
      points={fullCloud.cloud.positions} anchor={anchor} footerTarget={null} initialTool={editing.tool}
      onSceneChange={setEdit} onView={view => scene.current?.mapEditView(view)}
      onClose={() => { setEdit(null); setAnchor(null); editing.onClose() }}
      onOpenMap={editing.onOpenMap} onMapsChanged={editing.onMapsChanged} onExitGuardChange={editing.onExitGuardChange} />}
  </div>
}
