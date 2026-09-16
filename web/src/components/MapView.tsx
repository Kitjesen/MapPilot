import { useState, useEffect, useCallback, useRef } from 'react'
import { ArrowLeft, Map, FolderOpen, Trash2, RefreshCw, Save, Pencil, Navigation, ChevronDown, Check, MoreHorizontal, X, Search } from 'lucide-react'
import type { MapInfo, SessionEvent, ToastKind } from '../types'
import * as api from '../services/api'
import { mapIsActivationReady, mapSaveBlockedReason, navigationRuntimeReady, navigationSessionReady } from '../services/mapReadiness'
import { PointCloudViewer, type PointCloudPick } from './PointCloudViewer'
import { PromptModal, ConfirmModal } from './Modal'
import { text, type Locale } from '../i18n'
import { isObservationMode } from '../services/observationMode.ts'
import styles from './MapView.module.css'

interface MapViewProps {
  initialSelectedMap: string | null
  onReturnLive: () => void
  session: SessionEvent['data'] | null
  showToast: (msg: string, kind?: ToastKind) => void
  locale: Locale
  motionStartAllowed: boolean
  motionStartBlockedReason: string
}
// ── Map card ───────────────────────────────────────────────────
function formatSaveMapSummary(r: api.SaveMapResult): string {
  const source = r.map_save_source ?? r.source ?? 'unknown'
  const savedMapReloc = r.saved_map_relocalization_supported ?? r.relocalization_supported
  const relocText = savedMapReloc === undefined ? '未知' : savedMapReloc ? '支持' : '不支持'
  const recovery = r.restart_recovery_supported === undefined
    ? (r.recovery_method ?? 'unknown')
    : `${r.restart_recovery_supported ? 'restart' : 'no-restart'}${r.recovery_method ? `/${r.recovery_method}` : ''}`
  const warnings = r.warnings?.filter(Boolean) ?? []
  return [
    `来源：${source === 'unknown' ? '未知' : source}`,
    `保存地图重定位：${relocText}`,
    `恢复方式：${recovery === 'unknown' ? '未知' : recovery}`,
    warnings.length > 0 ? `警告：${warnings.join('; ')}` : null,
  ].filter((v): v is string => Boolean(v)).join(' | ')
}

interface SaveStatus {
  name: string
  state: 'saving' | 'saved' | 'failed'
  detail: string
  location?: string | null
  summary?: string | null
}

function saveMapStringField(r: api.SaveMapResult, keys: string[]): string | null {
  const record = r as unknown as Record<string, unknown>
  for (const key of keys) {
    const value = record[key]
    if (typeof value === 'string' && value.trim()) return value.trim()
  }
  return null
}

function formatSaveMapLocation(r: api.SaveMapResult, name: string): string {
  return saveMapStringField(r, [
    'path',
    'map_path',
    'map_dir',
    'save_dir',
    'directory',
    'pcd_path',
    'pcd',
  ]) ?? `网关地图目录 / ${name}`
}

function formatSaveMapDetail(r: api.SaveMapResult): string {
  const parts: string[] = []
  const df = r.dynamic_filter
  if (df && df.success && typeof df.dropped === 'number' && typeof df.orig_count === 'number' && df.orig_count > 0) {
    const pct = (100 * df.dropped / df.orig_count).toFixed(1)
    parts.push(`动态点清理 ${df.dropped}/${df.orig_count} (${pct}%)`)
  }
  if (r.size) parts.push(`大小 ${r.size}`)
  if (r.saved_map_relocalization_supported ?? r.relocalization_supported) {
    parts.push('支持重定位')
  }
  return parts.length ? parts.join(' · ') : '地图已保存'
}

interface CardProps {
  m: MapInfo
  selected: boolean
  readOnly: boolean
  navigationReady: boolean
  onPreview:  (name: string) => void
  onNavigate: (name: string) => void
  onRename:   (name: string) => void
  onDelete:   (name: string) => void
}
function MapCard({ m, selected, readOnly, navigationReady, onPreview, onNavigate, onRename, onDelete }: CardProps) {
  const [detailsOpen, setDetailsOpen] = useState(false)
  return (
    <li className={`${styles.mapRow} ${selected ? styles.mapRowSelected : ''}`}>
      <div className={styles.mapRowMain}>
        <button className={styles.mapChoice} onClick={() => onPreview(m.name)}
          disabled={!m.has_pcd} aria-pressed={selected} title={m.name}>
          <Map size={17} strokeWidth={1.6} />
          <span className={styles.mapName}>{m.name}</span>
          {selected && <Check size={16} />}
        </button>
        <button className={styles.iconButton} onClick={() => setDetailsOpen(value => !value)}
          aria-expanded={detailsOpen} aria-label={`${m.name} 详情与操作`} title="详情与操作">
          <MoreHorizontal size={18} />
        </button>
      </div>
      {!m.has_pcd && <span className={styles.rowHint}>暂无点云</span>}
      {detailsOpen && <div className={styles.rowDetails}>
        <p>{mapIsActivationReady(m) ? '导航数据齐全，使用前仍需定位' : '导航数据未就绪'}</p>
        {m.is_active && <p>当前加载的地图</p>}
        <div className={styles.mapMeta}>
          {m.has_pcd && <span>点云</span>}{m.has_octomap && <span>规划地图</span>}
          {!!m.size_mb && <span>{m.size_mb.toFixed(1)} MB</span>}
          {!!m.patch_count && <span>{m.patch_count} 子图</span>}
        </div>
        {!readOnly && <div className={styles.rowActions}>
          {navigationReady && mapIsActivationReady(m) && <button className={styles.quietButton}
            onClick={() => onNavigate(m.name)}><Navigation size={15} />选目标</button>}
          <button className={styles.quietButton} onClick={() => onRename(m.name)}><Pencil size={15} />重命名</button>
          <button className={styles.dangerButton} onClick={() => onDelete(m.name)}><Trash2 size={15} />删除</button>
        </div>}
      </div>}
    </li>
  )
}

// ── Main ───────────────────────────────────────────────────────
export function MapView({
  initialSelectedMap,
  onReturnLive,
  session,
  showToast,
  locale,
  motionStartAllowed,
  motionStartBlockedReason,
}: MapViewProps) {
  const observe = isObservationMode()
  const saveBlockedReason = mapSaveBlockedReason(session)
  const [maps,        setMaps       ] = useState<MapInfo[]>([])
  const [loading,     setLoading    ] = useState(true)
  const [error,       setError      ] = useState('')
  const [selectedMap, setSelectedMap] = useState<string | null>(initialSelectedMap)
  const [libraryOpen, setLibraryOpen] = useState(initialSelectedMap === null)
  const [search, setSearch] = useState('')
  const [pickedPoint, setPickedPoint] = useState<PointCloudPick | null>(null)
  const [goalPickingMap, setGoalPickingMap] = useState<string | null>(null)

  // Modal state
  const [saveOpen,   setSaveOpen  ] = useState(false)
  const [navigateFrom, setNavigateFrom] = useState<string | null>(null)
  const [navigationPendingMap, setNavigationPendingMap] = useState<string | null>(null)
  const [renameFrom, setRenameFrom] = useState<string | null>(null)
  const [deleteFrom, setDeleteFrom] = useState<string | null>(null)
  const [saveStatus, setSaveStatus] = useState<SaveStatus | null>(null)

  const hasAutoSelected = useRef(initialSelectedMap !== null)

  // Data
  const loadMaps = useCallback(async () => {
    setLoading(true); setError('')
    try {
      const data = await api.fetchMaps()
      setMaps(data)
      if (!hasAutoSelected.current) {
        const active = data.find(m => m.is_active)
        if (active) { setSelectedMap(active.name); hasAutoSelected.current = true }
      }
    }
    catch (e: unknown) {
      setError(`无法获取地图列表: ${e instanceof Error ? e.message : String(e)}`)
    } finally { setLoading(false) }
  }, [])

  useEffect(() => { loadMaps() }, [loadMaps])

  useEffect(() => { setPickedPoint(null) }, [selectedMap])

  const handleNavigate = (name: string) => setNavigateFrom(name)
  const handleDelete = (name: string) => setDeleteFrom(name)
  const handleRename = (name: string) => setRenameFrom(name)
  const handleSave = () => setSaveOpen(true)

  const ensureNavigationSession = async (mapName: string) => {
    const [session, navigation] = await Promise.all([
      api.fetchSession(),
      api.fetchNavigationStatus(),
    ])
    if (!navigationRuntimeReady(session, navigation, mapName)) {
      throw new Error(`当前导航未在地图 ${mapName} 就绪，请先确认已加载该地图、定位有效且可接收目标`)
    }
  }

  const confirmNavigate = async () => {
    const name = navigateFrom
    setNavigateFrom(null)
    if (!name || navigationPendingMap) return
    setNavigationPendingMap(name)
    try {
      await ensureNavigationSession(name)
      setSelectedMap(name)
      setGoalPickingMap(name)
      setLibraryOpen(false)
      showToast(`导航已就绪：${name}`, 'success')
      await loadMaps()
    } catch (error) {
      showToast(`导航就绪检查失败：${error instanceof Error ? error.message : String(error)}`, 'error')
    } finally {
      setNavigationPendingMap(null)
    }
  }

  const confirmDelete = async () => {
    const name = deleteFrom
    setDeleteFrom(null)
    if (!name) return
    try {
      await api.deleteMap(name); showToast(`已删除: ${name}`, 'success')
      if (selectedMap === name) { setSelectedMap(null); setLibraryOpen(true) }
      loadMaps()
    } catch { showToast(`删除失败: ${name}`, 'error') }
  }
  const confirmRename = async (newName: string) => {
    const oldName = renameFrom
    setRenameFrom(null)
    if (!oldName || newName === oldName) return
    try {
      await api.renameMap(oldName, newName); showToast(`已重命名: ${newName}`, 'success')
      if (selectedMap === oldName) setSelectedMap(newName); loadMaps()
    } catch { showToast('重命名失败', 'error') }
  }
  const confirmSave = async (name: string) => {
    setSaveOpen(false)
    const blocked = mapSaveBlockedReason(session)
    if (blocked) {
      showToast(blocked, 'error')
      return
    }
    setSaveStatus({
      name,
      state: 'saving',
      detail: '正在写入点云、清理动态点并生成导航地图。完成后会显示保存位置。',
    })
    try {
      const admission = await api.saveMap(name)
      const r = await api.waitForMapSaveOperation(admission)
      const savedName = r.name
      const summary = formatSaveMapSummary(r)
      setSaveStatus({
        name: savedName,
        state: 'saved',
        detail: formatSaveMapDetail(r),
        location: formatSaveMapLocation(r, savedName),
        summary,
      })
      hasAutoSelected.current = true
      setSelectedMap(savedName)
      setLibraryOpen(false)
      setGoalPickingMap(null)
      showToast(`已保存 ${savedName}`, 'success')
      if (r.warnings?.length) showToast(r.warnings.join('；'), 'info')
      loadMaps()
    }
    catch (e: unknown) {
      const message = e instanceof Error ? e.message : String(e)
      setSaveStatus({
        name,
        state: 'failed',
        detail: message || '保存失败，请检查 Gateway 日志。',
      })
      showToast(`保存失败：${message}`, 'error')
    }
  }

  const confirmPickedGoal = async () => {
    if (!pickedPoint || !selectedMap) return
    if (!motionStartAllowed) {
      showToast(motionStartBlockedReason, 'error')
      return
    }
    try {
      await ensureNavigationSession(selectedMap)
      const res = await api.navigateClick(pickedPoint.x, pickedPoint.y, {
        z: pickedPoint.z,
        source: 'map_click',
        target_type: 'map_point',
        label: 'point_cloud_click',
        metadata: { map_name: selectedMap, source_view: 'point_cloud' },
      })
      showToast(
        api.formatCommandAck(res, `3D goal (${pickedPoint.x.toFixed(2)}, ${pickedPoint.y.toFixed(2)})`),
        'success',
      )
      setPickedPoint(null)
    } catch (e: unknown) {
      showToast(api.formatCommandError(e, '3D 点目标失败'), 'error')
    }
  }

  const nameValidator = (v: string) => {
    if (!/^[a-zA-Z0-9_-]+$/.test(v)) return '仅支持字母、数字、下划线和横线'
    if (v.length > 32) return '名称过长 (最多 32 字符)'
    return null
  }

  const togglePreview = (name: string) => {
    hasAutoSelected.current = true
    setGoalPickingMap(null)
    setSelectedMap(name)
    setLibraryOpen(false)
  }

  const filteredMaps = maps.filter(map => map.name.toLowerCase().includes(search.trim().toLowerCase()))
  const selectedInfo = maps.find(map => map.name === selectedMap)
  const canPickGoal = !observe && selectedMap !== null && goalPickingMap === selectedMap
    && session !== null && navigationSessionReady(session, selectedMap)
  const selectedNavigationReady = !observe && selectedMap !== null && selectedInfo !== undefined
    && mapIsActivationReady(selectedInfo) && session !== null && navigationSessionReady(session, selectedMap)

  return (
    <section className={styles.mapTab} aria-label="已保存地图">
      <header className={styles.mapHeader}>
        <button className={styles.backButton} onClick={onReturnLive}><ArrowLeft size={17} />现场</button>
        <span className={styles.headerDivider} />
        <button className={styles.mapSelector} onClick={() => setLibraryOpen(value => !value)}
          aria-expanded={libraryOpen} aria-controls="saved-map-library" title={selectedMap ?? '选择地图'}>
          <Map size={17} strokeWidth={1.6} />
          <span>{selectedMap ?? '选择地图'}</span><ChevronDown size={15} />
        </button>
        {selectedMap && <details className={styles.snapshotInfo}>
          <summary>已保存</summary>
          <div className={styles.snapshotPopover}>
            <strong>整图快照</strong>
            <p>显示保存时的地图。补扫后再次保存，即可更新。</p>
            {saveStatus?.state === 'saved' && saveStatus.name === selectedMap && <details>
              <summary>保存详情</summary>
              <p>{saveStatus.detail}</p><p>{saveStatus.location}</p><p>{saveStatus.summary}</p>
            </details>}
          </div>
        </details>}
        <div className={styles.headerActions}>
          {selectedNavigationReady && !canPickGoal && <button className={styles.quietButton}
            onClick={() => handleNavigate(selectedMap!)}><Navigation size={16} />选目标</button>}
          {!observe && session?.product === 'map' && <button className={styles.primaryButton}
            onClick={handleSave} disabled={Boolean(saveBlockedReason) || saveStatus?.state === 'saving'}
            title={saveBlockedReason || '保存当前建图并查看整图'}>
            <Save size={16} />{saveStatus?.state === 'saving' ? '保存中…' : '保存地图'}
          </button>}
        </div>
      </header>
      {saveStatus && saveStatus.state !== 'saved' && <div
        className={`${styles.saveNotice} ${saveStatus.state === 'failed' ? styles.saveError : ''}`} role="status">
        <span>{saveStatus.state === 'saving' ? `正在保存 ${saveStatus.name}…` : `保存失败：${saveStatus.detail}`}</span>
        {saveStatus.state === 'failed' && <button className={styles.iconButton}
          onClick={() => setSaveStatus(null)} aria-label="关闭保存提示"><X size={16} /></button>}
      </div>}
      <div className={styles.mapWorkspace}>
        {libraryOpen && <aside id="saved-map-library" className={styles.library} aria-label="地图库">
          <div className={styles.libraryHeader}>
            <h2>{text(locale, 'Maps', '地图库')}<span>{maps.length}</span></h2>
            <button className={styles.iconButton} onClick={loadMaps} aria-label="刷新地图库" title="刷新">
              <RefreshCw size={16} /></button>
            <button className={styles.iconButton} onClick={() => setLibraryOpen(false)} aria-label="收起地图库" title="收起">
              <X size={17} /></button>
          </div>
          {(maps.length > 6 || search) && <label className={styles.searchField}>
            <Search size={16} /><input aria-label="搜索地图" placeholder="搜索地图" value={search}
              onChange={event => setSearch(event.target.value)} />
          </label>}
          <div className={styles.libraryScroll}>
            {loading && <p className={styles.stateMsg} role="status">正在读取地图…</p>}
            {error && <div className={styles.stateMsg} role="status">
              <p>{error}</p>{maps.length > 0 && <p>以下为上次读取的地图。</p>}
              <button className={styles.quietButton} onClick={loadMaps}>重试</button>
            </div>}
            {!loading && !error && maps.length === 0 && <div className={styles.emptyLibrary}>
              <FolderOpen size={28} strokeWidth={1.3} /><p>还没有保存的地图</p>
            </div>}
            {!loading && <ul className={styles.mapList}>
              {filteredMaps.map(map => <MapCard key={map.name} m={map} selected={selectedMap === map.name}
                readOnly={observe} navigationReady={session !== null && navigationSessionReady(session, map.name)}
                onPreview={togglePreview} onNavigate={handleNavigate} onRename={handleRename} onDelete={handleDelete} />)}
            </ul>}
            {!loading && maps.length > 0 && filteredMaps.length === 0 && <p className={styles.stateMsg}>没有找到匹配的地图</p>}
          </div>
        </aside>}
        <div className={styles.mapCanvas}>
          {selectedMap ? <PointCloudViewer mapName={selectedMap} pickedPoint={canPickGoal ? pickedPoint : null}
            onPick={canPickGoal ? setPickedPoint : undefined} /> : <div className={styles.emptyCanvas}>
              <Map size={36} strokeWidth={1.2} /><h2>查看已保存的地图</h2>
              <p>选择一张地图，查看完整建图范围。</p>
              {!libraryOpen && <button className={styles.quietButton} onClick={() => setLibraryOpen(true)}>打开地图库</button>}
            </div>}
          {canPickGoal && <div className={styles.pickPanel}>
            <div className={styles.pickInfo}>
              <strong>{pickedPoint ? '导航目标' : '点击地图选择目标'}</strong>
              {pickedPoint && <span>{pickedPoint.x.toFixed(2)}, {pickedPoint.y.toFixed(2)}, {pickedPoint.z.toFixed(2)} m</span>}
            </div>
            {pickedPoint && <button className={styles.primaryButton} onClick={confirmPickedGoal}
              disabled={!motionStartAllowed} title={motionStartAllowed ? '发送导航目标' : motionStartBlockedReason}>发送目标</button>}
            <button className={styles.quietButton} onClick={() => { setPickedPoint(null); setGoalPickingMap(null) }}>取消</button>
          </div>}
        </div>
      </div>

      <PromptModal
        open={saveOpen}
        title="保存地图"
        message="保存当前建图，完成后查看整图。"
        placeholder="例如 building_2f"
        confirmLabel="保存"
        icon={<Save size={18} />}
        validate={nameValidator}
        onConfirm={confirmSave}
        onCancel={() => setSaveOpen(false)}
      />

      <PromptModal
        open={renameFrom != null}
        title="重命名地图"
        message={renameFrom ? `将 "${renameFrom}" 重命名为：` : ''}
        placeholder="新地图名称"
        initialValue={renameFrom ?? ''}
        confirmLabel="重命名"
        icon={<Pencil size={18} />}
        validate={nameValidator}
        onConfirm={confirmRename}
        onCancel={() => setRenameFrom(null)}
      />

      <ConfirmModal
        open={navigateFrom != null}
        title="在此地图选目标"
        message={`检查当前导航是否已在“${navigateFrom ?? ''}”就绪。通过后将打开该地图以选择目标；发送目标后才会运动。`}
        confirmLabel="检查就绪"
        onConfirm={confirmNavigate}
        onCancel={() => setNavigateFrom(null)}
      />

      <ConfirmModal
        open={deleteFrom != null}
        title="删除地图"
        message={`确定要删除地图 "${deleteFrom}" 吗？此操作无法撤销。`}
        confirmLabel="删除"
        danger
        onConfirm={confirmDelete}
        onCancel={() => setDeleteFrom(null)}
      />
    </section>
  )
}
