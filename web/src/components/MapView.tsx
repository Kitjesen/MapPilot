import { useState, useEffect, useCallback, useRef } from 'react'
import { ArrowLeft, Map, FolderOpen, Trash2, RefreshCw, Save, Pencil, Navigation, ChevronDown, Check, MoreHorizontal, X, Search } from 'lucide-react'
import type { MapInfo, SessionEvent, ToastKind } from '../types'
import * as api from '../services/api'
import { formatMapSaveProgress, pendingMapSaveStatus, savedMapStatus, mapSaveProgressValue, mapSaveElapsedMs, formatMapSaveElapsed, type MapSaveStatus } from '../services/mapSavePresentation.ts'
import { mapIsActivationReady, mapSaveBlockedReason, navigationRuntimeReady, navigationSessionReady } from '../services/mapReadiness'
import { PointCloudViewer, type PointCloudPick } from './PointCloudViewer'
import { PromptModal, ConfirmModal } from './Modal'
import { text, type Locale } from '../i18n'
import { isObservationMode } from '../services/observationMode.ts'
import { parseInitialPose, type InitialPoseInput, type LocalizationInitialPose } from '../services/localizationInitialPose.ts'
import styles from './MapView.module.css'

interface MapViewProps {
  initialSelectedMap: string | null
  onUseMap: (name: string, initialPose?: LocalizationInitialPose) => void
  productSwitchAllowed: boolean
  productSwitchReason: string
  productSwitchMessage: string
  onReturnLive: () => void
  session: SessionEvent['data'] | null
  showToast: (msg: string, kind?: ToastKind) => void
  locale: Locale
  motionStartAllowed: boolean
  motionStartBlockedReason: string
}
// ── Map card ───────────────────────────────────────────────────
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
  onUseMap,
  productSwitchAllowed,
  productSwitchReason,
  productSwitchMessage,
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
  const [useMapFrom, setUseMapFrom] = useState<string | null>(null)
  const [useInitialPose, setUseInitialPose] = useState(false)
  const [initialPose, setInitialPose] = useState<InitialPoseInput>({ x: '0', y: '0', z: '0', yaw: '0' })
  useEffect(() => {
    setUseInitialPose(false)
    setInitialPose({ x: '0', y: '0', z: '0', yaw: '0' })
  }, [selectedMap])
  const [navigateFrom, setNavigateFrom] = useState<string | null>(null)
  const [navigationPendingMap, setNavigationPendingMap] = useState<string | null>(null)
  const [renameFrom, setRenameFrom] = useState<string | null>(null)
  const [deleteFrom, setDeleteFrom] = useState<string | null>(null)
  const [saveStatus, setSaveStatus] = useState<MapSaveStatus | null>(null)
  const [saveTiming, setSaveTiming] = useState({ elapsedMs: 0, observedAt: 0 })
  const [saveProgress, setSaveProgress] = useState<number | undefined>()
  const [saveClock, setSaveClock] = useState(0)
  const saveStartedAt = useRef(0)
  const saveActive = saveStatus?.state === 'saving' || saveStatus?.state === 'pending'
  useEffect(() => {
    if (!saveActive) return
    const timer = window.setInterval(() => setSaveClock(Date.now()), 1000)
    return () => window.clearInterval(timer)
  }, [saveActive])

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
  const confirmSave = async (name: string, existing?: api.SaveMapResult & { operation_id: string }) => {
    setSaveOpen(false)
    const blocked = mapSaveBlockedReason(session)
    if (!existing && blocked) {
      showToast(blocked, 'error')
      return
    }
    if (!existing) saveStartedAt.current = Date.now()
    const updateProgress = (result?: api.SaveMapResult) => {
      const now = Date.now()
      setSaveTiming({ elapsedMs: mapSaveElapsedMs(result, now - saveStartedAt.current), observedAt: now })
      setSaveClock(now)
      setSaveProgress(mapSaveProgressValue(result))
    }
    updateProgress(existing)
    setSaveStatus({
      name,
      state: 'saving',
      detail: existing ? formatMapSaveProgress(existing) : '正在提交保存请求',
    })
    try {
      const admission = existing ?? await api.saveMap(name)
      const outcome = await api.waitForMapSaveOperation(admission, {
        onProgress: progress => {
          updateProgress(progress)
          setSaveStatus({ name, state: 'saving', detail: formatMapSaveProgress(progress) })
        },
      })
      if (outcome.state === 'pending') {
        setSaveStatus(pendingMapSaveStatus(outcome.result, outcome.reason))
        return
      }
      const r = outcome.result
      updateProgress(r)
      const savedName = r.name
      setSaveStatus(savedMapStatus(r))
      hasAutoSelected.current = true
      setSelectedMap(savedName)
      setLibraryOpen(false)
      setGoalPickingMap(null)
      showToast(`已保存 ${savedName}`, 'success')
      if (r.warnings?.length) showToast(r.warnings.join('；'), 'info')
      loadMaps()
    }
    catch (e: unknown) {
      updateProgress()
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
          {!observe && selectedMap && selectedInfo && mapIsActivationReady(selectedInfo) && !selectedNavigationReady && (
            <button className={styles.primaryButton} disabled={!productSwitchAllowed}
              title={productSwitchReason || '加载这张地图并切换导航'} onClick={() => setUseMapFrom(selectedMap)}>
              <Navigation size={16} />使用此地图导航
            </button>
          )}
          {selectedNavigationReady && !canPickGoal && <button className={styles.quietButton}
            onClick={() => handleNavigate(selectedMap!)}><Navigation size={16} />选目标</button>}
          {!observe && session?.product === 'map' && <button className={styles.primaryButton}
            onClick={handleSave} disabled={Boolean(saveBlockedReason) || saveStatus?.state === 'saving' || saveStatus?.state === 'pending'}
            title={saveBlockedReason || '保存当前建图并查看整图'}>
            <Save size={16} />{saveStatus?.state === 'saving' ? '保存中…' : '保存地图'}
          </button>}
        </div>
      </header>
      {productSwitchMessage && <div className={styles.saveNotice} role="status">{productSwitchMessage}</div>}
      <ConfirmModal open={useMapFrom !== null} title="使用此地图导航"
        message={<>
          <p>{session?.product === 'map'
          ? '将结束本次建图并加载所选地图。请确认最新补扫已保存；定位成功后可以选择目标。'
          : '将停止当前任务并加载所选地图，定位成功后可以选择目标。'}</p>
          <label className={styles.poseOption}>
            <input type="checkbox" checked={useInitialPose} onChange={event => setUseInitialPose(event.target.checked)} />
            指定初始位姿
          </label>
          {useInitialPose ? <>
            <p>填写机身在所选地图中的位置。Z 是地图坐标中的机身高度，航向单位为弧度。</p>
            <div className={styles.poseFields}>
              {(['x', 'y', 'z', 'yaw'] as const).map(key => <label key={key}>
                {key === 'yaw' ? '航向（rad）' : `${key.toUpperCase()}（m）`}
                <input type="number" step="0.1" value={initialPose[key]}
                  onChange={event => setInitialPose(value => ({ ...value, [key]: event.target.value }))} />
              </label>)}
            </div>
          </> : <p>优先尝试此地图的上次定位；匹配失败后自动全局搜索。已知当前位置时可指定初值。</p>}
        </>}
        confirmLabel="切换导航" onCancel={() => setUseMapFrom(null)}
        onConfirm={() => {
          try {
            const pose = useInitialPose ? parseInitialPose(initialPose) : undefined
            const name = useMapFrom
            if (name) onUseMap(name, pose)
            setUseMapFrom(null)
          } catch (cause) { showToast(cause instanceof Error ? cause.message : String(cause), 'error') }
        }} />
      {saveStatus && <div
        className={`${styles.saveNotice} ${saveStatus.state === 'failed' ? styles.saveError : ''}`} role="status">
        <span>{saveStatus.name}：{saveStatus.state === 'failed' ? '保存失败：' : ''}{saveStatus.detail}</span>
        <span className={styles.saveTiming}>
          {saveActive ? '已等待' : '耗时'} {formatMapSaveElapsed(saveTiming.elapsedMs + (saveActive ? Math.max(0, saveClock - saveTiming.observedAt) : 0))}
          {saveActive && <>
            <progress aria-label="地图保存阶段进度" max={1} value={saveProgress} />
            <small>{saveStatus.state === 'pending' ? '状态待确认；显示最近进度' : '阶段进度，非剩余时间估计'}</small>
          </>}
        </span>
        {saveStatus.summary && <span>{saveStatus.summary}</span>}
        {saveStatus.state === 'pending' && <button className={styles.quietButton}
          onClick={() => void confirmSave(saveStatus.name, saveStatus.operation)}><RefreshCw size={16} />继续查询</button>}
        {(saveStatus.state === 'failed' || saveStatus.state === 'saved') && <button className={styles.iconButton}
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
        message="保存当前建图，完成后可查看整图。同名会替换原地图；要保留原图请使用新名称。"
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
