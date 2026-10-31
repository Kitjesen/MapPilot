import { useState, useEffect, useCallback, useRef } from 'react'
import { ArrowLeft, Map, FolderOpen, Trash2, RefreshCw, Save, Pencil, Navigation, ChevronDown, Check, MoreHorizontal, X, Search } from 'lucide-react'
import type { MapInfo, SessionEvent, ToastKind } from '../types'
import * as api from '../services/api'
import { formatMapSaveProgress, pendingMapSaveStatus, savedMapStatus, mapSaveProgressValue, mapSaveElapsedMs, formatMapSaveElapsed, type MapSaveStatus } from '../services/mapSavePresentation.ts'
import { mapIsActivationReady, mapSaveBlockedReason, mapUseStatus, navigationGoalQueueReady } from '../services/mapReadiness'
import { PointCloudViewer } from './PointCloudViewer'
import { PromptModal, ConfirmModal } from './Modal'
import { text, type Locale } from '../i18n'
import { isObservationMode } from '../services/observationMode.ts'
import { parseInitialPose, type InitialPoseInput, type LocalizationInitialPose } from '../services/localizationInitialPose.ts'
import { mapEditNameError } from '../services/mapEdit.ts'
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
}
// ── Map card ───────────────────────────────────────────────────
interface CardProps {
  m: MapInfo
  selected: boolean
  readOnly: boolean
  onPreview:  (name: string) => void
  onRename:   (name: string) => void
  onDelete:   (name: string) => void
}

const MAP_EVIDENCE_LABEL: Record<string, string> = {
  saved_rays: '射线建图',
  preview: '点云预览',
}

function MapCard({ m, selected, readOnly, onPreview, onRename, onDelete }: CardProps) {
  const [detailsOpen, setDetailsOpen] = useState(false)
  const activationBlockers = m.activation_blockers ?? []
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
      {activationBlockers[0]
        ? <span className={styles.rowHint}>无法用于导航：{activationBlockers[0]}</span>
        : !m.has_pcd && <span className={styles.rowHint}>暂无点云</span>}
      {detailsOpen && <div className={styles.rowDetails}>
        <p>{mapIsActivationReady(m) ? '导航数据齐全，使用前仍需定位' : '导航数据未就绪'}</p>
        {activationBlockers.map(blocker => <p key={blocker}>阻塞原因：{blocker}</p>)}
        {m.is_active && <p>当前加载的地图</p>}
        <div className={styles.mapMeta}>
          {m.has_pcd && <span>点云</span>}{m.has_octomap && <span>规划地图</span>}
          {m.map_evidence && MAP_EVIDENCE_LABEL[m.map_evidence] && <span>{MAP_EVIDENCE_LABEL[m.map_evidence]}</span>}
          {!!m.size_mb && <span>{m.size_mb.toFixed(1)} MB</span>}
          {!!m.patch_count && <span>{m.patch_count} 子图</span>}
        </div>
        {!readOnly && <div className={styles.rowActions}>
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
}: MapViewProps) {
  const observe = isObservationMode()
  const saveBlockedReason = mapSaveBlockedReason(session)
  const [maps,        setMaps       ] = useState<MapInfo[]>([])
  const [loading,     setLoading    ] = useState(true)
  const [error,       setError      ] = useState('')
  const [selectedMap, setSelectedMap] = useState<string | null>(initialSelectedMap)
  const [libraryOpen, setLibraryOpen] = useState(initialSelectedMap === null)
  const [search, setSearch] = useState('')
  const [previewRefresh, setPreviewRefresh] = useState(0)

  // Modal state
  const [saveOpen,   setSaveOpen  ] = useState(false)
  const [resolution, setResolution] = useState(0.1)
  const [resolutionCopyOpen, setResolutionCopyOpen] = useState(false)
  const [resolutionBuild, setResolutionBuild] = useState<{
    source: string; name: string; options: Record<string, unknown>; state: 'copying' | 'building' | 'ready' | 'failed'; message: string
  } | null>(null)
  const [useMapFrom, setUseMapFrom] = useState<string | null>(null)
  const [useInitialPose, setUseInitialPose] = useState(false)
  const [initialPose, setInitialPose] = useState<InitialPoseInput>({ x: '0', y: '0', z: '0', yaw: '0' })
  useEffect(() => {
    setUseInitialPose(false)
    setInitialPose({ x: '0', y: '0', z: '0', yaw: '0' })
  }, [selectedMap])
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
  const listRequest = useRef(0)

  // Data
  const loadMaps = useCallback(async () => {
    const request = ++listRequest.current
    setLoading(true); setError('')
    try {
      const data = await api.fetchMaps()
      if (request !== listRequest.current) return null
      setMaps(data)
      setSelectedMap(current => {
        if (current && data.some(map => map.name === current)) return current
        if (!hasAutoSelected.current) {
          const active = data.find(map => map.is_active)
          if (active) { hasAutoSelected.current = true; return active.name }
        }
        return null
      })
      return data
    }
    catch (e: unknown) {
      if (request !== listRequest.current) return null
      setMaps([])
      setSelectedMap(null)
      setLibraryOpen(true)
      setError(`无法获取地图列表: ${e instanceof Error ? e.message : String(e)}`)
      return null
    } finally { if (request === listRequest.current) setLoading(false) }
  }, [])

  useEffect(() => {
    void loadMaps()
    return () => { listRequest.current += 1 }
  }, [loadMaps, session?.product_session_id])

  const handleDelete = (name: string) => setDeleteFrom(name)
  const handleRename = (name: string) => setRenameFrom(name)
  const handleSave = () => setSaveOpen(true)

  const confirmDelete = async () => {
    const name = deleteFrom
    setDeleteFrom(null)
    if (!name) return
    try {
      await api.deleteMap(name); showToast(`已删除: ${name}`, 'success')
      if (selectedMap === name) { setSelectedMap(null); setLibraryOpen(true) }
      void loadMaps()
    } catch { showToast(`删除失败: ${name}`, 'error') }
  }
  const confirmRename = async (newName: string) => {
    const oldName = renameFrom
    setRenameFrom(null)
    if (!oldName || newName === oldName) return
    try {
      await api.renameMap(oldName, newName); showToast(`已重命名: ${newName}`, 'success')
      if (selectedMap === oldName) setSelectedMap(newName); void loadMaps()
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
      showToast(`已保存 ${savedName}`, 'success')
      if (r.warnings?.length) showToast(r.warnings.join('；'), 'info')
      await loadMaps()
      setPreviewRefresh(value => value + 1)
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

  const nameValidator = (v: string) => {
    if (!/^[a-zA-Z0-9_-]+$/.test(v)) return '仅支持字母、数字、下划线和横线'
    if (v.length > 32) return '名称过长 (最多 32 字符)'
    return null
  }

  const togglePreview = (name: string) => {
    hasAutoSelected.current = true
    setSelectedMap(name)
    setLibraryOpen(false)
  }

  const refreshMapsAndPreview = async () => {
    if (await loadMaps()) setPreviewRefresh(value => value + 1)
  }

  const rebuildResolution = async (target: { source: string; name: string; options: Record<string, unknown> }) => {
    setResolutionBuild({ ...target, state: 'building', message: '正在从保存的原始点云与射线重建导航地图…' })
    try {
      const result = await api.buildMapOctomap(target.name, target.options)
      setResolutionBuild({ ...target, state: 'ready', message: `已生成 ${Number(result.resolution ?? target.options.resolution) * 100} cm 导航地图；使用此地图导航后生效。原图 ${target.source} 已保留。` })
      hasAutoSelected.current = true
      setSelectedMap(target.name)
      await refreshMapsAndPreview()
    } catch (reason) {
      setResolutionBuild({ ...target, state: 'failed', message: `${target.name} 的源数据已保存，导航重建失败：${reason instanceof Error ? reason.message : String(reason)}` })
      await loadMaps()
    }
  }

  const createResolutionCopy = async (name: string) => {
    if (!selectedMap || !selectedInfo?.content_epoch) return
    setResolutionCopyOpen(false)
    const target = { source: selectedMap, name, options: { resolution } }
    setResolutionBuild({ ...target, state: 'copying', message: '正在复制地图源数据与测量射线…' })
    try {
      const copied = await api.saveEditedMap({ name: selectedMap, contentEpoch: selectedInfo.content_epoch }, name, [])
      await rebuildResolution({ ...target, options: { ...copied.build_options, resolution } })
    } catch (reason) {
      setResolutionBuild(null)
      showToast(`副本 ${name} 未确认保存：${reason instanceof Error ? reason.message : String(reason)}`, 'error')
      await loadMaps()
    }
  }

  const filteredMaps = maps.filter(map => map.name.toLowerCase().includes(search.trim().toLowerCase()))
  const selectedInfo = maps.find(map => map.name === selectedMap)
  const selectedNavigationMap = selectedMap !== null && session !== null
    && navigationGoalQueueReady(session, selectedMap)
  const selectedStatus = mapUseStatus(selectedInfo, selectedNavigationMap)

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
        {selectedMap && <details className={styles.snapshotInfo} key={selectedMap}>
          <summary aria-label={`地图状态：${selectedStatus.label}`}>{selectedStatus.label}</summary>
          <div className={styles.snapshotPopover}>
            <strong>地图状态</strong>
            <p>{selectedStatus.detail}</p>
            {selectedInfo && <dl className={styles.snapshotArtifacts}>
              <div><dt>保存点云</dt><dd>{selectedInfo.has_pcd ? '已保存' : '缺失'}</dd></div>
              <div><dt>规划地图</dt><dd data-missing={!selectedInfo.has_octomap}>{selectedInfo.has_octomap ? '已保存' : '缺失'}</dd></div>
            </dl>}
            {!observe && selectedInfo?.has_pcd && selectedInfo.content_epoch && <>
              <label className={styles.resolutionOption}>全局导航地图分辨率
                <select aria-label="全局导航地图分辨率" value={resolution} onChange={event => setResolution(Number(event.target.value))}>
                  <option value={0.05}>5 cm</option>
                  <option value={0.06}>6 cm</option>
                  <option value={0.1}>10 cm</option>
                  <option value={0.2}>20 cm</option>
                </select>
              </label>
              <button className={styles.quietButton} disabled={resolutionBuild?.state === 'copying' || resolutionBuild?.state === 'building'}
                onClick={() => setResolutionCopyOpen(true)}>另存为此分辨率导航地图</button>
              <p>重建全局规划实际读取的地图，实时局部避障保持原有精度。原图保留，生成后需使用新地图导航。</p>
            </>}
            {!!selectedInfo?.activation_blockers?.length && <details>
              <summary>服务原因</summary>
              {selectedInfo.activation_blockers.map(blocker => <p key={blocker}>{blocker}</p>)}
            </details>}
            {saveStatus?.state === 'saved' && saveStatus.name === selectedMap && <details>
              <summary>保存详情</summary>
              <p>{saveStatus.detail}</p><p>{saveStatus.location}</p><p>{saveStatus.summary}</p>
            </details>}
          </div>
        </details>}
        <div className={styles.headerActions}>
          <button className={styles.quietButton} onClick={() => void refreshMapsAndPreview()}
            aria-label="刷新地图" title="服务重连后重新读取地图">
            <RefreshCw size={16} />刷新地图
          </button>
          {!observe && selectedMap && selectedInfo && mapIsActivationReady(selectedInfo) && !selectedNavigationMap && (
            <button className={styles.primaryButton} disabled={!productSwitchAllowed}
              title={productSwitchReason || '加载这张地图并切换导航'} onClick={() => setUseMapFrom(selectedMap)}>
              <Navigation size={16} />使用此地图导航
            </button>
          )}
          {!observe && selectedNavigationMap && <button className={styles.primaryButton}
            onClick={onReturnLive}><Navigation size={16} />在现场中导航</button>}
          {!observe && session?.product === 'map' && <button className={styles.primaryButton}
            onClick={handleSave} disabled={Boolean(saveBlockedReason) || saveStatus?.state === 'saving' || saveStatus?.state === 'pending'}
            title={saveBlockedReason || '保存当前建图并查看整图'}>
            <Save size={16} />{saveStatus?.state === 'saving' ? '保存中…' : '保存地图'}
          </button>}
        </div>
      </header>
      {productSwitchMessage && <div className={styles.saveNotice} role="status">{productSwitchMessage}</div>}
      {resolutionBuild && <div className={styles.saveNotice} role="status">
        <span>{resolutionBuild.message}</span>
        {resolutionBuild.state === 'failed' && <button className={styles.quietButton}
          onClick={() => void rebuildResolution(resolutionBuild)}>重试导航重建</button>}
        {(resolutionBuild.state === 'ready' || resolutionBuild.state === 'failed') && <button className={styles.iconButton}
          aria-label="关闭导航地图重建提示" onClick={() => setResolutionBuild(null)}><X size={16} /></button>}
      </div>}
      <PromptModal open={resolutionCopyOpen} title="生成新分辨率导航地图"
        message={`将 ${selectedMap ?? ''} 另存并重建为 ${resolution * 100} cm 导航地图。原地图和当前导航不受影响。`}
        initialValue={`${(selectedMap ?? 'map').slice(0, 88)}-${Math.round(resolution * 100)}cm`}
        validate={name => mapEditNameError(name, selectedMap ?? '')} confirmLabel="复制并重建"
        onConfirm={name => void createResolutionCopy(name)} onCancel={() => setResolutionCopyOpen(false)} />
      <ConfirmModal open={useMapFrom !== null} title="使用此地图导航"
        message={<>
          <p>{session?.product === 'map'
          ? '将结束本次建图并加载所选地图。请确认最新补扫已保存；随后在现场选择目标并预览路线。'
          : '将停止当前任务并加载所选地图，随后在现场选择目标并预览路线。'}</p>
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
              <button className={styles.quietButton} onClick={() => void loadMaps()}>重试</button>
            </div>}
            {!loading && !error && maps.length === 0 && <div className={styles.emptyLibrary}>
              <FolderOpen size={28} strokeWidth={1.3} /><p>还没有保存的地图</p>
            </div>}
            {!loading && <ul className={styles.mapList}>
              {filteredMaps.map(map => <MapCard key={map.name} m={map} selected={selectedMap === map.name}
                readOnly={observe}
                onPreview={togglePreview} onRename={handleRename} onDelete={handleDelete} />)}
            </ul>}
            {!loading && maps.length > 0 && filteredMaps.length === 0 && <p className={styles.stateMsg}>没有找到匹配的地图</p>}
          </div>
        </aside>}
        <div className={styles.mapCanvas}>
          {selectedMap && selectedInfo ? <PointCloudViewer mapName={selectedMap}
              contentEpoch={selectedInfo.content_epoch} productSessionId={session?.product_session_id}
              refreshToken={previewRefresh} /> : <div className={styles.emptyCanvas}>
              <Map size={36} strokeWidth={1.2} /><h2>查看已保存的地图</h2>
              <p>选择一张地图，查看完整建图范围。</p>
              {!libraryOpen && <button className={styles.quietButton} onClick={() => setLibraryOpen(true)}>打开地图库</button>}
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
