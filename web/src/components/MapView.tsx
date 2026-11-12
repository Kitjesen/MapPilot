import { useState, useEffect, useCallback, useRef } from 'react'
import { ArrowLeft, Ban, BoxSelect, BrushCleaning, Layers, MapPin, Navigation, Pencil, RefreshCw, Save, X } from 'lucide-react'
import { useMapSettings } from '../hooks/useMapSettings.ts'
import type { MapInfo, SessionEvent, ToastKind } from '../types'
import * as api from '../services/api'
import { formatMapSaveProgress, pendingMapSaveStatus, savedMapStatus, archivedMapStatus, failedMapSaveStatus, mapSaveProgressValue, mapSaveElapsedMs, formatMapSaveElapsed, mappingCapacityNotice, type MapSaveStatus } from '../services/mapSavePresentation.ts'
import { mapIsActivationReady, mapSaveBlockedReason, mapUseStatus, navigationGoalQueueReady } from '../services/mapReadiness'
import type { MapEditorTool } from './MapEditor.tsx'
import { PromptModal, ConfirmModal } from './Modal'
import { MapStatusIcon } from './MapLibrary.tsx'
import { MapGallery } from './MapGallery.tsx'
import { useMapSaves } from '../hooks/useMapSaves.ts'
import { mapLibraryEntries, mapLibraryStatus } from '../services/mapSavePresentation.ts'
import type { Locale } from '../i18n'
import { isObservationMode } from '../services/observationMode.ts'
import { parseInitialPose, type InitialPoseInput, type LocalizationInitialPose } from '../services/localizationInitialPose.ts'
import { mapEditNameError } from '../services/mapEdit.ts'
import styles from './MapView.module.css'

interface MapViewProps {
  initialSelectedMap: string | null
  onUseMap: (name: string, initialPose?: LocalizationInitialPose) => void
  productSwitchAllowed: boolean
  productSwitchReason: string
  onReturnLive: () => void
  returnLabel?: string
  onMapEditorExitGuardChange?: (guard: ((proceed: () => void) => void) | null) => void
  session: SessionEvent['data'] | null
  slamDiag: Record<string, unknown> | null
  showToast: (msg: string, kind?: ToastKind) => void
  locale: Locale
}
// ── Main ───────────────────────────────────────────────────────
export function MapView({
  initialSelectedMap,
  onUseMap,
  productSwitchAllowed,
  productSwitchReason,
  onReturnLive,
  returnLabel = '导航',
  onMapEditorExitGuardChange,
  session,
  slamDiag,
  showToast,
}: MapViewProps) {
  const observe = isObservationMode()
  const mapSettings = useMapSettings()
  const saveBlockedReason = mapSaveBlockedReason(session)
  const capacityNotice = mappingCapacityNotice(session?.product, slamDiag)
  const [maps,        setMaps       ] = useState<MapInfo[]>([])
  const [loading,     setLoading    ] = useState(true)
  const [error,       setError      ] = useState('')
  const [selectedMap, setSelectedMap] = useState<string | null>(initialSelectedMap)
  const selectedInfo = maps.find(map => map.name === selectedMap)
  const [previewRefresh, setPreviewRefresh] = useState(0)
  const [editTool, setEditTool] = useState<MapEditorTool | null>(null)
  const editing = editTool !== null
  const [managementAvailable, setManagementAvailable] = useState(false)
  const registerEditGuard = useCallback((guard: ((proceed: () => void) => void) | null) => {
    onMapEditorExitGuardChange?.(guard)
  }, [onMapEditorExitGuardChange])
  useEffect(() => {
    let active = true
    void api.fetchAppCapabilities().then(result => {
      if (active) setManagementAvailable(result.features.map_management === true)
    }, () => { if (active) setManagementAvailable(false) })
    return () => { active = false }
  }, [session?.product_session_id])

  // Modal state
  const [saveOpen,   setSaveOpen  ] = useState(false)
  const [autoProcess, setAutoProcess] = useState(true)
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
  const handleSave = () => { setAutoProcess(true); setSaveOpen(true) }

  const confirmDelete = async () => {
    const name = deleteFrom
    setDeleteFrom(null)
    if (!name) return
    try {
      await api.deleteMap(name); showToast(`已删除: ${name}`, 'success')
      if (selectedMap === name) setSelectedMap(null)
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
    } catch (cause) { showToast(cause instanceof Error ? cause.message : '重命名失败', 'error') }
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
      const admission = existing ?? await api.saveMap(name, autoProcess)
      const outcome = await api.waitForMapSaveOperation(admission, {
        until: 'source',
        onProgress: progress => {
          updateProgress(progress)
          setSaveStatus({ name, state: 'saving', detail: formatMapSaveProgress(progress) })
        },
      })
      if (outcome.state === 'pending') {
        setSaveStatus(pendingMapSaveStatus(outcome.result, outcome.reason))
        return
      }
      if (outcome.state === 'source_saved') {
        updateProgress(outcome.result)
        setSaveStatus(archivedMapStatus(outcome.result))
        showToast(`PCD 已保存 ${outcome.result.name}`, 'success')
        return
      }
      const r = outcome.result
      updateProgress(r)
      const savedName = r.name
      setSaveStatus(savedMapStatus(r))
      hasAutoSelected.current = true
      setSelectedMap(savedName)
      showToast(`已保存 ${savedName}`, 'success')
      if (r.warnings?.length) showToast(r.warnings.join('；'), 'info')
      await loadMaps()
      setPreviewRefresh(value => value + 1)
    }
    catch (e: unknown) {
      updateProgress()
      const message = e instanceof Error ? e.message : String(e)
      setSaveStatus(failedMapSaveStatus(name, e))
      showToast(`保存失败：${message}`, 'error')
    }
  }

  const nameValidator = (v: string) => {
    if (!/^[a-zA-Z0-9_-]+$/.test(v)) return '仅支持字母、数字、下划线和横线'
    if (v.length > 32) return '名称过长 (最多 32 字符)'
    return null
  }

  const selectMap = useCallback((name: string) => {
    hasAutoSelected.current = true
    setSelectedMap(name)
  }, [])

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

  const saves = useMapSaves({
    refreshKey: `${session?.product_session_id ?? ''}:${saveStatus?.state ?? ''}`,
    currentOperationId: saveStatus && 'operation' in saveStatus ? saveStatus.operation?.operation_id : undefined,
    onContinue: result => void confirmSave(result.name, result),
    onComplete: result => void confirmSave(result.name, result),
  })
  const entries = mapLibraryEntries(maps, saves.history.operations)
  const settingsRefresh = useCallback(() => {
    void loadMaps()
    setPreviewRefresh(value => value + 1)
  }, [loadMaps])
  const setSettingsMap = mapSettings?.setMap
  useEffect(() => {
    setSettingsMap?.(selectedInfo?.content_epoch ? { name: selectedInfo.name,
      contentEpoch: selectedInfo.content_epoch, active: selectedInfo.is_active === true,
      readOnly: observe || !managementAvailable, onChanged: settingsRefresh } : null)
    return () => setSettingsMap?.(null)
  }, [selectedInfo?.name, selectedInfo?.content_epoch, selectedInfo?.is_active,
    observe, managementAvailable, settingsRefresh, setSettingsMap])
  const selectedNavigationMap = selectedMap !== null && session !== null
    && navigationGoalQueueReady(session, selectedMap)
  const selectedStatus = mapUseStatus(selectedInfo, selectedNavigationMap)
  const selectedIcon = mapLibraryStatus(selectedInfo, entries.find(entry => entry.name === selectedMap)?.operations[0], Boolean(saves.error))
  const editAvailable = Boolean(managementAvailable && selectedInfo?.has_pcd && selectedInfo.content_epoch)

  return (
    <section className={styles.mapTab} data-gallery-open="true" aria-label="已保存地图">
      {!editing && <div className={styles.toolsBar} role="group" aria-label="地图工具">
        {!observe && <>
          <button type="button" aria-label="裁剪点云" data-tool="crop" disabled={!editAvailable}
            title={managementAvailable ? '框选三维范围，删除选中或仅保留选区' : '地图编辑服务未连接'}
            onClick={() => setEditTool('crop')}><BoxSelect size={16} /><span>裁剪</span></button>
          <button type="button" aria-label="清洗点云" data-tool="clean" disabled={!editAvailable || !mapSettings}
            title={managementAvailable ? '降采样、离群点过滤和小簇清理，预览后另存' : '地图编辑服务未连接'}
            onClick={() => setEditTool('clean')}><BrushCleaning size={16} /><span>清洗</span></button>
          <span className={styles.toolDivider} aria-hidden="true" />
        </>}
        <button type="button" aria-label="设置禁行区" data-tool="no-go" disabled={observe || !editAvailable || selectedInfo?.is_active}
          title={selectedInfo?.is_active ? '退出当前地图导航后修改区域' : '在地图上框选并命名禁行区或玻璃障碍'} onClick={() => setEditTool('region')}>
          <Ban size={16} /><span>禁区</span></button>
        <button type="button" disabled={observe || !editAvailable} aria-label="人工补面" data-tool="surface"
          title="框选地面或墙面缺口，预览人工补面后另存并重建" onClick={() => setEditTool('surface')}>
          <Layers size={16} /><span>补面</span></button>
        <button type="button" disabled={observe || !editAvailable} aria-label="编辑巡检点" data-tool="point"
          title="在地图设置巡检点、区域名称与到点动作" onClick={() => setEditTool('point')}>
          <MapPin size={16} /><span>巡检点</span></button>
      </div>}
      <header className={styles.mapHeader}>
        <button className={styles.backButton} onClick={onReturnLive}><ArrowLeft size={17} />{returnLabel}</button>
        <div className={styles.headerActions}>
          <button className={styles.quietButton} disabled={editing} onClick={() => void refreshMapsAndPreview()}
            aria-label="刷新地图" title="重新读取地图"><RefreshCw size={16} /></button>
          {!observe && !editing && session?.product === 'map' && <button className={styles.primaryButton}
            onClick={handleSave} disabled={Boolean(saveBlockedReason) || saveStatus?.state === 'saving' || saveStatus?.state === 'pending'}
            title={saveBlockedReason || '保存当前建图并查看整图'}>
            <Save size={16} />{saveStatus?.state === 'saving' ? '保存中…' : '保存地图'}
          </button>}
        </div>
      </header>
      {capacityNotice && <div className={styles.saveNotice} role="status">{capacityNotice}</div>}
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
          ? '将结束本次建图并加载所选地图。请确认最新补扫已保存；随后在导航页选择目标并预览路线。'
          : '将停止当前任务并加载所选地图，随后在导航页选择目标并预览路线。'}</p>
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
      {saveStatus && !['archived', 'saved'].includes(saveStatus.state) && <div
        className={`${styles.saveNotice} ${saveStatus.state === 'failed' ? styles.saveError : ''}`} role="status">
        <span>{saveStatus.name}：{saveStatus.state === 'failed' ? saveStatus.operation?.operation?.source_saved ? '处理失败：' : '保存失败：' : ''}{saveStatus.detail}</span>
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
        {(saveStatus.state === 'failed' || saveStatus.state === 'saved' || saveStatus.state === 'archived') && <button className={styles.iconButton}
          onClick={() => setSaveStatus(null)} aria-label="关闭保存提示"><X size={16} /></button>}
      </div>}
      <div className={styles.mapWorkspace}>
        <aside id="saved-map-library" className={styles.library} aria-label="地图库">
          <MapGallery key={previewRefresh} open entries={entries} selectedMap={selectedMap} saves={saves}
            loading={loading} error={error} readOnly={observe} onSelect={selectMap}
            onRename={handleRename} onDelete={handleDelete}
            onRefresh={() => { void refreshMapsAndPreview(); void saves.load() }}
            editing={editTool && !observe && managementAvailable ? {
              tool: editTool, onClose: () => setEditTool(null),
              onOpenMap: name => { setEditTool(null); selectMap(name); void loadMaps() },
              onMapsChanged: () => void loadMaps(), onExitGuardChange: registerEditGuard,
            } : null}
            actions={<>
                {selectedMap && <details className={styles.mapDetailsToggle} key={selectedMap} data-hover-menu>
                  <summary aria-label={`地图状态：${selectedIcon.label}`} title={selectedIcon.label} data-status={selectedIcon.kind}><MapStatusIcon kind={selectedIcon.kind} /></summary>
                  <div className={styles.mapDetailsBody}>
                    <strong>{selectedIcon.label}</strong>
                    <p>{selectedIcon.detail}</p>
                    {selectedNavigationMap && <p>{selectedStatus.detail}</p>}
                    {selectedInfo && <dl className={styles.snapshotArtifacts}>
                      <div><dt>保存点云</dt><dd>{selectedInfo.has_pcd ? '已保存' : '缺失'}</dd></div>
                      <div><dt>规划地图</dt><dd data-missing={!selectedInfo.has_octomap}>{selectedInfo.has_octomap ? '已保存' : '缺失'}</dd></div>
                    </dl>}
                    {!observe && !editing && selectedInfo?.has_pcd && selectedInfo.content_epoch && <>
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
                {!observe && !editing && selectedMap && selectedInfo && mapIsActivationReady(selectedInfo) && !selectedNavigationMap && (
                  <button className={styles.primaryButton} disabled={!productSwitchAllowed}
                    title={productSwitchReason || '加载这张地图并切换导航'} onClick={() => setUseMapFrom(selectedMap)}>
                    <Navigation size={16} />使用此地图导航
                  </button>
                )}
                {!observe && !editing && selectedNavigationMap && <button className={styles.primaryButton}
                  onClick={onReturnLive}><Navigation size={16} />返回{returnLabel}</button>}
            </>} />
        </aside>
      </div>

      <PromptModal
        open={saveOpen}
        title="保存地图"
        message={`${capacityNotice ? `${capacityNotice} ` : ''}先保存建图点云存档，可下载 PCD。处理成功后，同名会替换原地图；要保留原图请使用新名称。`}
        placeholder="例如 building_2f"
        confirmLabel="保存"
        icon={<Save size={18} />}
        validate={nameValidator}
        onConfirm={confirmSave}
        onCancel={() => setSaveOpen(false)}
      >
        <label style={{ display: 'flex', gap: 8, marginTop: 14, alignItems: 'flex-start' }}>
          <input type="checkbox" checked={autoProcess} onChange={event => setAutoProcess(event.target.checked)} />
          <span>保存点云后自动后台处理<br /><small>取消勾选后，点击地图库中的状态图标开始处理。</small></span>
        </label>
      </PromptModal>

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
