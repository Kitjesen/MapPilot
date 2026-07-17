import { useState, useEffect, useCallback, useRef } from 'react'
import { Map, FolderOpen, Trash2, Star, RefreshCw, Save, Pencil, Navigation } from 'lucide-react'
import type { MapInfo, ToastKind } from '../types'
import * as api from '../services/api'
import { mapIsNavigationReady, navigationRuntimeReady } from '../services/mapReadiness'
import { PointCloudViewer, type PointCloudPick } from './PointCloudViewer'
import { PromptModal, ConfirmModal } from './Modal'
import { text, type Locale } from '../i18n'
import styles from './MapView.module.css'

interface MapViewProps {
  showToast: (msg: string, kind?: ToastKind) => void
  locale: Locale
}
// ── Map type classification ────────────────────────────────────
// 可导航地图 — has PCD + OctoMap artifact (OctoPlanner3D ready)
// 三维点云  — has PCD, no OctoMap artifact (raw LiDAR map)
// 空地图    — no PCD

interface Group { label: string; hint: string; maps: MapInfo[] }

function groupMaps(maps: MapInfo[]): Group[] {
  return [
    {
      label: '可导航地图',
      hint: '含 OctoMap，可用于 OctoPlanner3D 规划',
      maps: maps.filter(mapIsNavigationReady),
    },
    {
      label: '三维点云',
      hint: '原始 LiDAR 点云，需构建 OctoMap 后才能导航',
      maps: maps.filter(m => m.has_pcd && !mapIsNavigationReady(m)),
    },
    {
      label: '空地图',
      hint: '尚未采集数据',
      maps: maps.filter(m => !m.has_pcd),
    },
  ].filter(g => g.maps.length > 0)
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
  const opt = r.map_optimization
  if (opt && typeof opt === 'object') {
    const strategy = String(opt.strategy ?? 'map').toUpperCase()
    const status = String(opt.status ?? '')
    if (status === 'ok') parts.push(`${strategy} 已优化`)
    else if (status === 'unavailable') parts.push(`${strategy} 未安装`)
    else if (status === 'skipped') parts.push(`${strategy} 已跳过`)
    else if (status === 'failed') parts.push(`${strategy} 失败`)
  }
  const df = r.dynamic_filter
  if (df && df.success && typeof df.dropped === 'number' && typeof df.orig_count === 'number' && df.orig_count > 0) {
    const pct = (100 * df.dropped / df.orig_count).toFixed(1)
    parts.push(`动态点清理 ${df.dropped}/${df.orig_count} (${pct}%)`)
  }
  if (r.size) parts.push(`大小 ${r.size}`)
  if (r.saved_map_relocalization_supported ?? r.relocalization_supported) {
    parts.push('支持重定位')
  }
  return parts.length ? parts.join(' · ') : '已写入地图列表，可在右侧选择加载。'
}

interface CardProps {
  m: MapInfo
  selected: boolean
  onPreview:  (name: string) => void
  onNavigate: (name: string) => void
  onActivate: (name: string) => void
  onRename:   (name: string) => void
  onDelete:   (name: string) => void
}
function MapCard({ m, selected, onPreview, onNavigate, onActivate, onRename, onDelete }: CardProps) {
  return (
    <div className={[
      m.is_active ? styles.cardActive : styles.card,
      selected    ? styles.cardSelected : '',
    ].filter(Boolean).join(' ')}>
      <div className={styles.cardInfo}>
        <span className={styles.mapName}>
          {m.is_active && <Star size={12} className={styles.starIcon} />}
          {m.name}
        </span>
        <span className={styles.meta}>
          {m.has_pcd      && <span className={styles.tagPcd}>PCD</span>}
          {m.has_octomap && <span className={styles.tagTomo}>OctoMap</span>}
          {!m.has_pcd && !m.has_octomap && <span className={styles.tagEmpty}>空</span>}
          {!!m.patch_count && <span className={styles.tagEmpty}>{m.patch_count} patches</span>}
          {!!m.size_mb    && <span className={styles.tagEmpty}>{m.size_mb.toFixed(1)} MB</span>}
        </span>
      </div>
      <div className={styles.cardActions}>
        {mapIsNavigationReady(m) && (
          <button
            className={styles.btnTinyAccent}
            onClick={() => onNavigate(m.name)}
            title="切换到导航模式并重定位"
          >
            <Navigation size={11} /> 导航
          </button>
        )}
        {m.has_pcd && (
          <button
            className={selected ? styles.btnTinyActive : styles.btnTiny}
            onClick={() => onPreview(m.name)}
            title="3D 点云预览"
          >预览</button>
        )}
        <button className={styles.btnTiny} onClick={() => onRename(m.name)} title="重命名">
          <Pencil size={11} />
        </button>
        {!m.is_active && (
          <button className={styles.btnTinyAccent} onClick={() => onActivate(m.name)} title="激活地图（需确认）">
            <Star size={11} /> 激活
          </button>
        )}
        <button className={styles.btnTinyDanger} onClick={() => onDelete(m.name)} title="删除">
          <Trash2 size={11} />
        </button>
      </div>
    </div>
  )
}

// ── Main ───────────────────────────────────────────────────────
export function MapView({ showToast, locale }: MapViewProps) {
  const [maps,        setMaps       ] = useState<MapInfo[]>([])
  const [loading,     setLoading    ] = useState(true)
  const [error,       setError      ] = useState('')
  const [selectedMap, setSelectedMap] = useState<string | null>(null)
  const [splitPct,    setSplitPct   ] = useState(30)
  const [pickedPoint, setPickedPoint] = useState<PointCloudPick | null>(null)

  // Modal state
  const [saveOpen,   setSaveOpen  ] = useState(false)
  const [activateFrom, setActivateFrom] = useState<string | null>(null)
  const [navigateFrom, setNavigateFrom] = useState<string | null>(null)
  const [navigationPendingMap, setNavigationPendingMap] = useState<string | null>(null)
  const [renameFrom, setRenameFrom] = useState<string | null>(null)
  const [deleteFrom, setDeleteFrom] = useState<string | null>(null)
  const [saveStatus, setSaveStatus] = useState<SaveStatus | null>(null)

  // Resizable divider
  const containerRef    = useRef<HTMLDivElement>(null)
  const divDragRef      = useRef<{ startX: number; startPct: number } | null>(null)
  const hasAutoSelected = useRef(false)

  useEffect(() => {
    const onMove = (e: MouseEvent) => {
      if (!divDragRef.current || !containerRef.current) return
      const rect = containerRef.current.getBoundingClientRect()
      const pct  = ((e.clientX - rect.left) / rect.width) * 100
      setSplitPct(Math.max(20, Math.min(72, pct)))
    }
    const onUp = () => { divDragRef.current = null }
    window.addEventListener('mousemove', onMove)
    window.addEventListener('mouseup',   onUp)
    return () => {
      window.removeEventListener('mousemove', onMove)
      window.removeEventListener('mouseup',   onUp)
    }
  }, [])

  const onDividerDown = (e: React.MouseEvent) => {
    e.preventDefault()
    divDragRef.current = { startX: e.clientX, startPct: splitPct }
  }

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
      setMaps([])
    } finally { setLoading(false) }
  }, [])

  useEffect(() => { loadMaps() }, [loadMaps])

  useEffect(() => { setPickedPoint(null) }, [selectedMap])

  const handleActivate = (name: string) => setActivateFrom(name)
  const handleNavigate = (name: string) => setNavigateFrom(name)
  const handleDelete = (name: string) => setDeleteFrom(name)
  const handleRename = (name: string) => setRenameFrom(name)
  const handleSave = () => setSaveOpen(true)

  const confirmActivate = async () => {
    const name = activateFrom
    setActivateFrom(null)
    if (!name) return
    try {
      await api.activateMap(name)
      showToast(`已激活: ${name}`, 'success')
      loadMaps()
    } catch {
      showToast(`激活失败: ${name}`, 'error')
    }
  }

  const ensureNavigationSession = async (mapName: string) => {
    const [session, navigation] = await Promise.all([
      api.fetchSession(),
      api.fetchNavigationStatus(),
    ])
    if (!navigationRuntimeReady(session, navigation, mapName)) {
      const blockers = navigation.readiness?.blockers?.join(', ')
      throw new Error(
        blockers || `地图 ${mapName} 尚未完成导航切换和重定位`,
      )
    }
  }

  const waitForNavigationSession = async (mapName: string) => {
    const deadline = Date.now() + 60_000
    let lastError = ''
    while (Date.now() < deadline) {
      try {
        const [session, navigation] = await Promise.all([
          api.fetchSession(),
          api.fetchNavigationStatus(),
        ])
        if (navigationRuntimeReady(session, navigation, mapName)) return
        lastError = navigation.readiness?.blockers?.join(', ')
          || session.error
          || session.relocalization_state
          || session.mode
      } catch (error) {
        lastError = error instanceof Error ? error.message : String(error)
      }
      await new Promise(resolve => window.setTimeout(resolve, 1_000))
    }
    throw new Error(lastError || '导航运行时在 60 秒内未就绪')
  }

  const confirmNavigate = async () => {
    const name = navigateFrom
    setNavigateFrom(null)
    if (!name || navigationPendingMap) return
    setNavigationPendingMap(name)
    try {
      const [currentSession, currentNavigation] = await Promise.all([
        api.fetchSession(),
        api.fetchNavigationStatus(),
      ])
      if (!navigationRuntimeReady(currentSession, currentNavigation, name)) {
        const result = await api.switchProductSession('navigating', {
          currentProfile: currentSession.product_profile,
          mapName: name,
        })
        showToast(`导航切换已受理，等待服务重启和重定位：${result.status}`, 'info')
        await waitForNavigationSession(name)
      }
      setSelectedMap(name)
      showToast(`导航已就绪：${name}`, 'success')
      await loadMaps()
    } catch (error) {
      showToast(`导航切换失败：${error instanceof Error ? error.message : String(error)}`, 'error')
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
      if (selectedMap === name) setSelectedMap(null); loadMaps()
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
    setSaveStatus({
      name,
      state: 'saving',
      detail: '正在写入点云、清理动态点并生成导航地图。完成后会显示保存位置。',
    })
    try {
      const r = await api.saveMap(name)
      const summary = formatSaveMapSummary(r)
      setSaveStatus({
        name,
        state: 'saved',
        detail: formatSaveMapDetail(r),
        location: formatSaveMapLocation(r, name),
        summary,
      })
      showToast(`已保存 ${name}。${summary}`, r.warnings?.length ? 'info' : 'success')
      loadMaps()
    }
    catch (e: unknown) {
      const message = e instanceof Error ? e.message : String(e)
      setSaveStatus({
        name,
        state: 'failed',
        detail: message || '保存失败，请检查 Gateway 日志。',
      })
      showToast('保存失败', 'error')
    }
  }

  const confirmPickedGoal = async () => {
    if (!pickedPoint || !selectedMap) return
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

  const togglePreview = (name: string) =>
    setSelectedMap(prev => prev === name ? null : name)

  const groups = groupMaps(maps)

  return (
    <div className={styles.mapTab} ref={containerRef}>
      {/* Left: 3D viewer */}
      <div className={styles.viewerPanel} style={{ width: `${splitPct}%` }}>
        <PointCloudViewer mapName={selectedMap} pickedPoint={pickedPoint} onPick={setPickedPoint} />
      </div>

      {/* Drag divider */}
      <div
        className={styles.divider}
        onMouseDown={onDividerDown}
        title="拖拽调整宽度"
      />

      {/* Right: categorized map list */}
      <div className={styles.listPanel}>
        <div className={styles.listHeader}>
          <h2 className={styles.listTitle}><Map size={15} /> {text(locale, 'Map Management', '地图管理')}</h2>
          <div className={styles.listActions}>
            <button className={styles.btnSmallAccent} onClick={handleSave}>
              <Save size={13} /> 保存当前地图
            </button>
            <button className={styles.btnSmall} onClick={loadMaps}>
              <RefreshCw size={13} /> 刷新
            </button>
          </div>
        </div>

        {saveStatus && (
          <div
            className={[
              styles.saveStatusCard,
              saveStatus.state === 'saving' ? styles.saveStatusBusy : '',
              saveStatus.state === 'failed' ? styles.saveStatusError : '',
            ].filter(Boolean).join(' ')}
            title={saveStatus.summary ?? saveStatus.detail}
          >
            <div className={styles.saveStatusHeader}>
              <span>{saveStatus.state === 'saving' ? '保存进度' : saveStatus.state === 'saved' ? '保存结果' : '保存失败'}</span>
              <span className={styles.saveStatusName}>{saveStatus.name}</span>
            </div>
            <div className={styles.saveStatusDetail}>{saveStatus.detail}</div>
            {saveStatus.state === 'saving' && (
              <div className={styles.saveProgressBar} aria-label="保存进行中">
                <span />
              </div>
            )}
            {saveStatus.location && (
              <div className={styles.saveLocation}>
                <span>位置</span>
                <code>{saveStatus.location}</code>
              </div>
            )}
            {saveStatus.summary && (
              <div className={styles.saveSummary}>{saveStatus.summary}</div>
            )}
          </div>
        )}

        {pickedPoint && (
          <div className={styles.pickPanel}>
            <div className={styles.pickInfo}>
              <span className={styles.pickTitle}>3D 目标点</span>
              <span className={styles.pickCoords}>
                x={pickedPoint.x.toFixed(2)} y={pickedPoint.y.toFixed(2)} z={pickedPoint.z.toFixed(2)}
              </span>
            </div>
            <div className={styles.pickActions}>
              <button className={styles.btnTinyAccent} onClick={confirmPickedGoal}>发送目标</button>
              <button className={styles.btnTiny} onClick={() => setPickedPoint(null)}>取消</button>
            </div>
          </div>
        )}

        <div className={styles.listScroll}>
          {loading && <div className={styles.stateMsg}>加载中...</div>}
          {error   && (
            <div className={styles.stateMsg}>
              <p>{error}</p>
              <button className={styles.btnSmall} onClick={loadMaps}>
                <RefreshCw size={13} /> 重试
              </button>
            </div>
          )}
          {!loading && !error && maps.length === 0 && (
            <div className={styles.stateMsg}>
              <FolderOpen size={40} strokeWidth={1} />
              <p>暂无保存的地图</p>
            </div>
          )}

          {!loading && groups.map(g => (
            <div key={g.label} className={styles.group}>
              <div className={styles.groupHeader}>
                <span className={styles.groupLabel}>{g.label}</span>
                <span className={styles.groupCount}>{g.maps.length}</span>
                <span className={styles.groupHint}>{g.hint}</span>
              </div>
              <div className={styles.list}>
                {g.maps.map(m => (
                  <MapCard
                    key={m.name} m={m} selected={selectedMap === m.name}
                    onPreview={togglePreview} onNavigate={handleNavigate} onActivate={handleActivate}
                    onRename={handleRename}   onDelete={handleDelete}
                  />
                ))}
              </div>
            </div>
          ))}
        </div>
      </div>

      <PromptModal
        open={saveOpen}
        title="保存当前地图"
        message="保存当前 SLAM 建图结果。完成后会出现在地图列表，并在“保存结果”显示保存位置和处理摘要。"
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
        title="进入导航模式"
        message={`将使用“${navigateFrom ?? ''}”执行完整导航切换：重启产品服务、加载 OctoMap 并进行保存地图重定位。切换成功本身不会移动机器人，发送目标后才会运动。`}
        confirmLabel="切换并重定位"
        onConfirm={confirmNavigate}
        onCancel={() => setNavigateFrom(null)}
      />

      <ConfirmModal
        open={activateFrom != null}
        title="激活地图"
        message={`将 "${activateFrom ?? ''}" 设为当前活动地图？此操作不会移动机器人，但会切换后续定位/导航使用的地图。`}
        confirmLabel="激活"
        onConfirm={confirmActivate}
        onCancel={() => setActivateFrom(null)}
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
    </div>
  )
}
