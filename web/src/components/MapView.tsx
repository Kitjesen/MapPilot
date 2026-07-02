import { useState, useEffect, useCallback, useRef } from 'react'
import { Map, FolderOpen, Trash2, Star, RefreshCw, Save, Pencil } from 'lucide-react'
import type { MapInfo, ToastKind } from '../types'
import * as api from '../services/api'
import { PointCloudViewer, type PointCloudPick } from './PointCloudViewer'
import { PromptModal, ConfirmModal } from './Modal'
import styles from './MapView.module.css'

interface MapViewProps {
  showToast: (msg: string, kind?: ToastKind) => void
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
      maps: maps.filter(m => m.has_pcd && (m.navigation_ready === true || m.has_octomap === true)),
    },
    {
      label: '三维点云',
      hint: '原始 LiDAR 点云，需构建 OctoMap 后才能导航',
      maps: maps.filter(m => m.has_pcd && !(m.navigation_ready === true || m.has_octomap === true)),
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
  const relocText = savedMapReloc === undefined ? 'unknown' : savedMapReloc ? 'yes' : 'no'
  const recovery = r.restart_recovery_supported === undefined
    ? (r.recovery_method ?? 'unknown')
    : `${r.restart_recovery_supported ? 'restart' : 'no-restart'}${r.recovery_method ? `/${r.recovery_method}` : ''}`
  const warnings = r.warnings?.filter(Boolean) ?? []
  return [
    `source=${source}`,
    `saved-map relocalize=${relocText}`,
    `recovery=${recovery}`,
    warnings.length > 0 ? `warnings=${warnings.join('; ')}` : null,
  ].filter((v): v is string => Boolean(v)).join(' | ')
}

interface CardProps {
  m: MapInfo
  selected: boolean
  onPreview:  (name: string) => void
  onActivate: (name: string) => void
  onRename:   (name: string) => void
  onDelete:   (name: string) => void
}
function MapCard({ m, selected, onPreview, onActivate, onRename, onDelete }: CardProps) {
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
export function MapView({ showToast }: MapViewProps) {
  const [maps,        setMaps       ] = useState<MapInfo[]>([])
  const [loading,     setLoading    ] = useState(true)
  const [error,       setError      ] = useState('')
  const [selectedMap, setSelectedMap] = useState<string | null>(null)
  const [splitPct,    setSplitPct   ] = useState(30)
  const [pickedPoint, setPickedPoint] = useState<PointCloudPick | null>(null)

  // Modal state
  const [saveOpen,   setSaveOpen  ] = useState(false)
  const [activateFrom, setActivateFrom] = useState<string | null>(null)
  const [renameFrom, setRenameFrom] = useState<string | null>(null)
  const [deleteFrom, setDeleteFrom] = useState<string | null>(null)
  const [lastSaveSummary, setLastSaveSummary] = useState<string | null>(null)

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
    try {
      const r = await api.saveMap(name)
      const summary = formatSaveMapSummary(r)
      setLastSaveSummary(summary)
      showToast(`Saved ${name}. ${summary}`, r.warnings?.length ? 'info' : 'success')
      loadMaps()
    }
    catch {
      setLastSaveSummary(null)
      showToast('Save failed', 'error')
    }
  }

  const confirmPickedGoal = async () => {
    if (!pickedPoint) return
    try {
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
      showToast(api.formatCommandError(e, '3D point goal failed'), 'error')
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
          <h2 className={styles.listTitle}><Map size={15} /> 地图管理</h2>
          <div className={styles.listActions}>
            <button className={styles.btnSmallAccent} onClick={handleSave}>
              <Save size={13} /> 保存当前地图
            </button>
            <button className={styles.btnSmall} onClick={loadMaps}>
              <RefreshCw size={13} /> 刷新
            </button>
          </div>
        </div>

        {lastSaveSummary && (
          <div className={styles.stateMsg} title={lastSaveSummary}>
            Last save: {lastSaveSummary}
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
                    onPreview={togglePreview} onActivate={handleActivate}
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
        message="保存当前 SLAM 建图结果。系统会自动生成导航所需的 OctoMap 和 occupancy 数据。"
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
