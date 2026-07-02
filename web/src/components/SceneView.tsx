import { useRef, useEffect, useCallback, useState, memo } from 'react'
import {
  Compass, Grid3x3, Navigation, Route, Target, Bot, Layers as LayersIcon,
  PanelLeftClose, PanelLeftOpen, Save, Trash2, StopCircle, Pencil, X,
  MapPinned, Cloud, Maximize2, Radio, Activity, LocateFixed, VideoOff,
} from 'lucide-react'
import type {
  SSEState,
  MapInfo,
  PathPoint,
  ToastKind,
  SlamProfile,
  LocationEntry,
  PlanPreviewResponse,
  MapLifecycleResponse,
} from '../types'
import * as api from '../services/api'
import { useCamera } from '../hooks/useCamera'
import { useBinaryCloud } from '../hooks/useBinaryCloud'
import { PromptModal, ConfirmModal } from './Modal'
import { Scene3D, type Scene3DHandle } from './Scene3D'
import styles from './SceneView.module.css'

interface SceneViewProps {
  sseState:  SSEState
  showToast: (msg: string, kind?: ToastKind) => void
}

// ── Layer flags ────────────────────────────────────────────────
interface Layers {
  grid:    boolean
  cloud:   boolean
  trail:   boolean
  path:    boolean
  goal:    boolean
  robot:   boolean
  costmap: boolean
  slope:   boolean
}

const TRAIL_MAX = 300
const GOAL_SPEED_OPTIONS = [0.25, 0.4, 0.6]
const GOAL_RADIUS_OPTIONS = [0.25, 0.45, 0.8]

const MAP_GROUPS: Array<{ label: string; filter: (m: MapInfo) => boolean }> = [
  { label: '可导航地图', filter: m => m.has_pcd && (m.navigation_ready === true || m.has_octomap === true) },
  { label: '三维点云', filter: m => m.has_pcd && !(m.navigation_ready === true || m.has_octomap === true) },
  { label: '空地图',   filter: m => !m.has_pcd },
]

type WorkbenchZoneState = 'preblocked' | 'traversable' | 'clear'

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

function formatPlanSafetySummary(preview: PlanPreviewResponse | null | undefined): string {
  if (!preview) return ''
  const safety = preview.path_safety ?? {}
  const planner = preview.selected_planner ?? preview.planner
  const safetyOk = safety.ok === true ? 'ok' : safety.ok === false ? 'blocked' : ''
  const blocked = typeof safety.blocked_sample_count === 'number'
    ? `blocked=${safety.blocked_sample_count}`
    : null
  const clearance = typeof safety.min_clearance_m === 'number'
    ? `clearance=${safety.min_clearance_m.toFixed(2)}m`
    : null
  return [
    planner ? `planner=${planner}` : null,
    preview.plan_safety_policy ? `policy=${preview.plan_safety_policy}` : null,
    safetyOk ? `safety=${safetyOk}` : null,
    blocked,
    clearance,
    preview.fallback_reason ? `fallback=${preview.fallback_reason}` : null,
    preview.rejected_plans?.length ? `rejected=${preview.rejected_plans.length}` : null,
  ].filter((v): v is string => Boolean(v)).join(' | ')
}

function formatMapLifecycleSummary(r: MapLifecycleResponse): string {
  const status = r.ok || r.success ? 'ok' : 'failed'
  const name = r.name ?? r.map_id ?? r.active ?? ''
  const message = typeof r.message === 'string' && r.message.trim() ? r.message.trim() : ''
  const ready = r.navigation_ready === true ? 'nav-ready' : r.navigation_ready === false ? 'not nav-ready' : ''
  return [status, name, ready, message].filter(Boolean).join(' | ')
}

function parseWorkbenchBounds(text: string): Record<string, unknown> {
  const raw = text.trim()
  if (!raw) throw new Error('bounds JSON is empty')
  const value = JSON.parse(raw) as unknown
  if (!value || typeof value !== 'object' || Array.isArray(value)) {
    throw new Error('bounds must be a JSON object')
  }
  return value as Record<string, unknown>
}

function formatPlanPreviewFailure(
  preview: PlanPreviewResponse | null | undefined,
  reasons: string[] = [],
  error?: string | null,
): string {
  const safety = formatPlanSafetySummary(preview)
  const reason = reasons.slice(0, 3).join(' / ') || error || preview?.error || 'plan preview rejected'
  return safety ? `${reason} (${safety})` : reason
}

function SceneViewComponent({ sseState, showToast }: SceneViewProps) {
  const scene3DRef = useRef<Scene3DHandle>(null)

  // Trail: state so Scene3D re-renders on movement
  // Persist trail in sessionStorage so a page refresh or tab re-open doesn't
  // erase the last N minutes of track. Cleared on explicit 清除轨迹 click
  // or end of browser session. Keyed per active map so switching maps
  // doesn't carry over the wrong trail.
  const trailStorageKey = (map: string | null | undefined) =>
    `lingtu.trail.${map ?? 'none'}`
  const [trail, setTrail] = useState<Array<[number, number]>>(() => {
    try {
      // We don't yet know the map here; lazy-load on mount via useEffect below.
      return []
    } catch { return [] }
  })
  const prevTrailEndRef = useRef<[number, number] | null>(null)

  const [slamPending, setSlamPending] = useState<SlamProfile | null>(null)
  const [drawerOpen, setDrawerOpen] = useState(true)
  const [saveModalOpen, setSaveModalOpen] = useState(false)
  const [layers, setLayers] = useState<Layers>({
    grid: true, cloud: true, trail: true, path: true, goal: true, robot: true, costmap: true, slope: false,
  })
  const [maps, setMaps] = useState<MapInfo[]>([])
  // Default 0.12 (12cm sphere per point) — with ~18k points in a room-scale
  // scene this gives a visually dense cloud. User can shrink via slider.
  const [pointSize, setPointSize] = useState(0.12)
  const [savedMapFlat, setSavedMapFlat] = useState<number[] | undefined>(undefined)
  const [relocOpen, setRelocOpen] = useState(false)
  const [relocDropOpen, setRelocDropOpen] = useState(false)
  const [relocMap, setRelocMap] = useState('')
  const relocDropRef = useRef<HTMLDivElement>(null)
  const [relocX, setRelocX] = useState('0')
  const [relocY, setRelocY] = useState('0')
  const [relocYaw, setRelocYaw] = useState('0')
  const [relocPending, setRelocPending] = useState(false)
  const [lastSaveSummary, setLastSaveSummary] = useState<string | null>(null)
  // Track whether the user has manually edited reloc inputs; until then we
  // mirror live odometry so the defaults reflect the robot's current pose
  // instead of the unhelpful (0, 0, 0).
  const [relocDirty, setRelocDirty] = useState(false)
  const [pendingGoal, setPendingGoal] = useState<{ x: number; y: number } | null>(null)
  const [pendingGoalPreview, setPendingGoalPreview] = useState<PlanPreviewResponse | null>(null)
  const [goalMaxSpeed, setGoalMaxSpeed] = useState(0.4)
  const [goalAcceptanceRadius, setGoalAcceptanceRadius] = useState(0.45)
  const [locationName, setLocationName] = useState('')
  const [locationBusy, setLocationBusy] = useState<string | null>(null)
  const [locationDeleteTarget, setLocationDeleteTarget] = useState<LocationEntry | null>(null)
  const [locationsOverride, setLocationsOverride] = useState<LocationEntry[] | null>(null)
  const [workbenchMapName, setWorkbenchMapName] = useState('')
  const [workbenchImportPath, setWorkbenchImportPath] = useState('')
  const [workbenchVoxelSize, setWorkbenchVoxelSize] = useState('0.10')
  const [workbenchBoundsJson, setWorkbenchBoundsJson] = useState(
    '{"min_x":-5,"max_x":5,"min_y":-5,"max_y":5,"min_z":-1,"max_z":2}',
  )
  const [workbenchZoneState, setWorkbenchZoneState] = useState<WorkbenchZoneState>('preblocked')
  const [workbenchZoneRadius, setWorkbenchZoneRadius] = useState('0.5')
  const [workbenchBusy, setWorkbenchBusy] = useState<string | null>(null)
  const [workbenchSummary, setWorkbenchSummary] = useState<string | null>(null)

  // Map management modals
  const [mapContextMenu, setMapContextMenu] = useState<{ name: string; x: number; y: number } | null>(null)
  const [deleteTarget, setDeleteTarget] = useState<string | null>(null)
  const [renameTarget, setRenameTarget] = useState<string | null>(null)
  const [loadTarget, setLoadTarget] = useState<string | null>(null)
  // SLAM switch confirmation
  const [slamSwitchTarget, setSlamSwitchTarget] = useState<SlamProfile | null>(null)

  const { imgSrc: cameraImgSrc, connected: cameraConnected, lastFrameAt: cameraLastFrameAt } = useCamera()
  const cameraPipRecovered = cameraImgSrc != null && cameraLastFrameAt != null
  const cameraPipLabel = cameraPipRecovered ? '相机已恢复' : cameraConnected ? '等待画面' : '无信号'
  const cameraPipDotClass = cameraPipRecovered ? styles.camDotLive : cameraConnected ? styles.camDotWait : styles.camDotOff
  // Binary point-cloud channel (replaces SSE JSON map_cloud).  The hook
  // owns the WebSocket + decoder worker; we just consume the latest frame.
  const cloud = useBinaryCloud()

  const rawPath = sseState.globalPath?.points ?? []
  const path = rawPath.filter(
    (p): p is PathPoint =>
      p != null && typeof p.x === 'number' && typeof p.y === 'number'
  )
  const rawLocalPath = sseState.localPath?.points ?? []
  const localPathPts = rawLocalPath.filter(
    (p): p is PathPoint =>
      p != null && typeof p.x === 'number' && typeof p.y === 'number'
  )
  const odom   = sseState.odometry
  // Sanity filter: reject absurd values (SLAM may emit garbage when lost).
  // Reasonable bounds: |pos| < 10km, |vel| < 10 m/s (quadruped max ~3 m/s).
  const sanePos = (v: unknown): number =>
    typeof v === 'number' && Number.isFinite(v) && Math.abs(v) < 10000 ? v : 0
  const saneVel = (v: unknown): number =>
    typeof v === 'number' && Number.isFinite(v) && Math.abs(v) < 10 ? v : 0
  const saneYaw = (v: unknown): number =>
    typeof v === 'number' && Number.isFinite(v) ? v : 0

  const robotX = sanePos(odom?.x)
  const robotY = sanePos(odom?.y)
  const yaw    = saneYaw(odom?.yaw)
  const vx     = saneVel(odom?.vx)
  const odomValid = odom != null && sanePos(odom.x) === odom.x && sanePos(odom.y) === odom.y

  const missionState = sseState.missionStatus?.state ?? 'IDLE'
  const missionGoal  = sseState.missionStatus?.goal
  const hasGoal      = missionState === 'EXECUTING' || missionState === 'PLANNING'
  const slamMode     = sseState.slamStatus?.mode ?? '—'
  const slamHz       = sseState.slamStatus?.slam_hz ?? 0
  const session = sseState.session
  const localizationBackend = session?.localization_backend ?? session?.slam_profile ?? sseState.slamStatus?.mode ?? 'unknown'
  const savedMapRelocalizeSupported =
    session?.saved_map_relocalization_supported ??
    session?.relocalization_supported ??
    localizationBackend === 'localizer'
  const recoveryMethod = session?.recovery_method ?? '--'
  const relocalizeUnavailableMessage =
    `Backend ${localizationBackend} does not support saved-map relocalize; recovery=${recoveryMethod}`
  const navigationStatus = sseState.navigationStatus
  const activeCmdSource = navigationStatus?.control?.active_cmd_source ?? 'none'
  const activeCmdBlocksGoal = activeCmdSource !== '' && activeCmdSource !== 'none'
  const canAcceptGoal =
    navigationStatus?.readiness?.can_accept_goal ??
    navigationStatus?.can_accept_goal ??
    false
  const goalBlockers = [
    ...(navigationStatus?.readiness?.blockers ?? []),
    ...(navigationStatus?.feedback?.blockers ?? []),
    activeCmdBlocksGoal
      ? `当前控制源: ${navigationStatus?.control?.active_source?.label ?? activeCmdSource}`
      : null,
    !odomValid ? '无有效里程计' : null,
  ].filter((v): v is string => Boolean(v))
  const goalDisabledReason =
    canAcceptGoal && !activeCmdBlocksGoal
      ? ''
      : goalBlockers.slice(0, 4).join(' · ') || '导航未 ready，暂不允许下发目标'
  const canSendGoal = goalDisabledReason === ''
  const pendingGoalPlanSummary = formatPlanSafetySummary(pendingGoalPreview)
  const savedLocations = locationsOverride ?? sseState.locations?.locations ?? []
  const normalizedLocationName = locationName.trim()
  // Legacy SSE map_cloud now carries only metadata (count/seq) — points
  // are streamed over /ws/cloud and live in `cloud.positions`.

  // ── Load map list ─────────────────────────────────────────────
  const loadMaps = useCallback(async () => {
    try {
      const data = await api.fetchMaps()
      setMaps(data)
    } catch { /* noop */ }
  }, [])

  useEffect(() => { loadMaps() }, [loadMaps])

  useEffect(() => {
    setLocationsOverride(null)
  }, [sseState.locations?.count, sseState.locations?.ts])

  // Close dropdown on outside click
  useEffect(() => {
    if (!relocDropOpen) return
    const handler = (e: MouseEvent) => {
      if (relocDropRef.current && !relocDropRef.current.contains(e.target as Node)) {
        setRelocDropOpen(false)
      }
    }
    document.addEventListener('mousedown', handler)
    return () => document.removeEventListener('mousedown', handler)
  }, [relocDropOpen])

  // ── Trail tracking ────────────────────────────────────────────
  const activeMap = sseState.session?.active_map
  // Load persisted trail when active_map changes (or first mount).
  useEffect(() => {
    if (!activeMap) return
    try {
      const raw = sessionStorage.getItem(trailStorageKey(activeMap))
      if (!raw) return
      const parsed = JSON.parse(raw) as Array<[number, number]>
      if (Array.isArray(parsed)) {
        // Filter: finite + within plausible map bounds (|xy| < 100m).
        // Older sessions captured odom-frame divergence points (±1000m)
        // which, when re-played as a line string, drew a huge spiderweb
        // across the scene. This bound prunes them on load.
        const clean = parsed.filter(
          (p): p is [number, number] =>
            Array.isArray(p) && p.length === 2 &&
            Number.isFinite(p[0]) && Number.isFinite(p[1]) &&
            Math.abs(p[0]) < 100 && Math.abs(p[1]) < 100
        ).slice(-TRAIL_MAX)
        setTrail(clean)
        if (clean.length > 0) prevTrailEndRef.current = clean[clean.length - 1]
      }
    } catch { /* ignore */ }
  }, [activeMap])

  useEffect(() => {
    if (odom == null) return
    const last = prevTrailEndRef.current
    if (!last || Math.hypot(odom.x - last[0], odom.y - last[1]) > 0.05) {
      prevTrailEndRef.current = [odom.x, odom.y]
      setTrail(prev => {
        const next = [...prev, [odom.x, odom.y] as [number, number]]
        return next.length > TRAIL_MAX ? next.slice(next.length - TRAIL_MAX) : next
      })
    }
  }, [odom])

  // Persist trail on change (throttle: only every ~1 s to keep sessionStorage
  // writes cheap on long runs).
  const trailSaveThrottleRef = useRef(0)
  useEffect(() => {
    if (!activeMap) return
    const now = Date.now()
    if (now - trailSaveThrottleRef.current < 1000) return
    trailSaveThrottleRef.current = now
    try {
      sessionStorage.setItem(trailStorageKey(activeMap), JSON.stringify(trail))
    } catch { /* quota hit? ignore */ }
  }, [trail, activeMap])

  // ── Sync reloc inputs with odometry until user edits ──────────
  // When the panel is closed (or user hasn't edited yet) keep X/Y/Yaw mirroring
  // current odom so opening it shows useful defaults.  Once the user edits any
  // field we stop overwriting (relocDirty=true).
  useEffect(() => {
    if (relocDirty || !odom) return
    setRelocX(robotX.toFixed(2))
    setRelocY(robotY.toFixed(2))
    setRelocYaw(yaw.toFixed(3))
  }, [odom, robotX, robotY, yaw, relocDirty])

  // ── Default active map for reloc panel ─────────────────────────
  const activeMapName = sseState.session?.active_map ?? null
  const workbenchTargetMapName = workbenchMapName.trim() || activeMapName || ''
  useEffect(() => {
    if (!relocMap && activeMapName) setRelocMap(activeMapName)
  }, [activeMapName, relocMap])

  // ── Handlers ──────────────────────────────────────────────────
  const handlePendingGoal = useCallback(async (x: number, y: number) => {
    if (!canSendGoal) {
      setPendingGoal({ x, y })
      setPendingGoalPreview(null)
      showToast(`Map point selected; navigation blocked: ${goalDisabledReason}`, 'info')
      return
    }
    try {
      const candidate = await api.constructGoalCandidate({
        x,
        y,
        source: 'map_click',
        target_type: 'map_point',
        label: 'scene_click',
        acceptance_radius_m: goalAcceptanceRadius,
        max_speed_mps: goalMaxSpeed,
      })
      if (!candidate.ok || candidate.preview?.feasible === false) {
        const reason = candidate.reasons.slice(0, 3).join(' / ') || candidate.error || '目标预检未通过'
        showToast(`目标不可达: ${reason}`, 'error')
        return
      }
      const target = candidate.target
      setPendingGoal({ x: target?.x ?? x, y: target?.y ?? y })
      setPendingGoalPreview(candidate.preview ?? null)
    } catch (e: unknown) {
      showToast(`目标预检失败: ${e instanceof Error ? e.message : String(e)}`, 'error')
    }
  }, [canSendGoal, goalAcceptanceRadius, goalDisabledReason, goalMaxSpeed, showToast])

  const handleSceneRelocalize = useCallback(async (x: number, y: number) => {
    if (!savedMapRelocalizeSupported) {
      showToast(relocalizeUnavailableMessage, 'error')
      return
    }
    const mapName = sseState.session?.active_map
    if (!mapName) {
      showToast('请先加载一张地图后再重定位', 'error')
      return
    }
    const useYaw = typeof odom?.yaw === 'number' && Number.isFinite(odom.yaw) ? odom.yaw : 0
    showToast(`重定位中… (${x.toFixed(2)}, ${y.toFixed(2)})`, 'info')
    try {
      await api.relocalize(mapName, x, y, useYaw)
      const q = sseState.session?.icp_quality
      const qStr = typeof q === 'number' ? ` quality=${q.toFixed(2)}` : ''
      showToast(`重定位成功${qStr}`, 'success')
    } catch (e: unknown) {
      showToast(`重定位失败: ${e instanceof Error ? e.message : String(e)}`, 'error')
    }
  }, [savedMapRelocalizeSupported, relocalizeUnavailableMessage, sseState.session, odom, showToast])

  const handleConfirmGoal = useCallback(async () => {
    if (!pendingGoal) return
    if (!canSendGoal) {
      showToast(`不能下发目标: ${goalDisabledReason}`, 'error')
      return
    }
    const { x, y } = pendingGoal
    try {
      const res = await api.sendGoal(x, y, {
        source: 'map_click',
        target_type: 'map_point',
        label: 'scene_click',
        acceptance_radius_m: goalAcceptanceRadius,
        max_speed_mps: goalMaxSpeed,
      })
      setPendingGoal(null)
      setPendingGoalPreview(null)
      showToast(api.formatCommandAck(res, `目标 (${x.toFixed(2)}, ${y.toFixed(2)})`), 'success')
    } catch (e: unknown) {
      showToast(api.formatCommandError(e, '发送目标失败'), 'error')
    }
  }, [canSendGoal, goalAcceptanceRadius, goalDisabledReason, goalMaxSpeed, pendingGoal, showToast])

  const handleSaveCurrentLocation = useCallback(async () => {
    if (!odomValid) {
      showToast('No valid odometry for saving a location', 'error')
      return
    }
    if (!normalizedLocationName) {
      showToast('Location name is required', 'error')
      return
    }
    setLocationBusy('save')
    try {
      const res = await api.saveLocation({
        name: normalizedLocationName,
        x: robotX,
        y: robotY,
        z: 0,
        yaw,
        tags: ['web'],
        source: 'web',
      })
      if (!res.ok) throw new Error(res.message || res.error || res.status)
      setLocationsOverride(res.locations.locations)
      setLocationName('')
      showToast(`Saved location ${normalizedLocationName}`, 'success')
    } catch (e: unknown) {
      showToast(`Save location failed: ${e instanceof Error ? e.message : String(e)}`, 'error')
    } finally {
      setLocationBusy(null)
    }
  }, [normalizedLocationName, odomValid, robotX, robotY, yaw, showToast])

  const handleNavigateLocation = useCallback(async (loc: LocationEntry) => {
    if (!canSendGoal) {
      showToast(`Cannot send goal: ${goalDisabledReason}`, 'error')
      return
    }
    setLocationBusy(`nav:${loc.name}`)
    try {
      const candidate = await api.constructGoalCandidate({
        x: loc.x,
        y: loc.y,
        z: loc.z,
        yaw: loc.yaw ?? undefined,
        source: 'saved_location',
        target_type: 'saved_location',
        label: loc.name,
        location_name: loc.name,
        acceptance_radius_m: goalAcceptanceRadius,
        max_speed_mps: goalMaxSpeed,
      })
      if (!candidate.ok || candidate.preview?.feasible === false) {
        const reason = formatPlanPreviewFailure(
          candidate.preview,
          candidate.reasons,
          candidate.error,
        )
        throw new Error(reason)
      }
      const target = candidate.target
      const res = await api.sendGoal(target?.x ?? loc.x, target?.y ?? loc.y, {
        z: target?.z ?? loc.z,
        yaw: target?.yaw ?? loc.yaw ?? 0,
        source: 'saved_location',
        target_type: 'saved_location',
        label: loc.name,
        acceptance_radius_m: goalAcceptanceRadius,
        max_speed_mps: goalMaxSpeed,
        metadata: { location_name: loc.name },
      })
      showToast(api.formatCommandAck(res, `Goal ${loc.name}`), 'success')
    } catch (e: unknown) {
      showToast(`Location goal failed: ${e instanceof Error ? e.message : String(e)}`, 'error')
    } finally {
      setLocationBusy(null)
    }
  }, [canSendGoal, goalAcceptanceRadius, goalDisabledReason, goalMaxSpeed, showToast])

  const handleUpdateLocationToCurrent = useCallback(async (loc: LocationEntry) => {
    if (!odomValid) {
      showToast('No valid odometry for updating a location', 'error')
      return
    }
    setLocationBusy(`update:${loc.name}`)
    try {
      const res = await api.updateLocation(loc.name, {
        name: loc.name,
        x: robotX,
        y: robotY,
        z: loc.z ?? 0,
        yaw,
        tags: loc.tags,
        source: 'web',
        metadata: { ...(loc.metadata ?? {}), updated_from: 'web_current_pose' },
      })
      if (!res.ok) throw new Error(res.message || res.error || res.status)
      setLocationsOverride(res.locations.locations)
      showToast(`Updated location ${loc.name}`, 'success')
    } catch (e: unknown) {
      showToast(`Update location failed: ${e instanceof Error ? e.message : String(e)}`, 'error')
    } finally {
      setLocationBusy(null)
    }
  }, [odomValid, robotX, robotY, yaw, showToast])

  const handleDeleteLocation = useCallback(async () => {
    if (!locationDeleteTarget) return
    const loc = locationDeleteTarget
    setLocationDeleteTarget(null)
    setLocationBusy(`delete:${loc.name}`)
    try {
      const res = await api.deleteLocation(loc.name)
      if (!res.ok) throw new Error(res.message || res.error || res.status)
      setLocationsOverride(res.locations.locations)
      showToast(`Deleted location ${loc.name}`, 'success')
    } catch (e: unknown) {
      showToast(`Delete location failed: ${e instanceof Error ? e.message : String(e)}`, 'error')
    } finally {
      setLocationBusy(null)
    }
  }, [locationDeleteTarget, showToast])

  const handleClearTrail = useCallback(() => {
    setTrail([])
    prevTrailEndRef.current = null
    try {
      if (activeMap) sessionStorage.removeItem(trailStorageKey(activeMap))
    } catch { /* ignore */ }
  }, [activeMap])

  const handleClearCloud = useCallback(async () => {
    try {
      await api.resetMapCloud()
      showToast('已清除累积点云', 'success')
    } catch (e) {
      showToast(`清除失败: ${e instanceof Error ? e.message : String(e)}`, 'error')
    }
  }, [showToast])

  const handleSaveMap = () => setSaveModalOpen(true)

  const confirmSaveMap = async (name: string) => {
    setSaveModalOpen(false)
    // 告诉用户动态过滤 + PGO 正在跑,保存需要较长时间
    showToast(`正在保存并清洗动态障碍: ${name}…`, 'info')
    try {
      const r = await api.saveMap(name)
      const summary = formatSaveMapSummary(r)
      setLastSaveSummary(summary)
      const df = r.dynamic_filter
      if (df && df.success && df.dropped !== undefined && df.orig_count) {
        const pct = (100 * df.dropped / df.orig_count).toFixed(1)
        showToast(`已保存: ${name} · 清除 ${df.dropped} 动态点 (${pct}%)`, 'success')
      } else {
        showToast(`已保存: ${name}`, 'success')
      }
      showToast(`Map save details: ${summary}`, r.warnings?.length ? 'info' : 'success')
      loadMaps()
    } catch { showToast('保存失败', 'error') }
  }

  const publishWorkbenchResult = useCallback((r: MapLifecycleResponse) => {
    const summary = formatMapLifecycleSummary(r)
    setWorkbenchSummary(summary)
    showToast(summary, r.ok || r.success ? 'success' : 'error')
  }, [showToast])

  const handleWorkbenchImportPcd = useCallback(async () => {
    const name = workbenchMapName.trim()
    const sourcePath = workbenchImportPath.trim()
    if (!name) {
      showToast('Map name is required for PCD import', 'error')
      return
    }
    if (!sourcePath) {
      showToast('PCD source path is required', 'error')
      return
    }
    setWorkbenchBusy('import')
    try {
      const res = await api.importPcdMap(name, sourcePath, Number(workbenchVoxelSize) || 0)
      publishWorkbenchResult(res)
      loadMaps()
    } catch (e: unknown) {
      showToast(`Import PCD failed: ${e instanceof Error ? e.message : String(e)}`, 'error')
    } finally {
      setWorkbenchBusy(null)
    }
  }, [loadMaps, publishWorkbenchResult, showToast, workbenchImportPath, workbenchMapName, workbenchVoxelSize])

  const handleWorkbenchCrop = useCallback(async () => {
    if (!workbenchTargetMapName) {
      showToast('Select or type a map name before crop', 'error')
      return
    }
    setWorkbenchBusy('crop')
    try {
      const bounds = parseWorkbenchBounds(workbenchBoundsJson)
      const res = await api.cropMap(workbenchTargetMapName, bounds)
      publishWorkbenchResult(res)
      loadMaps()
    } catch (e: unknown) {
      showToast(`Crop failed: ${e instanceof Error ? e.message : String(e)}`, 'error')
    } finally {
      setWorkbenchBusy(null)
    }
  }, [loadMaps, publishWorkbenchResult, showToast, workbenchBoundsJson, workbenchTargetMapName])

  const handleWorkbenchBuildOctomap = useCallback(async () => {
    if (!workbenchTargetMapName) {
      showToast('Select or type a map name before build', 'error')
      return
    }
    setWorkbenchBusy('build')
    try {
      const res = await api.buildMapOctomap(workbenchTargetMapName)
      publishWorkbenchResult(res)
      loadMaps()
    } catch (e: unknown) {
      showToast(`Build OctoMap failed: ${e instanceof Error ? e.message : String(e)}`, 'error')
    } finally {
      setWorkbenchBusy(null)
    }
  }, [loadMaps, publishWorkbenchResult, showToast, workbenchTargetMapName])

  const handleWorkbenchMarkZone = useCallback(async () => {
    if (!workbenchTargetMapName) {
      showToast('Select or type a map name before marking', 'error')
      return
    }
    if (!pendingGoal) {
      showToast('Click a point in the 3D map first', 'error')
      return
    }
    setWorkbenchBusy('mark')
    try {
      const res = await api.markMapZone(workbenchTargetMapName, {
        state: workbenchZoneState,
        shape: 'sphere',
        center: { x: pendingGoal.x, y: pendingGoal.y, z: 0 },
        radius: Number(workbenchZoneRadius) || 0.5,
      })
      publishWorkbenchResult(res)
      loadMaps()
    } catch (e: unknown) {
      showToast(`Mark zone failed: ${e instanceof Error ? e.message : String(e)}`, 'error')
    } finally {
      setWorkbenchBusy(null)
    }
  }, [
    loadMaps,
    pendingGoal,
    publishWorkbenchResult,
    showToast,
    workbenchTargetMapName,
    workbenchZoneRadius,
    workbenchZoneState,
  ])

  const handleWorkbenchValidatePlan = useCallback(async () => {
    if (!workbenchTargetMapName) {
      showToast('Select or type a map name before preview', 'error')
      return
    }
    if (!pendingGoal) {
      showToast('Click a target point in the 3D map first', 'error')
      return
    }
    setWorkbenchBusy('preview')
    try {
      const res = await api.validateMapPlan(workbenchTargetMapName, pendingGoal.x, pendingGoal.y, 0)
      const preview = res.preview as PlanPreviewResponse | undefined
      const summary = [
        res.ok || res.success ? 'plan ok' : 'plan blocked',
        preview?.selected_planner ?? preview?.planner ?? '',
        preview ? formatPlanSafetySummary(preview) : '',
      ].filter(Boolean).join(' | ')
      setWorkbenchSummary(summary)
      showToast(summary, res.ok || res.success ? 'success' : 'error')
    } catch (e: unknown) {
      showToast(`Plan preview failed: ${e instanceof Error ? e.message : String(e)}`, 'error')
    } finally {
      setWorkbenchBusy(null)
    }
  }, [pendingGoal, showToast, workbenchTargetMapName])

  const handleActivate = (name: string) => {
    if (maps.find(m => m.name === name)?.is_active) {
      showToast(`当前已激活: ${name}`, 'info')
      return
    }
    setLoadTarget(name)
  }

  const confirmLoadMap = async () => {
    if (!loadTarget) return
    const name = loadTarget
    setLoadTarget(null)
    // Clear old saved map first — don't show stale data
    setSavedMapFlat(undefined)
    try {
      await api.activateMap(name)
      if (!savedMapRelocalizeSupported) {
        showToast(`Loaded ${name}. ${relocalizeUnavailableMessage}`, 'info')
        loadMaps()
        try {
          const pts = await api.fetchSavedMapPoints(name)
          setSavedMapFlat(pts)
        } catch { /* PCD not available */ }
        return
      }
      await api.relocalize(name, 0, 0, 0)
      showToast(`已加载: ${name}，重定位到原点`, 'success')
      loadMaps()
      try {
        const pts = await api.fetchSavedMapPoints(name)
        setSavedMapFlat(pts)
      } catch { /* PCD not available */ }
    } catch (e: unknown) {
      showToast(`加载失败: ${e instanceof Error ? e.message : String(e)}`, 'error')
    }
  }

  const handleDeleteMap = async () => {
    if (!deleteTarget) return
    const name = deleteTarget
    setDeleteTarget(null)
    try {
      await api.deleteMap(name)
      showToast(`已删除: ${name}`, 'success')
      loadMaps()
      if (savedMapFlat !== undefined) setSavedMapFlat(undefined)
    } catch (e: unknown) {
      showToast(`删除失败: ${e instanceof Error ? e.message : String(e)}`, 'error')
    }
  }

  const confirmRenameMap = async (newName: string) => {
    if (!renameTarget) return
    const oldName = renameTarget
    setRenameTarget(null)
    try {
      await api.renameMap(oldName, newName)
      showToast(`已重命名: ${oldName} → ${newName}`, 'success')
      loadMaps()
    } catch (e: unknown) {
      showToast(`重命名失败: ${e instanceof Error ? e.message : String(e)}`, 'error')
    }
  }

  const handleSwitchSlam = (profile: SlamProfile) => {
    if (slamPending) return
    setSlamSwitchTarget(profile)
  }

  const confirmSwitchSlam = async () => {
    if (!slamSwitchTarget) return
    const profile = slamSwitchTarget
    setSlamSwitchTarget(null)
    setSlamPending(profile)
    try {
      await api.switchSlamMode(profile)
      showToast(`已切换: ${profile === 'fastlio2' ? 'SLAM建图' : '导航巡航'}`, 'success')
    } catch (e: unknown) {
      showToast(`切换失败: ${e instanceof Error ? e.message : String(e)}`, 'error')
    } finally {
      setSlamPending(null)
    }
  }

  const handleStop = async () => {
    try {
      const res = await api.sendStop()
      showToast(api.formatCommandAck(res, '停止指令'), 'info')
    } catch (e: unknown) {
      showToast(api.formatCommandError(e, '停止失败'), 'error')
    }
  }

  const handleCancelNavigation = async () => {
    try {
      const res = await api.cancelNavigation('web_cancel')
      showToast(api.formatCommandAck(res, 'Navigation cancel'), 'info')
    } catch (e: unknown) {
      showToast(api.formatCommandError(e, 'Cancel navigation failed'), 'error')
    }
  }

  const handleRelocalize = async () => {
    if (!savedMapRelocalizeSupported) {
      showToast(relocalizeUnavailableMessage, 'error')
      return
    }
    if (!relocMap) { showToast('请先选择地图', 'error'); return }
    setRelocPending(true)
    try {
      await api.relocalize(relocMap, parseFloat(relocX) || 0, parseFloat(relocY) || 0, parseFloat(relocYaw) || 0)
      showToast(`重定位已发起: ${relocMap}`, 'success')
      setRelocOpen(false)
      // Load saved map cloud only after relocalization (coordinate frames now aligned)
      try {
        const pts = await api.fetchSavedMapPoints(relocMap)
        setSavedMapFlat(pts)
      } catch { /* PCD not available — ignore */ }
    } catch (e: unknown) {
      showToast(`重定位失败: ${e instanceof Error ? e.message : String(e)}`, 'error')
    } finally {
      setRelocPending(false)
    }
  }

  const toggleLayer = (key: keyof Layers) =>
    setLayers(prev => ({ ...prev, [key]: !prev[key] }))

  const LayerBtn = ({
    k, icon, label,
  }: { k: keyof Layers; icon: React.ReactNode; label: string }) => (
    <button
      className={layers[k] ? styles.layerBtnActive : styles.layerBtn}
      onClick={() => toggleLayer(k)}
    >
      {icon} {label}
    </button>
  )

  return (
    <div className={styles.sceneView}>
      {/* Toolbar */}
      <div className={styles.toolbar}>
        <span className={styles.toolbarTitle}>
          <span className={styles.toolbarTitleIcon}><Compass size={13} /></span>
          场景视图
        </span>

        <span className={styles.divider} />

        <div className={styles.layerGroup}>
          <LayerBtn k="grid"  icon={<Grid3x3 size={11} />}   label="网格"  />
          <LayerBtn k="cloud" icon={<Cloud size={11} />}      label="点云"  />
          <LayerBtn k="trail" icon={<Route size={11} />}      label="轨迹"  />
          <LayerBtn k="path"  icon={<Navigation size={11} />} label="路径"  />
          <LayerBtn k="goal"  icon={<Target size={11} />}     label="目标"  />
          <LayerBtn k="robot"   icon={<Bot size={11} />}        label="本机"  />
          <LayerBtn k="costmap" icon={<LayersIcon size={11} />} label="代价"  />
          {/* 坡度层暂时隐藏 — slope grid TF 对齐未彻底修复,看起来飘.
              保留 Scene3D / SSE 渲染代码,待 TF 修好后恢复按钮. */}
        </div>

        <span className={styles.divider} />

        <div className={styles.pointSizeRow} title="点云粒子大小">
          <Cloud size={10} className={styles.pointSizeIcon} />
          <input
            type="range" min={0.02} max={0.4} step={0.01}
            value={pointSize}
            onChange={e => setPointSize(parseFloat(e.target.value))}
            className={styles.pointSlider}
          />
          <span className={styles.pointSizeVal}>{pointSize.toFixed(2)}</span>
        </div>

        <div className={styles.toolbarSpacer} />

        <button className={styles.toolbarBtn} onClick={() => scene3DRef.current?.resetCamera()} title="重置到自动视野">
          <Maximize2 size={12} /> 重置视野
        </button>
        <button className={styles.toolbarBtn} onClick={handleClearTrail}>
          <Trash2 size={12} /> 清除轨迹
        </button>
        <button className={styles.toolbarBtn} onClick={handleClearCloud} title="仅清浏览器可视化，不动 SLAM ikd-tree">
          <Cloud size={12} /> 清除点云
        </button>
        <button className={styles.toolbarBtnPrimary} onClick={handleSaveMap}>
          <Save size={12} /> 保存地图
        </button>
      </div>

      {/* Workspace */}
      <div className={[styles.workspace, drawerOpen ? '' : styles.drawerClosed].filter(Boolean).join(' ')}>
        {/* Left: map drawer */}
        <div className={styles.drawer}>
          <div className={styles.drawerHeader}>
            <span className={styles.drawerTitle}>地图</span>
            <div style={{ display: 'flex', gap: 4 }}>
              {savedMapFlat !== undefined && (
                <button
                  className={styles.drawerToggle}
                  onClick={() => setSavedMapFlat(undefined)}
                  title="清除已加载地图"
                >
                  <X size={14} />
                </button>
              )}
              <button
                className={styles.drawerToggle}
                onClick={() => setDrawerOpen(!drawerOpen)}
                title={drawerOpen ? '收起' : '展开'}
              >
                {drawerOpen ? <PanelLeftClose size={14} /> : <PanelLeftOpen size={14} />}
              </button>
            </div>
          </div>
          <div className={styles.drawerBody}>
            {lastSaveSummary && (
              <div className={styles.hintBadge} title={lastSaveSummary}>
                Last save: {lastSaveSummary}
              </div>
            )}
            <div className={styles.mapWorkbench}>
              <div className={styles.workbenchHeader}>
                <span>Map Workbench</span>
                <span className={styles.workbenchActiveMap}>
                  {workbenchTargetMapName || 'no map'}
                </span>
              </div>
              <input
                className={styles.workbenchInput}
                value={workbenchMapName}
                onChange={(e) => setWorkbenchMapName(e.target.value)}
                placeholder={activeMapName ? `active: ${activeMapName}` : 'map name'}
              />
              <input
                className={styles.workbenchInput}
                value={workbenchImportPath}
                onChange={(e) => setWorkbenchImportPath(e.target.value)}
                placeholder="PCD path on gateway host"
              />
              <div className={styles.workbenchRow}>
                <input
                  className={styles.workbenchInput}
                  value={workbenchVoxelSize}
                  onChange={(e) => setWorkbenchVoxelSize(e.target.value)}
                  placeholder="voxel m"
                  inputMode="decimal"
                />
                <button
                  type="button"
                  className={styles.workbenchButton}
                  disabled={workbenchBusy != null}
                  onClick={handleWorkbenchImportPcd}
                >
                  Import PCD
                </button>
              </div>
              <textarea
                className={styles.workbenchTextarea}
                value={workbenchBoundsJson}
                onChange={(e) => setWorkbenchBoundsJson(e.target.value)}
                spellCheck={false}
              />
              <div className={styles.workbenchRow}>
                <button
                  type="button"
                  className={styles.workbenchButton}
                  disabled={workbenchBusy != null}
                  onClick={handleWorkbenchCrop}
                >
                  Crop
                </button>
                <button
                  type="button"
                  className={styles.workbenchButton}
                  disabled={workbenchBusy != null}
                  onClick={handleWorkbenchBuildOctomap}
                >
                  Build OctoMap
                </button>
              </div>
              <div className={styles.workbenchRow}>
                <select
                  className={styles.workbenchInput}
                  value={workbenchZoneState}
                  onChange={(e) => setWorkbenchZoneState(e.target.value as WorkbenchZoneState)}
                >
                  <option value="preblocked">preblocked</option>
                  <option value="traversable">traversable</option>
                  <option value="clear">clear</option>
                </select>
                <input
                  className={styles.workbenchInput}
                  value={workbenchZoneRadius}
                  onChange={(e) => setWorkbenchZoneRadius(e.target.value)}
                  placeholder="radius m"
                  inputMode="decimal"
                />
              </div>
              <div className={styles.workbenchRow}>
                <button
                  type="button"
                  className={styles.workbenchButton}
                  disabled={workbenchBusy != null || !pendingGoal}
                  onClick={handleWorkbenchMarkZone}
                  title={pendingGoal ? 'Mark selected point in active OctoMap' : 'Click a map point first'}
                >
                  Mark Zone
                </button>
                <button
                  type="button"
                  className={styles.workbenchButton}
                  disabled={workbenchBusy != null || !pendingGoal}
                  onClick={handleWorkbenchValidatePlan}
                  title={pendingGoal ? 'Run no-motion route preview' : 'Click a target point first'}
                >
                  Preview
                </button>
              </div>
              {pendingGoal && (
                <div className={styles.workbenchHint}>
                  point {pendingGoal.x.toFixed(2)}, {pendingGoal.y.toFixed(2)}
                </div>
              )}
              {workbenchSummary && (
                <div className={styles.workbenchHint} title={workbenchSummary}>
                  {workbenchSummary}
                </div>
              )}
            </div>
            {maps.length === 0 && (
              <div className={styles.emptyState}>
                <MapPinned size={32} className={styles.emptyIcon} strokeWidth={1.4} />
                <div className={styles.emptyTitle}>暂无地图</div>
                <div className={styles.emptyHint}>保存当前场景来创建第一张地图</div>
                <button
                  type="button"
                  className={styles.emptyCta}
                  onClick={handleSaveMap}
                >
                  <Save size={11} /> 保存地图
                </button>
              </div>
            )}
            {MAP_GROUPS.map(g => {
              const groupMaps = maps.filter(g.filter)
              if (groupMaps.length === 0) return null
              return (
                <div key={g.label} className={styles.mapGroup}>
                  <div className={styles.mapGroupTitle}>{g.label}</div>
                  {groupMaps.map(m => (
                    <button
                      key={m.name}
                      className={m.is_active ? styles.mapItemActive : styles.mapItem}
                      onClick={() => handleActivate(m.name)}
                      onContextMenu={(e) => {
                        e.preventDefault()
                        const rect = (e.currentTarget as HTMLElement).getBoundingClientRect()
                        setMapContextMenu({ name: m.name, x: rect.right + 4, y: rect.top })
                      }}
                      title="左键加载确认 · 右键管理"
                    >
                      <span>{m.name}</span>
                      {m.has_octomap && <span className={styles.mapBadge}>O</span>}
                    </button>
                  ))}
                </div>
              )
            })}
            {/* Context menu */}
            {mapContextMenu && (
              <div
                className={styles.contextMenu}
                style={{ position: 'fixed', left: mapContextMenu.x, top: mapContextMenu.y, zIndex: 100 }}
                onClick={() => setMapContextMenu(null)}
              >
                <button className={styles.contextMenuItem} onClick={() => { setLoadTarget(mapContextMenu.name); setMapContextMenu(null) }}>
                  <LocateFixed size={12} /> 加载并重定位
                </button>
                <button className={styles.contextMenuItem} onClick={() => { setRenameTarget(mapContextMenu.name); setMapContextMenu(null) }}>
                  <Pencil size={12} /> 重命名
                </button>
                <button className={[styles.contextMenuItem, styles.contextMenuDanger].join(' ')} onClick={() => { setDeleteTarget(mapContextMenu.name); setMapContextMenu(null) }}>
                  <Trash2 size={12} /> 删除
                </button>
              </div>
            )}
          </div>
        </div>

        {/* Center: 3D scene */}
        <div className={styles.canvasArea}>
          <div className={styles.canvasWrap}>
            <Scene3D
              ref={scene3DRef}
              cloud={cloud}
              savedMapFlat={savedMapFlat ?? sseState.savedMap?.points}
              costmap={sseState.costmap ?? null}
              slopeGrid={sseState.slopeGrid ?? null}
              sceneGraph={sseState.sceneGraph ?? null}
              robotX={robotX}
              robotY={robotY}
              yaw={yaw}
              trail={trail}
              path={path}
              localPath={localPathPts}
              layers={layers}
              pointSize={pointSize}
              onPendingGoal={handlePendingGoal}
              onRelocalize={handleSceneRelocalize}
              pendingGoal={pendingGoal}
            />
            <div className={styles.canvasOverlayTop}>
              <span className={styles.scaleLabel}>3D 场景视图  ·  拖拽旋转  ·  滚轮缩放  ·  点击放置目标  ·  Shift+点击重定位</span>
            </div>
            <div className={styles.robotOverlay}>
              <span>位置 {odomValid ? `(${robotX.toFixed(2)}, ${robotY.toFixed(2)})` : '(--, --)'}</span>
              <span>航向 {odomValid ? `${((yaw * 180) / Math.PI).toFixed(1)}°` : '--°'}</span>
              <span>速度 {odomValid ? `${vx.toFixed(2)} m/s` : '-- m/s'}</span>
              <span>{slamMode === '—' || slamMode === 'stop' ? '⚠ SLAM 离线' : missionState}</span>
            </div>
            {pendingGoal && (
              <div className={styles.goalConfirmPanel}>
                <span className={styles.goalConfirmLabel}>导航目标</span>
                <span className={styles.goalConfirmCoords}>
                  ({pendingGoal.x.toFixed(2)}, {pendingGoal.y.toFixed(2)})
                </span>
                <div className={styles.goalControlGroup}>
                  <label className={styles.goalControl}>
                    <span>Speed</span>
                    <select
                      className={styles.goalSelect}
                      value={goalMaxSpeed}
                      onChange={(e) => setGoalMaxSpeed(Number(e.target.value))}
                    >
                      {GOAL_SPEED_OPTIONS.map((speed) => (
                        <option key={speed} value={speed}>{speed.toFixed(2)} m/s</option>
                      ))}
                    </select>
                  </label>
                  <label className={styles.goalControl}>
                    <span>Radius</span>
                    <select
                      className={styles.goalSelect}
                      value={goalAcceptanceRadius}
                      onChange={(e) => setGoalAcceptanceRadius(Number(e.target.value))}
                    >
                      {GOAL_RADIUS_OPTIONS.map((radius) => (
                        <option key={radius} value={radius}>{radius.toFixed(2)} m</option>
                      ))}
                    </select>
                  </label>
                </div>
                {pendingGoalPlanSummary && (
                  <span className={styles.goalPlanSummary} title={pendingGoalPlanSummary}>
                    {pendingGoalPlanSummary}
                  </span>
                )}
                {goalDisabledReason && (
                  <span className={styles.goalConfirmReason} title={goalDisabledReason}>
                    {goalDisabledReason}
                  </span>
                )}
                <button
                  className={styles.goalConfirmBtn}
                  onClick={handleConfirmGoal}
                  disabled={!canSendGoal}
                  title={goalDisabledReason || '发送导航目标'}
                >
                  <Navigation size={12} /> 发送
                </button>
                <button
                  className={styles.goalCancelBtn}
                  onClick={() => {
                    setPendingGoal(null)
                    setPendingGoalPreview(null)
                  }}
                >
                  取消
                </button>
              </div>
            )}
            {/* Camera PiP — draggable */}
            <div
              className={styles.cameraPip}
              onMouseDown={(e) => {
                const pip = e.currentTarget
                const rect = pip.getBoundingClientRect()
                const ox = e.clientX - rect.left
                const oy = e.clientY - rect.top
                const onMove = (ev: MouseEvent) => {
                  const parent = pip.parentElement!.getBoundingClientRect()
                  pip.style.left = `${ev.clientX - parent.left - ox}px`
                  pip.style.top = `${ev.clientY - parent.top - oy}px`
                  pip.style.right = 'auto'
                  pip.style.bottom = 'auto'
                }
                const onUp = () => {
                  document.removeEventListener('mousemove', onMove)
                  document.removeEventListener('mouseup', onUp)
                  pip.style.cursor = 'grab'
                }
                pip.style.cursor = 'grabbing'
                document.addEventListener('mousemove', onMove)
                document.addEventListener('mouseup', onUp)
              }}
            >
              <div className={styles.cameraPipHeader}>
                <span className={cameraPipDotClass} />
                {cameraPipLabel}
              </div>
              {cameraImgSrc
                ? <img src={cameraImgSrc} className={styles.cameraPipImg} alt="camera" draggable={false} />
                : <div className={styles.cameraPipEmpty}><VideoOff size={18} opacity={0.35} /></div>
              }
            </div>
          </div>
        </div>

        {/* Right: side panel */}
        <div className={styles.sidePanel}>
          <div className={styles.statCard}>
            <div className={styles.statCardTitle}>机器人状态</div>
            <div className={styles.statGrid}>
              <div className={styles.statItem}>
                <span className={styles.statLabel}>X</span>
                <span className={styles.statValueHl}>{robotX.toFixed(2)}</span>
              </div>
              <div className={styles.statItem}>
                <span className={styles.statLabel}>Y</span>
                <span className={styles.statValueHl}>{robotY.toFixed(2)}</span>
              </div>
              <div className={styles.statItem}>
                <span className={styles.statLabel}>航向</span>
                <span className={styles.statValue}>
                  {((yaw * 180) / Math.PI).toFixed(0)}°
                </span>
              </div>
              <div className={styles.statItem}>
                <span className={styles.statLabel}>线速度</span>
                <span className={styles.statValue}>{vx.toFixed(2)} m/s</span>
              </div>
            </div>
          </div>

          <div className={styles.statCard}>
            <div className={styles.statCardTitle}>导航任务</div>
            <div className={styles.statGrid}>
              <div className={styles.statItem} style={{ gridColumn: '1 / span 2' }}>
                <span className={styles.statLabel}>状态</span>
                <span className={hasGoal ? styles.goalBadgeActive : styles.goalBadgeIdle}>
                  {missionState}
                </span>
              </div>
              <div className={styles.statItem} style={{ gridColumn: '1 / span 2' }}>
                <span className={styles.statLabel}>目标</span>
                <span className={styles.statValueDim}>
                  {missionGoal ?? '无'}
                </span>
              </div>
            </div>
          </div>

          <div className={styles.statCard}>
            <div className={styles.statCardTitle}>
              <MapPinned size={11} style={{ marginRight: 4, verticalAlign: 'middle' }} />
              Locations
            </div>
            <div className={styles.locationSaveRow}>
              <input
                className={styles.locationNameInput}
                value={locationName}
                onChange={e => setLocationName(e.target.value)}
                placeholder="Name current pose"
                maxLength={48}
              />
              <button
                type="button"
                className={styles.locationSaveBtn}
                onClick={handleSaveCurrentLocation}
                disabled={!odomValid || !normalizedLocationName || locationBusy !== null}
                title={odomValid ? 'Save current robot pose' : 'No valid odometry'}
              >
                <Save size={12} />
              </button>
            </div>
            <div className={styles.locationList}>
              {savedLocations.length === 0 && (
                <div className={styles.locationEmpty}>No saved locations</div>
              )}
              {savedLocations.slice(0, 8).map(loc => {
                const navBusy = locationBusy === `nav:${loc.name}`
                const updateBusy = locationBusy === `update:${loc.name}`
                const deleteBusy = locationBusy === `delete:${loc.name}`
                const disabled = locationBusy !== null
                return (
                  <div className={styles.locationItem} key={loc.name}>
                    <button
                      type="button"
                      className={styles.locationMain}
                      onClick={() => handleNavigateLocation(loc)}
                      disabled={!canSendGoal || disabled}
                      title={canSendGoal ? `Navigate to ${loc.name}` : goalDisabledReason}
                    >
                      <span className={styles.locationName}>{loc.name}</span>
                      <span className={styles.locationCoords}>
                        {loc.x.toFixed(2)}, {loc.y.toFixed(2)}
                      </span>
                      {loc.tags.length > 0 && (
                        <span className={styles.locationTags}>{loc.tags.slice(0, 2).join(' / ')}</span>
                      )}
                    </button>
                    <div className={styles.locationActions}>
                      <button
                        type="button"
                        className={styles.locationIconBtn}
                        onClick={() => handleUpdateLocationToCurrent(loc)}
                        disabled={!odomValid || disabled}
                        title={odomValid ? 'Update to current pose' : 'No valid odometry'}
                      >
                        {updateBusy ? <Activity size={12} /> : <Pencil size={12} />}
                      </button>
                      <button
                        type="button"
                        className={`${styles.locationIconBtn} ${styles.locationDangerBtn}`}
                        onClick={() => setLocationDeleteTarget(loc)}
                        disabled={disabled}
                        title="Delete location"
                      >
                        {deleteBusy ? <Activity size={12} /> : <Trash2 size={12} />}
                      </button>
                    </div>
                    {navBusy && <span className={styles.locationBusyLine} />}
                  </div>
                )
              })}
            </div>
          </div>

          <div className={styles.statCard}>
            <div className={styles.statCardTitle}>
              <Radio size={11} style={{ marginRight: 4, verticalAlign: 'middle' }} />
              SLAM 模式
            </div>
            <div className={styles.slamModeGroup}>
              <button
                className={slamMode === 'fastlio2' ? styles.slamModeBtnActive : styles.slamModeBtn}
                onClick={() => handleSwitchSlam('fastlio2')}
                disabled={slamPending !== null}
              >
                <Navigation size={11} />
                建图
                {slamMode === 'fastlio2' && <span className={styles.slamDot} />}
              </button>
              <button
                className={slamMode === 'localizer' ? styles.slamModeBtnActive : styles.slamModeBtn}
                onClick={() => handleSwitchSlam('localizer')}
                disabled={slamPending !== null}
              >
                <Activity size={11} />
                巡航
                {slamMode === 'localizer' && <span className={styles.slamDot} />}
              </button>
            </div>
            <div className={styles.statGrid}>
              <div className={styles.statItem}>
                <span className={styles.statLabel}>频率</span>
                <span className={styles.statValue}>{slamHz.toFixed(1)} Hz</span>
              </div>
              <div className={styles.statItem}>
                <span className={styles.statLabel}>点云</span>
                <span className={styles.statValue}>
                  {sseState.slamStatus?.map_points?.toLocaleString() ?? '—'}
                </span>
              </div>
            </div>

            <button
              className={styles.relocBtn}
              onClick={() => setRelocOpen(v => !v)}
              disabled={!savedMapRelocalizeSupported}
              title={savedMapRelocalizeSupported ? 'Saved-map relocalize is available' : relocalizeUnavailableMessage}
            >
              <LocateFixed size={11} />
              重定位
            </button>

            {relocOpen && (
              <div className={styles.relocPanel}>
                {!savedMapRelocalizeSupported && (
                  <div className={styles.hintBadge}>{relocalizeUnavailableMessage}</div>
                )}
                {/* Custom dropdown */}
                <div className={styles.customSelect} ref={relocDropRef}>
                  <button
                    type="button"
                    className={styles.customSelectTrigger}
                    onClick={() => setRelocDropOpen(v => !v)}
                  >
                    <span className={relocMap ? styles.customSelectValue : styles.customSelectPlaceholder}>
                      {relocMap || '— 选择地图 —'}
                    </span>
                    <svg width="10" height="6" viewBox="0 0 10 6" fill="none" className={relocDropOpen ? styles.customSelectArrowOpen : styles.customSelectArrow}>
                      <path d="M1 1L5 5L9 1" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round"/>
                    </svg>
                  </button>
                  {relocDropOpen && (
                    <div className={styles.customSelectList}>
                      {maps.length === 0 && (
                        <div className={styles.customSelectEmpty}>暂无地图</div>
                      )}
                      {maps.map(m => (
                        <button
                          key={m.name}
                          type="button"
                          className={m.name === relocMap ? styles.customSelectItemActive : styles.customSelectItem}
                          onClick={() => { setRelocMap(m.name); setRelocDropOpen(false) }}
                        >
                          {m.name}
                        </button>
                      ))}
                    </div>
                  )}
                </div>
                <div className={styles.relocInputRow}>
                  <label>X</label>
                  <input className={styles.relocInput} type="number" step="0.1"
                    value={relocX}
                    onChange={e => { setRelocDirty(true); setRelocX(e.target.value) }} />
                  <label>Y</label>
                  <input className={styles.relocInput} type="number" step="0.1"
                    value={relocY}
                    onChange={e => { setRelocDirty(true); setRelocY(e.target.value) }} />
                  <label>Yaw</label>
                  <input className={styles.relocInput} type="number" step="0.1"
                    value={relocYaw}
                    onChange={e => { setRelocDirty(true); setRelocYaw(e.target.value) }} />
                </div>
                <button
                  className={styles.relocConfirmBtn}
                  onClick={handleRelocalize}
                  disabled={relocPending || !relocMap || !savedMapRelocalizeSupported}
                >
                  {relocPending ? '定位中…' : '确认重定位'}
                </button>
              </div>
            )}
          </div>

          <button
            className={styles.cancelNavBtn}
            onClick={handleCancelNavigation}
            disabled={!hasGoal}
            title={hasGoal ? 'Cancel current navigation mission' : 'No active navigation mission'}
          >
            <X size={15} />
            Cancel Nav
          </button>
          <button className={styles.eStopBtn} onClick={handleStop}>
            <StopCircle size={16} />
            紧急停止
          </button>
        </div>
      </div>

      {/* Click-away to close context menu */}
      {mapContextMenu && (
        <div style={{ position: 'fixed', inset: 0, zIndex: 99 }} onClick={() => setMapContextMenu(null)} />
      )}

      {/* Load map confirm */}
      <ConfirmModal
        open={loadTarget !== null}
        title="Load map"
        message={savedMapRelocalizeSupported
          ? `Load "${loadTarget ?? ''}" and relocalize into this saved map frame?`
          : `Load "${loadTarget ?? ''}" without relocalize. ${relocalizeUnavailableMessage}`}
        confirmLabel={savedMapRelocalizeSupported ? 'Load and relocalize' : 'Load map only'}
        onConfirm={confirmLoadMap}
        onCancel={() => setLoadTarget(null)}
      />

      {/* Delete map confirm */}
      <ConfirmModal
        open={deleteTarget !== null}
        title="删除地图"
        message={`确认删除「${deleteTarget ?? ''}」？此操作不可恢复。`}
        confirmLabel="删除"
        danger
        onConfirm={handleDeleteMap}
        onCancel={() => setDeleteTarget(null)}
      />

      <ConfirmModal
        open={locationDeleteTarget !== null}
        title="Delete location"
        message={`Delete location "${locationDeleteTarget?.name ?? ''}"?`}
        confirmLabel="Delete"
        danger
        onConfirm={handleDeleteLocation}
        onCancel={() => setLocationDeleteTarget(null)}
      />

      {/* Rename map */}
      <PromptModal
        open={renameTarget !== null}
        title="重命名地图"
        message={`当前名称: ${renameTarget ?? ''}`}
        placeholder="新名称"
        initialValue={renameTarget ?? ''}
        confirmLabel="重命名"
        icon={<Pencil size={18} />}
        validate={(v) => {
          if (!/^[a-zA-Z0-9_-]+$/.test(v)) return '仅支持字母、数字、下划线和横线'
          if (v === renameTarget) return '名称未变'
          return null
        }}
        onConfirm={confirmRenameMap}
        onCancel={() => setRenameTarget(null)}
      />

      {/* SLAM switch confirm */}
      <ConfirmModal
        open={slamSwitchTarget !== null}
        title="切换 SLAM 模式"
        message={slamSwitchTarget === 'fastlio2'
          ? '切换到建图模式？当前导航巡航将停止，开始实时建图。'
          : '切换到巡航模式？需要先加载一张已有地图用于定位。'}
        confirmLabel={slamSwitchTarget === 'fastlio2' ? '开始建图' : '切换巡航'}
        onConfirm={confirmSwitchSlam}
        onCancel={() => setSlamSwitchTarget(null)}
      />

      <PromptModal
        open={saveModalOpen}
        title="保存当前地图"
        message="保存当前 SLAM 建图结果。系统会自动生成导航所需的 OctoMap 和 occupancy 数据。"
        placeholder="例如 building_2f"
        confirmLabel="保存"
        icon={<Save size={18} />}
        validate={(v) => {
          if (!/^[a-zA-Z0-9_-]+$/.test(v)) return '仅支持字母、数字、下划线和横线'
          if (v.length > 32) return '名称过长 (最多 32 字符)'
          return null
        }}
        onConfirm={confirmSaveMap}
        onCancel={() => setSaveModalOpen(false)}
      />
    </div>
  )
}

export const SceneView = memo(SceneViewComponent)
