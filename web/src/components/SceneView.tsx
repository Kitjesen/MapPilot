import { useRef, useEffect, useCallback, useMemo, useState, memo, type ReactNode } from 'react'
import {
  Compass, Grid3x3, Navigation, Route, Target, Bot,
  PanelLeftClose, PanelLeftOpen, Save, Trash2, StopCircle, Pencil, X,
  MapPinned, Cloud, Maximize2, Radio, Activity, LocateFixed, VideoOff, CircleDot,
  RefreshCw, Gamepad2,
} from 'lucide-react'
import type {
  SSEState,
  MapInfo,
  PathPoint,
  ToastKind,
  LocationEntry,
  PlanPreviewResponse,
  MapLifecycleResponse,
  NavigationDdsSnapshotResponse,
  ExplorationStatusResponse,
  RecordingStatusResponse,
} from '../types'
import * as api from '../services/api'
import {
  mapIsActivationReady,
} from '../services/mapReadiness'
import { presentNavigationOperatorStatus } from '../services/navigationOperatorState'
import { useCamera } from '../hooks/useCamera'
import { useBinaryCloud } from '../hooks/useBinaryCloud'
import {
  cloudFramesShareCoordinateEpoch,
  savedMapNeedsSceneRebind,
} from '../workers/cloudDecoderCore.ts'
import { PromptModal, ConfirmModal } from './Modal'
import { Scene3D, type Scene3DHandle } from './Scene3D'
import { FloatingWidget } from './FloatingWidget'
import { RecordingPanel } from './RecordingPanel'
import { TeleopPanel } from './TeleopPanel'
import { text, type Locale } from '../i18n'
import styles from './SceneView.module.css'
import {
  LOCAL_PLANNER_DIAGNOSTICS_POLL_MS,
  localPlannerSampleWarning,
  shouldPollLocalPlannerDiagnostics,
} from '../services/localPlannerDiagnostics'
import {
  RECORDING_STATUS_POLL_MS,
  recordingNeedsRecovery,
  recordingStatusIsActive,
} from '../services/recordingStatus'
import {
  resolveElevationLayer,
  type ElevationLayerState,
} from './scene3d/layers/elevationLayer.ts'
import {
  resolveNativeTraversabilityLayer,
  type NativeTraversabilityLayerState,
} from './scene3d/layers/traversabilityLayer.ts'

interface SceneViewProps {
  sseState:  SSEState
  showToast: (msg: string, kind?: ToastKind) => void
  locale: Locale
  motionStartAllowed: boolean
  motionStartBlockedReason: string
  onElevationSubscriptionChange?: (enabled: boolean) => void
}

// ── Layer flags ────────────────────────────────────────────────
interface Layers {
  grid:    boolean
  cloud:   boolean
  trail:   boolean
  path:    boolean
  goal:    boolean
  robot:   boolean
  elevation: boolean
  nativeTraversability: boolean
  localPlanner: boolean
}

const TRAIL_MAX = 300
const GOAL_SPEED_OPTIONS = [0.25, 0.4, 0.6]
const GOAL_RADIUS_OPTIONS = [0.25, 0.45, 0.8]

const MAP_GROUPS: Array<{ label: string; filter: (m: MapInfo) => boolean }> = [
  { label: '地图可激活', filter: mapIsActivationReady },
  { label: '地图产物未就绪', filter: m => m.has_pcd && !mapIsActivationReady(m) },
  { label: '空地图',   filter: m => !m.has_pcd },
]

type WorkbenchZoneState = 'preblocked' | 'traversable' | 'clear'

interface SaveStatus {
  name: string
  state: 'saving' | 'saved' | 'failed'
  detail: string
  location?: string | null
  summary?: string | null
}

function sceneLayerLegendClass(
  status: ElevationLayerState['status'] | NativeTraversabilityLayerState['status'],
  warn: boolean = false,
): string {
  const classNames = [styles.sceneLayerLegend]
  if (status === 'error') {
    classNames.push(styles.sceneLayerLegendError)
  } else if (status === 'stale' || warn) {
    classNames.push(styles.sceneLayerLegendStale)
  }
  return classNames.join(' ')
}

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
  return parts.length ? parts.join(' · ') : '已写入地图列表，可在左侧选择加载。'
}

function formatPlanSummary(preview: PlanPreviewResponse | null | undefined): string {
  if (!preview) return ''
  return [
    preview.planner ? `规划器=${preview.planner}` : null,
    `路径点=${preview.count}`,
  ].filter((v): v is string => Boolean(v)).join(' | ')
}

function formatMapLifecycleSummary(r: MapLifecycleResponse): string {
  const status = r.ok || r.success ? '成功' : '失败'
  const name = r.name ?? r.map_id ?? r.active ?? ''
  const message = typeof r.message === 'string' && r.message.trim() ? r.message.trim() : ''
  const ready = r.activation_ready === true ? '地图可激活' : r.activation_ready === false ? '地图产物未就绪' : ''
  return [status, name, ready, message].filter(Boolean).join(' | ')
}

function parseWorkbenchBounds(text: string): Record<string, unknown> {
  const raw = text.trim()
  if (!raw) throw new Error('边界 JSON 为空')
  const value = JSON.parse(raw) as unknown
  if (!value || typeof value !== 'object' || Array.isArray(value)) {
    throw new Error('边界必须是 JSON 对象')
  }
  return value as Record<string, unknown>
}

function numericMetric(data: Record<string, unknown>, key: string): number | undefined {
  const value = data[key]
  if (typeof value === 'number' && Number.isFinite(value)) return value
  if (typeof value === 'string') {
    const parsed = Number(value)
    return Number.isFinite(parsed) ? parsed : undefined
  }
  return undefined
}

function formatHz(value: number | undefined): string {
  return typeof value === 'number' && Number.isFinite(value) ? `${value.toFixed(1)} Hz` : '—'
}

function formatCount(value: number | undefined): string {
  return typeof value === 'number' && Number.isFinite(value) ? Math.round(value).toLocaleString() : '—'
}

function normalizeDisplayZero(value: number, digits: number): number {
  const threshold = 0.5 * 10 ** -digits
  return Math.abs(value) < threshold ? 0 : value
}

function formatTelemetryValue(value: number, digits: number): string {
  return normalizeDisplayZero(value, digits).toFixed(digits)
}

function uniqueStrings(values: string[]): string[] {
  const seen = new Set<string>()
  const out: string[] = []
  for (const raw of values) {
    const value = raw.trim()
    if (!value || seen.has(value)) continue
    seen.add(value)
    out.push(value)
  }
  return out
}

function blockerLabel(value: string): string {
  const labels: Record<string, string> = {
    navigation_session_inactive: '当前不是导航产品会话',
    no_active_command_source: '没有活动控制源',
    odometry_missing: '里程计缺失',
    map_not_ready: '地图未就绪',
    localizer_not_ready: '定位未就绪',
  }
  return labels[value] ?? value
}

function productLabel(value: string | null | undefined): string {
  const labels: Record<string, string> = {
    teleop: '遥控',
    teleop_avoid: '避障遥控',
    map: '建图',
    tracking: '跟踪',
    nav: '导航',
    inspection: '巡检',
    explore: '探索',
  }
  return labels[value ?? ''] ?? value ?? '空闲'
}

function shouldShowSavedMapForProduct(product: string | null | undefined): boolean {
  return product === 'nav'
    || product === 'tracking'
    || product === 'inspection'
    || product === 'explore'
}

function formatPlanPreviewFailure(
  preview: PlanPreviewResponse | null | undefined,
  reasons: string[] = [],
  error?: string | null,
): string {
  const safety = formatPlanSummary(preview)
  const reason = reasons.slice(0, 3).join(' / ') || error || preview?.error || 'plan preview rejected'
  return safety ? `${reason} (${safety})` : reason
}

interface LayerButtonProps {
  active:  boolean
  icon:    ReactNode
  label:   string
  onClick: () => void
}

function LayerButton({ active, icon, label, onClick }: LayerButtonProps) {
  return (
    <button
      type="button"
      className={active ? styles.layerBtnActive : styles.layerBtn}
      onClick={onClick}
    >
      {icon}
      <span>{label}</span>
    </button>
  )
}

function showSceneDebugTools(): boolean {
  try {
    const params = new URLSearchParams(window.location.search)
    return params.has('debug_nav')
  } catch {
    return false
  }
}

function SceneViewComponent({
  sseState,
  showToast,
  locale,
  motionStartAllowed,
  motionStartBlockedReason,
  onElevationSubscriptionChange,
}: SceneViewProps) {
  const scene3DRef = useRef<Scene3DHandle>(null)
  const recordingStatusPollInFlight = useRef(false)
  const sceneDebugTools = showSceneDebugTools()

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

  const [drawerOpen, setDrawerOpen] = useState(true)
  const [saveModalOpen, setSaveModalOpen] = useState(false)
  const [recordingPanelOpen, setRecordingPanelOpen] = useState(false)
  const [teleopPanelOpen, setTeleopPanelOpen] = useState(false)
  const [recordingStatus, setRecordingStatus] = useState<RecordingStatusResponse | null>(null)
  const [recordingStatusError, setRecordingStatusError] = useState<string | null>(null)
  const [layers, setLayers] = useState<Layers>({
    grid: true, cloud: true, trail: true, path: true, goal: true, robot: true,
    elevation: false, nativeTraversability: false,
    localPlanner: sceneDebugTools,
  })
  const [rasterNowS, setRasterNowS] = useState(() => Date.now() / 1000)
  const [localPlannerSnapshot, setLocalPlannerSnapshot] = useState<NavigationDdsSnapshotResponse | null>(null)
  const [maps, setMaps] = useState<MapInfo[]>([])
  // Default 0.32 keeps the live registered cloud visible when the current
  // frame only carries a few thousand points. User can shrink via slider.
  const [pointSize, setPointSize] = useState(0.32)
  const [savedMapCloud, setSavedMapCloud] = useState<api.SavedMapPointCloud | undefined>()
  const savedMapFlat = savedMapCloud?.points
  const [relocOpen, setRelocOpen] = useState(false)
  const [relocDropOpen, setRelocDropOpen] = useState(false)
  const [relocMap, setRelocMap] = useState('')
  const relocDropRef = useRef<HTMLDivElement>(null)
  const [relocX, setRelocX] = useState('0')
  const [relocY, setRelocY] = useState('0')
  const [relocYaw, setRelocYaw] = useState('0')
  const [relocPending, setRelocPending] = useState(false)
  const [restartLocalizationPending, setRestartLocalizationPending] = useState(false)
  const [saveStatus, setSaveStatus] = useState<SaveStatus | null>(null)
  // Track whether the user has manually edited reloc inputs; until then we
  // mirror live odometry so the defaults reflect the robot's current pose
  // instead of the unhelpful (0, 0, 0).
  const [relocDirty, setRelocDirty] = useState(false)
  const [pendingGoal, setPendingGoal] = useState<{ x: number; y: number } | null>(null)
  const [pendingGoalPreview, setPendingGoalPreview] = useState<PlanPreviewResponse | null>(null)
  const [explorationStatus, setExplorationStatus] = useState<ExplorationStatusResponse | null>(null)
  const [explorationStatusLoading, setExplorationStatusLoading] = useState(false)
  const [directedExplorationBusy, setDirectedExplorationBusy] = useState(false)
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
  const [mapSwitchBusy, setMapSwitchBusy] = useState<string | null>(null)

  const refreshRecordingStatus = useCallback(async () => {
    if (recordingStatusPollInFlight.current) return
    recordingStatusPollInFlight.current = true
    try {
      setRecordingStatus(await api.fetchRecordingStatus())
      setRecordingStatusError(null)
    } catch (cause: unknown) {
      setRecordingStatusError(cause instanceof Error ? cause.message : String(cause))
    } finally {
      recordingStatusPollInFlight.current = false
    }
  }, [])

  useEffect(() => {
    void refreshRecordingStatus()
    const timer = window.setInterval(
      () => void refreshRecordingStatus(),
      RECORDING_STATUS_POLL_MS,
    )
    return () => window.clearInterval(timer)
  }, [refreshRecordingStatus])

  useEffect(() => {
    if (!layers.elevation && !layers.nativeTraversability) return
    setRasterNowS(Date.now() / 1000)
    const timer = window.setInterval(() => setRasterNowS(Date.now() / 1000), 1000)
    return () => window.clearInterval(timer)
  }, [layers.elevation, layers.nativeTraversability])

  const { imgSrc: cameraImgSrc, connected: cameraConnected, lastFrameAt: cameraLastFrameAt } = useCamera()
  const cameraPipRecovered = cameraImgSrc != null && cameraLastFrameAt != null
  const cameraPipLabel = cameraPipRecovered ? '相机已恢复' : cameraConnected ? '等待画面' : '无信号'
  const recordingRecoveryRequired = recordingNeedsRecovery(recordingStatus)
  const recordingActive = recordingStatusIsActive(recordingStatus)
  const recordingToolbarState = recordingStatusError
    ? '状态未知'
    : recordingStatus === null
      ? '状态未知'
    : recordingRecoveryRequired
      ? '需恢复'
      : recordingStatus?.state === 'preparing'
        ? '准备中'
        : recordingStatus?.state === 'recording'
          ? '录制中'
          : recordingStatus?.state === 'stopping'
            ? '停止中'
            : recordingStatus?.state === 'failed'
              ? '失败'
              : recordingActive
                ? '进行中'
                : recordingStatus?.healthy === false
                  ? '异常'
                  : recordingStatus?.available === false
                    ? '不可用'
                    : recordingStatus?.state === 'completed'
                      ? '已完成'
                      : recordingStatus?.state === 'idle'
                        ? '空闲'
                        : null
  const recordingToolbarTitle = recordingRecoveryRequired
    ? '原生录制会话状态异常，需要打开面板停止并恢复'
    : recordingStatusError
      ? '录制状态刷新失败：' + recordingStatusError
      : recordingStatus === null
        ? '正在获取原生录制状态'
      : '在场景中配置并管理原生数据录制'
  const cameraPipDotClass = cameraPipRecovered ? styles.camDotLive : cameraConnected ? styles.camDotWait : styles.camDotOff
  // Binary point-cloud channel (replaces SSE JSON map_cloud).  The hook
  // owns the WebSocket + decoder worker; we just consume the latest frame.
  const scanOverlayEnabled = useMemo(() => {
    if (typeof window === 'undefined') return false
    const params = new URLSearchParams(window.location.search)
    const flag = params.get('scan') ?? params.get('debug_scan')
    return flag == null || !['0', 'false', 'off'].includes(flag.toLowerCase())
  }, [])
  const cloud = useBinaryCloud('/ws/cloud', '/api/v1/map/points?max_points=60000', 4)
  const scanCloud = useBinaryCloud(scanOverlayEnabled ? '/ws/scan' : null, null, 10)
  const alignedScanCloud = scanOverlayEnabled
    && cloudFramesShareCoordinateEpoch(cloud, scanCloud)
    ? scanCloud
    : null
  const localPlannerDiagnosticsEnabled = shouldPollLocalPlannerDiagnostics(
    sceneDebugTools,
    layers.localPlanner,
  )

  useEffect(() => {
    if (!localPlannerDiagnosticsEnabled) {
      setLocalPlannerSnapshot(null)
      return
    }

    let disposed = false
    let inFlight = false
    const refresh = async () => {
      if (inFlight) return
      inFlight = true
      try {
        const snapshot = await api.fetchNavigationDdsSnapshot()
        if (!disposed) setLocalPlannerSnapshot(snapshot)
      } catch {
        // The layer is diagnostic-only. Failed reads hide the old snapshot so
        // stale obstacles are never presented as current planner input.
        if (!disposed) setLocalPlannerSnapshot(null)
      } finally {
        inFlight = false
      }
    }

    void refresh()
    const timer = window.setInterval(refresh, LOCAL_PLANNER_DIAGNOSTICS_POLL_MS)
    return () => {
      disposed = true
      window.clearInterval(timer)
    }
  }, [localPlannerDiagnosticsEnabled])

  const mapPath = sseState.globalPath?.frame_id === 'map' ? sseState.globalPath : null
  const mapLocalPath = sseState.localPath?.frame_id === 'map' ? sseState.localPath : null
  const mapSceneGraph = sseState.sceneGraph?.frame_id === 'map' ? sseState.sceneGraph : null
  const rawPath = mapPath?.points ?? []
  const path = rawPath.filter(
    (p): p is PathPoint =>
      p != null && typeof p.x === 'number' && typeof p.y === 'number'
  )
  const rawLocalPath = mapLocalPath?.points ?? []
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

  const mapOdom = odom?.frame_id === 'map' ? odom : null
  const robotX = sanePos(mapOdom?.x)
  const robotY = sanePos(mapOdom?.y)
  const yaw    = saneYaw(mapOdom?.yaw)
  const vx     = saneVel(mapOdom?.vx)
  const odomValid = mapOdom != null && sanePos(mapOdom.x) === mapOdom.x && sanePos(mapOdom.y) === mapOdom.y
  const poseAvailable = odomValid
  const displayRobotX = poseAvailable ? formatTelemetryValue(robotX, 2) : '--'
  const displayRobotY = poseAvailable ? formatTelemetryValue(robotY, 2) : '--'
  const displayYawDeg = poseAvailable ? `${formatTelemetryValue((yaw * 180) / Math.PI, 0)}°` : '--'
  const displaySpeed = poseAvailable ? `${formatTelemetryValue(vx, 2)} m/s` : '-- m/s'

  const navigationStatus = sseState.navigationStatus
  const operatorView = presentNavigationOperatorStatus(navigationStatus, {
    locale,
    legacyTaskState: sseState.missionStatus?.state,
  })
  const missionState = operatorView.task.state
  const missionStateLabel = operatorView.task.label
  const targetGoal = navigationStatus?.target?.goal
  const missionGoal = targetGoal && Number.isFinite(targetGoal.x) && Number.isFinite(targetGoal.y)
    ? `(${formatTelemetryValue(targetGoal.x, 2)}, ${formatTelemetryValue(targetGoal.y, 2)})`
    : operatorView.source === 'legacy'
      ? sseState.missionStatus?.goal
      : null
  const hasGoal = ['PLANNING', 'EXECUTING', 'RECOVERING', 'PAUSED'].includes(missionState)
  const slamHz       = sseState.slamStatus?.slam_hz ?? 0
  const slamDiag = sseState.slamDiag?.data ?? {}
  const processedScanHz = numericMetric(slamDiag, 'processed_scan_hz') ?? slamHz
  const lidarInputHz = numericMetric(slamDiag, 'lidar_input_hz')
  const displayedMapPoints = numericMetric(slamDiag, 'map_points') ?? sseState.slamStatus?.map_points
  const rawSession = sseState.session
  const session = rawSession
  const localizationBackend = session?.localization_backend ?? session?.slam_profile ?? sseState.slamStatus?.mode ?? 'unknown'
  const activeMap = sseState.session?.active_map
  const savedMapRelocalizeSupported =
    session?.saved_map_relocalization_supported ??
    session?.relocalization_supported ??
    false
  const recoveryMethod = session?.recovery_method ?? '--'
  const relocalizeUnavailableMessage =
    `当前后端 ${localizationBackend} 不支持保存地图重定位；恢复方式：${recoveryMethod}`
  const activeCmdSource = navigationStatus?.control?.active_cmd_source ?? 'none'
  const activeCmdBlocksGoal = operatorView.source === 'legacy'
    && activeCmdSource !== ''
    && activeCmdSource !== 'none'
  const canAcceptGoal = operatorView.goalAdmission.state === 'ACCEPTING'
  const currentProduct = session?.product
  const isExplorationSession = currentProduct === 'explore'
  const activeMapName = activeMap ?? null
  const showSavedMapInScene = Boolean(activeMapName && shouldShowSavedMapForProduct(currentProduct))
  const savedMapForScene = showSavedMapInScene
    && savedMapCloud?.mapName === activeMapName
    ? savedMapCloud
    : undefined
  const elevationState = useMemo<ElevationLayerState>(() => (
    layers.elevation
      ? resolveElevationLayer(sseState.mapScene, {
          nowS: rasterNowS,
          savedMapFrameId: savedMapForScene?.frameId,
        })
      : { status: 'unavailable', message: '最低观测高程图层未启用' }
  ), [layers.elevation, rasterNowS, savedMapForScene?.frameId, sseState.mapScene])
  const nativeTraversabilityState = useMemo<NativeTraversabilityLayerState>(() => (
    layers.nativeTraversability
      ? resolveNativeTraversabilityLayer(sseState.nativeTraversability, {
          nowS: rasterNowS,
          allowedFrameIds: [sseState.mapScene?.frame_id, savedMapForScene?.frameId],
        })
      : { status: 'unavailable', message: '控制可通行性图层未启用' }
  ), [layers.nativeTraversability, rasterNowS, savedMapForScene?.frameId, sseState.nativeTraversability, sseState.mapScene?.frame_id])
  const localPlannerSampleWarningText = localPlannerSampleWarning(localPlannerSnapshot)
  const goalBlockers = uniqueStrings([
    ...operatorView.goalAdmission.blockers,
    activeCmdBlocksGoal
      ? `当前控制源: ${navigationStatus?.control?.active_source?.label ?? activeCmdSource}`
      : null,
    !poseAvailable ? '定位尚未恢复' : null,
  ].filter((v): v is string => Boolean(v)))
  const goalBlockerText = goalBlockers.map(blockerLabel).slice(0, 3).join(' · ')
  const goalDisabledReason =
    !motionStartAllowed
      ? motionStartBlockedReason
        : canAcceptGoal && !activeCmdBlocksGoal
      ? ''
      : goalBlockerText || '导航暂未就绪'
  const goalBlockedHint =
    goalBlockers.includes('navigation_session_inactive')
      ? `已选地图点；当前是${productLabel(currentProduct)}会话，不会下发导航目标。请先切到导航、跟踪或巡检模式。`
      : `已选地图点；暂不下发目标：${goalDisabledReason}`
  const canSendGoal = goalDisabledReason === ''
  const pendingGoalPlanSummary = formatPlanSummary(pendingGoalPreview)
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
    if (odom == null || !poseAvailable) return
    const last = prevTrailEndRef.current
    if (!last || Math.hypot(odom.x - last[0], odom.y - last[1]) > 0.05) {
      prevTrailEndRef.current = [odom.x, odom.y]
      setTrail(prev => {
        const next = [...prev, [odom.x, odom.y] as [number, number]]
        return next.length > TRAIL_MAX ? next.slice(next.length - TRAIL_MAX) : next
      })
    }
  }, [odom, poseAvailable])

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
    if (relocDirty || !odom || !poseAvailable) return
    setRelocX(robotX.toFixed(2))
    setRelocY(robotY.toFixed(2))
    setRelocYaw(yaw.toFixed(3))
  }, [odom, poseAvailable, robotX, robotY, yaw, relocDirty])

  // ── Default active map for reloc panel ─────────────────────────
  const workbenchTargetMapName = workbenchMapName.trim() || (showSavedMapInScene ? activeMapName : '') || ''
  const savedMapAutoLoadRef = useRef<string | null>(null)
  useEffect(() => {
    if (!relocMap && activeMapName) setRelocMap(activeMapName)
  }, [activeMapName, relocMap])

  useEffect(() => {
    savedMapAutoLoadRef.current = null
    setSavedMapCloud(undefined)
  }, [activeMapName, showSavedMapInScene])

  useEffect(() => {
    if (!showSavedMapInScene) return
    if (!activeMapName) return
    const savedMapNeedsEpochRebind = savedMapCloud !== undefined
      && savedMapNeedsSceneRebind(
        { frameId: cloud.frameId, epoch: cloud.epoch },
        { frameId: savedMapCloud.frameId, epoch: savedMapCloud.epoch },
      )
    if (savedMapFlat !== undefined && !savedMapNeedsEpochRebind) return
    const sceneBindingKey = `${activeMapName}:${cloud.frameId ?? 'unknown'}:${cloud.epoch ?? 'unknown'}`
    if (savedMapAutoLoadRef.current === sceneBindingKey) return

    let cancelled = false
    savedMapAutoLoadRef.current = sceneBindingKey
    api.fetchSavedMapPointCloud(activeMapName)
      .then(savedMap => {
        if (!cancelled) setSavedMapCloud(savedMap)
      })
      .catch(() => {
        // Map may be live-only or PCD may not exist yet; keep the live cloud visible.
      })
    return () => { cancelled = true }
  }, [
    activeMapName,
    cloud.epoch,
    cloud.frameId,
    savedMapCloud,
    savedMapFlat,
    showSavedMapInScene,
  ])

  // ── Handlers ──────────────────────────────────────────────────
  const handlePendingGoal = useCallback(async (x: number, y: number) => {
    if (!canSendGoal) {
      setPendingGoal({ x, y })
      setPendingGoalPreview(null)
      showToast(goalBlockedHint, 'info')
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
  }, [canSendGoal, goalAcceptanceRadius, goalBlockedHint, goalMaxSpeed, showToast])

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

  const handleDirectedExploration = useCallback(async () => {
    if (!pendingGoal || !isExplorationSession || directedExplorationBusy) return
    if (!motionStartAllowed) {
      showToast(`不能引导探索：${motionStartBlockedReason}`, 'error')
      return
    }

    const { x, y } = pendingGoal
    setDirectedExplorationBusy(true)
    setExplorationStatusLoading(true)
    try {
      const status = await api.fetchExplorationStatus()
      setExplorationStatus(status)
      const nativeTareActive = status.tare?.runtime === 'native_dds'
      if (
        !status.available
        || status.backend !== 'tare'
        || !status.exploring
        || !nativeTareActive
      ) {
        showToast(`不能引导探索: ${status.reason || '活动原生 TARE 探索未就绪'}`, 'error')
        return
      }

      await api.setDirectedExplorationTarget(x, y, {
        ttl_s: api.DIRECTED_EXPLORATION_TTL_S,
        reason: 'web_scene_selected_point',
      })
      setPendingGoal(null)
      setPendingGoalPreview(null)
      showToast(
        `已引导探索至 (${x.toFixed(2)}, ${y.toFixed(2)})，持续 ${api.DIRECTED_EXPLORATION_TTL_S} 秒`,
        'success',
      )
    } catch (e: unknown) {
      showToast(api.formatCommandError(e, '引导探索失败'), 'error')
    } finally {
      setExplorationStatusLoading(false)
      setDirectedExplorationBusy(false)
    }
  }, [
    directedExplorationBusy,
    isExplorationSession,
    motionStartAllowed,
    motionStartBlockedReason,
    pendingGoal,
    showToast,
  ])

  const handleSaveCurrentLocation = useCallback(async () => {
    if (!poseAvailable) {
      showToast('当前没有有效里程计，无法保存位置', 'error')
      return
    }
    if (!normalizedLocationName) {
      showToast('请先输入位置名称', 'error')
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
      showToast(`已保存位置：${normalizedLocationName}`, 'success')
    } catch (e: unknown) {
      showToast(`保存位置失败：${e instanceof Error ? e.message : String(e)}`, 'error')
    } finally {
      setLocationBusy(null)
    }
  }, [normalizedLocationName, poseAvailable, robotX, robotY, yaw, showToast])

  const handleNavigateLocation = useCallback(async (loc: LocationEntry) => {
    if (!canSendGoal) {
      showToast(`无法下发目标：${goalDisabledReason}`, 'error')
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
    if (!poseAvailable) {
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
  }, [poseAvailable, robotX, robotY, yaw, showToast])

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

  const handleRestartLocalization = useCallback(async () => {
    if (restartLocalizationPending) return
    setRestartLocalizationPending(true)
    try {
      if (!session?.active_map) throw new Error('当前没有激活地图')
      const result = await api.globalRelocalize(session.active_map)
      showToast(result.message || (result.ok ? '重定位已接受' : '重定位被拒绝'), result.ok ? 'success' : 'error')
    } catch (e: unknown) {
      showToast(`重定位失败：${e instanceof Error ? e.message : String(e)}`, 'error')
    } finally {
      setRestartLocalizationPending(false)
    }
  }, [restartLocalizationPending, session?.active_map, showToast])

  const handleSaveMap = () => setSaveModalOpen(true)

  const confirmSaveMap = async (name: string) => {
    setSaveModalOpen(false)
    setSaveStatus({
      name,
      state: 'saving',
      detail: '正在写入点云、清理动态点并生成导航地图。完成后会显示保存位置。',
    })
    showToast(`正在保存并清洗动态障碍: ${name}…`, 'info')
    try {
      const admission = await api.saveMap(name)
      const r = await api.waitForMapSaveOperation(admission)
      const summary = formatSaveMapSummary(r)
      const location = formatSaveMapLocation(r, name)
      const detail = formatSaveMapDetail(r)
      setSaveStatus({
        name,
        state: 'saved',
        detail,
        location,
        summary,
      })
      const df = r.dynamic_filter
      if (df && df.success && df.dropped !== undefined && df.orig_count) {
        const pct = (100 * df.dropped / df.orig_count).toFixed(1)
        showToast(`已保存: ${name} · 清除 ${df.dropped} 动态点 (${pct}%)`, 'success')
      } else {
        showToast(`已保存: ${name}`, 'success')
      }
      showToast(`Map save details: ${summary}`, r.warnings?.length ? 'info' : 'success')
      loadMaps()
    } catch (e: unknown) {
      const message = e instanceof Error ? e.message : String(e)
      setSaveStatus({
        name,
        state: 'failed',
        detail: message || '保存失败，请检查 Gateway 日志。',
      })
      showToast('保存失败', 'error')
    }
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
      showToast('请先输入地图名称', 'error')
      return
    }
    if (!sourcePath) {
      showToast('请先填写 PCD 文件路径', 'error')
      return
    }
    setWorkbenchBusy('import')
    try {
      const res = await api.importPcdMap(name, sourcePath, Number(workbenchVoxelSize) || 0)
      publishWorkbenchResult(res)
      loadMaps()
    } catch (e: unknown) {
      showToast(`导入 PCD 失败：${e instanceof Error ? e.message : String(e)}`, 'error')
    } finally {
      setWorkbenchBusy(null)
    }
  }, [loadMaps, publishWorkbenchResult, showToast, workbenchImportPath, workbenchMapName, workbenchVoxelSize])

  const handleWorkbenchCrop = useCallback(async () => {
    if (!workbenchTargetMapName) {
      showToast('请先选择或输入地图名称', 'error')
      return
    }
    setWorkbenchBusy('crop')
    try {
      const bounds = parseWorkbenchBounds(workbenchBoundsJson)
      const res = await api.cropMap(workbenchTargetMapName, bounds)
      publishWorkbenchResult(res)
      loadMaps()
    } catch (e: unknown) {
      showToast(`裁剪失败：${e instanceof Error ? e.message : String(e)}`, 'error')
    } finally {
      setWorkbenchBusy(null)
    }
  }, [loadMaps, publishWorkbenchResult, showToast, workbenchBoundsJson, workbenchTargetMapName])

  const handleWorkbenchBuildOctomap = useCallback(async () => {
    if (!workbenchTargetMapName) {
      showToast('请先选择或输入地图名称', 'error')
      return
    }
    setWorkbenchBusy('build')
    try {
      const res = await api.buildMapOctomap(workbenchTargetMapName)
      publishWorkbenchResult(res)
      loadMaps()
    } catch (e: unknown) {
      showToast(`构建 OctoMap 失败：${e instanceof Error ? e.message : String(e)}`, 'error')
    } finally {
      setWorkbenchBusy(null)
    }
  }, [loadMaps, publishWorkbenchResult, showToast, workbenchTargetMapName])

  const handleWorkbenchMarkZone = useCallback(async () => {
    if (!workbenchTargetMapName) {
      showToast('请先选择或输入地图名称', 'error')
      return
    }
    if (!pendingGoal) {
      showToast('请先在场景中选择一个点', 'error')
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
      showToast(`标记区域失败：${e instanceof Error ? e.message : String(e)}`, 'error')
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
      showToast('请先选择或输入地图名称', 'error')
      return
    }
    if (!pendingGoal) {
      showToast('请先在场景中选择目标点', 'error')
      return
    }
    setWorkbenchBusy('preview')
    try {
      const res = await api.validateMapPlan(workbenchTargetMapName, pendingGoal.x, pendingGoal.y, 0)
      const preview = res as unknown as PlanPreviewResponse
      const summary = [
        preview.ok ? 'plan ok' : 'plan blocked',
        preview.planner ?? '',
        formatPlanSummary(preview),
      ].filter(Boolean).join(' | ')
      setWorkbenchSummary(summary)
      showToast(summary, preview.ok ? 'success' : 'error')
    } catch (e: unknown) {
      showToast(`路径预览失败：${e instanceof Error ? e.message : String(e)}`, 'error')
    } finally {
      setWorkbenchBusy(null)
    }
  }, [pendingGoal, showToast, workbenchTargetMapName])

  const handleActivate = (name: string) => {
    if (mapSwitchBusy) {
      showToast(`地图 ${mapSwitchBusy} 正在加载预览，请等待完成`, 'info')
      return
    }
    setRelocMap(name)
    setLoadTarget(name)
  }

  const confirmLoadMap = async () => {
    if (!loadTarget || mapSwitchBusy) return
    const name = loadTarget
    setMapSwitchBusy(name)
    try {
      const savedMap = await api.fetchSavedMapPointCloud(name)
      setSavedMapCloud(savedMap)
      showToast(`已预览 ${name}`, 'info')
      loadMaps()
    } catch (e: unknown) {
      showToast(`加载失败: ${e instanceof Error ? e.message : String(e)}`, 'error')
    } finally {
      setMapSwitchBusy(null)
      setLoadTarget(null)
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
      if (savedMapFlat !== undefined) setSavedMapCloud(undefined)
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
      showToast(api.formatCommandAck(res, '取消导航'), 'info')
    } catch (e: unknown) {
      showToast(api.formatCommandError(e, '取消导航失败'), 'error')
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
        const savedMap = await api.fetchSavedMapPointCloud(relocMap)
        setSavedMapCloud(savedMap)
      } catch { /* PCD not available — ignore */ }
    } catch (e: unknown) {
      showToast(`重定位失败: ${e instanceof Error ? e.message : String(e)}`, 'error')
    } finally {
      setRelocPending(false)
    }
  }

  const handleGlobalRelocalize = async () => {
    if (!savedMapRelocalizeSupported) {
      showToast(relocalizeUnavailableMessage, 'error')
      return
    }
    if (!relocMap) {
      showToast('请先选择地图', 'error')
      return
    }
    setRelocPending(true)
    try {
      const res = await api.globalRelocalize(relocMap)
      const message = res.message || res.status || (res.ok ? 'accepted' : 'rejected')
      showToast(res.ok ? `全局重定位: ${message}` : `全局重定位失败: ${message}`, res.ok ? 'success' : 'error')
      if (res.ok && relocMap) {
        try {
          const savedMap = await api.fetchSavedMapPointCloud(relocMap)
          setSavedMapCloud(savedMap)
        } catch { /* PCD not available — ignore */ }
      }
    } catch (e: unknown) {
      showToast(`全局重定位失败: ${e instanceof Error ? e.message : String(e)}`, 'error')
    } finally {
      setRelocPending(false)
    }
  }

  const toggleLayer = useCallback((key: keyof Layers) => {
    setLayers(prev => {
      const enabled = !prev[key]
      if (key === 'elevation') onElevationSubscriptionChange?.(enabled)
      return { ...prev, [key]: enabled }
    })
  }, [onElevationSubscriptionChange])

  const LayerBtn = useCallback(({
    k, icon, label,
  }: { k: keyof Layers; icon: ReactNode; label: string }) => (
    <LayerButton
      active={layers[k]}
      icon={icon}
      label={label}
      onClick={() => toggleLayer(k)}
    />
  ), [layers, toggleLayer])

  return (
    <div className={styles.sceneView}>
      {/* Toolbar */}
      <div className={styles.toolbar}>
        <span className={styles.toolbarTitle}>
          <span className={styles.toolbarTitleIcon}><Compass size={13} /></span>
          {text(locale, 'Scene View', '场景视图')}
        </span>

        <span className={styles.divider} />

        <div className={styles.layerGroup}>
          <LayerBtn k="grid"  icon={<Grid3x3 size={11} />}   label="网格"  />
          <LayerBtn k="cloud" icon={<Cloud size={11} />}      label="点云"  />
          <LayerBtn k="trail" icon={<Route size={11} />}      label="轨迹"  />
          <LayerBtn k="path"  icon={<Navigation size={11} />} label="路径"  />
          <LayerBtn k="goal"  icon={<Target size={11} />}     label="目标"  />
          <LayerBtn k="robot"   icon={<Bot size={11} />}        label="本机"  />
          <LayerBtn k="elevation" icon={<MapPinned size={11} />} label="最低观测高程" />
          <LayerBtn k="nativeTraversability" icon={<Activity size={11} />} label="控制可通行性"  />
          <LayerBtn k="localPlanner" icon={<Radio size={11} />} label="局部安全诊断（采样）" />
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

        <button className={styles.toolbarBtn} onClick={() => scene3DRef.current?.resetCamera()} title="仅复位 3D 视角，不重启定位">
          <Maximize2 size={12} /> 视角复位
        </button>
        {sceneDebugTools && (
          <>
            <button className={styles.toolbarBtn} onClick={handleClearTrail}>
              <Trash2 size={12} /> 清除轨迹
            </button>
            <button className={styles.toolbarBtn} onClick={handleClearCloud} title="仅清理浏览器显示，不修改底层定位">
              <Cloud size={12} /> 清除点云
            </button>
          </>
        )}
        <button
          className={styles.toolbarBtnPrimary}
          onClick={handleRestartLocalization}
          disabled={restartLocalizationPending}
          title="在当前地图上重新定位"
        >
          <RefreshCw size={12} /> {restartLocalizationPending ? '重定位中...' : '重新定位'}
        </button>
        <button
          className={styles.toolbarBtn}
          onClick={() => setRelocOpen(v => !v)}
          disabled={!savedMapRelocalizeSupported}
          title={savedMapRelocalizeSupported ? '打开保存地图重定位面板；Shift+点击场景可设置初始位姿' : relocalizeUnavailableMessage}
        >
          <LocateFixed size={12} /> 手动重定位
        </button>
        <button
          className={styles.toolbarBtn}
          onClick={handleGlobalRelocalize}
          disabled={relocPending || !savedMapRelocalizeSupported}
          title={savedMapRelocalizeSupported ? '基于当前保存地图执行全局重定位' : relocalizeUnavailableMessage}
        >
          <LocateFixed size={12} /> 自动匹配
        </button>
        <button className={styles.toolbarBtnPrimary} onClick={handleSaveMap}>
          <Save size={12} /> 保存地图
        </button>
        <button
          className={styles.toolbarBtn}
          onClick={() => setTeleopPanelOpen(open => !open)}
          aria-expanded={teleopPanelOpen}
          aria-controls="scene-teleop"
          title="Web 遥控入口：只在 teleop/teleop_avoid Product 且 Gateway 声明 teleop_ws 时可用"
        >
          <Gamepad2 size={12} />
          {teleopPanelOpen ? '收起遥控' : '遥控'}
        </button>
        <button
          className={styles.toolbarBtn}
          onClick={() => setRecordingPanelOpen(open => !open)}
          aria-expanded={recordingPanelOpen}
          aria-controls="scene-recording"
          title={recordingToolbarTitle}
        >
          <CircleDot size={12} />
          {recordingPanelOpen ? '收起录制' : '录制'}
          {recordingToolbarState && <span aria-live="polite"> · {recordingToolbarState}</span>}
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
                  onClick={() => setSavedMapCloud(undefined)}
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
            {sceneDebugTools && (
              <div className={styles.mapWorkbench}>
                <div className={styles.workbenchHeader}>
                  <span>地图调试</span>
                  <span className={styles.workbenchActiveMap}>
                    {workbenchTargetMapName || '未选择地图'}
                  </span>
                </div>
                <input
                  className={styles.workbenchInput}
                  value={workbenchMapName}
                  onChange={(e) => setWorkbenchMapName(e.target.value)}
                  placeholder={showSavedMapInScene && activeMapName ? `当前地图：${activeMapName}` : '地图名称'}
                />
                <input
                  className={styles.workbenchInput}
                  value={workbenchImportPath}
                  onChange={(e) => setWorkbenchImportPath(e.target.value)}
                  placeholder="网关主机上的 PCD 路径"
                />
                <div className={styles.workbenchRow}>
                  <input
                    className={styles.workbenchInput}
                    value={workbenchVoxelSize}
                    onChange={(e) => setWorkbenchVoxelSize(e.target.value)}
                    placeholder="体素尺寸 m"
                    inputMode="decimal"
                  />
                  <button
                    type="button"
                    className={styles.workbenchButton}
                    disabled={workbenchBusy != null}
                    onClick={handleWorkbenchImportPcd}
                  >
                    导入 PCD
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
                    裁剪
                  </button>
                  <button
                    type="button"
                    className={styles.workbenchButton}
                    disabled={workbenchBusy != null}
                    onClick={handleWorkbenchBuildOctomap}
                  >
                    构建 OctoMap
                  </button>
                </div>
                <div className={styles.workbenchRow}>
                  <select
                    className={styles.workbenchInput}
                    value={workbenchZoneState}
                    onChange={(e) => setWorkbenchZoneState(e.target.value as WorkbenchZoneState)}
                  >
                    <option value="preblocked">预阻挡</option>
                    <option value="traversable">可通行</option>
                    <option value="clear">清除</option>
                  </select>
                  <input
                    className={styles.workbenchInput}
                    value={workbenchZoneRadius}
                    onChange={(e) => setWorkbenchZoneRadius(e.target.value)}
                    placeholder="半径 m"
                    inputMode="decimal"
                  />
                </div>
                <div className={styles.workbenchRow}>
                  <button
                    type="button"
                    className={styles.workbenchButton}
                    disabled={workbenchBusy != null || !pendingGoal}
                    onClick={handleWorkbenchMarkZone}
                    title={pendingGoal ? '在当前 OctoMap 标记选中点' : '请先点击地图点'}
                  >
                    标记区域
                  </button>
                  <button
                    type="button"
                    className={styles.workbenchButton}
                    disabled={workbenchBusy != null || !pendingGoal}
                    onClick={handleWorkbenchValidatePlan}
                    title={pendingGoal ? '执行不下发运动的路径预览' : '请先点击目标点'}
                  >
                    预览路径
                  </button>
                </div>
                {pendingGoal && (
                  <div className={styles.workbenchHint}>
                    点位 {pendingGoal.x.toFixed(2)}, {pendingGoal.y.toFixed(2)}
                  </div>
                )}
                {workbenchSummary && (
                  <div className={styles.workbenchHint} title={workbenchSummary}>
                    {workbenchSummary}
                  </div>
                )}
              </div>
            )}
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
                      className={m.is_active && showSavedMapInScene ? styles.mapItemActive : styles.mapItem}
                      onClick={() => handleActivate(m.name)}
                      disabled={mapSwitchBusy !== null}
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
              scanCloud={alignedScanCloud}
              savedMapFlat={savedMapForScene?.points}
              savedMapFrameId={savedMapForScene?.frameId}
              savedMapEpoch={savedMapForScene?.epoch}
              elevationState={elevationState}
              nativeTraversabilityState={nativeTraversabilityState}
              sceneGraph={mapSceneGraph}
              robotX={robotX}
              robotY={robotY}
              robotValid={poseAvailable}
              yaw={yaw}
              trail={trail}
              path={path}
              localPath={localPathPts}
              localPlannerSnapshot={localPlannerSnapshot}
              layers={layers}
              pointSize={pointSize}
              onPendingGoal={handlePendingGoal}
              onRelocalize={handleSceneRelocalize}
              pendingGoal={pendingGoal}
            />
            <div className={styles.canvasOverlayTop}>
              <span className={styles.scaleLabel}>3D 场景视图  ·  拖拽旋转  ·  滚轮缩放  ·  点击放置目标  ·  Shift+点击重定位</span>
            </div>
            {recordingPanelOpen && (
              <FloatingWidget
                id="scene-recording"
                defaultPos={{ x: 18, y: 18 }}
                defaultSize={{ w: 420, h: 650 }}
                minSize={{ w: 320, h: 360 }}
              >
                <RecordingPanel
                  embedded
                  showToast={showToast}
                  locale={locale}
                  status={recordingStatus}
                  statusError={recordingStatusError}
                  refreshStatus={refreshRecordingStatus}
                  onClose={() => setRecordingPanelOpen(false)}
                />
              </FloatingWidget>
            )}
            {teleopPanelOpen && (
              <FloatingWidget
                id="scene-teleop"
                defaultPos={{ x: 456, y: 18 }}
                defaultSize={{ w: 360, h: 470 }}
                minSize={{ w: 320, h: 360 }}
              >
                <TeleopPanel
                  sseState={sseState}
                  showToast={showToast}
                />
              </FloatingWidget>
            )}
            {(layers.elevation || layers.nativeTraversability || layers.localPlanner) && (
              <div className={styles.sceneLegendStack} aria-live="polite">
                {layers.elevation && (
                  <div
                    className={sceneLayerLegendClass(elevationState.status)}
                    aria-label="最低观测高程图层图例"
                  >
                    <strong>最低观测高程</strong>
                    {elevationState.status === 'ready' && (
                      <>
                        <span className={styles.elevationRamp} />
                        <span className={styles.legendRange}>
                          <b>{elevationState.minZ.toFixed(2)} m</b>
                          <b>{elevationState.maxZ.toFixed(2)} m</b>
                        </span>
                      </>
                    )}
                    <small>{elevationState.message}</small>
                  </div>
                )}
                {layers.nativeTraversability && (
                  <div
                    className={sceneLayerLegendClass(nativeTraversabilityState.status)}
                    aria-label="控制可通行性图例"
                  >
                    <strong>控制可通行性</strong>
                    <div className={styles.costLegendItems}>
                      <span><i className={styles.costSoft} />0 低运动风险</span>
                      <span><i className={styles.costLethal} />100 native 控制风险</span>
                    </div>
                    <small>{nativeTraversabilityState.message}</small>
                  </div>
                )}
                {layers.localPlanner && (
                  <div className={styles.localPlannerLegend} aria-label="局部安全诊断（采样）图层图例">
                    <strong>局部安全诊断（采样）</strong>
                    <span><i className={`${styles.legendMark} ${styles.legendObstacle}`} />障碍点</span>
                    <span><i className={`${styles.legendMark} ${styles.legendTerrain}`} />Terrain 风险</span>
                    <span><i className={`${styles.legendLine} ${styles.legendCandidate}`} />候选轨迹</span>
                    <span><i className={`${styles.legendLine} ${styles.legendSelected}`} />选中轨迹</span>
                    {localPlannerSampleWarningText && <small>{localPlannerSampleWarningText}</small>}
                  </div>
                )}
              </div>
            )}
            {pendingGoal && (
              <div className={styles.goalConfirmPanel}>
                <span className={styles.goalConfirmLabel}>导航目标</span>
                <span className={styles.goalConfirmCoords}>
                  ({pendingGoal.x.toFixed(2)}, {pendingGoal.y.toFixed(2)})
                </span>
                <div className={styles.goalControlGroup}>
                  <label className={styles.goalControl}>
                    <span>速度</span>
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
                    <span>半径</span>
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
                {isExplorationSession && (
                  <button
                    className={styles.goalConfirmBtn}
                    onClick={handleDirectedExploration}
                    disabled={directedExplorationBusy || explorationStatusLoading || !motionStartAllowed}
                    title={!motionStartAllowed
                      ? motionStartBlockedReason
                      : explorationStatus?.exploring
                      ? '当前 TARE 探索运行中；将此点作为探索偏好，持续 30 秒'
                      : '将此点作为 TARE 探索偏好，持续 30 秒'}
                  >
                    <Route size={12} /> {directedExplorationBusy ? '引导中…' : '引导探索至此 (30 秒)'}
                  </button>
                )}

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
                <span className={`${styles.statValueHl} ${styles.robotPositionValue}`}>{displayRobotX}</span>
              </div>
              <div className={styles.statItem}>
                <span className={styles.statLabel}>Y</span>
                <span className={`${styles.statValueHl} ${styles.robotPositionValue}`}>{displayRobotY}</span>
              </div>
              <div className={styles.statItem}>
                <span className={styles.statLabel}>航向</span>
                <span className={`${styles.statValue} ${styles.robotYawValue}`}>{displayYawDeg}</span>
              </div>
              <div className={styles.statItem}>
                <span className={styles.statLabel}>线速度</span>
                <span className={`${styles.statValue} ${styles.robotSpeedValue}`}>{displaySpeed}</span>
              </div>
            </div>
          </div>

          <div className={styles.statCard}>
            <div className={styles.statCardTitle}>导航任务</div>
            <div className={styles.statGrid}>
              <div className={styles.statItem} style={{ gridColumn: '1 / span 2' }}>
                <span className={styles.statLabel}>状态</span>
                <span className={hasGoal ? styles.goalBadgeActive : styles.goalBadgeIdle}>
                  {missionStateLabel}
                </span>
              </div>
              <div className={styles.statItem} style={{ gridColumn: '1 / span 2' }}>
                <span className={styles.statLabel}>目标准入</span>
                <span className={styles.statValueDim}>
                  {operatorView.goalAdmission.label}
                </span>
              </div>
              <div className={styles.statItem} style={{ gridColumn: '1 / span 2' }}>
                <span className={styles.statLabel}>控制权</span>
                <span className={styles.statValueDim}>
                  {operatorView.control.label}
                  {operatorView.control.resumeRequired ? text(locale, ' · Resume required', ' · 需要恢复') : ''}
                </span>
              </div>
              <div className={styles.statItem} style={{ gridColumn: '1 / span 2' }}>
                <span className={styles.statLabel}>运动</span>
                <span className={styles.statValueDim}>
                  {operatorView.motion.permission.label} · {operatorView.motion.observation.label}
                </span>
              </div>
              <div className={styles.statItem} style={{ gridColumn: '1 / span 2' }}>
                <span className={styles.statLabel}>停稳确认</span>
                <span className={styles.statValueDim}>
                  {operatorView.motion.stopConfirmation.label}
                </span>
              </div>
              <div className={styles.statItem} style={{ gridColumn: '1 / span 2' }}>
                <span className={styles.statLabel}>目标</span>
                <span className={styles.statValueDim}>
                  {missionGoal ?? '无'}
                </span>
              </div>
              <div className={styles.statItem} style={{ gridColumn: '1 / span 2' }}>
                <span className={styles.statLabel}>下一步</span>
                <span className={styles.statValueDim}>
                  {operatorView.summary.nextAction}
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
                disabled={!poseAvailable || !normalizedLocationName || locationBusy !== null}
                title={poseAvailable ? '保存当前机器人位姿' : '当前没有有效里程计'}
              >
                <Save size={12} />
              </button>
            </div>
            <div className={styles.locationList}>
              {savedLocations.length === 0 && (
                <div className={styles.locationEmpty}>暂无保存位置</div>
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
                        disabled={!poseAvailable || disabled}
                        title={poseAvailable ? 'Update to current pose' : 'No valid odometry'}
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
              SLAM 状态
            </div>
            <div className={`${styles.statGrid} ${styles.statGridThree}`}>
              <div className={styles.statItem}>
                <span className={styles.statLabel} title="定位处理后的扫描输出频率；原始雷达输入单独显示。">扫描</span>
                <span className={styles.statValue}>{formatHz(processedScanHz)}</span>
              </div>
              <div className={styles.statItem}>
                <span className={styles.statLabel}>雷达</span>
                <span className={styles.statValue}>{formatHz(lidarInputHz)}</span>
              </div>
              <div className={styles.statItem}>
                <span className={styles.statLabel}>点数</span>
                <span className={styles.statValue}>{formatCount(displayedMapPoints)}</span>
              </div>
            </div>

            <button
              className={styles.relocBtn}
              onClick={() => setRelocOpen(v => !v)}
              disabled={!savedMapRelocalizeSupported}
              title={savedMapRelocalizeSupported ? '支持保存地图重定位' : relocalizeUnavailableMessage}
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
                  <label>航向</label>
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
            title={hasGoal ? '取消当前导航任务' : '当前没有导航任务'}
          >
            <X size={15} />
            取消导航
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
        title="预览点云地图"
        message={`在场景中预览「${loadTarget ?? ''}」的保存点云，不改变机器人当前任务或激活地图。`}
        confirmLabel="预览地图"
        busy={mapSwitchBusy !== null}
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

      <PromptModal
        open={saveModalOpen}
        title="保存当前地图"
        message="保存当前 SLAM 建图结果。完成后会出现在左侧地图列表，并在左侧“保存结果”显示保存位置和处理摘要。"
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
