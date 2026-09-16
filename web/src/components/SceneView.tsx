import { isObservationMode } from '../services/observationMode.ts'
import { usePlanningMap } from '../hooks/usePlanningMap.ts'
import { navigationPreviewIsCurrent, planningCellAt, planningCellLabel, planningMapUnavailableLabel } from '../services/planningMap.ts'
import { resolveMappingObservation, mappingObservationPointAt } from '../services/mappingObservation.ts'
import { estimateSceneTime, freshSource, projectScenePose, projectScenePath, scanCanStandAlone, currentNativeLocalPath, scenePoseEpoch, sceneTrailStorageKey, sceneRelocalizationSeed, activeMapRelocalizationTarget } from '../services/sceneTelemetry.ts'
import { useRef, useEffect, useCallback, useMemo, useState, memo, type ReactNode } from 'react'
import {
  Grid3x3, Navigation, Route, Target, Bot,
  ArrowLeft, Save, Trash2, Pencil, X,
  MapPinned, Cloud, Maximize2, Radio, Activity, LocateFixed, VideoOff, CircleDot,
  RefreshCw, Gamepad2, Layers2, Camera, SlidersHorizontal, MoreHorizontal, Info, MousePointer2,
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
  CameraMediaStatus,
} from '../types'
import * as api from '../services/api'
import {
  mapIsActivationReady,
  mapSaveBlockedReason,
} from '../services/mapReadiness'
import { presentNavigationStatus } from '../services/navigationStatus'
import { useCamera } from '../hooks/useCamera'
import { useBinaryCloud } from '../hooks/useBinaryCloud'
import {
  cloudFramesShareCoordinateEpoch,
  savedMapNeedsSceneRebind,
} from '../workers/cloudDecoderCore.ts'
import { PromptModal, ConfirmModal } from './Modal'
import { Scene3D, type Scene3DHandle } from './Scene3D'
import { RobotJointStatus } from './RobotJointStatus.tsx'
import { safetyEnvelopeFromRunPlan, type SafetyEnvelope } from './scene3d/layers/safetyEnvelope.ts'
import { RecordingPanel } from './RecordingPanel'
import { TeleopPanel } from './TeleopPanel'
import type { Locale } from '../i18n'
import styles from './SceneView.module.css'
import {
  LOCAL_PLANNER_DIAGNOSTICS_POLL_MS,
  localPlannerSampleWarning,
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
  onOpenSavedMap: (name: string | null) => void
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
const GOAL_SPEED_OPTIONS = [0.2, 0.25, 0.4, 0.5, 0.6]
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
      aria-pressed={active}
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
  onOpenSavedMap,
}: SceneViewProps) {
  const scene3DRef = useRef<Scene3DHandle>(null)
  const savePreviewSession = useRef<string | null>(null)
  useEffect(() => {
    savePreviewSession.current = sseState.session?.product_session_id ?? ''
    return () => { savePreviewSession.current = null }
  }, [sseState.session?.product_session_id])
  const recordingStatusPollInFlight = useRef(false)
  const observe = isObservationMode()
  const sceneDebugTools = showSceneDebugTools()
  const robotModel = sseState.robotModel === 'unitree/go2' ? 'go2'
    : sseState.robotModel === 'doso/thunder_v4' ? 'thunder_v4' : undefined

  // History belongs to one map and one published SLAM coordinate era.
  const [trailState, setTrailState] = useState<{ key: string | null; points: Array<[number, number, number]> }>({ key: null, points: [] })

  const [drawerOpen, setDrawerOpen] = useState(false)
  const [inspectorOpen, setInspectorOpen] = useState(false)
  const [workspaceTool, setWorkspaceTool] = useState<'maps' | 'locations' | 'localization'>('maps')
  const [inspectorTab, setInspectorTab] = useState<'status' | 'layers' | 'tools'>('status')
  const inspectorTabs = [
    { key: 'status', label: '状态', icon: Activity },
    { key: 'layers', label: '图层', icon: Layers2 },
    ...(!observe ? [{ key: 'tools' as const, label: '操作', icon: SlidersHorizontal }] : []),
  ] as const
  const dismissOnlyCanvasClick = useRef(false)
  const openWorkspaceTool = (tool: 'maps' | 'locations' | 'localization') => {
    setWorkspaceTool(tool)
    setTeleopMode(false)
    setDrawerOpen(true)
    if (tool === 'localization') setRelocOpen(true)
  }
  const [followRobot, setFollowRobot] = useState(true)
  const [cameraPreference, setCameraPreference] = useState<boolean | null>(null)
  const cameraVisible = cameraPreference ?? sseState.session?.product !== 'map'
  const [cameraStatus, setCameraStatus] = useState<CameraMediaStatus | null>(null)
  const [saveModalOpen, setSaveModalOpen] = useState(false)
  const [recordingPanelOpen, setRecordingPanelOpen] = useState(false)
  const [teleopMode, setTeleopMode] = useState(false)
  const [resumePending, setResumePending] = useState(false)
  const [resumeError, setResumeError] = useState<string | null>(null)
  const [liveScanPreference, setLiveScanPreference] = useState<boolean | null>(null)
  const [mapView, setMapView] = useState<'planning' | 'points'>('planning')
  const [mappingView, setMappingView] = useState<'coverage' | 'points' | 'global'>('global')
  const [mappingProbe, setMappingProbe] = useState<{ x: number; y: number; epoch: string | null } | null>(null)
  const [recordingStatus, setRecordingStatus] = useState<RecordingStatusResponse | null>(null)
  const [recordingStatusError, setRecordingStatusError] = useState<string | null>(null)
  const [layers, setLayers] = useState<Layers>({
    grid: true, cloud: true, trail: false, path: true, goal: true, robot: true,
    elevation: false, nativeTraversability: false,
    localPlanner: false,
  })
  const [localNowS, setLocalNowS] = useState(() => Date.now() / 1000)
  const rasterNowS = estimateSceneTime(localNowS, sseState.stateSnapshot?.ts, sseState.stateSnapshotReceivedAt)
  const [localPlannerSnapshot, setLocalPlannerSnapshot] = useState<NavigationDdsSnapshotResponse | null>(null)
  const [safetyView, setSafetyView] = useState<'slice' | 'volume'>('slice')
  const [envelopeConfig, setEnvelopeConfig] = useState<{ session: string; shape: SafetyEnvelope | null } | null>(null)
  const [maps, setMaps] = useState<MapInfo[]>([])
  const [mapListStatus, setMapListStatus] = useState<'loading' | 'ready' | 'error'>('loading')
  const mapListRequestInFlight = useRef(false)
  const [pointSize, setPointSize] = useState(0.12)
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
  const [pendingGoal, setPendingGoal] = useState<{ x: number; y: number; z?: number } | null>(null)
  const [pendingGoalPreview, setPendingGoalPreview] = useState<PlanPreviewResponse | null>(null)
  const [goalPreviewPending, setGoalPreviewPending] = useState(false)
  const [goalSendPending, setGoalSendPending] = useState(false)
  const [goalPreviewError, setGoalPreviewError] = useState<string | null>(null)
  const goalPreviewRequest = useRef(0)
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
    if (observe) return
    void refreshRecordingStatus()
    const timer = window.setInterval(
      () => void refreshRecordingStatus(),
      RECORDING_STATUS_POLL_MS,
    )
    return () => window.clearInterval(timer)
  }, [refreshRecordingStatus, observe])

  useEffect(() => {
    setLocalNowS(Date.now() / 1000)
    const timer = window.setInterval(() => setLocalNowS(Date.now() / 1000), 1000)
    return () => window.clearInterval(timer)
  }, [])

  const { imgSrc: cameraImgSrc, connected: cameraConnected, lastFrameAt: cameraLastFrameAt } = useCamera(cameraVisible ? '/ws/camera' : '')
  const cameraPipRecovered = sseState.connected && cameraConnected && cameraImgSrc != null
    && cameraLastFrameAt != null && Date.now() - cameraLastFrameAt < 3000
  const cameraPipLabel = cameraPipRecovered ? '实时画面'
    : !sseState.connected ? '连接已断开'
      : cameraStatus?.status === 'not_loaded' ? '相机未启用'
        : cameraLastFrameAt ? '画面已暂停' : '等待相机画面'
  useEffect(() => {
    if (!cameraVisible || !sseState.connected) return
    let disposed = false
    let timer: ReturnType<typeof setTimeout>
    const poll = async () => {
      try {
        const status = await api.fetchCameraStatus()
        if (!disposed) setCameraStatus(status)
      } catch {
        if (!disposed) setCameraStatus(null)
      } finally {
        if (!disposed) timer = setTimeout(poll, 3000)
      }
    }
    void poll()
    return () => { disposed = true; clearTimeout(timer) }
  }, [cameraVisible, sseState.connected])
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
  const localCloud = useBinaryCloud('/ws/cloud', '/api/v1/map/points?max_points=60000', 4)
  const showGlobalMapping = sseState.session?.product === 'map' && mappingView === 'global'
  const globalCloud = useBinaryCloud(null,
    showGlobalMapping ? '/api/v1/map/global/points?max_points=120000' : null, 1)
  const cloud = showGlobalMapping ? globalCloud : localCloud
  const scanCloud = useBinaryCloud(scanOverlayEnabled ? '/ws/scan' : null, null, 10)
  const alignedScanCloud = scanOverlayEnabled
    && freshSource(scanCloud.stampS, rasterNowS)
    && (cloudFramesShareCoordinateEpoch(cloud, scanCloud) || scanCanStandAlone(scanCloud, cloud))
    ? scanCloud
    : null
  // Live motion status and source-timestamped paths remain available even
  // when the optional diagnostic obstacle layer is hidden.
  const localPlannerDiagnosticsEnabled = true

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

  const localization = sseState.stateSnapshot?.localization
  const poseEpoch = scenePoseEpoch(sseState.session?.product_session_id, sseState.slamDiag?.data ?? localization)
  const sourceTransform = localization?.map_odom_tf
  const transformStamp = sourceTransform && typeof sourceTransform === 'object'
    ? (sourceTransform as Record<string, unknown>).ts : null
  const mapFromOdom = freshSource(transformStamp, rasterNowS, 7) ? sourceTransform : null
  const odom = sseState.odometry
  const telemetryFresh = sseState.connected && freshSource(odom?.ts, rasterNowS)
  const mapOdom = telemetryFresh ? projectScenePose(odom, mapFromOdom) : null
  const poseAvailable = mapOdom !== null
  const robotX = mapOdom?.x ?? 0
  const robotY = mapOdom?.y ?? 0
  const robotZ = mapOdom?.z ?? 0
  const yaw = mapOdom?.yaw ?? 0
  const displayRobotX = poseAvailable ? formatTelemetryValue(robotX, 2) : '--'
  const displayRobotY = poseAvailable ? formatTelemetryValue(robotY, 2) : '--'
  const displayRobotZ = poseAvailable ? formatTelemetryValue(mapOdom.z ?? 0, 2) : '--'
  const displayYawDeg = poseAvailable ? `${formatTelemetryValue((yaw * 180) / Math.PI, 0)}°` : '--'
  const displaySpeed = poseAvailable ? `${formatTelemetryValue(mapOdom.vx, 2)} m/s` : '-- m/s'
  const nativeStatus = localPlannerSnapshot?.nav_endpoint
  const nativeFresh = sseState.connected && freshSource(nativeStatus?.stamp_s, rasterNowS)
  const nativeProduct = nativeStatus?.native_product as { product_session_id?: string; session_id?: string } | undefined
  const nativeSession = nativeProduct?.product_session_id ?? nativeProduct?.session_id ?? ''
  const safetyEnvelope = nativeFresh && envelopeConfig?.session === nativeSession ? envelopeConfig.shape : null
  useEffect(() => {
    if (!sseState.connected || !nativeSession) return
    let disposed = false
    void api.fetchRuntimeDataflow().then(data => {
      if (!disposed) setEnvelopeConfig({ session: nativeSession, shape: safetyEnvelopeFromRunPlan(data.run_plan) })
    }).catch(() => {
      if (!disposed) setEnvelopeConfig(null)
    })
    return () => { disposed = true }
  }, [nativeSession, sseState.connected])
  const nativePath = (key: string): PathPoint[] => {
    const value = nativeStatus?.[key]
    if (!nativeFresh || !Array.isArray(value)) return []
    const points = value.map(p => Array.isArray(p) ? { x: p[0], y: p[1], z: p[2] ?? 0 } : p)
      .filter((p): p is PathPoint => p != null && Number.isFinite(p.x) && Number.isFinite(p.y))
    return projectScenePath(points, nativeStatus?.planning_frame_id, mapFromOdom)
  }
  const path = localPlannerDiagnosticsEnabled
    ? nativePath('global_path')
    : sseState.connected
      ? projectScenePath(sseState.globalPath?.points ?? [], sseState.globalPath?.frame_id, mapFromOdom) : []
  const localPathPts = localPlannerDiagnosticsEnabled
    ? currentNativeLocalPath(nativeStatus, sseState.connected, rasterNowS, mapFromOdom)
    : sseState.connected && freshSource(sseState.localPath?.stamp_s, rasterNowS)
      ? projectScenePath(sseState.localPath?.points ?? [], sseState.localPath?.frame_id, mapFromOdom) : []
  const mapSceneGraph = sseState.sceneGraph?.frame_id === 'map' ? sseState.sceneGraph : null
  const lastLocal = nativeStatus?.last_local as Record<string, unknown> | undefined
  const tracking = lastLocal?.tracking as Record<string, unknown> | undefined
  const teleop = nativeStatus?.teleop as Record<string, unknown> | undefined
  const request = teleop?.request as Record<string, unknown> | undefined
  const requestedMotion = request && [request.vx, request.vy, request.wz].some(v => typeof v === 'number' && Math.abs(v) > 0.001)
  const finalCommand = nativeStatus?.final_cmd_vel as Record<string, unknown> | undefined
  const commandValues = (value: Record<string, unknown> | undefined) =>
    [value?.vx, value?.vy, value?.wz].map(v => typeof v === 'number' && Number.isFinite(v) ? formatTelemetryValue(v, 2) : '—')
  const poseLabel = !sseState.connected ? '未连接'
    : !telemetryFresh ? '定位未更新'
      : !poseAvailable ? '等待定位' : '定位有效'

  const navigationStatus = sseState.navigationStatus
  const navigationFresh = sseState.connected && freshSource(navigationStatus?.ts, rasterNowS, 7)
  const navigationView = presentNavigationStatus(navigationFresh ? navigationStatus : null, locale)
  useEffect(() => {
    if (!resumePending) return
    const confirmed = navigationFresh && !navigationView.control.resumeRequired
    const timer = setTimeout(() => {
      setResumePending(false)
      setResumeError(confirmed ? null : '未收到恢复确认，请检查控制详情后重试')
    }, confirmed ? 0 : 8000)
    return () => clearTimeout(timer)
  }, [resumePending, navigationFresh, navigationView.control.resumeRequired])
  const missionState = navigationView.task.state
  const missionStateLabel = navigationView.task.label
  const hasGoal = ['PLANNING', 'EXECUTING', 'RECOVERING', 'PAUSED'].includes(missionState)
  const plannerLabel = !nativeFresh ? '规划未更新'
    : localPathPts.length > 1 && tracking?.active === true
      ? tracking.execution_frozen === true ? '轨迹已暂停' : '正在跟踪轨迹'
      : requestedMotion || hasGoal ? '等待可通行路径'
        : '无运动请求'
  const currentPlannerIssue = nativeFresh && (requestedMotion || hasGoal || missionState === 'FAILED')
    && localPathPts.length < 2 && typeof lastLocal?.reason === 'string'
    && !['superseded_by_new_goal', 'local_plan_pending'].includes(lastLocal.reason)
    && lastLocal.path_found === false ? lastLocal.reason : null
  const plannerIssueLabel = currentPlannerIssue === 'scan_initialization_failed' ? '局部规划初始化失败'
    : currentPlannerIssue === 'local_trajectory_blocked_near_robot' ? '机器人附近路径受阻'
      : currentPlannerIssue
  const motionHeld = navigationView.motion.permission.state === 'HELD'
    || navigationView.motion.permission.state === 'ESTOPPED'
  const plannerAttention = missionState === 'FAILED' ? plannerIssueLabel : null
  const operatorAttention = navigationView.control.resumeRequired
    || navigationView.motion.permission.state === 'ESTOPPED' || !!plannerAttention
  const slamHz       = sseState.slamStatus?.slam_hz ?? 0
  const slamDiag = sseState.slamDiag?.data ?? {}
  const processedScanHz = numericMetric(slamDiag, 'processed_scan_hz') ?? slamHz
  const lidarInputHz = numericMetric(slamDiag, 'lidar_input_hz')
  const displayedMapPoints = numericMetric(slamDiag, 'map_points') ?? sseState.slamStatus?.map_points
  const rawSession = sseState.session
  const session = rawSession
  const saveBlockedReason = mapSaveBlockedReason(session)
  const localizationBackend = session?.localization_backend ?? session?.slam_profile ?? sseState.slamStatus?.mode ?? 'unknown'
  const activeMap = sseState.session?.active_map
  const trailKey = sceneTrailStorageKey(activeMap, poseEpoch)
  const trail = trailState.key === trailKey ? trailState.points : []
  const savedMapRelocalizeSupported =
    session?.saved_map_relocalization_supported ??
    session?.relocalization_supported ??
    false
  const recoveryMethod = session?.recovery_method ?? '--'
  const relocalizeUnavailableMessage =
    `当前后端 ${localizationBackend} 不支持保存地图重定位；恢复方式：${recoveryMethod}`
  const canAcceptGoal = navigationView.goalAdmission.state === 'ACCEPTING'
  const currentProduct = session?.product
  const isExplorationSession = currentProduct === 'explore'
  const isMappingSession = currentProduct === 'map'
  const activeMapName = activeMap ?? null
  const showSavedMapInScene = Boolean(activeMapName && shouldShowSavedMapForProduct(currentProduct))
  const liveScanVisible = liveScanPreference ?? (!isMappingSession && !showSavedMapInScene)
  const planningLayer = usePlanningMap(sseState.connected, showSavedMapInScene ? activeMapName : null,
    session?.product_session_id, localNowS)
  const planningMap = planningLayer.map
  const showPlanningMap = mapView === 'planning' && planningMap !== null
  const initialMapView = useRef<string | null>(null)
  useEffect(() => {
    const key = `${session?.product_session_id}:${activeMapName}`
    if (!showPlanningMap || initialMapView.current === key) return
    initialMapView.current = key
    scene3DRef.current?.topView()
  }, [showPlanningMap, session?.product_session_id, activeMapName])
  const savedMapForScene = showSavedMapInScene
    && savedMapCloud?.mapName === activeMapName
    ? savedMapCloud
    : undefined
  const mappingObservation = useMemo(() => resolveMappingObservation(
    sseState.connected && isMappingSession ? sseState.mapScene : null, { nowS: rasterNowS },
  ), [sseState.connected, isMappingSession, sseState.mapScene, rasterNowS])
  const showMappingObservation = isMappingSession && mappingView === 'coverage'
  const canProbeMapping = sceneDebugTools && showMappingObservation && mappingObservation.status === 'ready'
  const currentMappingProbe = canProbeMapping && mappingProbe?.epoch === poseEpoch
    ? mappingProbe : null
  const mappingProbeCell = currentMappingProbe
    ? mappingObservationPointAt(mappingObservation, currentMappingProbe.x, currentMappingProbe.y) : null
  const mappingProbeLabel = mappingProbeCell?.value === 100 ? '障碍回波 · 高于附近地面'
    : mappingProbeCell?.value === 0 ? '支撑候选 · 已观测低表面'
      : mappingProbeCell?.value === -1 ? '未确认 · 缺少地面依据'
        : '当前窗口之外'
  const mappingProbeGround = mappingProbeCell?.ground
  const initialMappingView = useRef<string | null>(null)
  useEffect(() => {
    const key = `${session?.product_session_id}:${poseEpoch}:${mappingView}`
    if (!isMappingSession || !poseAvailable || initialMappingView.current === key) return
    initialMappingView.current = key
    if (mappingView === 'coverage') scene3DRef.current?.topView(6)
    else scene3DRef.current?.resetCamera()
  }, [isMappingSession, session?.product_session_id, poseEpoch, poseAvailable, mappingView])
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
  const goalDisabledReason =
    observe ? '只读监控' : navigationView.control.resumeRequired ? '请先点击上方“恢复控制”' : !motionStartAllowed
      ? motionStartBlockedReason
      : canAcceptGoal
      ? ''
      : '导航暂未就绪'
  const canSendGoal = goalDisabledReason === ''
  const previewDisabledReason = !sseState.connected ? '连接断开，等待恢复'
    : !poseAvailable ? '等待有效定位'
      : showSavedMapInScene && !planningMap ? '等待当前地图' : ''
  const previewCurrent = navigationPreviewIsCurrent(pendingGoalPreview, mapOdom)
  const pendingGoalPlanSummary = previewCurrent
    ? `预览可达${typeof pendingGoalPreview?.distance_m === 'number' ? ` · 路程 ${pendingGoalPreview.distance_m.toFixed(1)} m` : ''}`
    : ''
  const pendingGoalDistance = pendingGoal && mapOdom
    ? Math.hypot(pendingGoal.x - mapOdom.x, pendingGoal.y - mapOdom.y) : null
  const savedLocations = locationsOverride ?? sseState.locations?.locations ?? []
  const normalizedLocationName = locationName.trim()
  // Legacy SSE map_cloud now carries only metadata (count/seq) — points
  // are streamed over /ws/cloud and live in `cloud.positions`.

  // ── Load map list ─────────────────────────────────────────────
  const loadMaps = useCallback(async () => {
    if (mapListRequestInFlight.current) return
    mapListRequestInFlight.current = true
    setMapListStatus('loading')
    try {
      const data = await api.fetchMaps()
      setMaps(data)
      setMapListStatus('ready')
    } catch {
      setMapListStatus('error')
    } finally {
      mapListRequestInFlight.current = false
    }
  }, [])

  useEffect(() => { if (!observe) void loadMaps() }, [loadMaps, observe])

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
  // Hide the previous era immediately, then restore only matching XYZ history.
  useEffect(() => {
    let points: Array<[number, number, number]> = []
    try {
      const raw = trailKey ? sessionStorage.getItem(trailKey) : null
      const parsed: unknown = raw ? JSON.parse(raw) : null
      if (Array.isArray(parsed)) {
        points = parsed.filter(
          (p): p is [number, number, number] =>
            Array.isArray(p) && p.length === 3 && p.every(Number.isFinite) &&
            Math.abs(p[0]) < 100 && Math.abs(p[1]) < 100
        ).slice(-TRAIL_MAX)
      }
    } catch { /* ignore */ }
    setTrailState({ key: trailKey, points })
  }, [trailKey])

  useEffect(() => {
    if (!poseAvailable || trailKey === null) return
    setTrailState(previous => {
      const points = previous.key === trailKey ? previous.points : []
      const last = points.at(-1)
      if (last && Math.hypot(robotX - last[0], robotY - last[1], robotZ - last[2]) <= 0.05) return previous
      return { key: trailKey, points: [...points, [robotX, robotY, robotZ] as [number, number, number]].slice(-TRAIL_MAX) }
    })
  }, [poseAvailable, robotX, robotY, robotZ, trailKey])

  // Persist trail on change (throttle: only every ~1 s to keep sessionStorage
  // writes cheap on long runs).
  const trailSaveThrottleRef = useRef(0)
  useEffect(() => {
    if (!trailKey || trailState.key !== trailKey) return
    const now = Date.now()
    if (now - trailSaveThrottleRef.current < 1000) return
    trailSaveThrottleRef.current = now
    try {
      sessionStorage.setItem(trailKey, JSON.stringify(trailState.points))
    } catch { /* quota hit? ignore */ }
  }, [trailState, trailKey])

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
    ++goalPreviewRequest.current
    setPendingGoal(null)
    setPendingGoalPreview(null)
    setGoalPreviewPending(false)
    setGoalPreviewError(null)
  }, [activeMapName, showSavedMapInScene, session?.product_session_id, sseState.connected])

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
  const handleResumeControl = useCallback(async () => {
    setResumePending(true)
    setResumeError(null)
    try {
      const response = await api.resumeNavigation()
      if (!response.ok) throw new Error(response.status || '恢复请求被拒绝')
    } catch (error: unknown) {
      const message = api.formatCommandError(error, '恢复控制失败')
      setResumeError(message)
      setResumePending(false)
    }
  }, [])

  const handlePendingGoal = useCallback(async (x: number, y: number, selectedZ?: number) => {
    const request = ++goalPreviewRequest.current
    const z = selectedZ ?? planningMap?.origin[2]
    setPendingGoal({ x, y, z })
    setPendingGoalPreview(null)
    setGoalPreviewError(null)
    setGoalPreviewPending(false)
    setFollowRobot(false)
    if (showSavedMapInScene && !planningMap) {
      setGoalPreviewError('等待当前高度通行图，暂时无法预览目标')
      return
    }
    if (previewDisabledReason) {
      setGoalPreviewError(previewDisabledReason)
      return
    }
    setGoalPreviewPending(true)
    try {
      const candidate = await api.constructGoalCandidate({
        x,
        y,
        z,
        source: 'map_click',
        target_type: 'map_point',
        label: 'scene_click',
        acceptance_radius_m: goalAcceptanceRadius,
        max_speed_mps: goalMaxSpeed,
      })
      if (request !== goalPreviewRequest.current) return
      if (!candidate.ok || candidate.preview?.feasible === false) {
        const reason = candidate.reasons.slice(0, 3).join(' / ') || candidate.error || '目标预检未通过'
        setGoalPreviewError(reason === 'empty_path'
          ? '未找到连接当前位置的路径。可换选附近绿区，或检查沿途支撑与障碍。'
          : `路径预览未通过：${reason}`)
        return
      }
      const target = candidate.target
      setPendingGoal({ x: target?.x ?? x, y: target?.y ?? y, z: target?.z })
      setPendingGoalPreview(candidate.preview ?? null)
    } catch (e: unknown) {
      if (request !== goalPreviewRequest.current) return
      setGoalPreviewError(`预览失败：${e instanceof Error ? e.message : String(e)}`)
    } finally {
      if (request === goalPreviewRequest.current) setGoalPreviewPending(false)
    }
  }, [previewDisabledReason, goalAcceptanceRadius, goalMaxSpeed, planningMap, showSavedMapInScene])

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
    const seed = sceneRelocalizationSeed(x, y, mapOdom)
    if (!seed) {
      showToast('当前没有有效的地图定位，无法确定初始朝向', 'error')
      return
    }
    showToast(`重定位中… (${x.toFixed(2)}, ${y.toFixed(2)})`, 'info')
    try {
      await api.relocalize(mapName, seed.x, seed.y, seed.yaw)
      const q = sseState.session?.icp_quality
      const qStr = typeof q === 'number' ? ` quality=${q.toFixed(2)}` : ''
      showToast(`重定位成功${qStr}`, 'success')
    } catch (e: unknown) {
      showToast(`重定位失败: ${e instanceof Error ? e.message : String(e)}`, 'error')
    }
  }, [savedMapRelocalizeSupported, relocalizeUnavailableMessage, sseState.session, mapOdom, showToast])

  const handleConfirmGoal = useCallback(async () => {
    if (!pendingGoal || goalSendPending) return
    if (goalPreviewPending || !previewCurrent) return
    if (!canSendGoal) {
      showToast(`不能下发目标: ${goalDisabledReason}`, 'error')
      return
    }
    const { x, y } = pendingGoal
    setGoalSendPending(true)
    try {
      const res = await api.sendGoal(x, y, {
        z: pendingGoal.z,
        source: 'map_click',
        target_type: 'map_point',
        label: 'scene_click',
        acceptance_radius_m: goalAcceptanceRadius,
        max_speed_mps: goalMaxSpeed,
      })
      if (!res.ok) throw new Error(api.formatCommandAck(res, '导航请求'))
      setPendingGoal(null)
      setPendingGoalPreview(null)
      showToast(api.formatCommandAck(res, `目标 (${x.toFixed(2)}, ${y.toFixed(2)})`), 'success')
    } catch (e: unknown) {
      setGoalPreviewError(api.formatCommandError(e, '发送目标失败'))
    } finally {
      setGoalSendPending(false)
    }
  }, [canSendGoal, goalAcceptanceRadius, goalDisabledReason, goalMaxSpeed, pendingGoal, previewCurrent, goalPreviewPending, goalSendPending, showToast])

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
    setTrailState({ key: trailKey, points: [] })
    try {
      if (trailKey) sessionStorage.removeItem(trailKey)
    } catch { /* ignore */ }
  }, [trailKey])

  const handleClearCloud = useCallback(async () => {
    try {
      await api.resetMapCloud()
      showToast('已清除局部地图点预览', 'success')
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

  const handleSaveMap = () => {
    setTeleopMode(false)
    setSaveModalOpen(true)
  }

  const confirmSaveMap = async (name: string) => {
    const initiatingSession = savePreviewSession.current
    setSaveModalOpen(false)
    if (saveBlockedReason) {
      showToast(saveBlockedReason, 'error')
      return
    }
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
      const savedName = r.name
      const location = formatSaveMapLocation(r, savedName)
      const detail = formatSaveMapDetail(r)
      setSaveStatus({
        name: savedName,
        state: 'saved',
        detail,
        location,
        summary,
      })
      const df = r.dynamic_filter
      if (df && df.success && df.dropped !== undefined && df.orig_count) {
        const pct = (100 * df.dropped / df.orig_count).toFixed(1)
        showToast(`已保存: ${savedName} · 清除 ${df.dropped} 动态点 (${pct}%)`, 'success')
      } else {
        showToast(`已保存: ${savedName}`, 'success')
      }
      if (r.warnings?.length) showToast(r.warnings.join('；'), 'info')
      if (initiatingSession !== null && savePreviewSession.current === initiatingSession) {
        loadMaps()
        onOpenSavedMap(savedName)
      }
    } catch (e: unknown) {
      const message = e instanceof Error ? e.message : String(e)
      setSaveStatus({
        name,
        state: 'failed',
        detail: message || '保存失败，请检查 Gateway 日志。',
      })
      showToast(`保存失败：${message}`, 'error')
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
    const mapName = activeMapRelocalizationTarget(session)
    if (!mapName) {
      showToast('请先激活地图', 'error')
      return
    }
    setRelocPending(true)
    try {
      const res = await api.globalRelocalize(mapName)
      const message = res.message || res.status || (res.ok ? 'accepted' : 'rejected')
      showToast(res.ok ? `全局重定位: ${message}` : `全局重定位失败: ${message}`, res.ok ? 'success' : 'error')
      if (res.ok) {
        try {
          const savedMap = await api.fetchSavedMapPointCloud(mapName)
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

  const sceneDisplayControls = <>
    {isMappingSession && <div className={styles.mappingViewSwitch} role="group" aria-label="建图显示方式">
      <button className={styles.toolbarBtn} aria-pressed={mappingView === 'global'}
        title="查看本次累计建图，已验证的闭环在后台校正"
        onClick={() => { setMappingView('global'); setLayers(value => ({ ...value, cloud: true })) }}><MapPinned size={15} />整图</button>
      <button className={styles.toolbarBtn} aria-pressed={mappingView === 'coverage'}
        title="显示相对附近地面的障碍与支撑候选，不是通行图"
        onClick={() => setMappingView('coverage')}><Grid3x3 size={15} />局部投影</button>
      <button className={styles.toolbarBtn} aria-pressed={mappingView === 'points'}
        title="显示当前局部地图表面的三维点"
        onClick={() => { setMappingView('points'); setLayers(value => ({ ...value, cloud: true })) }}><Cloud size={15} />局部地图</button>
    </div>}
    {showSavedMapInScene && <button className={showPlanningMap ? styles.toolbarBtnSelected : styles.toolbarBtn}
      aria-pressed={showPlanningMap} onClick={() => setMapView(value => value === 'planning' ? 'points' : 'planning')}
      title="切换通行图与原始保存点云"><MapPinned size={15} /><span>{mapView === 'planning' ? '通行图' : '点云'}</span></button>}
    <button className={liveScanVisible ? styles.toolbarBtnSelected : styles.toolbarBtn}
      aria-pressed={liveScanVisible} onClick={() => setLiveScanPreference(!liveScanVisible)}
      title="只叠加当前扫描，不改变地图图层"><Radio size={15} /><span>当前扫描</span></button>
    <button className={styles.toolbarBtn} aria-pressed={cameraVisible}
      onClick={() => {
        setCameraPreference(!cameraVisible)
        if (isMappingSession && !cameraVisible) setInspectorOpen(true)
      }} title="显示相机实时画面">
      <Camera size={15} /><span>相机</span>
    </button>
    <button className={followRobot ? styles.toolbarBtnSelected : styles.toolbarBtn}
      onClick={() => setFollowRobot(value => !value)} aria-pressed={followRobot}
      aria-label="跟随机器人" title="跟随机器人">
      <LocateFixed size={15} /><span>跟随</span>
    </button>
    <button className={styles.toolbarBtn} onClick={() => scene3DRef.current?.resetCamera()}
      aria-label="空间视角" title="空间视角"><Maximize2 size={15} /><span>空间</span></button>
    <button className={styles.toolbarBtn} onClick={() => scene3DRef.current?.topView(isMappingSession ? 6 : undefined)}
      aria-label="俯视当前数据" title="俯视当前数据"><Grid3x3 size={15} /><span>俯视</span></button>
    <button className={styles.toolbarBtn} disabled={!savedMapForScene?.points.length && !(isMappingSession && (cloud.count || mappingObservation.status === 'ready'))}
      onClick={() => { setFollowRobot(false); scene3DRef.current?.fitMap() }}
      aria-label={isMappingSession ? '查看当前数据全范围' : '查看完整保存地图'}
      title={isMappingSession ? '查看当前数据全范围' : '查看完整保存地图'}><Maximize2 size={15} /><span>{isMappingSession && !showGlobalMapping ? '局部范围' : '全图'}</span></button>
  </>

  return (
    <div className={`${styles.sceneView} ${isMappingSession ? styles.mappingScene : ''}`}>


      {/* Workspace */}
      <div className={[
        styles.workspace,
        drawerOpen && !observe ? styles.toolsWorkspace : '',
        isMappingSession && !inspectorOpen && !(drawerOpen && !observe) ? styles.inspectorCollapsed : '',
      ].filter(Boolean).join(' ')}>
        {/* Tools open alongside the scene without replacing its live state. */}
        {!observe && drawerOpen && <>
        <div className={styles.drawer}>
          <div className={styles.drawerHeader}>
            <button className={styles.backToInspector} onClick={() => {
              setDrawerOpen(false)
              requestAnimationFrame(() => document.getElementById(isMappingSession && !inspectorOpen ? 'scene-inspector-toggle' : 'scene-tab-tools')?.focus())
            }}>
              <ArrowLeft size={16} /> {isMappingSession ? '关闭工具' : '返回操作'}
            </button>
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

            </div>
          </div>
          <div className={styles.workspaceTabs} role="tablist" aria-label="场景工具">
            <button role="tab" aria-selected={workspaceTool === 'maps'} onClick={() => setWorkspaceTool('maps')}>地图</button>
            <button role="tab" aria-selected={workspaceTool === 'locations'} onClick={() => setWorkspaceTool('locations')}>位置</button>
            <button role="tab" aria-selected={workspaceTool === 'localization'} onClick={() => openWorkspaceTool('localization')}>定位</button>
          </div>
          <div className={styles.drawerBody}>
            {workspaceTool === 'maps' && <>
            <div className={styles.mapToolbar}>
              <button className={styles.toolbarBtn} onClick={() => void loadMaps()} disabled={mapListStatus === 'loading'}><RefreshCw size={13} />刷新地图库</button>
            </div>
            {saveBlockedReason && <p role="status">{saveBlockedReason}</p>}
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
                {saveStatus.state === 'saved' && <button className={styles.toolbarBtn}
                  onClick={() => onOpenSavedMap(saveStatus.name)}><MapPinned size={13} /> 查看完整地图</button>}
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
            <details className={styles.mapWorkbench}>
              <summary>地图编辑</summary>
              <div className={styles.mapWorkbenchBody}>
                <div className={styles.workbenchHeader}>
                  <span>编辑地图产物</span>
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
                {!observe && pendingGoal && (
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
            </details>
            {mapListStatus === 'loading' && (
              <p className={styles.statusLine} role="status">
                {maps.length > 0 ? '正在刷新地图列表，以下为上次读取的地图。' : '正在加载地图列表…'}
              </p>
            )}
            {mapListStatus === 'error' && (
              <div className={styles.statusLine} role="alert">
                <p>{maps.length > 0 ? '地图列表读取失败，以下为上次读取的地图。' : '地图列表读取失败。'}</p>
                <button className={styles.toolbarBtn} onClick={() => void loadMaps()}><RefreshCw size={13} /> 重试</button>
              </div>
            )}
            {mapListStatus === 'ready' && maps.length === 0 && (
              <div className={styles.emptyState}>
                <MapPinned size={32} className={styles.emptyIcon} strokeWidth={1.4} />
                <div className={styles.emptyTitle}>暂无地图</div>
                <div className={styles.emptyHint}>保存当前场景来创建第一张地图</div>

              </div>
            )}
            {MAP_GROUPS.map(g => {
              const groupMaps = maps.filter(g.filter)
              if (groupMaps.length === 0) return null
              return (
                <div key={g.label} className={styles.mapGroup}>
                  <div className={styles.mapGroupTitle}>{g.label}</div>
                  {groupMaps.map(m => (
                    <div className={styles.mapRow} key={m.name}>
                    <button
                      className={m.is_active && showSavedMapInScene ? styles.mapItemActive : styles.mapItem}
                      onClick={() => onOpenSavedMap(m.name)}
                      onContextMenu={(e) => {
                        e.preventDefault()
                        const rect = (e.currentTarget as HTMLElement).getBoundingClientRect()
                        setMapContextMenu({ name: m.name, x: rect.right + 4, y: rect.top })
                      }}
                      title="查看整图快照"
                    >
                      <span>{m.name}</span>
                    </button>
                    <button className={styles.mapActionsButton} aria-label={`管理地图 ${m.name}`}
                      onClick={e => {
                        const rect = e.currentTarget.getBoundingClientRect()
                        setMapContextMenu({ name: m.name, x: Math.min(rect.left, window.innerWidth - 190), y: rect.bottom + 4 })
                      }}>
                      <MoreHorizontal size={15} />
                    </button>
                    </div>
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
                <button className={styles.contextMenuItem} onClick={() => { onOpenSavedMap(mapContextMenu.name); setMapContextMenu(null) }}>
                  <LocateFixed size={12} /> 预览地图
                </button>
                <button className={styles.contextMenuItem} onClick={() => { setRenameTarget(mapContextMenu.name); setMapContextMenu(null) }}>
                  <Pencil size={12} /> 重命名
                </button>
                <button className={[styles.contextMenuItem, styles.contextMenuDanger].join(' ')} onClick={() => { setDeleteTarget(mapContextMenu.name); setMapContextMenu(null) }}>
                  <Trash2 size={12} /> 删除
                </button>
              </div>
            )}
            </>}
            {workspaceTool === 'locations' && <>
          <div className={styles.statCard}>
            <div className={styles.statCardTitle}>
              <MapPinned size={11} style={{ marginRight: 4, verticalAlign: 'middle' }} />
              常用位置
            </div>
            <div className={styles.locationSaveRow}>
              <input
                className={styles.locationNameInput}
                value={locationName}
                onChange={e => setLocationName(e.target.value)}
                placeholder="为当前位置命名"
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
              {savedLocations.map(loc => {
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
                      title={canSendGoal ? `前往 ${loc.name}` : goalDisabledReason}
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
                        title={poseAvailable ? '更新为当前位置' : '当前没有有效定位'}
                      >
                        {updateBusy ? <Activity size={12} /> : <Pencil size={12} />}
                      </button>
                      <button
                        type="button"
                        className={`${styles.locationIconBtn} ${styles.locationDangerBtn}`}
                        onClick={() => setLocationDeleteTarget(loc)}
                        disabled={disabled}
                        title="删除位置"
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

          </>}

            {workspaceTool === 'localization' && relocOpen && (
              <div className={styles.relocPanel}>
                <div className={styles.statCardTitle}>定位</div>
                <p className={styles.statusLine} role="status">{poseLabel}</p>
                <p className={styles.statusLine}>{activeMapName ? `当前地图：${activeMapName}` : '当前未激活地图'}</p>
                <div className={styles.localizationActions}>
                  <button className={styles.toolbarBtn} onClick={handleRestartLocalization}
                    disabled={restartLocalizationPending || !activeMapName}
                    title={activeMapName ? '在当前地图上重新定位' : '请先激活地图'}>
                    <RefreshCw size={14} /> {restartLocalizationPending ? '重定位中…' : '重新定位'}
                  </button>
                  <button className={styles.toolbarBtn} onClick={handleGlobalRelocalize}
                    disabled={relocPending || !activeMapName || !savedMapRelocalizeSupported}
                    title={!activeMapName ? '请先激活地图' : savedMapRelocalizeSupported ? '在当前保存地图自动匹配位置' : relocalizeUnavailableMessage}>
                    <LocateFixed size={14} /> 自动匹配
                  </button>
                </div>
                {!savedMapRelocalizeSupported && <p className={styles.statusLine}>{relocalizeUnavailableMessage}</p>}
                <details className={styles.statusDetails}>
                  <summary>手动设置初始位姿</summary>
                  <p className={styles.statusLine}>选择保存地图并设置地图中的位置与朝向。支持时也可 Shift+点击场景进行重定位。</p>
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
                      {mapListStatus === 'loading' && (
                        <div className={styles.customSelectEmpty} role="status">
                          {maps.length > 0 ? '正在刷新，以下为上次读取的地图。' : '正在加载地图列表…'}
                        </div>
                      )}
                      {mapListStatus === 'error' && (
                        <div className={styles.customSelectEmpty} role="alert">
                          <p>{maps.length > 0 ? '地图列表读取失败，以下为上次读取的地图。' : '地图列表读取失败。'}</p>
                          <button className={styles.toolbarBtn} onClick={() => void loadMaps()}><RefreshCw size={13} /> 重试</button>
                        </div>
                      )}
                      {mapListStatus === 'ready' && maps.length === 0 && (
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
                </details>
              </div>
            )}
          </div>
          <div className={styles.controlSummary}>
            <div><span>控制权</span><strong>{navigationView.control.label}</strong></div>
            <div><span>运动许可</span><strong>{navigationView.motion.permission.label}</strong></div>
          </div>
        </div>
        </>}

        {/* Center: 3D scene */}
        <div className={`${styles.canvasArea} ${isMappingSession ? styles.mappingCanvasArea : ''}`}>
          <div className={styles.canvasHeader}>
            <span>{isMappingSession ? '建图' : <><Grid3x3 size={15} />现场地图</>}
              <small>{isMappingSession
                ? observe ? '只读' : session?.env === 'sim' ? '仿真' : '实时'
                : `${observe ? '只读 · ' : session?.env === 'sim' ? '仿真 · ' : ''}${activeMapName || '当前环境'}`}</small>
            </span>
            <div className={styles.cameraActions}>
              {!observe && (currentProduct === 'teleop' || currentProduct === 'teleop_avoid' || currentProduct === 'map') && (
                <button className={teleopMode ? styles.toolbarBtnSelected : styles.toolbarBtn}
                  aria-pressed={teleopMode} onClick={() => setTeleopMode(value => !value)}>
                  <Gamepad2 size={15} /><span>{teleopMode ? '退出遥控' : isMappingSession ? '遥控' : '遥控模式'}</span>
                </button>
              )}
              {isMappingSession && <>
                {!observe && <button className={styles.toolbarBtnPrimary} onClick={handleSaveMap}
                  disabled={Boolean(saveBlockedReason) || saveStatus?.state === 'saving'}
                  title={saveBlockedReason || '保存当前建图结果后查看整图快照'}>
                  <Save size={15} /><span>{saveStatus?.state === 'saving' ? '保存中…' : '保存地图'}</span>
                </button>}
                <button className={styles.toolbarBtn} onClick={() => onOpenSavedMap(null)}
                  title="查看保存的完整地图"><MapPinned size={15} /><span>已存地图</span></button>
              </>}
              {isMappingSession ? <details className={styles.mappingDisplayMenu} name="lingtu-menu">
                <summary><SlidersHorizontal size={15} /> 视图</summary>
                <div className={styles.mappingDisplayOptions} onClick={event => {
                  if ((event.target as HTMLElement).closest('button')) event.currentTarget.parentElement?.removeAttribute('open')
                }}>{sceneDisplayControls}</div>
              </details> : sceneDisplayControls}
              {isMappingSession && <button className={inspectorOpen && !drawerOpen ? styles.toolbarBtnSelected : styles.toolbarBtn}
                id="scene-inspector-toggle" aria-controls="scene-inspector" aria-expanded={inspectorOpen && !drawerOpen}
                onClick={() => {
                  setInspectorOpen(!inspectorOpen || drawerOpen)
                  setDrawerOpen(false)
                }} title="状态、图层与操作"><Activity size={15} />状态</button>}
            </div>
          </div>
        {(!isMappingSession || operatorAttention || motionHeld || !sseState.connected) && <div className={`${styles.sceneAlert} ${!operatorAttention ? styles.sceneAlertQuiet : ''}`} role="status">
          <Info size={16} aria-hidden="true" />
          <span className={styles.sceneAlertMessage} title={plannerAttention || navigationView.motion.permission.label}>{navigationView.control.resumeRequired ? '控制已暂停' : motionHeld || navigationView.motion.permission.state === 'ESTOPPED' ? navigationView.motion.permission.label : plannerAttention || (sseState.connected ? '连接正常' : '等待连接')}</span>
          <span className={styles.sceneAlertDetail}>{resumePending ? '等待机器人确认，已选目标保留' : navigationView.control.resumeRequired && resumeError ? resumeError : isMappingSession ? '' : plannerAttention ? '检查周围障碍，或重新选点' : motionHeld ? '查看控制详情' : ''}</span>
          {!observe && navigationView.control.resumeRequired && <button className={styles.toolbarBtn}
            disabled={resumePending || !sseState.connected} onClick={handleResumeControl}
            title="解除控制暂停，保留已选目标，仍需确认后开始导航">
            {resumePending ? '恢复中…' : '恢复控制'}
          </button>}
          {isMappingSession && motionHeld && !navigationView.control.resumeRequired && <button className={styles.toolbarBtn}
            onClick={() => { setInspectorOpen(true); setDrawerOpen(false); setInspectorTab('status') }}>控制详情</button>}
        </div>}
          {!observe && teleopMode && !drawerOpen && (
            <TeleopPanel key={currentProduct} sseState={sseState} showToast={showToast}
              onExit={() => setTeleopMode(false)} />
          )}
          <div className={styles.canvasWrap}
            onPointerDownCapture={() => {
              // A canvas click closes menus before any map interaction.
              const openMenus = document.querySelectorAll<HTMLDetailsElement>('details[name="lingtu-menu"][open]')
              dismissOnlyCanvasClick.current = openMenus.length > 0
              openMenus.forEach(menu => { menu.open = false })
            }}
            onMouseUpCapture={event => {
              if (!dismissOnlyCanvasClick.current) return
              dismissOnlyCanvasClick.current = false
              event.preventDefault()
              event.stopPropagation()
            }}>
            <Scene3D
              ref={scene3DRef}
              cloud={cloud}
              scanCloud={alignedScanCloud}
              scanVisible={liveScanVisible}
              savedMapFlat={savedMapForScene?.points}
              savedMapFrameId={savedMapForScene?.frameId}
              savedMapEpoch={savedMapForScene?.epoch}
              savedMapVisible={mapView !== 'planning'}
              planningMap={planningMap}
              planningMapVisible={showPlanningMap}
              mappingMode={isMappingSession}
              mappingObservation={mappingObservation}
              mappingObservationVisible={showMappingObservation}
              elevationState={elevationState}
              nativeTraversabilityState={nativeTraversabilityState}
              sceneGraph={mapSceneGraph}
              robotX={robotX}
              robotY={robotY}
              robotZ={robotZ}
              orientation={mapOdom?.orientation}
              robotModel={robotModel}
              jointTelemetry={sseState.jointTelemetry}
              poseStampS={odom?.ts ?? null}
              poseEpoch={poseEpoch}
              followRobot={followRobot}
              robotValid={poseAvailable}
              yaw={yaw}
              trail={trail}
              path={hasGoal ? path : []}
              localPath={hasGoal || requestedMotion ? localPathPts : []}
                localPlannerSnapshot={nativeFresh ? localPlannerSnapshot : null}
                safetyEnvelope={safetyEnvelope}
                safetyView={safetyView}
              layers={{ ...layers, cloud: layers.cloud && !showSavedMapInScene
                && !showMappingObservation }}
              pointSize={pointSize}
              onPendingGoal={isMappingSession
                ? canProbeMapping ? (x, y) => setMappingProbe({ x, y, epoch: poseEpoch }) : undefined
                : observe || goalSendPending ? undefined : handlePendingGoal}
              onRelocalize={!isMappingSession && !observe && workspaceTool === 'localization' && drawerOpen ? handleSceneRelocalize : undefined}
              pendingGoal={isMappingSession ? currentMappingProbe && { ...currentMappingProbe,
                z: mappingObservation.status === 'ready' ? mappingObservation.layer.origin[2] : 0 } : pendingGoal}
              pendingGoalRadius={isMappingSession ? 0.15 : goalAcceptanceRadius}
              pendingGoalLabel={isMappingSession ? '观测检查' : '待确认目标'}
              previewPath={!isMappingSession && previewCurrent ? pendingGoalPreview?.path : undefined}
            />
            {!isMappingSession && !observe && workspaceTool === 'localization' && drawerOpen && <div className={styles.canvasOverlayTop}>
              <span className={styles.scaleLabel}>Shift + 点击地图设置初始位置</span>
            </div>}
            {!sseState.connected && (
              <div className={styles.sceneDisconnected} role="status">
                <div className={styles.connectionIcon}><Radio size={26} strokeWidth={1.4} aria-hidden="true" /></div>
                <strong>实时数据连接中断</strong>
                <span>正在重连，请检查网络与监控服务</span>
              </div>
            )}
            {(showSavedMapInScene || layers.elevation || layers.nativeTraversability || layers.localPlanner) && (
              <div className={styles.sceneLegendStack}>
                {showSavedMapInScene && !layers.localPlanner && <div className={styles.mapReadingKey}>
                  {showPlanningMap && planningMap ? <>
                    <strong>通行图 · 查询高度 {planningMap.origin[2].toFixed(2)} m</strong>
                    <div className={styles.planningMapLegend}>
                      <span><i style={{ background: '#63ae98' }} />可通行</span>
                      <span><i style={{ background: '#d57164' }} />受阻</span>
                      <span><i style={{ background: '#71767f' }} />缺少支撑</span>
                    </div>
                    <small>二维投影，非地面表面 · 运动中检查局部障碍</small>
                  </> : <>
                    <strong>{mapView === 'planning' ? planningMapUnavailableLabel(planningLayer.reason) : '保存点云 · 静态'}</strong>
                    {mapView === 'points' && <span>点：已扫描表面 · 空白不代表可走</span>}
                  </>}
                </div>}
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
                    <strong title="实时风险叠加；透明格表示风险值为 0，不代表已验证可通行。">实时风险</strong>
                    <div className={styles.costLegendItems}>
                      <span><i className={styles.costSoft} />低</span>
                      <span><i className={styles.costLethal} />高</span>
                    </div>
                    {nativeTraversabilityState.status !== 'ready' && <small>{nativeTraversabilityState.message}</small>}
                  </div>
                )}
                {layers.localPlanner && (
                  <div className={styles.localPlannerLegend} aria-label="局部安全诊断（采样）图层图例">
                    <strong>{safetyView === 'slice' ? '机身高度切片' : '三维诊断'} · 抽样</strong>
                    <span><i className={styles.legendLine} style={{ background: '#59bfff' }} />机身包络</span>
                    <span><i className={styles.legendMark} style={{ background: '#fff' }} />前后查询点</span>
                    <span><i className={styles.legendMark} style={{ background: '#ff6b57' }} />膨胀占据格</span>
                    <small>抽样空白不代表可通行</small>
                    <details className={styles.safetyLegendDetails}>
                    <summary>尺寸与判读说明</summary>
                    <p>{safetyEnvelope
                      ? `半径 ${(safetyEnvelope.radius * 100).toFixed(0)} cm · 前后偏移 ±${(safetyEnvelope.offset * 100).toFixed(0)} cm · 上/下 ${(safetyEnvelope.above * 100).toFixed(0)}/${(safetyEnvelope.below * 100).toFixed(0)} cm`
                      : '等待当前运行配置与定位'}</p>
                    <p>半径和上下净空已计入红格，以白点查询；切片只显示与机身中心高度相交的格子。</p>
                    <p>{nativeFresh && nativeStatus?.local_map?.collision?.live
                      ? `接口返回 ${nativeStatus.local_map.collision.occupied_points_returned ?? 0} / ${nativeStatus.local_map.collision.occupied_points_total ?? '—'} 个占据格，切片仅展示其中一部分。`
                      : '占据格数据未就绪'}</p>
                    {safetyView === 'volume' && <>
                    <span><i className={`${styles.legendMark} ${styles.legendObstacle}`} />障碍点</span>
                    <span><i className={`${styles.legendMark} ${styles.legendTerrain}`} />Terrain 风险</span>
                    <span><i className={`${styles.legendLine} ${styles.legendCandidate}`} />候选轨迹</span>
                    <span><i className={`${styles.legendLine} ${styles.legendSelected}`} />选中轨迹</span>
                    {localPlannerSampleWarningText && <small>{localPlannerSampleWarningText}</small>}
                    </>}
                    </details>
                  </div>
                )}
              </div>
            )}
            {currentMappingProbe && <div className={`${styles.goalConfirmPanel} ${styles.mappingProbe}`} aria-label="观测位置检查"
              onPointerDown={event => event.stopPropagation()}>
              <div className={styles.goalConfirmHeader}>
                <strong>{mappingObservation.status !== 'ready' ? '观测数据未就绪' : mappingProbeLabel}</strong>
                <span className={styles.goalConfirmCoords}>({currentMappingProbe.x.toFixed(2)}, {currentMappingProbe.y.toFixed(2)})</span>
                <button className={styles.goalCancelBtn} onClick={() => setMappingProbe(null)} aria-label="关闭观测检查"><X size={16} /></button>
              </div>
              <p>{mappingObservation.status !== 'ready' ? '连接恢复并收到新观测后再检查。'
                : mappingProbeCell?.value === -1 ? '这里缺少有效观测。请从另一位置或朝向补扫。'
                : mappingProbeCell?.value === 100 ? '此处有高于附近地面的回波；切换空间点云可查看三维形态。'
                : mappingProbeCell?.value === 0 ? '此处观测到与附近低表面相连的支撑候选；仍需通行图和路径检查。'
                : '这里不在当前滚动窗口内，不能判断是否已经建图。'}</p>
              <small>{mappingProbeGround
                ? `局部表面拟合 · 高度 ${mappingProbeGround.heightM.toFixed(2)} m · 残差 ${(mappingProbeGround.roughnessM * 100).toFixed(1)} cm · ${Math.round(mappingProbeGround.supportCount)} 个细 XY 支撑`
                : '局部表面拟合不可用；这不是置信度或通行判定。'}</small>
              <small>仅检查地图，不发送运动目标</small>
            </div>}
            {!isMappingSession && !observe && pendingGoal && (
              <div className={styles.goalConfirmPanel} aria-label="导航目标" onPointerDown={event => event.stopPropagation()}>
                <div className={styles.goalConfirmHeader}>
                <span className={styles.goalConfirmLabel}>已选目标{pendingGoalDistance !== null ? ` · ${pendingGoalDistance.toFixed(1)} m` : ''}</span>
                <span className={styles.goalConfirmCoords}>
                  ({pendingGoal.x.toFixed(2)}, {pendingGoal.y.toFixed(2)})
                </span>
                </div>
                <div className={styles.goalControlGroup}>
                  <label className={styles.goalControl}>
                    <span>速度</span>
                    <select
                      className={styles.goalSelect}
                      value={goalMaxSpeed}
                      disabled={goalSendPending || goalPreviewPending}
                      onChange={(e) => { setGoalMaxSpeed(Number(e.target.value)); setPendingGoalPreview(null); setGoalPreviewError(null) }}
                    >
                      {GOAL_SPEED_OPTIONS.map((speed) => (
                        <option key={speed} value={speed}>{speed.toFixed(2)} m/s</option>
                      ))}
                    </select>
                  </label>
                  <label className={styles.goalControl}>
                    <span>到点范围</span>
                    <select
                      className={styles.goalSelect}
                      value={goalAcceptanceRadius}
                      disabled={goalSendPending || goalPreviewPending}
                      onChange={(e) => { setGoalAcceptanceRadius(Number(e.target.value)); setPendingGoalPreview(null); setGoalPreviewError(null) }}
                    >
                      {GOAL_RADIUS_OPTIONS.map((radius) => (
                        <option key={radius} value={radius}>{radius.toFixed(2)} m</option>
                      ))}
                    </select>
                  </label>
                </div>
                <div className={styles.goalFeedback} role="status">
                  <span className={styles.goalPlanSummary}>
                    {goalSendPending ? '正在提交导航…' : goalPreviewPending ? '正在预览路径…'
                      : goalPreviewError || pendingGoalPlanSummary
                        || (pendingGoalPreview?.feasible && !previewCurrent ? '当前位置已变化，请重新预览路径' : '')
                        || planningCellLabel(planningCellAt(planningMap, pendingGoal.x, pendingGoal.y))}
                  </span>
                  <span className={styles.goalConfirmReason}>
                    {previewCurrent && goalDisabledReason ? goalDisabledReason : '预览不会移动机器人，确认后才开始导航'}
                  </span>
                </div>
                <div className={styles.goalConfirmActions}>
                <button
                  className={styles.goalConfirmBtn}
                  onClick={previewCurrent ? handleConfirmGoal : () => void handlePendingGoal(pendingGoal.x, pendingGoal.y, pendingGoal.z)}
                  disabled={goalSendPending || goalPreviewPending || (previewCurrent ? !canSendGoal : !!previewDisabledReason)}
                  title={previewCurrent ? goalDisabledReason || '确认后开始导航' : previewDisabledReason || '仅预览，不移动机器人'}
                >
                  <Navigation size={16} /> {goalSendPending ? '提交中…' : goalPreviewPending ? '规划中…' : previewCurrent ? '开始导航' : goalPreviewError || pendingGoalPreview ? '重新预览' : '预览路径'}
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
                  disabled={goalSendPending}
                  onClick={() => {
                    ++goalPreviewRequest.current
                    setPendingGoal(null)
                    setPendingGoalPreview(null)
                    setGoalPreviewPending(false)
                    setGoalPreviewError(null)
                  }}
                >
                  取消
                </button>
                </div>
              </div>
            )}

          </div>
          <div className={`${styles.sceneFooter} ${isMappingSession ? styles.mappingFooter : ''}`}>
            {isMappingSession ? <>
              <div className={styles.mappingSourceStatus} role="status">
                  <span>{showMappingObservation ? '局部投影' : showGlobalMapping ? '累计建图' : '局部地图'}</span>
                  {showGlobalMapping && (cloud.mappingSummary?.droppedFrames ?? 0) > 0
                    ? <span role="status">建图数据有缺失，请先保存原始记录</span>
                    : showGlobalMapping && cloud.mappingSummary?.state === 'optimizer_quality_failed'
                      ? <span role="status">本轮校正未收敛，保留上一版</span>
                    : showGlobalMapping && (cloud.mappingSummary?.registrationRejections ?? 0) > 0
                      ? <span title="部分关键帧几何配准不足，仍保留原始点云；这些区域的闭环校正尚未验证。">部分区域待校正</span>
                    : showGlobalMapping && (cloud.mappingSummary?.optimizations ?? 0) > 0
                      ? <span>闭环已校正</span> : null}
                  {!showMappingObservation && cloud.count === 0 && <span>等待点云</span>}
                  {showMappingObservation && mappingObservation.status !== 'ready' && <span role="status">观测未更新</span>}
                  {!showMappingObservation && cloud.count > 0 && (!sseState.connected || (!showGlobalMapping && !freshSource(cloud.stampS, rasterNowS))) && <span role="status">缓存画面</span>}
              </div>
              <span className={styles.mappingSpeed} title={`实测前进速度 · ${poseLabel}`}>{displaySpeed}</span>
            </> : <>
              <span><i className={styles.routeKey} />执行轨迹 <i className={styles.globalKey} />全局路径 <i className={styles.trailKey} />已走轨迹</span>
              <span className={styles.canvasHint}><MousePointer2 size={14} />{observe ? '拖动旋转 · 滚轮缩放' : '选点后确认导航'}</span>
            </>}
          </div>
        </div>

        {/* Right: side panel */}
        <aside className={styles.sidePanel} id="scene-inspector" aria-label="现场面板"
          hidden={(drawerOpen && !observe) || (isMappingSession && !inspectorOpen)}>
          <div className={styles.inspectorTabs} role="tablist" aria-label="现场面板内容"
            onKeyDown={event => {
              if (!['ArrowLeft', 'ArrowRight', 'Home', 'End'].includes(event.key)) return
              event.preventDefault()
              const tabs = [...event.currentTarget.querySelectorAll<HTMLButtonElement>('[role="tab"]')]
              const index = tabs.indexOf(document.activeElement as HTMLButtonElement)
              const next = event.key === 'Home' ? 0 : event.key === 'End' ? tabs.length - 1
                : (index + (event.key === 'ArrowRight' ? 1 : -1) + tabs.length) % tabs.length
              tabs[next]?.click()
              tabs[next]?.focus()
            }}>
            {inspectorTabs.map(item => <button key={item.key} role="tab" id={`scene-tab-${item.key}`}
              aria-selected={inspectorTab === item.key} aria-controls={`scene-panel-${item.key}`}
              tabIndex={inspectorTab === item.key ? 0 : -1} onClick={() => setInspectorTab(item.key)}>
              <item.icon size={16} />{item.label}
            </button>)}
          </div>
            {/* Camera is optional and opens its stream only while visible. */}
            {cameraVisible && <div
              className={styles.cameraDock}
              aria-label="相机实时画面"
              onPointerDown={event => event.stopPropagation()}
            >
              <div className={styles.cameraPipHeader}>
                <span className={cameraPipDotClass} />
                {cameraPipLabel}
                <button onClick={() => setCameraPreference(false)} aria-label="收起相机"><X size={16} /></button>
              </div>
              {cameraPipRecovered && cameraImgSrc
                ? <img src={cameraImgSrc} className={styles.cameraPipImg} alt="机器人相机实时画面" draggable={false} />
                : <div className={styles.cameraPipEmpty}><VideoOff size={24} /><span>{cameraStatus?.status === 'not_loaded'
                  ? '当前运行模式未接入相机' : '收到新画面后自动显示'}</span></div>
              }
              {cameraPipRecovered && cameraStatus?.available && <div className={styles.cameraPipInfo}>
                <span>彩色 {cameraStatus.color.fps.toFixed(0)} 帧/秒</span>
                <span>深度 {cameraStatus.depth.fps.toFixed(0)} 帧/秒</span>
              </div>}
            </div>}
          <div className={styles.inspectorBody} role="tabpanel" id="scene-panel-status"
            aria-labelledby="scene-tab-status" hidden={inspectorTab !== 'status'} tabIndex={0}>
          <section className={styles.statCard} aria-label="机器人位姿">
            <div className={styles.statCardTitle}>{robotModel === 'go2' ? 'Go2' : robotModel === 'thunder_v4' ? 'Thunder v4' : '机器人'}<span role="status">{poseLabel}</span></div>
            <div className={styles.speedOverview}>
              <strong className={styles.robotSpeedValue}>{displaySpeed}</strong><span>实测前进速度</span>
            </div>
            {robotModel === 'go2' && <RobotJointStatus stream={sseState.jointTelemetry} />}
            <details className={styles.statusDetails}>
              <summary>位姿详情</summary>
            <div className={styles.poseGrid}>
              <div className={styles.statItem}><span className={styles.statLabel}>X · m</span><span className={`${styles.statValue} ${styles.robotPositionValue}`}>{displayRobotX}</span></div>
              <div className={styles.statItem}><span className={styles.statLabel}>Y · m</span><span className={`${styles.statValue} ${styles.robotPositionValue}`}>{displayRobotY}</span></div>
              <div className={styles.statItem}><span className={styles.statLabel}>Z · m</span><span className={styles.statValue}>{displayRobotZ}</span></div>
            </div>
            <div className={styles.metricRow}><span>朝向</span><strong className={styles.robotYawValue}>{displayYawDeg}</strong></div>
            </details>
          </section>

          <section className={styles.statCard} aria-label="运动与规划">
            <div className={styles.statCardTitle}>规划<span>{missionStateLabel}</span></div>
            {sseState.connected && <p className={styles.statusLine} role="status">{plannerLabel}</p>}
            {!observe && sseState.connected && !canSendGoal && <p className={styles.statusLine}>{goalDisabledReason}</p>}
            <details className={styles.statusDetails}>
              <summary>控制详情</summary>
            <table className={styles.commandTable} aria-label="请求与输出速度">
              <thead><tr><th></th><th>前后</th><th>左右</th><th>转向</th></tr></thead>
              <tbody>
                <tr><th scope="row">请求</th>{commandValues(nativeFresh ? request : undefined).map((v, i) => <td key={i}>{v}</td>)}</tr>
                <tr><th scope="row">输出</th>{commandValues(nativeFresh ? finalCommand : undefined).map((v, i) => <td key={i}>{v}</td>)}</tr>
              </tbody>
            </table>
            <p className={styles.units}>平移 m/s · 转向 rad/s</p>
            <div className={styles.metricRow}><span>当前运动</span><strong>{navigationView.motion.observation.label}</strong></div>
              <div className={styles.metricRow}><span>导航准备</span><strong>{navigationView.goalAdmission.label}</strong></div>
              <div className={styles.metricRow}><span>控制权</span><strong>{navigationView.control.label}</strong></div>
              <div className={styles.metricRow}><span>运动许可</span><strong>{navigationView.motion.permission.label}</strong></div>
              <div className={styles.metricRow}><span>停稳确认</span><strong>{navigationView.motion.stopConfirmation.label}</strong></div>
              {sceneDebugTools && <p className={styles.units}>{String(lastLocal?.reason ?? '—')}</p>}
            </details>
          </section>

          <section className={styles.statCard} aria-label="雷达与定位">
            <div className={styles.statCardTitle}>雷达<span>{alignedScanCloud ? `${scanCloud.count.toLocaleString()} 点` : '未更新'}</span></div>
            <details className={styles.statusDetails}>
              <summary>雷达详情</summary>
            <p className={styles.sourceNote}>当前扫描，非完整碰撞栅格。</p>
            <div className={styles.metricRow}><span>定位更新</span><strong>{telemetryFresh ? formatHz(processedScanHz) : '—'}</strong></div>
            <div className={styles.metricRow}><span>雷达输入</span><strong>{telemetryFresh ? formatHz(lidarInputHz) : '—'}</strong></div>
            <div className={styles.metricRow}><span>IMU 输入</span><strong>{telemetryFresh ? formatHz(numericMetric(slamDiag, 'imu_input_hz')) : '—'}</strong></div>
            <div className={styles.metricRow}><span>局部地图点</span><strong>{cloud.count > 0 ? `${cloud.count.toLocaleString()} 点` : '当前未发布'}</strong></div>
            {sceneDebugTools && <div className={styles.metricRow}><span>后端地图点数</span><strong>{formatCount(displayedMapPoints)}</strong></div>}
            </details>
          </section>

          </div>
          <div className={`${styles.inspectorBody} ${styles.layerSettings}`} role="tabpanel" id="scene-panel-layers"
            aria-labelledby="scene-tab-layers" hidden={inspectorTab !== 'layers'} tabIndex={0}>
                {isMappingSession && <details className={styles.mappingReadingKey} aria-label="建图显示图例">
                  <summary>图例</summary>
                  {showMappingObservation ? <>
                    <p className={styles.mappingTruth}>地面相对高度 · 不代表可通行</p>
                    <div className={styles.planningMapLegend}>
                      <span><i style={{ background: 'rgba(83, 133, 129, 0.49)' }} />支撑候选</span>
                      <span><i style={{ background: 'rgba(201, 111, 99, 0.90)' }} />障碍回波</span>
                      <span><i className={styles.unobservedSwatch} />未确认</span>
                    </div>
                    {mappingObservation.status !== 'ready' && <span>{!sseState.connected
                      ? '已断开 · 观测图暂不可用' : mappingObservation.message}</span>}
                    {mappingObservation.status !== 'ready' && cloud.count > 0 && <span>仅显示点云参考，暂不能判读观测覆盖</span>}
                    {mappingObservation.status === 'ready' && <div className={styles.observationArea}>
                      <span>已分类 {((mappingObservation.freeCount + mappingObservation.occupiedCount) * mappingObservation.layer.resolution ** 2).toFixed(1)} m²</span>
                      <span>未确认 {((mappingObservation.unknownCount) * mappingObservation.layer.resolution ** 2).toFixed(1)} m²</span>
                    </div>}
                  </> : <>
                    <span>{cloud.count.toLocaleString()} 个{showGlobalMapping ? '整图预览' : '局部地图'}点</span>
                    <span>{showGlobalMapping
                      ? (cloud.count > 0 ? '本次累计建图 · 显示经过采样' : '等待整图数据')
                      : '当前局部窗口 · 切换整图查看累计范围'}</span>
                  </>}
                  {sceneDebugTools && <details><summary>诊断说明</summary>
                    <p>红格表示比附近支撑候选高 12–80 cm 的占据回波。地面回波保留在三维地图中，不再直接染红。</p>
                    <p>二维投影不是地面高度。底图放在脚下便于观察，点云保留原始三维高度。</p>
                    <p>点击格子可查看局部表面拟合高度、残差和细 XY 支撑数；缺失时不作推断。</p>
                    {mappingObservation.status === 'ready' && <>
                      <p>当前窗口 {(mappingObservation.layer.cols * mappingObservation.layer.resolution).toFixed(1)} × {(mappingObservation.layer.rows * mappingObservation.layer.resolution).toFixed(1)} m</p>
                    </>}
                    <p>灰格缺少可靠的地面依据，可从不同位置和朝向补扫。窗口外不表示从未建图。</p>
                    <p>蓝绿格是已观测支撑候选，灰格是缺少依据。20 cm 网格不检查机身净空；导航仍使用三维碰撞和通行图。</p>
                    <p>局部投影和空间点云都来自当前局部窗口；完整 SLAM 建图只在保存地图后查看。这里不计算建图完成百分比。</p>
                  </details>}
                </details>}

            {!isMappingSession && <div className={styles.dataExplanation}>
              <Info size={16} aria-hidden="true" />
              <p>点云空白 ≠ 可通行</p>
            </div>}
            <span className={styles.menuLabel}>显示图层</span>
            <div className={styles.layerGroup}>
              {isMappingSession ? <LayerButton active={mappingView === 'points'} icon={<Cloud size={13} />} label="空间点云"
                onClick={() => { setMappingView(value => value === 'points' ? 'coverage' : 'points'); setLayers(value => ({ ...value, cloud: true })) }} />
                : <LayerBtn k="cloud" icon={<Cloud size={13} />} label="在线地图点云" />}
              <LayerButton active={liveScanVisible} icon={<Radio size={13} />} label="实时雷达"
                onClick={() => setLiveScanPreference(!liveScanVisible)} />
              <LayerBtn k="path" icon={<Navigation size={13} />} label="规划路径" />
              <LayerBtn k="trail" icon={<Route size={13} />} label="行走轨迹" />
              <LayerBtn k="goal" icon={<Target size={13} />} label="目标点" />
              <LayerBtn k="robot" icon={<Bot size={13} />} label="机器人" />
              <LayerBtn k="grid" icon={<Grid3x3 size={13} />} label="参考网格" />
              <LayerBtn k="nativeTraversability" icon={<Activity size={13} />} label="局部风险栅格" />
              <LayerButton active={cameraVisible} icon={<Camera size={13} />} label="相机画面"
                onClick={() => setCameraPreference(!cameraVisible)} />
            </div>
            <div className={styles.diagnosticLayer}>
              <LayerBtn k="localPlanner" icon={<Radio size={13} />} label="安全范围" />
              {layers.localPlanner && <div className={styles.layerBtns}>
                <LayerButton active={safetyView === 'slice'} icon={<Grid3x3 size={13} />} label="高度切片"
                  onClick={() => { setSafetyView('slice'); scene3DRef.current?.topView() }} />
                <LayerButton active={safetyView === 'volume'} icon={<Layers2 size={13} />} label="三维诊断"
                  onClick={() => setSafetyView('volume')} />
              </div>}
              <p>采样显示，非完整碰撞栅格。</p>
            </div>
            <label className={styles.pointSizeRow}>
              点云大小
              <input type="range" min={0.02} max={0.2} step={0.005} value={pointSize}
                onChange={e => setPointSize(parseFloat(e.target.value))} className={styles.pointSlider} />
            </label>
            <details className={styles.advancedTools}>
              <summary>地形与诊断</summary>
              <div className={styles.layerGroup}>
              <LayerBtn k="elevation" icon={<MapPinned size={13} />} label="最低观测高程" />
              </div>
              {!observe && <div className={styles.toolGroup}>
                <button onClick={handleClearTrail}><Trash2 size={14} /> 清除显示轨迹</button>
                <button onClick={handleClearCloud} title="清除显示点云缓存，不修改底层定位"><Cloud size={14} /> 清除显示点云</button>
              </div>}
            </details>
            <details className={styles.advancedTools}>
              <summary>操作说明</summary>
              <p className={styles.statusLine}>{observe ? '拖动旋转视角，滚轮缩放。' : '拖动旋转视角，滚轮缩放。点击选择目标，确认后才会开始导航。'}</p>
              <p className={styles.statusLine}>机身跟随定位；Go2 腿部跟随实测关节角。无关节数据时显示站姿，断流后保留最后姿态。网格仅作坐标参考。</p>
            </details>
          </div>
          {!observe && <div className={`${styles.inspectorBody} ${styles.operationSettings}`} role="tabpanel" id="scene-panel-tools"
            aria-labelledby="scene-tab-tools" hidden={inspectorTab !== 'tools'} tabIndex={0}>
            {recordingPanelOpen && inspectorTab === 'tools' && (
              <div id="scene-recording">
                <RecordingPanel embedded showToast={showToast} locale={locale}
                  status={recordingStatus} statusError={recordingStatusError}
                  refreshStatus={refreshRecordingStatus} onClose={() => setRecordingPanelOpen(false)} />
              </div>
            )}
              <div className={styles.toolGroup}>
                <button onClick={() => openWorkspaceTool('maps')}><MapPinned size={14} /> 地图</button>
                <button onClick={() => openWorkspaceTool('locations')}><Target size={14} /> 常用位置</button>
                <button onClick={() => openWorkspaceTool('localization')}><LocateFixed size={14} /> 定位</button>
              </div>
              <div className={styles.toolGroup}>
                <span className={styles.menuLabel}>数据记录</span>
                <button onClick={() => {
                  setRecordingPanelOpen(open => !open)
                }} aria-expanded={recordingPanelOpen} aria-controls="scene-recording" title={recordingToolbarTitle}>
                  <CircleDot size={12} /> {recordingPanelOpen ? '收起录制' : '录制'}
                  {recordingToolbarState && <span className={styles.menuMeta}> · {recordingToolbarState}</span>}
                </button>
              </div>
          </div>}
          <div className={styles.controlSummary}>
            <div><span>控制权</span><strong>{navigationView.control.label}</strong></div>
            <div><span>运动许可</span><strong>{navigationView.motion.permission.label}</strong></div>
          </div>
          {!observe && hasGoal && <div className={styles.motionActions}>
            <button className={styles.cancelNavBtn} onClick={handleCancelNavigation} title="取消当前导航任务">
              <X size={15} /> 取消导航
            </button>
          </div>}
        </aside>
      </div>

      {/* Click-away to close context menu */}
      {mapContextMenu && (
        <div style={{ position: 'fixed', inset: 0, zIndex: 99 }} onClick={() => setMapContextMenu(null)} />
      )}

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
        title="删除位置"
        message={`确认删除位置「${locationDeleteTarget?.name ?? ''}」？`}
        confirmLabel="删除"
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
        title="保存地图"
        message="保存完成后打开整图快照。"
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
