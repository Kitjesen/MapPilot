import { isObservationMode } from '../services/observationMode.ts'
import { parseInitialPose } from '../services/localizationInitialPose.ts'
import { navigationGoalErrorMessage, navigationPreviewErrorMessage, navigationPreviewIsCurrent, navigationPlanEndpoint, sceneGoalHeight, navigationSupportHeightFromRunPlan } from '../services/navigationPreview.ts'
import { resolveMappingObservation, mappingObservationPointAt } from '../services/mappingObservation.ts'
import { estimateSceneTime, freshSource, freshScenePose, projectScenePose, projectScenePath, scanCanStandAlone, currentNativeLocalPath, scenePoseEpoch, sceneTrailStorageKey, sceneRelocalizationSeed, activeMapRelocalizationTarget } from '../services/sceneTelemetry.ts'
import { useRef, useEffect, useCallback, useMemo, useState, memo, type ReactNode, type PointerEvent as ReactPointerEvent } from 'react'
import { createPortal } from 'react-dom'
import {
  Grid3x3, Navigation, Route, Target, Bot,
  ArrowLeft, Save, Trash2, Pencil, X,
  MapPinned, Cloud, Maximize2, Minimize2, Radio, Activity, LocateFixed, CircleDot,
  RefreshCw, Gamepad2, Layers2, Camera, SlidersHorizontal, Info,
  ChevronUp, ChevronDown, Play, Pause,
} from 'lucide-react'
import type {
  SSEState,
  AgentTaskStatusResponse,
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
  InspectionTaskStatusResponse,
  InspectionRoute,
  InspectionRouteRequest,
} from '../types'
import * as api from '../services/api'
import { formatMapSaveProgress, pendingMapSaveStatus, savedMapStatus, archivedMapStatus, failedMapSaveStatus, mapSaveProgressValue, mapSaveElapsedMs, formatMapSaveElapsed, mappingCapacityNotice, mappingFailureNotice, type MapSaveStatus } from '../services/mapSavePresentation.ts'
import {
  mapSaveBlockedReason,
} from '../services/mapReadiness'
import { liveNavigationStatus, operatorReasonMessage, presentNavigationStatus } from '../services/navigationStatus'
import { useCamera } from '../hooks/useCamera'
import { useWHEP } from '../hooks/useWHEP'
import { useVideoFrame } from '../hooks/useVideoFrame'
import { useRotatedVideoLayout } from '../hooks/useRotatedVideoLayout'
import { useBinaryCloud } from '../hooks/useBinaryCloud'
import { useOctomap } from '../hooks/useOctomap.ts'
import { CLOUD_HEIGHT_GRADIENT, CLOUD_HEIGHT_MIN, CLOUD_HEIGHT_MAX, cloudColorRange } from '../services/cloudHeightColor.ts'
import {
  cloudFramesShareCoordinateEpoch,
  savedMapNeedsSceneRebind,
} from '../workers/cloudDecoderCore.ts'
import { PromptModal, ConfirmModal } from './Modal'
import { MapLibrary } from './MapLibrary.tsx'
import { useMapSaves } from '../hooks/useMapSaves.ts'
import { mapLibraryEntries } from '../services/mapSavePresentation.ts'
import { Scene3D, type Scene3DHandle } from './Scene3D'
import type { GoalPickMode } from './scene3d/goalPicking.ts'
import { StairEditor, type StairEditorHandle } from './StairEditor'
import { MapEditor, type MapEditorHandle } from './MapEditor'
import type { MapEditScene, MapEditView } from '../types/mapEdit.ts'
import { HeightRangeControl } from './HeightRangeControl'
import { CameraTargetOverlay } from './CameraTargetOverlay'
import { deriveSceneTarget } from '../services/sceneTarget'
import type { StairScene } from '../types/stairs'
import { RobotJointStatus } from './RobotJointStatus.tsx'
import { safetyEnvelopeFromRunPlan, type SafetyEnvelope } from './scene3d/layers/safetyEnvelope.ts'
import { RecordingPanel } from './RecordingPanel'
import { TeleopPanel } from './TeleopPanel'
import { supportsWebTeleop } from '../services/teleopControl.ts'
import { mapLocalizationLabel, mapLocalizationState } from '../services/mapLabel.ts'
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
import {
  EMPTY_ROUTE_DRAFT,
  appendRouteDraftPoint,
  buildInspectionRouteRequest,
  ensureRouteStartIntent,
  moveRouteDraftPoint,
  removeRouteDraftPoint,
  resolveRouteDraftIdentity,
  routeDraftSignature,
  routeDraftFromSaved,
  saveAndStartRouteDraft,
  selectUniqueOpenRouteTask,
  sameRouteDraftIdentity,
  type RouteDraft,
  type RouteDraftIdentity,
  type RouteStartIntent,
} from '../services/routeDraft.ts'

interface SceneViewProps {
  workspace?: 'mapping' | 'navigation'
  onResetEstop?: () => void
  resetEstopReason?: string
  instructionStatus?: AgentTaskStatusResponse | null
  onTaskPanelMount?: (node: HTMLDivElement | null) => void
  sseState:  SSEState
  showToast: (msg: string, kind?: ToastKind) => void
  locale: Locale
  motionStartAllowed: boolean
  motionStartBlockedReason: string
  instructionActive: boolean
  onElevationSubscriptionChange?: (enabled: boolean) => void
  onOpenSavedMap: (name: string | null) => void
  initialRoute?: InspectionRoute | null
  onRouteSaved?: (route: InspectionRoute) => void
  onStartMapping: () => void
  productSwitchAllowed: boolean
  productSwitchReason: string
  onMapEditorExitGuardChange?: (guard: ((proceed: () => void) => void) | null) => void
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
const GOAL_SPEED_OPTIONS = [0.2, 0.25, 0.4, 0.5, 0.6, 0.8, 1.0]
const GOAL_RADIUS_OPTIONS = [0.25, 0.45, 0.8]

type NavigationSelectionMode = 'browse' | 'single' | 'route'

function routeTaskStateLabel(status: InspectionTaskStatusResponse | null): string {
  if (!status) return '等待任务状态'
  const labels: Record<string, string> = {
    ACCEPTED: '等待原生执行器确认',
    RUNNING: '路线执行中',
    PAUSED: '路线已暂停',
    SUCCEEDED: '路线已完成',
    COMPLETED: '路线已完成',
    FAILED: '路线执行失败',
    CANCELLED: '路线已取消',
  }
  return labels[status.current_state] ?? (status.current_state || '状态未知')
}

type WorkbenchZoneState = 'occupied' | 'free'

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

function normalizeGoalHeight(value: number | null | undefined): number | null {
  if (typeof value !== 'number' || !Number.isFinite(value)) return null
  // Keep planner precision; only remove floating-point noise around the ground plane.
  return normalizeDisplayZero(value, 2)
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
  const reason = reasons.slice(0, 3).map(item => operatorReasonMessage(item, 'zh')).join(' / ')
    || operatorReasonMessage(error || preview?.error || 'plan preview rejected', 'zh')
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
  workspace = 'navigation',
  onResetEstop,
  resetEstopReason,
  instructionStatus,
  onTaskPanelMount,
  sseState,
  showToast,
  locale,
  motionStartAllowed,
  motionStartBlockedReason,
  instructionActive,
  onElevationSubscriptionChange,
  onOpenSavedMap,
  initialRoute,
  onRouteSaved,
  onStartMapping,
  productSwitchAllowed,
  productSwitchReason,
  onMapEditorExitGuardChange,
}: SceneViewProps) {
  const isMappingWorkspace = workspace === 'mapping'
  const scene3DRef = useRef<Scene3DHandle>(null)
  const workspaceRef = useRef<HTMLDivElement>(null)
  const navigationPanelsRef = useRef<HTMLDivElement>(null)
  useEffect(() => {
    const workspace = workspaceRef.current
    const panels = navigationPanelsRef.current
    if (!workspace || !panels) return
    const reserveTaskSpace = () => {
      const inset = Math.max(0, workspace.getBoundingClientRect().bottom - panels.getBoundingClientRect().top) + 12
      workspace.style.setProperty('--scene-task-inset', `${inset}px`)
    }
    // Task details and the instruction composer can change the bottom area's height.
    const observer = new ResizeObserver(reserveTaskSpace)
    observer.observe(workspace)
    observer.observe(panels)
    reserveTaskSpace()
    return () => {
      observer.disconnect()
      workspace.style.removeProperty('--scene-task-inset')
    }
  }, [])
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
  const [selectedWorkspaceTool, setWorkspaceTool] = useState<'maps' | 'locations' | 'localization'>('maps')
  const workspaceTool = isMappingWorkspace ? 'maps' : selectedWorkspaceTool
  const [inspectorTab, setInspectorTab] = useState<'status' | 'layers' | 'tools'>('status')
  const layerTrigger = useRef<HTMLButtonElement>(null)
  const inspectorPanel = useRef<HTMLElement>(null)
  const layerDismissTimer = useRef<ReturnType<typeof setTimeout> | undefined>(undefined)
  const cancelLayerDismiss = useCallback(() => {
    clearTimeout(layerDismissTimer.current)
    layerDismissTimer.current = undefined
  }, [])
  useEffect(() => cancelLayerDismiss, [inspectorOpen, inspectorTab, cancelLayerDismiss])
  const leaveLayers = (event: ReactPointerEvent) => {
    if (event.pointerType !== 'mouse' || inspectorTab !== 'layers') return
    cancelLayerDismiss()
    const next = event.relatedTarget instanceof Node ? event.relatedTarget : null
    if (next && (layerTrigger.current?.contains(next) || inspectorPanel.current?.contains(next))) return
    layerDismissTimer.current = setTimeout(() => setInspectorOpen(false), 220)
  }
  const inspectorTabs = [
    { key: 'status', label: '诊断', icon: Activity },
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
  const [cameraPreference, setCameraPreference] = useState(true)
  const cameraVisible = cameraPreference
  const [cameraExpanded, setCameraExpanded] = useState(false)
  const cameraToggleRef = useRef<HTMLButtonElement>(null)
  const [cameraStatus, setCameraStatus] = useState<CameraMediaStatus | null>(null)
  const [saveModalOpen, setSaveModalOpen] = useState(false)
  const [autoProcess, setAutoProcess] = useState(true)
  const [recordingView, setRecordingView] = useState<'record' | 'history' | null>(null)
  const recordingPanelOpen = recordingView !== null
  const [sensorDiagnosticsVisible, setSensorDiagnosticsVisible] = useState(false)
  const [sceneDockTarget, setSceneDockTarget] = useState<HTMLElement | null>(null)
  const [teleopMode, setTeleopMode] = useState(false)
  const [resumePending, setResumePending] = useState(false)
  const [resumeError, setResumeError] = useState<string | null>(null)
  const [liveScanPreference, setLiveScanPreference] = useState<boolean | null>(null)
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
  const [envelopeConfig, setEnvelopeConfig] = useState<{ session: string; shape: SafetyEnvelope | null; supportHeight: number | null } | null>(null)
  const [maps, setMaps] = useState<MapInfo[]>([])
  const [mapListStatus, setMapListStatus] = useState<'loading' | 'ready' | 'error'>('loading')
  const mapListRequestInFlight = useRef(false)
  const [pointSize, setPointSize] = useState(0.18)
  const [cloudColorMode, setCloudColorMode] = useState<'height' | 'neutral'>('height')
  const [mapRepresentation, setMapRepresentation] = useState<'points' | 'voxels'>('points')
  const [voxelScale, setVoxelScale] = useState(2)
  const [cloudHeightModePreference, setCloudHeightModePreference] = useState<'all' | 'follow' | 'fixed' | null>(null)
  const [cloudHeightAboveBase, setCloudHeightAboveBase] = useState(1)
  const [cloudHeightBelowBase, setCloudHeightBelowBase] = useState(1)
  const [cloudHeightBase, setCloudHeightBase] = useState<{ epoch: string | null; z: number } | null>(null)
  const [fixedCloudHeight, setFixedCloudHeight] = useState<{ epoch: string; z: number; input: string; below?: number; above?: number } | null>(null)
  const [savedMapCloud, setSavedMapCloud] = useState<api.SavedMapPointCloud | undefined>()
  const [stairEditorOpen, setStairEditorOpen] = useState(false)
  const [mapEditorKey, setMapEditorKey] = useState<string | null>(null)
  const [mapEditState, setMapEditState] = useState<{ key: string; scene: MapEditScene } | null>(null)
  const [mapEditAnchor, setMapEditAnchor] = useState<{ x: number; y: number } | null>(null)
  const [mapEditFooter, setMapEditFooter] = useState<HTMLDivElement | null>(null)
  const mapEditorRef = useRef<MapEditorHandle>(null)
  const stairEditorRef = useRef<StairEditorHandle>(null)
  const [stairScene, setStairScene] = useState<{ key: string; value: StairScene } | null>(null)
  const [savedMapLoadError, setSavedMapLoadError] = useState<string | null>(null)
  const savedMapFlat = savedMapCloud?.points
  const [relocOpen, setRelocOpen] = useState(false)
  const [relocDropOpen, setRelocDropOpen] = useState(false)
  const [relocMap, setRelocMap] = useState('')
  const relocDropRef = useRef<HTMLDivElement>(null)
  const [relocX, setRelocX] = useState('0')
  const [relocY, setRelocY] = useState('0')
  const [relocZ, setRelocZ] = useState('0')
  const [relocYaw, setRelocYaw] = useState('0')
  const [relocPending, setRelocPending] = useState(false)
  const [restartLocalizationPending, setRestartLocalizationPending] = useState(false)
  const [saveStatus, setSaveStatus] = useState<MapSaveStatus | null>(null)
  const [saveTiming, setSaveTiming] = useState({ elapsedMs: 0, observedAt: 0 })
  const [saveProgress, setSaveProgress] = useState<number | undefined>()
  const [saveClock, setSaveClock] = useState(0)
  const saveStartedAt = useRef(0)
  const saveActive = saveStatus?.state === 'saving' || saveStatus?.state === 'pending'
  useEffect(() => {
    setSceneDockTarget(document.getElementById('scene-actions-slot'))
  }, [])
  useEffect(() => {
    if (!saveActive) return
    const timer = window.setInterval(() => setSaveClock(Date.now()), 1000)
    return () => window.clearInterval(timer)
  }, [saveActive])
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
  const [navigationSelectionMode, setNavigationSelectionMode] = useState<NavigationSelectionMode>(initialRoute ? 'route' : 'single')
  const [goalPickMode, setGoalPickMode] = useState<'plane' | 'surface'>('plane')
  const [routeDraft, setRouteDraft] = useState<RouteDraft>(() => initialRoute ? routeDraftFromSaved(initialRoute) : EMPTY_ROUTE_DRAFT)
  const [routeName, setRouteName] = useState(initialRoute?.name || '连续点路线')
  const [savedDraftRoute, setSavedDraftRoute] = useState<{ signature: string; id: string; revision: number } | null>(() => initialRoute ? {
    signature: routeDraftSignature(routeDraftFromSaved(initialRoute), 0.45, initialRoute.name || '连续点路线'),
    id: initialRoute.id, revision: initialRoute.revision ?? 1,
  } : null)
  const [routeStartIntent, setRouteStartIntent] = useState<RouteStartIntent | null>(null)
  const [routeTaskId, setRouteTaskId] = useState<string | null>(null)
  const [routeTaskStatus, setRouteTaskStatus] = useState<InspectionTaskStatusResponse | null>(null)
  const [routeTaskError, setRouteTaskError] = useState<string | null>(null)
  const [routeActionPending, setRouteActionPending] = useState<'save' | 'start' | 'pause' | 'resume' | 'cancel' | null>(null)
  const [explorationStatus, setExplorationStatus] = useState<ExplorationStatusResponse | null>(null)
  const [explorationStatusLoading, setExplorationStatusLoading] = useState(false)
  const [directedExplorationBusy, setDirectedExplorationBusy] = useState(false)
  const [goalMaxSpeed, setGoalMaxSpeed] = useState<number | null>(null)
  const [goalYaw, setGoalYaw] = useState<number | undefined>(undefined)
  const [goalHeadingEditing, setGoalHeadingEditing] = useState(false)
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
  const [workbenchZoneState, setWorkbenchZoneState] = useState<WorkbenchZoneState>('occupied')
  const [workbenchZoneRadius, setWorkbenchZoneRadius] = useState('0.5')
  const [workbenchBusy, setWorkbenchBusy] = useState<string | null>(null)
  const [workbenchSummary, setWorkbenchSummary] = useState<string | null>(null)
  // Map management modals
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

  const whep = useWHEP('/api/v1/webrtc/whep', 'cam', cameraVisible && sseState.connected)
  const whepConnected = whep.connected
  const whepReconnect = whep.reconnect
  const whepRotationDegrees = whep.rotationDegrees
  const whepStream = whep.stream
  const cameraMediaRef = useRef<HTMLDivElement>(null)
  const cameraVideoRef = useRef<HTMLVideoElement>(null)
  const cameraVideoFrame = useVideoFrame(
    cameraVideoRef,
    whepStream,
    cameraVisible && sseState.connected && whepConnected,
  )
  const cameraVideoLayout = useRotatedVideoLayout(
    cameraMediaRef,
    cameraVideoRef,
    whepRotationDegrees,
    whepStream != null,
  )
  const useWebRTCCamera = whepConnected && cameraVideoFrame.hasFrame && cameraVideoLayout.ready
  const { imgSrc: cameraImgSrc, connected: cameraConnected, lastFrameAt: cameraLastFrameAt } = useCamera(
    cameraVisible && !useWebRTCCamera ? '/ws/camera' : '',
  )
  const jpegCameraRecovered = cameraConnected && cameraImgSrc != null
    && cameraLastFrameAt != null && Date.now() - cameraLastFrameAt < 3000
  const cameraPipRecovered = sseState.connected && (useWebRTCCamera || jpegCameraRecovered)
  const sceneTarget = deriveSceneTarget({ task: instructionStatus, sseState,
    nowS: localNowS, cameraLive: cameraPipRecovered })
  useEffect(() => {
    if (!cameraExpanded) return
    const close = (event: KeyboardEvent) => {
      if (event.key !== 'Escape') return
      event.preventDefault()
      setCameraExpanded(false)
      cameraToggleRef.current?.focus()
    }
    document.addEventListener('keydown', close, true)
    return () => document.removeEventListener('keydown', close, true)
  }, [cameraExpanded])
  const cameraHasFrame = useWebRTCCamera || jpegCameraRecovered
  const cameraPipLabel = cameraHasFrame ? '实时画面'
    : cameraStatus?.status === 'not_loaded' ? '相机未启用'
      : cameraLastFrameAt ? '画面已暂停' : '等待相机画面'
  useEffect(() => {
    if (cameraVideoFrame.stalled && whepConnected) whepReconnect()
  }, [cameraVideoFrame.stalled, whepConnected, whepReconnect])
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
    showGlobalMapping ? '/api/v1/map/global/points?max_points=120000&format=binary' : null, 1)
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
  const odom = sseState.odometry
  const telemetryFresh = sseState.connected && freshScenePose(odom?.ts, rasterNowS, sseState.session?.env, localization)
  const mapOdom = telemetryFresh ? projectScenePose(odom) : null
  const poseAvailable = mapOdom !== null
  const robotX = mapOdom?.x ?? 0
  const robotY = mapOdom?.y ?? 0
  const robotZ = mapOdom?.z ?? 0
  const yaw = mapOdom?.yaw ?? 0
  const displayRobotX = poseAvailable ? formatTelemetryValue(robotX, 2) : '--'
  const displayRobotY = poseAvailable ? formatTelemetryValue(robotY, 2) : '--'
  const displayRobotZ = poseAvailable ? formatTelemetryValue(mapOdom.z ?? 0, 2) : '--'
  const displayYawDeg = poseAvailable ? `${formatTelemetryValue((yaw * 180) / Math.PI, 0)}°` : '--'
  const displaySpeedValue = poseAvailable ? formatTelemetryValue(mapOdom.vx, 2) : '--'
  const displaySessionDistance = poseAvailable && typeof odom?.session_distance_m === 'number'
    ? formatTelemetryValue(odom.session_distance_m, 1)
    : '—'
  const nativeStatus = localPlannerSnapshot?.nav_endpoint
  const nativeFresh = sseState.connected && freshSource(nativeStatus?.stamp_s, rasterNowS)
  const nativeProduct = nativeStatus?.native_product as { product_session_id?: string; session_id?: string } | undefined
  const nativeSession = nativeProduct?.product_session_id ?? nativeProduct?.session_id ?? ''
  const safetyEnvelope = nativeFresh && envelopeConfig?.session === nativeSession ? envelopeConfig.shape : null
  const navigationSupportHeight = envelopeConfig?.session === nativeSession ? envelopeConfig.supportHeight : null
  useEffect(() => {
    if (!sseState.connected || !nativeSession) return
    let disposed = false
    void api.fetchRuntimeDataflow().then(data => {
      if (disposed) return
      if (data.runtime_boundary.product_session_id !== nativeSession) {
        setEnvelopeConfig(null)
        return
      }
      setEnvelopeConfig({ session: nativeSession, shape: safetyEnvelopeFromRunPlan(data.run_plan),
        supportHeight: navigationSupportHeightFromRunPlan(data.run_plan) })
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
    return projectScenePath(points, nativeStatus?.planning_frame_id)
  }
  const path = localPlannerDiagnosticsEnabled
    ? nativePath('global_path')
    : sseState.connected
      ? projectScenePath(sseState.globalPath?.points ?? [], sseState.globalPath?.frame_id) : []
  const localPathPts = localPlannerDiagnosticsEnabled
    ? currentNativeLocalPath(nativeStatus, sseState.connected, rasterNowS)
    : sseState.connected && freshSource(sseState.localPath?.stamp_s, rasterNowS)
      ? projectScenePath(sseState.localPath?.points ?? [], sseState.localPath?.frame_id) : []
  const mapSceneGraph = sseState.sceneGraph?.frame_id === 'map' ? sseState.sceneGraph : null
  const lastLocal = nativeStatus?.last_local as Record<string, unknown> | undefined
  const tracking = lastLocal?.tracking as Record<string, unknown> | undefined
  const teleop = nativeStatus?.teleop as Record<string, unknown> | undefined
  const request = teleop?.request as Record<string, unknown> | undefined
  const requestedMotion = request && [request.vx, request.vy, request.wz].some(v => typeof v === 'number' && Math.abs(v) > 0.001)
  const finalCommand = nativeStatus?.final_cmd_vel as Record<string, unknown> | undefined
  const commandValues = (value: Record<string, unknown> | undefined) =>
    [value?.vx, value?.vy, value?.wz].map(v => typeof v === 'number' && Number.isFinite(v) ? formatTelemetryValue(v, 2) : '—')
  const poseLabel = mapLocalizationLabel(mapLocalizationState(sseState, localNowS), locale)

  const navigationStatus = liveNavigationStatus(sseState, localNowS)
  const navigationFresh = navigationStatus !== null
  const navigationView = presentNavigationStatus(navigationStatus, locale)
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
  const waitingForRoute = ['EXECUTING', 'RECOVERING'].includes(missionState)
    && navigationStatus?.task.reason === 'replan_waiting'
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
  const plannerIssueLabel = currentPlannerIssue === 'local_no_path' ? '局部规划无路径'
    : currentPlannerIssue === 'local_blocked' ? '局部目标被局部规划器阻挡'
    : currentPlannerIssue === 'local_near_field_stop' ? '近场障碍停车请求'
    : currentPlannerIssue === 'execution_stalled' ? '运动进展不足'
    : currentPlannerIssue === 'invalid_input' ? '局部规划输入无效'
    : currentPlannerIssue === 'scan_initialization_failed' ? '局部规划初始化失败'
    : currentPlannerIssue === 'local_trajectory_blocked_near_robot' ? '机器人附近路径受阻'
      : currentPlannerIssue ? operatorReasonMessage(currentPlannerIssue, locale) : null
  const motionHeld = navigationView.motion.permission.state === 'HELD'
    || navigationView.motion.permission.state === 'ESTOPPED'
  const motionHoldMessage = navigationView.motion.message
    || (waitingForRoute && navigationView.motion.permission.state === 'HELD' ? missionStateLabel : '')
    || navigationView.motion.permission.label
  const plannerAttention = waitingForRoute ? missionStateLabel : missionState === 'FAILED' ? plannerIssueLabel : null
  const operatorAttention = navigationView.control.resumeRequired
    || navigationView.motion.permission.state === 'ESTOPPED' || !!plannerAttention
  const slamHz       = sseState.slamStatus?.slam_hz ?? 0
  const slamDiag = sseState.slamDiag?.data ?? {}
  const capacityNotice = mappingCapacityNotice(sseState.session?.product, slamDiag)
  const mappingNotice = (sseState.connected ? mappingFailureNotice(sseState.session?.product, slamDiag) : null) ?? capacityNotice
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
  const savedMapForScene = showSavedMapInScene
    && savedMapCloud?.mapName === activeMapName
    ? savedMapCloud
    : undefined
  const stairMapKey = `${savedMapForScene?.mapName}:${savedMapForScene?.contentEpoch}`
  const mapSourceKey = `${stairMapKey}:${savedMapForScene?.frameId}`
  const mapEditing = mapEditorKey === mapSourceKey && !!savedMapForScene && !observe && !isMappingWorkspace && !isMappingSession
  const shownMapEdit = mapEditing && mapEditState?.key === mapSourceKey ? mapEditState.scene : null
  const onMapEditSceneChange = useCallback((scene: MapEditScene) => {
    setMapEditState({ key: mapSourceKey, scene })
  }, [mapSourceKey])
  const onMapEditView = useCallback((view: MapEditView) => { scene3DRef.current?.mapEditView(view) }, [])
  const closeMapEditor = useCallback(() => {
    setMapEditorKey(null)
    requestAnimationFrame(() => document.querySelector<HTMLElement>('[aria-label="视图设置"]')?.focus())
  }, [])
  const openMapEditor = () => {
    setMapEditorKey(mapSourceKey); setStairEditorOpen(false); setInspectorOpen(false); setDrawerOpen(false)
    setTeleopMode(false); setFollowRobot(false); setLayers(value => ({ ...value, cloud: true }))
  }
  useEffect(() => {
    if (mapEditorKey && mapEditorKey !== mapSourceKey) showToast('地图来源已变化，旧草稿不会应用到新地图。', 'error')
  }, [mapEditorKey, mapSourceKey, showToast])
  const onStairSceneChange = useCallback((value: StairScene) => {
    setStairScene({ key: stairMapKey, value })
  }, [stairMapKey])
  const stairEditing = stairEditorOpen && !!savedMapForScene && !observe && !isMappingWorkspace && !isMappingSession
  const shownStairs = stairEditing && stairScene?.key === stairMapKey ? stairScene.value : null
  const showNavigationMap = mapRepresentation === 'voxels' && showSavedMapInScene && !mapEditing && !stairEditing
  const navigationMap = useOctomap(savedMapForScene?.mapName, savedMapForScene?.contentEpoch,
    savedMapForScene?.frameId ?? undefined, showNavigationMap && layers.cloud)
  const navigationGeometry = navigationMap.status === 'ready' ? navigationMap.map : undefined
  const mapColorHeightRange = useMemo(() => showNavigationMap && navigationGeometry
    ? cloudColorRange(navigationGeometry.cells, 4)
    : cloudColorRange(savedMapForScene?.points ?? []), [showNavigationMap, navigationGeometry, savedMapForScene?.points])
  const cloudHeightKey = `${activeMap ?? ''}:${poseEpoch ?? ''}`
  const cloudHeightMode = cloudHeightModePreference ?? 'all'
  const lastCloudHeight = cloudHeightBase?.epoch === cloudHeightKey ? cloudHeightBase.z : null
  const followedCloudHeight = poseAvailable ? robotZ : lastCloudHeight
  const currentFixedHeight = fixedCloudHeight?.epoch === cloudHeightKey ? fixedCloudHeight : null
  const fixedHeightBase = followedCloudHeight ?? (mapColorHeightRange ? (mapColorHeightRange.min + mapColorHeightRange.max) / 2 : null)
  const displayHeight = cloudHeightMode === 'fixed' ? currentFixedHeight?.z ?? fixedHeightBase ?? 0 : followedCloudHeight
  const cloudMinZ = cloudHeightMode !== 'all' && displayHeight !== null ? displayHeight - (cloudHeightMode === 'fixed' ? currentFixedHeight?.below ?? 1 : cloudHeightBelowBase) : null
  const cloudMaxZ = cloudHeightMode !== 'all' && displayHeight !== null ? displayHeight + (cloudHeightMode === 'fixed' ? currentFixedHeight?.above ?? 1 : cloudHeightAboveBase) : null
  useEffect(() => {
    if (!poseAvailable) return
    setCloudHeightBase(current => current?.epoch === cloudHeightKey && current.z === robotZ
      ? current : { epoch: cloudHeightKey, z: robotZ })
  }, [poseAvailable, cloudHeightKey, robotZ])
  useEffect(() => {
    if (cloudHeightMode !== 'fixed' || fixedHeightBase === null) return
    setFixedCloudHeight(current => current?.epoch === cloudHeightKey ? current
      : { epoch: cloudHeightKey, z: fixedHeightBase, input: fixedHeightBase.toFixed(2) })
  }, [cloudHeightMode, cloudHeightKey, fixedHeightBase])
  const colorHeightRange = useMemo(() => cloudMinZ !== null && cloudMaxZ !== null
    ? { min: cloudMinZ, max: cloudMaxZ } : mapColorHeightRange, [cloudMinZ, cloudMaxZ, mapColorHeightRange])
  const cloudHeightDomain = useMemo(() => {
    let min = -3, max = 3
    const points = savedMapForScene?.points ?? []
    for (let i = 2; i < points.length; i += 3) {
      if (Number.isFinite(points[i])) { min = Math.min(min, points[i] - 0.5); max = Math.max(max, points[i] + 0.5) }
    }
    return { min, max }
  }, [savedMapForScene?.points])
  const routeDraftIdentity = useMemo<RouteDraftIdentity | null>(() => {
    const activeRecord = maps.find(map => map.name === activeMapName && map.is_active)
    return resolveRouteDraftIdentity(
      activeMapName,
      activeRecord?.content_epoch,
      savedMapForScene?.contentEpoch,
    )
  }, [activeMapName, maps, savedMapForScene?.contentEpoch])
  const routeTaskOpen = Boolean(routeTaskId && !routeTaskStatus?.terminal)
  const routeDraftLocked = routeTaskOpen || routeActionPending !== null
  const normalizedRouteName = routeName.trim() || '连续点路线'
  const currentRouteSignature = routeDraftSignature(routeDraft, goalAcceptanceRadius, normalizedRouteName)
  const routeDraftSaved = savedDraftRoute?.signature === currentRouteSignature
  const runningRouteMatchesDraft = Boolean(
    routeTaskOpen
      && savedDraftRoute
      && routeTaskStatus?.identity.route_id === savedDraftRoute.id
      && routeTaskStatus.identity.route_revision === savedDraftRoute.revision
      && routeTaskStatus.identity.map_id === routeDraft.identity?.mapId
      && routeTaskStatus.identity.map_content_epoch === routeDraft.identity?.contentEpoch,
  )
  const showRouteDraft = !routeTaskOpen || runningRouteMatchesDraft
  const routeTaskPointCount = routeTaskStatus?.found
    && routeTaskStatus.progress.point_count > 0
    ? routeTaskStatus.progress.point_count
    : null
  const routePointCountLabel = routeTaskOpen
    ? routeTaskPointCount === null ? '读取路线中' : `${routeTaskPointCount} 个点`
    : `${routeDraft.points.length} 个点`
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
  const routeDisabledReason =
    observe ? '只读监控' : navigationView.control.resumeRequired ? '请先点击“解除控制暂停”' : !motionStartAllowed
      ? motionStartBlockedReason
      : canAcceptGoal
      ? ''
      : '导航暂未就绪'
  const goalDisabledReason = observe ? '只读监控'
    : instructionActive ? '请先结束当前指令任务，再开始地图导航' : ''
  const canSendGoal = goalDisabledReason === ''
  const canStartRoute = routeDisabledReason === ''
  const robotStatusMessage = !sseState.connected ? '等待实时连接恢复'
    : navigationView.control.resumeRequired ? '控制已暂停'
      : motionHeld ? motionHoldMessage
        : operatorAttention ? plannerAttention
          : !observe && !isMappingSession && !canStartRoute ? routeDisabledReason : null
  const routeStartHint = routeTaskOpen ? ''
    : routeActionPending === 'start' ? '正在保存路线并提交启动请求…'
      : routeDraft.points.length === 1 ? '再添加一个点即可开始路线'
        : routeDraft.points.length === 0 ? '点击三维地图添加路线点位'
          : !sseState.connected ? '实时连接已断开，恢复后可开始路线'
            : !routeDraftIdentity ? '正在读取活动地图版本…'
              : !canStartRoute ? routeDisabledReason
                : '点位顺序确认后即可开始路线'
  const previewDisabledReason = !sseState.connected ? '连接断开，等待恢复'
    : !odom ? '等待机器人位姿'
      : !telemetryFresh ? '定位数据已过期，等待更新'
        : !poseAvailable ? '定位坐标无效，等待地图坐标位姿' : ''
  const previewCurrent = navigationPreviewIsCurrent(pendingGoalPreview, mapOdom)
  const plannedGoal = useMemo(() => previewCurrent ? navigationPlanEndpoint(pendingGoalPreview, pendingGoal) : null,
    [previewCurrent, pendingGoalPreview, pendingGoal])
  const goalPreviewMessage = goalPreviewError ? navigationPreviewErrorMessage(goalPreviewError) : null
  const previewDistance = previewCurrent && typeof pendingGoalPreview?.distance_m === 'number'
    && Number.isFinite(pendingGoalPreview.distance_m) ? pendingGoalPreview.distance_m : null
  const selectedGoalDistance = previewDistance ?? (pendingGoal && poseAvailable && typeof pendingGoal.z === 'number'
    ? Math.hypot(pendingGoal.x - robotX, pendingGoal.y - robotY, pendingGoal.z - robotZ) : null)
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

  useEffect(() => {
    if (!observe) void loadMaps()
  }, [activeMapName, loadMaps, observe, session?.product_session_id, sseState.connected])

  useEffect(() => {
    setLocationsOverride(null)
  }, [sseState.locations?.count, sseState.locations?.ts])

  useEffect(() => {
    if (!routeDraft.identity) return
    const changed = !activeMapName || routeDraft.identity.mapId !== activeMapName
      || Boolean(routeDraftIdentity && !sameRouteDraftIdentity(routeDraft.identity, routeDraftIdentity))
    if (!changed) return
    setRouteDraft(EMPTY_ROUTE_DRAFT)
    setSavedDraftRoute(null)
    setRouteStartIntent(null)
    setRouteName('连续点路线')
  }, [activeMapName, routeDraft.identity, routeDraftIdentity])

  useEffect(() => {
    setRouteStartIntent(null)
  }, [goalAcceptanceRadius])

  useEffect(() => {
    if (!routeTaskId || !sseState.connected) return
    let cancelled = false
    const refresh = async () => {
      try {
        const status = await api.fetchInspectionTask(routeTaskId)
        if (cancelled) return
        setRouteTaskStatus(status)
        setRouteTaskError(null)
      } catch (error: unknown) {
        if (!cancelled) setRouteTaskError(error instanceof Error ? error.message : String(error))
      }
    }
    void refresh()
    if (routeTaskStatus?.terminal) return () => { cancelled = true }
    const timer = window.setInterval(() => void refresh(), 1500)
    return () => {
      cancelled = true
      window.clearInterval(timer)
    }
  }, [routeTaskId, routeTaskStatus?.terminal, sseState.connected])

  useEffect(() => {
    if (navigationSelectionMode !== 'route' || !sseState.connected || !activeMapName) return
    if (routeTaskId) return
    let cancelled = false
    api.fetchInspectionTasks({ mapId: activeMapName, includeTerminal: false, limit: 2 })
      .then(result => {
        if (cancelled) return
        const selection = selectUniqueOpenRouteTask(result.tasks)
        if (selection.kind === 'one') {
          setRouteTaskId(selection.task.task_id)
          setRouteTaskStatus(selection.task)
          setRouteTaskError(null)
        } else if (selection.kind === 'ambiguous') {
          setRouteTaskError('存在多个未结束任务，请到巡检任务页选择要查看的任务')
        }
      })
      .catch(error => {
        if (!cancelled) setRouteTaskError(error instanceof Error ? error.message : String(error))
      })
    return () => { cancelled = true }
  }, [
    activeMapName,
    navigationSelectionMode,
    routeTaskId,
    sseState.connected,
    sseState.inspectionTaskEvent?.event_id,
  ])

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
  // When the panel is closed (or user hasn't edited yet) keep X/Y/Z/Yaw mirroring
  // current odom so opening it shows useful defaults.  Once the user edits any
  // field we stop overwriting (relocDirty=true).
  useEffect(() => {
    if (relocDirty || !odom || !poseAvailable) return
    setRelocX(robotX.toFixed(2))
    setRelocY(robotY.toFixed(2))
    setRelocZ(robotZ.toFixed(2))
    setRelocYaw(yaw.toFixed(3))
  }, [odom, poseAvailable, robotX, robotY, robotZ, yaw, relocDirty])

  // ── Default active map for reloc panel ─────────────────────────
  const workbenchTargetMapName = workbenchMapName.trim() || (showSavedMapInScene ? activeMapName : '') || ''
  useEffect(() => {
    if (!relocMap && activeMapName) setRelocMap(activeMapName)
  }, [activeMapName, relocMap])

  useEffect(() => {
    setPendingGoal(null)
    setGoalHeadingEditing(false)
  }, [activeMapName, showSavedMapInScene, session?.product_session_id])

  useEffect(() => {
    setSavedMapCloud(undefined)
    setSavedMapLoadError(null)
    ++goalPreviewRequest.current
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
    if (!sseState.connected) return
    let cancelled = false
    let retryTimer: ReturnType<typeof setTimeout> | null = null
    const loadSavedMap = () => {
      api.fetchSavedMapPointCloud(activeMapName)
        .then(savedMap => {
          if (!cancelled) {
            setSavedMapCloud(savedMap)
            setSavedMapLoadError(null)
          }
        })
        .catch((error: unknown) => {
          // Activation and reconnect may briefly precede the scene binding.
          if (!cancelled) {
            setSavedMapLoadError(error instanceof Error ? error.message : String(error))
            retryTimer = setTimeout(loadSavedMap, 1500)
          }
        })
    }
    loadSavedMap()
    return () => {
      cancelled = true
      if (retryTimer !== null) clearTimeout(retryTimer)
    }
  }, [
    activeMapName,
    cloud.epoch,
    cloud.frameId,
    savedMapCloud,
    savedMapFlat,
    showSavedMapInScene,
    session?.product_session_id,
    sseState.connected,
  ])

  // ── Handlers ──────────────────────────────────────────────────
  const invalidatePendingGoalPreview = useCallback(() => {
    ++goalPreviewRequest.current
    setPendingGoalPreview(null)
    setGoalPreviewPending(false)
    setGoalPreviewError(null)
  }, [])

  const clearPendingGoal = useCallback(() => {
    invalidatePendingGoalPreview()
    setPendingGoal(null)
    setGoalYaw(undefined)
    setGoalHeadingEditing(false)
  }, [invalidatePendingGoalPreview])

  const selectNavigationMode = useCallback((mode: NavigationSelectionMode) => {
    if (routeDraftLocked) return
    setNavigationSelectionMode(mode)
    if (mode === 'route') clearPendingGoal()
  }, [clearPendingGoal, routeDraftLocked])

  const handleRouteDraftPoint = useCallback((x: number, y: number, selectedZ?: number, mode: GoalPickMode = 'plane') => {
    if (routeDraftLocked) return
    if (!routeDraftIdentity) {
      showToast('等待活动地图版本加载完成后再编辑连续点路线', 'error')
      return
    }
    const z = sceneGoalHeight(undefined, selectedZ, mode, navigationSupportHeight)
    if (z === null) {
      showToast(mode === 'surface' ? '等待当前机器人支撑高度，请稍后重新选点' : '等待有效的目标高度，请确认定位已更新', 'error')
      return
    }
    setRouteDraft(current => appendRouteDraftPoint(current, routeDraftIdentity, {
      x,
      y,
      z,
    }))
    setSavedDraftRoute(null)
    setRouteStartIntent(null)
    setRouteTaskError(null)
    setFollowRobot(false)
  }, [routeDraftIdentity, routeDraftLocked, showToast, navigationSupportHeight])

  const editRouteDraft = useCallback((edit: (draft: RouteDraft) => RouteDraft) => {
    if (routeDraftLocked) return
    setRouteDraft(edit)
    setSavedDraftRoute(null)
    setRouteStartIntent(null)
    setRouteTaskError(null)
  }, [routeDraftLocked])

  const saveRouteRequest = useCallback(async (request: InspectionRouteRequest) => {
    const saved = await api.saveInspectionRoute(request)
    const savedDraft = routeDraftFromSaved(saved.route)
    const signature = routeDraftSignature(savedDraft, goalAcceptanceRadius, normalizedRouteName)
    setRouteDraft(savedDraft)
    setSavedDraftRoute({ signature, id: saved.route.id, revision: saved.route.revision ?? 1 })
    setRouteStartIntent(current => current?.signature === currentRouteSignature ? { ...current, signature } : current)
    onRouteSaved?.(saved.route)
    return saved
  }, [goalAcceptanceRadius, normalizedRouteName, currentRouteSignature, onRouteSaved])

  const saveRouteDraft = useCallback(async () => {
    if (routeDraftLocked || routeDraftSaved || !routeDraft.identity || routeDraft.points.length < 2) return
    const intent = ensureRouteStartIntent(routeStartIntent, currentRouteSignature,
      () => api.makeRequestId('inspection_start'), () => `scene-${Date.now().toString(36)}`)
    setRouteStartIntent(intent)
    setRouteActionPending('save')
    setRouteTaskError(null)
    try {
      await saveRouteRequest(buildInspectionRouteRequest(routeDraft, {
        id: intent.routeId, name: normalizedRouteName, tolerance: goalAcceptanceRadius,
      }))
      showToast('路线已保存，可在巡检任务中继续编辑', 'success')
    } catch (error: unknown) {
      setRouteTaskError(api.formatCommandError(error, '保存路线失败'))
    } finally {
      setRouteActionPending(null)
    }
  }, [routeDraftLocked, routeDraftSaved, routeDraft, routeStartIntent, currentRouteSignature, saveRouteRequest,
    normalizedRouteName, goalAcceptanceRadius, showToast])

  const startRouteDraft = useCallback(async () => {
    if (routeActionPending || routeTaskOpen) return
    if (!routeDraft.identity || routeDraft.points.length < 2) {
      setRouteTaskError('连续点路线至少需要两个点位')
      return
    }
    if (!routeDraftIdentity || !sameRouteDraftIdentity(routeDraft.identity, routeDraftIdentity)) {
      setRouteTaskError('活动地图已变化，请重新选择路线点位')
      return
    }
    const signature = currentRouteSignature
    const intent = ensureRouteStartIntent(
      routeStartIntent,
      signature,
      () => api.makeRequestId('inspection_start'),
      () => `scene-${Date.now().toString(36)}`,
    )
    setRouteStartIntent(intent)
    setRouteActionPending('start')
    setRouteTaskError(null)
    try {
      const saved = savedDraftRoute
      let receipt
      if (!saved || saved.signature !== signature) {
        const request = buildInspectionRouteRequest(routeDraft, {
          id: intent.routeId,
          name: normalizedRouteName,
          tolerance: goalAcceptanceRadius,
        })
        receipt = await saveAndStartRouteDraft(request, intent.requestId, {
          save: saveRouteRequest,
          start: api.startInspectionTask,
        })
      } else {
        receipt = await api.startInspectionTask(saved.id, {
          map_id: routeDraft.identity.mapId,
          revision: saved.revision,
          request_id: intent.requestId,
        })
      }
      if (!receipt.accepted || !receipt.task_id) throw new Error('原生路线执行器未接受任务')
      setRouteStartIntent(null)
      setRouteTaskId(receipt.task_id)
      setRouteTaskStatus(null)
      showToast('路线命令已提交，等待原生任务确认', 'success')
    } catch (error: unknown) {
      setRouteTaskError(api.formatCommandError(error, '开始路线失败'))
    } finally {
      setRouteActionPending(null)
    }
  }, [goalAcceptanceRadius, routeActionPending, routeDraft, routeDraftIdentity, routeStartIntent, routeTaskOpen,
    savedDraftRoute, currentRouteSignature, normalizedRouteName, saveRouteRequest, showToast])

  const runRouteTaskAction = useCallback(async (action: 'pause' | 'resume' | 'cancel') => {
    if (!routeTaskId || routeActionPending || !sseState.connected) return
    setRouteActionPending(action)
    setRouteTaskError(null)
    try {
      const receipt = action === 'pause' ? await api.pauseInspectionTask(routeTaskId)
        : action === 'resume' ? await api.resumeInspectionTask(routeTaskId)
          : await api.cancelInspectionTask(routeTaskId)
      if (!receipt.accepted) throw new Error(`路线${action === 'pause' ? '暂停' : action === 'resume' ? '恢复' : '取消'}请求未被接受`)
      const status = await api.fetchInspectionTask(routeTaskId)
      setRouteTaskStatus(status)
      showToast('路线命令已提交，等待原生任务确认', 'success')
    } catch (error: unknown) {
      setRouteTaskError(api.formatCommandError(error, '路线操作失败'))
    } finally {
      setRouteActionPending(null)
    }
  }, [routeActionPending, routeTaskId, showToast, sseState.connected])

  const handleResumeControl = useCallback(async () => {
    setResumePending(true)
    setResumeError(null)
    try {
      const response = await api.resumeNavigation()
      if (!response.ok) throw new Error(response.status || '恢复请求被拒绝')
      showToast('解除控制暂停请求已受理，等待机器人确认', 'info')
    } catch (error: unknown) {
      const message = api.formatCommandError(error, '解除控制暂停失败')
      setResumeError(message)
      setResumePending(false)
    }
  }, [showToast])

  const selectPendingGoal = useCallback((x: number, y: number, selectedZ?: number, mode: GoalPickMode = 'plane') => {
    invalidatePendingGoalPreview()
    const z = normalizeGoalHeight(sceneGoalHeight(mapOdom?.z, selectedZ, mode, navigationSupportHeight))
    const target = { x, y, z: z ?? undefined }
    setPendingGoal(target)
    setGoalYaw(undefined)
    setGoalHeadingEditing(false)
    setGoalPreviewError(z === null
      ? mode === 'surface' ? '等待当前机器人支撑高度，请稍后重新选点' : '等待有效的三维定位高度' : null)
    setFollowRobot(false)
    return target
  }, [invalidatePendingGoalPreview, mapOdom?.z, navigationSupportHeight])

  const handlePendingGoal = useCallback(async (x: number, y: number, selectedZ?: number) => {
    if (goalHeadingEditing) return
    const request = ++goalPreviewRequest.current
    const z = normalizeGoalHeight(selectedZ)
    setPendingGoalPreview(null)
    setGoalPreviewError(null)
    setGoalPreviewPending(false)
    if (previewDisabledReason) {
      setGoalPreviewError(previewDisabledReason)
      return
    }
    if (z === null) {
      setGoalPreviewError('等待有效的三维定位高度')
      return
    }
    setGoalPreviewPending(true)
    try {
      const candidate = await api.constructGoalCandidate({
        x,
        y,
        z,
        yaw: goalYaw,
        source: 'map_click',
        target_type: 'map_point',
        label: 'scene_click',
        acceptance_radius_m: goalAcceptanceRadius,
        max_speed_mps: goalMaxSpeed ?? undefined,
      })
      if (request !== goalPreviewRequest.current) return
      if (!candidate.ok || candidate.preview?.feasible === false) {
        const reasons = candidate.reasons.length ? candidate.reasons : candidate.preview?.reasons ?? []
        const rawReason = [...new Set(reasons)].join('；') || candidate.error || '目标预检未通过'
        setGoalPreviewError(rawReason === 'empty_path' ? '未找到可行路线' : `无法生成路线：${rawReason}`)
        return
      }
      if (!navigationPlanEndpoint(candidate.preview ?? null, { x, y, z })) {
        setGoalPreviewError('规划失败：preview_result_invalid')
        return
      }
      setPendingGoalPreview(candidate.preview!)
    } catch (e: unknown) {
      if (request !== goalPreviewRequest.current) return
      setGoalPreviewError(`规划失败：${e instanceof Error ? e.message : String(e)}`)
    } finally {
      if (request === goalPreviewRequest.current) setGoalPreviewPending(false)
    }
  }, [previewDisabledReason, goalAcceptanceRadius, goalHeadingEditing, goalMaxSpeed, goalYaw])

  useEffect(() => {
    if (!pendingGoal || previewCurrent || goalSendPending || isMappingWorkspace || isMappingSession || observe
      || goalHeadingEditing || navigationSelectionMode !== 'single') return
    if (typeof pendingGoal.z !== 'number' || !Number.isFinite(pendingGoal.z)) return
    const timer = window.setTimeout(() => {
      void handlePendingGoal(pendingGoal.x, pendingGoal.y, pendingGoal.z)
    }, 220)
    return () => window.clearTimeout(timer)
  }, [
    goalAcceptanceRadius,
    goalMaxSpeed,
    goalHeadingEditing,
    goalYaw,
    goalSendPending,
    handlePendingGoal,
    isMappingSession,
    isMappingWorkspace,
    navigationSelectionMode,
    observe,
    pendingGoal,
    previewCurrent,
  ])

  // Let keyboard users dismiss a selected target without reaching for the card.
  useEffect(() => {
    if (!pendingGoal || isMappingSession || observe) return
    const handleKeyDown = (event: KeyboardEvent) => {
      if (event.key !== 'Escape' || event.defaultPrevented || goalSendPending || document.fullscreenElement) return
      const target = event.target as HTMLElement | null
      if (target?.closest('input, select, textarea, [contenteditable="true"]')) return
      clearPendingGoal()
    }
    window.addEventListener('keydown', handleKeyDown)
    return () => window.removeEventListener('keydown', handleKeyDown)
  }, [clearPendingGoal, goalSendPending, isMappingSession, observe, pendingGoal])

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
      await api.relocalize(mapName, seed.x, seed.y, seed.yaw, seed.z)
      const q = sseState.session?.icp_quality
      const qStr = typeof q === 'number' ? ` quality=${q.toFixed(2)}` : ''
      showToast(`重定位成功${qStr}`, 'success')
    } catch (e: unknown) {
      showToast(`重定位: ${e instanceof Error ? e.message : String(e)}`, 'error')
    }
  }, [savedMapRelocalizeSupported, relocalizeUnavailableMessage, sseState.session, mapOdom, showToast])

  const handleGoalHeadingChange = useCallback((heading: number | undefined) => {
    if ((heading !== undefined && !Number.isFinite(heading)) || goalSendPending) return
    invalidatePendingGoalPreview()
    setGoalYaw(heading)
  }, [goalSendPending, invalidatePendingGoalPreview])

  const handleConfirmGoal = useCallback(async () => {
    if (!pendingGoal || goalSendPending) return
    if (goalHeadingEditing || goalPreviewPending || !previewCurrent || !plannedGoal) return
    if (!canSendGoal) {
      showToast(`不能下发目标: ${goalDisabledReason}`, 'error')
      return
    }
    const { x, y } = pendingGoal
    setGoalSendPending(true)
    try {
      const res = await api.sendGoal(x, y, {
        z: pendingGoal.z,
        yaw: goalYaw,
        source: 'map_click',
        target_type: 'map_point',
        label: 'scene_click',
        acceptance_radius_m: goalAcceptanceRadius,
        max_speed_mps: goalMaxSpeed ?? undefined,
      })
      if (!res.ok) throw new Error(api.formatCommandAck(res, '导航请求'))
      setPendingGoal(null)
      setPendingGoalPreview(null)
      showToast(api.formatCommandAck(res, `目标 (${x.toFixed(2)}, ${y.toFixed(2)})`), 'info')
    } catch (e: unknown) {
      setGoalPreviewError(navigationGoalErrorMessage(e, locale))
    } finally {
      setGoalSendPending(false)
    }
  }, [canSendGoal, goalAcceptanceRadius, goalDisabledReason, goalHeadingEditing, goalMaxSpeed, goalYaw, locale, pendingGoal, plannedGoal, previewCurrent, goalPreviewPending, goalSendPending, showToast])

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
        use_current_pose: true,
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
  }, [normalizedLocationName, poseAvailable, showToast])

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
        max_speed_mps: goalMaxSpeed ?? undefined,
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
        max_speed_mps: goalMaxSpeed ?? undefined,
        metadata: { location_name: loc.name },
      })
      showToast(api.formatCommandAck(res, `Goal ${loc.name}`), 'success')
    } catch (e: unknown) {
      showToast(navigationGoalErrorMessage(e, locale), 'error')
    } finally {
      setLocationBusy(null)
    }
  }, [canSendGoal, goalAcceptanceRadius, goalDisabledReason, goalMaxSpeed, locale, showToast])

  const handleUpdateLocationToCurrent = useCallback(async (loc: LocationEntry) => {
    if (!poseAvailable) {
      showToast('No valid odometry for updating a location', 'error')
      return
    }
    setLocationBusy(`update:${loc.name}`)
    try {
      const res = await api.updateLocation(loc.name, {
        name: loc.name,
        use_current_pose: true,
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
  }, [poseAvailable, showToast])

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
    setAutoProcess(true)
    setTeleopMode(false)
    setSaveModalOpen(true)
  }

  const confirmSaveMap = async (name: string, existing?: api.SaveMapResult & { operation_id: string }) => {
    const initiatingSession = savePreviewSession.current
    setSaveModalOpen(false)
    if (!existing && saveBlockedReason) {
      showToast(saveBlockedReason, 'error')
      return
    }
    openWorkspaceTool('maps')
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
      showToast(`已保存: ${savedName}`, 'success')
      if (r.warnings?.length) showToast(r.warnings.join('；'), 'info')
      if (initiatingSession !== null && savePreviewSession.current === initiatingSession) {
        loadMaps()
      }
    } catch (e: unknown) {
      updateProgress()
      const message = e instanceof Error ? e.message : String(e)
      setSaveStatus(failedMapSaveStatus(name, e))
      showToast(`保存失败：${message}`, 'error')
    }
  }

  const mapSaves = useMapSaves({
    refreshKey: `${session?.product_session_id ?? ''}:${saveStatus?.state ?? ''}`,
    currentOperationId: saveStatus && 'operation' in saveStatus ? saveStatus.operation?.operation_id : undefined,
    onContinue: result => void confirmSaveMap(result.name, result),
    onComplete: result => void confirmSaveMap(result.name, result),
  })
  const mapEntries = mapLibraryEntries(maps, mapSaves.history.operations)

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
        center: { x: pendingGoal.x, y: pendingGoal.y, z: pendingGoal.z ?? 0 },
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

  const handleRelocalize = async () => {
    if (!savedMapRelocalizeSupported) {
      showToast(relocalizeUnavailableMessage, 'error')
      return
    }
    if (!relocMap) { showToast('请先选择地图', 'error'); return }
    setRelocPending(true)
    try {
      const seed = parseInitialPose({ x: relocX, y: relocY, z: relocZ, yaw: relocYaw })
      await api.relocalize(relocMap, seed.x, seed.y, seed.yaw, seed.z)
      showToast(`重定位已完成: ${relocMap}`, 'success')
      setRelocOpen(false)
      // Load saved map cloud only after relocalization (coordinate frames now aligned)
      try {
        const savedMap = await api.fetchSavedMapPointCloud(relocMap)
        setSavedMapCloud(savedMap)
      } catch { /* PCD not available — ignore */ }
    } catch (e: unknown) {
      showToast(`重定位: ${e instanceof Error ? e.message : String(e)}`, 'error')
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
      showToast(`全局重定位: ${e instanceof Error ? e.message : String(e)}`, 'error')
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

  const sceneDockControls = <>
    {!isMappingWorkspace && <>
      <button className={styles.toolbarBtnPrimary} onClick={() => onOpenSavedMap(null)}
        title="选择已保存地图并准备导航"><MapPinned size={15} /><span>选地图</span></button>
      {!observe && <button className={styles.toolbarBtn} disabled={isMappingSession}
        title={isMappingSession ? '当前正在建图，请先保存并使用地图导航' : '在当前地图确认机器人位置'}
        onClick={() => openWorkspaceTool('localization')}><LocateFixed size={15} /><span>定位</span></button>}
    </>}
    {!isMappingWorkspace && !observe && !isMappingSession && <details className={styles.mappingDisplayMenu} name="lingtu-menu" data-hover-menu>
      <summary aria-label="目标选择" title="同层按当前机身高度选点；三维表面按所选地面和机器人支撑高度设置目标">
        <Target size={15} /><span>目标</span>
      </summary>
      <div className={styles.mappingDisplayOptions} onClick={event => {
        if ((event.target as HTMLElement).closest('button')) event.currentTarget.parentElement?.removeAttribute('open')
      }}>
        {!stairEditing && !mapEditing && !(workspaceTool === 'localization' && drawerOpen) && (
          <div className={styles.navigationModeSwitch} role="tablist" aria-label="目标选择模式"
            onPointerDown={event => event.stopPropagation()}>
            <button type="button" role="tab" aria-selected={navigationSelectionMode === 'browse'}
              className={navigationSelectionMode === 'browse' ? styles.navigationModeActive : ''}
              disabled={routeDraftLocked}
              onClick={() => selectNavigationMode('browse')}>
              <Maximize2 size={14} />浏览
            </button>
            <button type="button" role="tab" aria-selected={navigationSelectionMode === 'single'}
              className={navigationSelectionMode === 'single' ? styles.navigationModeActive : ''}
              disabled={routeDraftLocked}
              onClick={() => selectNavigationMode('single')}>
              <Target size={14} />选目标
            </button>
            <button type="button" role="tab" aria-selected={navigationSelectionMode === 'route'}
              className={navigationSelectionMode === 'route' ? styles.navigationModeActive : ''}
              disabled={routeDraftLocked && navigationSelectionMode !== 'route'}
              onClick={() => selectNavigationMode('route')}>
              <Route size={14} />连续点{routeDraft.points.length > 0 ? ` ${routeDraft.points.length}` : ''}
            </button>
          </div>
        )}
        <button type="button" className={goalPickMode === 'plane' ? styles.toolbarBtnSelected : styles.toolbarBtn}
          aria-pressed={goalPickMode === 'plane'} title="在当前机身高度的水平面上选择目标位置"
          onClick={() => setGoalPickMode('plane')}>同层</button>
        <button type="button" className={goalPickMode === 'surface' ? styles.toolbarBtnSelected : styles.toolbarBtn}
          aria-pressed={goalPickMode === 'surface'} title="选择目标地面，按当前机器人支撑高度设置机身目标；也可按住 Alt 点击"
          onClick={() => setGoalPickMode('surface')}>三维表面</button>
      </div>
    </details>}
    {isMappingWorkspace && !observe && !isMappingSession && <button className={styles.toolbarBtnPrimary}
      disabled={!productSwitchAllowed} title={productSwitchReason || '停止当前任务并开始建图'}
      onClick={onStartMapping}><MapPinned size={15} /><span>开始建图</span></button>}
    {isMappingWorkspace && isMappingSession && !observe && <button className={styles.toolbarBtnPrimary} onClick={handleSaveMap}
      disabled={Boolean(saveBlockedReason) || saveStatus?.state === 'saving' || saveStatus?.state === 'pending'}
      title={saveBlockedReason || '保存当前建图结果后查看整图快照'}>
      <Save size={15} /><span>{saveStatus?.state === 'saving' ? '保存中…' : '保存地图'}</span>
    </button>}
    {isMappingWorkspace && <button className={styles.toolbarBtn} onClick={() => onOpenSavedMap(null)}
      aria-label="已存地图" title="查看保存的完整地图"><MapPinned size={15} /><span>已存地图</span></button>}
  </>

  return (
    <div className={`${styles.sceneView} ${isMappingWorkspace ? styles.mappingScene : ''}`}>
      {sceneDockTarget ? createPortal(sceneDockControls, sceneDockTarget) : null}


      {/* Workspace */}
      <div ref={workspaceRef} data-inspector-open={inspectorOpen} className={[
        styles.workspace,
        drawerOpen && !observe ? styles.toolsWorkspace : '',
        !inspectorOpen && !(drawerOpen && !observe) ? styles.inspectorCollapsed : '',
      ].filter(Boolean).join(' ')}>
        {/* Tools open alongside the scene without replacing its live state. */}
        {!observe && drawerOpen && <>
        <div className={styles.drawer}>
          <div className={styles.drawerHeader}>
            <button className={styles.backToInspector} onClick={() => {
              setDrawerOpen(false)
              requestAnimationFrame(() => (isMappingWorkspace && !inspectorOpen
                ? document.querySelector<HTMLElement>('summary[aria-label="显示设置"]')
                : document.getElementById('scene-tab-tools'))?.focus())
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
            {!isMappingWorkspace && <button role="tab" aria-selected={workspaceTool === 'locations'} onClick={() => setWorkspaceTool('locations')}>位置</button>}
            {!isMappingWorkspace && <button role="tab" aria-selected={workspaceTool === 'localization'} onClick={() => openWorkspaceTool('localization')}>定位</button>}
          </div>
          <div className={styles.drawerBody}>
            {workspaceTool === 'maps' && <>
            <div className={styles.mapToolbar}>
              <button className={styles.toolbarBtn} onClick={() => { void loadMaps(); void mapSaves.load() }} disabled={mapListStatus === 'loading'}><RefreshCw size={13} />刷新地图库</button>
            </div>
            {saveBlockedReason && <p role="status">{saveBlockedReason}</p>}
            {saveStatus && !['archived', 'saved'].includes(saveStatus.state) && (
              <div
                className={[
                  styles.saveStatusCard,
                  saveStatus.state === 'saving' ? styles.saveStatusBusy : '',
                  saveStatus.state === 'failed' ? styles.saveStatusError : '',
                ].filter(Boolean).join(' ')}
                title={saveStatus.summary ?? saveStatus.detail}
              >
                <div className={styles.saveStatusHeader}>
                  <span>{saveStatus.state === 'saving' ? '保存进度' : saveStatus.state === 'saved' ? '保存结果' : saveStatus.state === 'archived' ? '存档已保存' : saveStatus.state === 'pending' ? '等待确认' : '保存失败'}</span>
                  <span className={styles.saveStatusName}>{saveStatus.name}</span>
                </div>
                <div className={styles.saveStatusDetail}>{saveStatus.detail}</div>
                {saveStatus.state === 'pending' && <button className={styles.toolbarBtn}
                  onClick={() => void confirmSaveMap(saveStatus.name, saveStatus.operation)}><RefreshCw size={13} />继续查询</button>}
                {saveStatus.state === 'saved' && <button className={styles.toolbarBtn}
                  onClick={() => onOpenSavedMap(saveStatus.name)}><MapPinned size={13} /> 查看完整地图</button>}
                <div className={styles.saveStatusDetail}>
                  {saveActive ? '已等待' : '耗时'} {formatMapSaveElapsed(saveTiming.elapsedMs + (saveActive ? Math.max(0, saveClock - saveTiming.observedAt) : 0))}
                </div>
                {saveActive && <>
                  <progress style={{ width: '100%' }} aria-label="地图保存阶段进度" max={1} value={saveProgress} />
                  <div className={styles.saveStatusDetail}>{saveStatus.state === 'pending' ? '状态待确认；显示最近进度' : '阶段进度，非剩余时间估计'}</div>
                </>}
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
              <summary>地图产物工具</summary>
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
                    <option value="occupied">添加占据</option>
                    <option value="free">设为空闲</option>
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
                    title={pendingGoal ? '修改 OctoMap 占据；设为空闲不代表地面可通行' : '请先点击地图点'}
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
            {mapListStatus === 'ready' && !mapSaves.loading && !mapSaves.error && mapEntries.length === 0 && (
              <div className={styles.emptyState}>
                <MapPinned size={32} className={styles.emptyIcon} strokeWidth={1.4} />
                <div className={styles.emptyTitle}>暂无地图</div>
                <div className={styles.emptyHint}>保存当前场景来创建第一张地图</div>

              </div>
            )}
            <MapLibrary entries={mapEntries} saves={mapSaves}
              selectedMap={showSavedMapInScene ? activeMapName ?? null : null} readOnly={observe}
              onPreview={onOpenSavedMap} onRename={setRenameTarget} onDelete={setDeleteTarget} />
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
                  <p className={styles.statusLine}>填写机身在地图中的位置（X/Y/Z，米）与航向（弧度）。Z 是地图坐标中的机身高度。Shift+点击保留当前地图高度与朝向。</p>
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
                  <label>Z</label>
                  <input className={styles.relocInput} type="number" step="0.1" aria-label="初始位姿 Z（米）"
                    value={relocZ}
                    onChange={e => { setRelocDirty(true); setRelocZ(e.target.value) }} />
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
        </div>
        </>}

        {/* Center: 3D scene */}
        <div className={`${styles.canvasArea} ${isMappingWorkspace ? styles.mappingCanvasArea : ''}`}>
          <div className={styles.canvasWrap}
            onPointerDownCapture={event => {
              // Dismiss other menus without swallowing controls inside the open menu.
              const openMenus = [...document.querySelectorAll<HTMLDetailsElement>('details[name="lingtu-menu"][open]')]
                .filter(menu => !menu.contains(event.target as Node))
              dismissOnlyCanvasClick.current = openMenus.length > 0
              openMenus.forEach(menu => { menu.open = false })
              if (openMenus.length > 0) {
                event.preventDefault()
                event.stopPropagation()
              }
            }}
            onMouseUpCapture={event => {
              if (!dismissOnlyCanvasClick.current) return
              dismissOnlyCanvasClick.current = false
              event.preventDefault()
              event.stopPropagation()
            }}>
            <div className={styles.canvasControls}>
            {!observe && supportsWebTeleop(currentProduct) && teleopMode && !drawerOpen && (
              <div className={styles.canvasTeleopDock}>
                <TeleopPanel key={currentProduct} sseState={sseState} showToast={showToast}
                  onExit={() => setTeleopMode(false)} />
              </div>
            )}
            {!isMappingWorkspace && isMappingSession && <div className={styles.sceneAlert} role="status">
              <MapPinned size={16} /><span>当前正在建图，请先保存地图，再选择地图开始导航。</span>
            </div>}
            {isMappingWorkspace && !isMappingSession && <div className={styles.sceneAlert} role="status">
              <MapPinned size={16} /><span>点击“开始建图”创建建图会话，完成后保存地图。</span>
            </div>}
            {mappingNotice && <div className={styles.sceneAlert} role="status">
              <Info size={16} aria-hidden="true" />
              <span className={styles.capacityNotice}>{mappingNotice}</span>
            </div>}
            {(layers.elevation || layers.nativeTraversability || layers.localPlanner) && (
              <div className={styles.sceneLegendStack}>
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
            </div>
            {/* Keep one camera preview inside the map, including browser full screen. */}
            {cameraVisible && <div
              className={`${styles.cameraPip} ${cameraExpanded && cameraHasFrame ? styles.cameraPipExpanded : ''} ${pendingGoal ? styles.cameraPipWithGoal : ''} ${!cameraHasFrame ? styles.cameraDockEmpty : ''}`}
              aria-label="相机实时画面"
              aria-hidden={!cameraHasFrame}
              onPointerDown={event => event.stopPropagation()}
            >
              {cameraHasFrame && <div className={styles.cameraPipHeader}>
                <button type="button" onClick={() => {
                  if (!cameraExpanded) setInspectorOpen(false)
                  setCameraExpanded(value => !value)
                }}
                  aria-label={cameraExpanded ? '还原相机' : '放大相机'} title={cameraExpanded ? '还原相机' : '放大相机'} aria-pressed={cameraExpanded}>
                  {cameraExpanded ? <Minimize2 size={17} /> : <Maximize2 size={17} />}
                </button>
              </div>}
              {whepStream || (jpegCameraRecovered && cameraImgSrc)
                ? <div ref={cameraMediaRef} className={styles.cameraPipMedia}>
                  {whepStream && <video
                    ref={cameraVideoRef}
                    className={`${styles.cameraPipImg} ${!useWebRTCCamera ? styles.cameraPipVideoPending : ''}`}
                    style={cameraVideoLayout.style}
                    autoPlay muted playsInline
                  />}
                  {!useWebRTCCamera && jpegCameraRecovered && cameraImgSrc
                    ? <img src={cameraImgSrc} className={styles.cameraPipImg} alt="机器人相机实时画面" draggable={false} />
                    : null}
                </div>
                : null
              }
              <CameraTargetOverlay target={sceneTarget} />
            </div>}
            <div className={styles.mapTools} onPointerDown={event => event.stopPropagation()}>
              {!observe && supportsWebTeleop(currentProduct) && (
                <button type="button" className={`${styles.cameraToggle} ${styles.teleopToggle}`}
                  aria-label={teleopMode ? '退出遥控' : currentProduct === 'nav' ? '人工接管' : '遥控'}
                  title={teleopMode ? '退出遥控' : currentProduct === 'nav' ? '人工接管' : '遥控'}
                  aria-pressed={teleopMode} onClick={() => {
                    setDrawerOpen(false)
                    setTeleopMode(value => !value)
                  }}>
                  <Gamepad2 size={19} aria-hidden="true" /><span>{teleopMode ? '退出遥控' : currentProduct === 'nav' ? '人工接管' : '遥控'}</span>
                </button>
              )}
              <div className={styles.sceneToolGroup} role="group" aria-label="场景显示工具">
              <button ref={cameraToggleRef} type="button" id="scene-camera-toggle" className={`${styles.cameraToggle} ${styles.cameraEntry}`}
                aria-label={cameraVisible ? '收起相机' : '显示相机'} aria-pressed={cameraVisible}
                title={cameraVisible ? cameraPipLabel : '显示相机'}
                onClick={() => { setCameraExpanded(false); setCameraPreference(!cameraVisible) }}>
                <Camera size={18} aria-hidden="true" /><span>相机</span>
                {cameraVisible && <small className={styles.cameraState} role="status" aria-label={cameraPipLabel}
                  title={cameraStatus?.available
                    ? `${cameraPipLabel} · 彩色 ${cameraStatus.color.fps.toFixed(0)} 帧/秒 · 深度 ${cameraStatus.depth.fps.toFixed(0)} 帧/秒`
                    : cameraPipLabel}>{cameraHasFrame ? '实时' : cameraStatus?.status === 'not_loaded' ? '未启用' : cameraLastFrameAt ? '中断' : '等待'}</small>}
              </button>
              <button ref={layerTrigger} type="button" className={styles.cameraToggle} aria-label="图层" title="图层"
                onPointerEnter={cancelLayerDismiss} onPointerLeave={leaveLayers}
                aria-expanded={inspectorOpen && !drawerOpen && inspectorTab === 'layers'} aria-controls="scene-inspector"
                onClick={() => {
                  setInspectorOpen(!(inspectorOpen && !drawerOpen && inspectorTab === 'layers'))
                  setInspectorTab('layers')
                  setDrawerOpen(false)
                }}>
                <Layers2 size={18} aria-hidden="true" /><span>图层</span>
              </button>
              </div>

            </div>
            {!mapEditing && !stairEditing && <div className={styles.centerTelemetry} aria-label="运行数据">
              <span title="SLAM 实测前进速度"><span>速度</span><strong>{displaySpeedValue}</strong><small>m/s</small></span>
              <span title="机器人尚未上报 Wi-Fi RSSI"><span>信号</span><strong>—</strong></span>
              <span title="本次定位进程累计里程，不含全局位置修正"><span>里程</span><strong>{displaySessionDistance}</strong><small>m</small></span>
              <span><span>朝向</span><strong>{displayYawDeg}</strong></span>
            </div>}
            <Scene3D
              ref={scene3DRef}
              cloud={cloud}
              scanCloud={alignedScanCloud}
              scanVisible={liveScanVisible && !mapEditing}
              savedMapFlat={savedMapForScene?.points}
              savedMapFrameId={savedMapForScene?.frameId}
              savedMapEpoch={savedMapForScene?.epoch}
              savedMapVisible={layers.cloud && !showNavigationMap}
              navigationMap={showNavigationMap && layers.cloud && navigationMap.status === 'ready' ? navigationMap.map : undefined}
              voxelScale={voxelScale}
              cloudColorRange={colorHeightRange}
              mapEdit={mapEditing ? shownMapEdit ?? { selecting: false, selection: null, view: 'draft', operations: [], showDeleted: false } : null}
              onMapEditBounds={bounds => mapEditorRef.current?.setBounds(bounds)}
              onMapEditAnchor={setMapEditAnchor}
              stairAnnotations={shownStairs?.annotations}
              stairDraft={shownStairs?.draft}
              stairTool={inspectorTab === 'tools' ? shownStairs?.tool : null}
              onStairBounds={bounds => stairEditorRef.current?.setBounds(bounds)}
              onStairPoint={point => stairEditorRef.current?.setPoint(point)}
              cloudMaxZ={cloudMaxZ}
              cloudMinZ={cloudMinZ}
              cloudHeightStyle={cloudHeightMode === 'follow' ? 'fade' : 'clip'}
              mappingObservation={mappingObservation}
              mappingObservationVisible={showMappingObservation}
              elevationState={elevationState}
              nativeTraversabilityState={nativeTraversabilityState}
              sceneGraph={mapEditing ? null : mapSceneGraph}
              robotX={robotX}
              robotY={robotY}
              robotZ={robotZ}
              orientation={mapOdom?.orientation}
              robotModel={robotModel}
              jointTelemetry={sseState.jointTelemetry}
              poseStampS={odom?.ts ?? null}
              poseEpoch={poseEpoch}
              followRobot={followRobot && !stairEditing && !mapEditing}
              robotValid={poseAvailable}
              yaw={yaw}
              trail={trail}
              path={hasGoal ? path : []}
              localPath={hasGoal || requestedMotion ? localPathPts : []}
                localPlannerSnapshot={nativeFresh ? localPlannerSnapshot : null}
                safetyEnvelope={safetyEnvelope}
                safetyView={safetyView}
              layers={{ ...layers, ...(mapEditing ? { trail: false, path: false, goal: false, robot: false, elevation: false, nativeTraversability: false, localPlanner: false } : {}), cloud: layers.cloud && !savedMapForScene
                && !showMappingObservation }}
              pointSize={pointSize}
              cloudColorMode={cloudColorMode}
              onPendingGoal={isMappingWorkspace
                ? canProbeMapping ? (x, y) => setMappingProbe({ x, y, epoch: poseEpoch }) : undefined
                : isMappingSession || observe || stairEditing || mapEditing || goalSendPending || routeDraftLocked || navigationSelectionMode === 'browse' ? undefined
                  : navigationSelectionMode === 'route' ? handleRouteDraftPoint : selectPendingGoal}
              goalPickMode={goalPickMode}
              onRelocalize={!isMappingWorkspace && !isMappingSession && !observe && !stairEditing && !mapEditing && workspaceTool === 'localization' && drawerOpen ? handleSceneRelocalize : undefined}
              pendingGoal={stairEditing || mapEditing || (!isMappingWorkspace && isMappingSession) ? null : isMappingSession ? currentMappingProbe && { ...currentMappingProbe,
                z: mappingObservation.status === 'ready' ? mappingObservation.layer.origin[2] : 0 }
                : !isMappingWorkspace && navigationSelectionMode === 'single' ? pendingGoal : null}
              pendingGoalYaw={isMappingSession ? undefined : goalYaw}
              pendingGoalLabel={isMappingSession ? '观测检查' : '点击目标'}
              plannedGoal={!isMappingWorkspace && !isMappingSession && !mapEditing && !stairEditing && !goalHeadingEditing && navigationSelectionMode === 'single' ? plannedGoal : null}
              goalHeadingEditing={!observe && !isMappingWorkspace && !isMappingSession && !mapEditing && !stairEditing && navigationSelectionMode === 'single' && goalHeadingEditing}
              onGoalHeadingChange={goalHeadingEditing && !observe && !isMappingWorkspace && !isMappingSession && !mapEditing && !stairEditing && !goalSendPending && !routeDraftLocked && navigationSelectionMode === 'single' ? handleGoalHeadingChange : undefined}
              onGoalHeadingComplete={() => setGoalHeadingEditing(false)}
              previewPath={!stairEditing && !mapEditing && !isMappingWorkspace && !isMappingSession && previewCurrent ? pendingGoalPreview?.path : undefined}
              routeDraftPoints={!stairEditing && !mapEditing && !isMappingWorkspace && !isMappingSession && navigationSelectionMode === 'route' && showRouteDraft
                ? routeDraft.points : undefined}
            />
            {mapEditing && savedMapForScene && savedMapForScene.contentEpoch != null && <MapEditor
              key={mapSourceKey} ref={mapEditorRef}
              source={{ name: savedMapForScene.mapName, contentEpoch: savedMapForScene.contentEpoch, frameId: savedMapForScene.frameId || 'map' }}
              points={savedMapForScene.points} anchor={mapEditAnchor} footerTarget={mapEditFooter}
              cameraExpanded={cameraExpanded} onExitGuardChange={onMapEditorExitGuardChange}
              onSceneChange={onMapEditSceneChange} onView={onMapEditView} onClose={closeMapEditor}
              onOpenMap={onOpenSavedMap} onMapsChanged={() => void loadMaps()} />}
            {!isMappingWorkspace && !isMappingSession && !observe && workspaceTool === 'localization' && drawerOpen && <div className={styles.canvasOverlayTop}>
              <span className={styles.scaleLabel}>Shift + 点击地图设置初始位置</span>
            </div>}
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
                : mappingProbeCell?.value === 0 ? '此处观测到与附近低表面相连的支撑候选；仍需三维路径检查。'
                : '这里不在当前滚动窗口内，不能判断是否已经建图。'}</p>
              <small>{mappingProbeGround
                ? `局部表面拟合 · 高度 ${mappingProbeGround.heightM.toFixed(2)} m · 残差 ${(mappingProbeGround.roughnessM * 100).toFixed(1)} cm · ${Math.round(mappingProbeGround.supportCount)} 个细 XY 支撑`
                : '局部表面拟合不可用；这不是置信度或通行判定。'}</small>
              <small>仅检查地图，不发送运动目标</small>
            </div>}
            <div ref={navigationPanelsRef} data-chat-container="scene" className={styles.navigationPanels} data-selection-mode={navigationSelectionMode} data-map-editing={mapEditing}>
            {mapEditing && <div ref={setMapEditFooter} />}
            {!isMappingWorkspace && !isMappingSession && !observe && !stairEditing && !mapEditing && navigationSelectionMode === 'route' && (
              <div className={`${styles.goalConfirmPanel} ${styles.routeDraftPanel}`} aria-label="连续点路线"
                onPointerDown={event => event.stopPropagation()}>
                <div className={styles.goalConfirmHeader}>
                  <span className={styles.goalConfirmLabel}>连续点路线 · {routePointCountLabel}</span>

                </div>
                {!routeTaskOpen && <input className={styles.routeNameInput} aria-label="路线名称" value={routeName}
                  maxLength={80} disabled={routeDraftLocked} placeholder="路线名称"
                  onChange={event => { setRouteName(event.target.value); setRouteStartIntent(null) }} />}
                {showRouteDraft && routeDraft.points.length > 0 ? (
                  <ol className={styles.routeDraftList}>
                    {routeDraft.points.map((point, index) => (
                      <li key={`${index}:${point.x}:${point.y}:${point.z}`}>
                        <button type="button" className={styles.routePointNumber} aria-label={`查看第 ${index + 1} 个点`}
                          onClick={() => { setFollowRobot(false); scene3DRef.current?.focusPoint(point.x, point.y, point.z) }}>{index + 1}</button>
                        <span className={styles.routePointCoords}>
                          {routeTaskOpen && point.id && routeTaskStatus?.progress.current_point_id === point.id ? '当前 · ' : ''}
                          {point.enabled === false ? '停用 · ' : ''}高度 {point.z.toFixed(2)} m
                        </span>
                        <span className={styles.routePointActions}>
                          <button type="button" disabled={routeDraftLocked || index === 0}
                            title={`上移第 ${index + 1} 个点`} aria-label={`上移第 ${index + 1} 个点`}
                            onClick={() => editRouteDraft(draft => moveRouteDraftPoint(draft, index, -1))}>
                            <ChevronUp size={15} />
                          </button>
                          <button type="button" disabled={routeDraftLocked || index === routeDraft.points.length - 1}
                            title={`下移第 ${index + 1} 个点`} aria-label={`下移第 ${index + 1} 个点`}
                            onClick={() => editRouteDraft(draft => moveRouteDraftPoint(draft, index, 1))}>
                            <ChevronDown size={15} />
                          </button>
                          <button type="button" disabled={routeDraftLocked}
                            title={`删除第 ${index + 1} 个点`} aria-label={`删除第 ${index + 1} 个点`}
                            onClick={() => editRouteDraft(draft => removeRouteDraftPoint(draft, index))}>
                            <X size={15} />
                          </button>
                        </span>
                      </li>
                    ))}
                  </ol>
                ) : routeTaskOpen ? (
                  <div className={styles.routeDraftEmpty}>可在“巡检任务”页查看这项任务的完整路线</div>
                ) : (
                  <div className={styles.routeDraftEmpty}>点击地图添加第一个三维点位</div>
                )}
                {(routeTaskId || routeTaskError) && (
                  <div className={styles.routeTaskStatus} role="status">
                    <strong>{routeTaskError || routeTaskStateLabel(routeTaskStatus)}</strong>
                    {routeTaskStatus?.found && routeTaskStatus.progress.known && (
                      <span>
                        已完成 {routeTaskStatus.progress.completed_points} / {routeTaskStatus.progress.point_count} 点
                        {(routeTaskStatus.progress.current_point_number ?? 0) > 0 ? ` · 当前第 ${routeTaskStatus.progress.current_point_number} 点` : ''}
                        {routeTaskStatus.reason ? ` · ${operatorReasonMessage(routeTaskStatus.reason, locale)}` : ''}
                      </span>
                    )}
                    {!sseState.connected && routeTaskId && <span>连接中断，任务编号已保留；恢复后继续读取同一任务</span>}
                    {routeActionPending && routeActionPending !== 'start' && routeActionPending !== 'save' && (
                      <span>{routeActionPending === 'pause' ? '正在提交暂停请求…'
                        : routeActionPending === 'resume' ? '正在提交恢复请求…'
                          : '正在提交取消请求…'}</span>
                    )}
                  </div>
                )}
                <div className={styles.routeDraftActions}>
                  {!routeTaskOpen && (
                    <button type="button" className={styles.goalCancelBtn}
                      disabled={routeDraftLocked || routeDraft.points.length === 0}
                      onClick={() => editRouteDraft(() => EMPTY_ROUTE_DRAFT)}>
                      清空
                    </button>
                  )}
                  {!routeTaskOpen && (
                    <button type="button" className={styles.goalCancelBtn}
                      disabled={routeDraftLocked || routeDraftSaved || routeDraft.points.length < 2 || !routeDraftIdentity || !sseState.connected}
                      onClick={() => void saveRouteDraft()}>
                      {routeActionPending === 'save' ? '保存中…' : routeDraftSaved ? '已保存' : '保存路线'}
                    </button>
                  )}
                  {!routeTaskOpen && (
                    <button type="button" className={styles.goalConfirmBtn}
                      disabled={routeActionPending !== null || routeDraft.points.length < 2
                        || !routeDraftIdentity || !sseState.connected || !canStartRoute}
                      title={!routeDraftIdentity ? '等待活动地图版本' : !canStartRoute ? routeDisabledReason : '保存路线并提交一次原生巡检任务'}
                      onClick={() => void startRouteDraft()}>
                      <Play size={16} />{routeActionPending === 'start' ? '提交中…' : '开始路线'}
                    </button>
                  )}
                  {routeTaskOpen && routeTaskStatus?.can_pause && (
                    <button type="button" className={styles.goalConfirmBtn}
                      disabled={routeActionPending !== null || !sseState.connected}
                      onClick={() => void runRouteTaskAction('pause')}>
                      <Pause size={16} />{routeActionPending === 'pause' ? '暂停请求中…' : '暂停'}
                    </button>
                  )}
                  {routeTaskOpen && routeTaskStatus?.can_resume && (
                    <button type="button" className={styles.goalConfirmBtn}
                      disabled={routeActionPending !== null || !sseState.connected}
                      onClick={() => void runRouteTaskAction('resume')}>
                      <Play size={16} />{routeActionPending === 'resume' ? '恢复请求中…' : '恢复'}
                    </button>
                  )}
                  {routeTaskOpen && routeTaskStatus?.can_cancel && (
                    <button type="button" className={styles.goalCancelBtn}
                      disabled={routeActionPending !== null || !sseState.connected}
                      onClick={() => void runRouteTaskAction('cancel')}>
                      {routeActionPending === 'cancel' ? '取消请求中…' : '取消路线'}
                    </button>
                  )}
                </div>
                {!routeTaskOpen && <div className={styles.routeActionHint} role="status">{routeStartHint}</div>}
              </div>
            )}
            {!isMappingWorkspace && !isMappingSession && !observe && !stairEditing && !mapEditing && navigationSelectionMode === 'single' && pendingGoal && (
              <section className={`${styles.goalConfirmPanel} ${styles.singleGoalPanel}`} aria-label="导航目标" onPointerDown={event => event.stopPropagation()}>
                <div className={styles.goalConfirmHeader}>
                  <button type="button" className={`${styles.goalConfirmLabel} ${styles.goalFocusButton}`}
                    aria-label="查看所选目标" title="将视角移到所选目标"
                    disabled={typeof pendingGoal.z !== 'number'}
                    onClick={() => {
                      if (typeof pendingGoal.z !== 'number') return
                      setFollowRobot(false)
                      scene3DRef.current?.focusPoint(pendingGoal.x, pendingGoal.y, pendingGoal.z)
                    }}><Target size={17} aria-hidden="true" />新目标</button>
                  <button type="button" className={styles.goalDismissBtn} aria-label="清除目标"
                    disabled={goalSendPending} onClick={clearPendingGoal}><X size={19} aria-hidden="true" /></button>
                </div>
                {selectedGoalDistance !== null && <div className={styles.goalMetrics}>
                  <div><span>{previewDistance !== null ? '路程' : '直线距离'}</span>
                    <strong>{selectedGoalDistance !== null ? selectedGoalDistance.toFixed(1) : '—'}<small>m</small></strong></div>
                </div>}
                {plannedGoal && plannedGoal.offset >= .001 && <p className={styles.goalOffsetNotice} role="status">规划终点偏移 {(plannedGoal.offset * 100).toFixed(1)} cm</p>}
                <div className={styles.goalFooter}>
                  <span className={styles.goalPlanSummary} role="status" aria-live="polite" aria-atomic="true"
                    data-state={goalPreviewError ? 'error' : previewCurrent && goalDisabledReason ? 'blocked' : previewCurrent ? 'ready' : 'pending'}>
                    {goalHeadingEditing ? '拖动设定朝向，或选择不限朝向' : goalSendPending ? '正在提交导航…' : goalPreviewPending ? '正在规划路线…'
                      : goalPreviewMessage || (previewCurrent ? goalDisabledReason || '路线已就绪' : '')
                        || (pendingGoalPreview?.feasible && !previewCurrent ? '起点已变化，正在更新路线…' : '')
                        || '准备规划路线…'}
                  </span>
                  <details className={styles.goalSettings} name="lingtu-menu"
                    onKeyDown={event => {
                      if (event.key === 'Escape') {
                        event.currentTarget.open = false
                        event.currentTarget.querySelector('summary')?.focus()
                        event.stopPropagation()
                      }
                    }}>
                    <summary>
                      <span>详情</span>
                      <ChevronDown className={styles.goalSettingsChevron} size={15} aria-hidden="true" />
                    </summary>
                    <div className={styles.goalSettingsBody}>
                  <dl className={styles.goalEndpoints} aria-label="目标位置对照">
                  <div><dt>点击目标</dt><dd>{pendingGoal.x.toFixed(2)}, {pendingGoal.y.toFixed(2)}, {pendingGoal.z?.toFixed(2) ?? '—'} m</dd></div>
                  <div data-kind="planned"><dt>规划终点</dt><dd>{plannedGoal ? `${plannedGoal.x.toFixed(2)}, ${plannedGoal.y.toFixed(2)}, ${plannedGoal.z.toFixed(2)} m` : '等待有效预览'}</dd></div>
                  <div><dt>端点偏移</dt><dd>{plannedGoal ? plannedGoal.offset < .001 ? '重合' : `${(plannedGoal.offset * 100).toFixed(1)} cm` : '—'}<span> · 到达范围 {goalAcceptanceRadius.toFixed(2)} m</span></dd></div>
                  </dl>
                      {goalPreviewError && <p className={styles.goalPreviewReason}>{goalPreviewError}</p>}
                      <div className={styles.goalHeadingRow}>
                        <label>终点朝向（可选）<input key={goalYaw} type="number" step="1" aria-label="到达朝向（度）"
                          defaultValue={goalYaw === undefined ? '' : (goalYaw * 180 / Math.PI).toFixed(1)} placeholder="不限" disabled={goalSendPending}
                          onKeyDown={event => { if (event.key === 'Enter') event.currentTarget.blur() }}
                          onBlur={event => {
                            const degrees = event.currentTarget.valueAsNumber
                            if (!event.currentTarget.value.trim()) { handleGoalHeadingChange(undefined); setGoalHeadingEditing(false); return }
                            if (!Number.isFinite(degrees)) { event.currentTarget.value = goalYaw === undefined ? '' : (goalYaw * 180 / Math.PI).toFixed(1); return }
                            handleGoalHeadingChange(Math.atan2(Math.sin(degrees * Math.PI / 180), Math.cos(degrees * Math.PI / 180)))
                            setGoalHeadingEditing(false)
                          }} /><span>°</span></label>
                        <button type="button" aria-pressed={goalHeadingEditing} disabled={goalSendPending || typeof pendingGoal.z !== 'number'}
                          onClick={() => { invalidatePendingGoalPreview(); setGoalHeadingEditing(true) }}>
                          <Navigation size={15} />图上设置</button>
                        <button type="button" disabled={goalSendPending}
                          onClick={() => { handleGoalHeadingChange(undefined); setGoalHeadingEditing(false) }}>不限朝向</button>
                      </div>
                      <div className={styles.goalControlGroup}>
                        {(['x', 'y'] as const).map(axis => (
                          <label className={styles.goalControl} key={axis}>
                            <span>位置 {axis.toUpperCase()}</span>
                            <input key={`${axis}:${pendingGoal[axis]}`} type="number" step="0.1" className={styles.goalSelect}
                              defaultValue={pendingGoal[axis].toFixed(2)} disabled={goalSendPending}
                              aria-label={`目标 ${axis.toUpperCase()}（米）`}
                              onKeyDown={event => { if (event.key === 'Enter') event.currentTarget.blur() }}
                              onBlur={event => {
                                const value = event.currentTarget.valueAsNumber
                                if (!Number.isFinite(value)) { event.currentTarget.value = pendingGoal[axis].toFixed(2); return }
                                if (value === pendingGoal[axis]) return
                                invalidatePendingGoalPreview()
                                setPendingGoal({ ...pendingGoal, [axis]: value })
                              }} />
                          </label>
                        ))}
                        <label className={styles.goalControl}>
                          <span>速度上限</span>
                          <select className={styles.goalSelect} value={goalMaxSpeed ?? ''} disabled={goalSendPending}
                            onChange={(event) => { invalidatePendingGoalPreview(); setGoalMaxSpeed(event.target.value === '' ? null : Number(event.target.value)) }}>
                            <option value="">跟随导航配置</option>
                            {GOAL_SPEED_OPTIONS.map((speed) => <option key={speed} value={speed}>{speed} m/s</option>)}
                          </select>
                        </label>
                        <label className={styles.goalControl}>
                          <span>到达范围</span>
                          <select className={styles.goalSelect} value={goalAcceptanceRadius} disabled={goalSendPending}
                            onChange={(event) => { invalidatePendingGoalPreview(); setGoalAcceptanceRadius(Number(event.target.value)) }}>
                            {GOAL_RADIUS_OPTIONS.map((radius) => <option key={radius} value={radius}>{radius.toFixed(2)} m</option>)}
                          </select>
                        </label>
                        <label className={styles.goalControl}>
                          <span>机身高度</span>
                          <input className={styles.goalSelect} type="number" step="0.05"
                            value={typeof pendingGoal.z === 'number' ? pendingGoal.z.toFixed(2) : ''}
                            disabled={goalSendPending} aria-label="目标高度（米）" placeholder="目标高度"
                            onChange={(event) => {
                              if (!event.target.value.trim()) {
                                invalidatePendingGoalPreview()
                                setPendingGoal({ ...pendingGoal, z: undefined })
                                setGoalPreviewError('请输入目标高度')
                                return
                              }
                              const z = Number(event.target.value)
                              if (!Number.isFinite(z)) return
                              invalidatePendingGoalPreview()
                              setPendingGoal({ ...pendingGoal, z })
                            }} />
                        </label>

                      </div>
                      <div className={styles.goalSettingsFooter}>
                        <button type="button" disabled={goalSendPending} onClick={() => {
                          invalidatePendingGoalPreview(); setGoalMaxSpeed(null); setGoalAcceptanceRadius(0.45); setGoalYaw(undefined); setGoalHeadingEditing(false)
                        }}>恢复默认</button>
                      </div>

                    </div>
                  </details>
                {goalPreviewError && !goalPreviewPending && !goalSendPending ? (
                  <button className={styles.goalRetryBtn} type="button"
                    disabled={!!previewDisabledReason || typeof pendingGoal.z !== 'number' || goalHeadingEditing}
                    title={previewDisabledReason || (typeof pendingGoal.z !== 'number' ? '请输入目标高度' : '重新预览路线')}
                    onClick={() => void handlePendingGoal(pendingGoal.x, pendingGoal.y, pendingGoal.z)}>
                    <RefreshCw size={15} aria-hidden="true" />重试
                  </button>
                ) : (
                <button
                  className={styles.goalConfirmBtn}
                  onClick={handleConfirmGoal}
                  disabled={goalSendPending || goalHeadingEditing || goalPreviewPending || !previewCurrent || !plannedGoal || !canSendGoal}
                  title={goalHeadingEditing ? '请完成朝向设置，或选择不限朝向' : goalDisabledReason || (!previewCurrent ? '正在准备路线' : hasGoal ? '提交新目标，替换当前导航任务' : '开始导航')}
                >
                  <Navigation size={16} aria-hidden="true" />{goalHeadingEditing ? '设置朝向中' : goalSendPending ? '提交中…'
                    : goalPreviewPending || !previewCurrent ? '规划中…' : hasGoal ? '切换到此目标' : '开始导航'}
                </button>
                )}
                </div>
                {isExplorationSession && (
                  <button
                    className={styles.goalSecondaryAction}
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
              </section>
            )}

            <div ref={onTaskPanelMount} data-chat-slot="scene" className={styles.taskPanelSlot}
              onPointerDown={event => event.stopPropagation()} />
            </div>
          </div>
        </div>

        {/* Right: side panel */}
        <aside ref={inspectorPanel} className={styles.sidePanel} id="scene-inspector" aria-label="现场面板"
          onPointerEnter={cancelLayerDismiss} onPointerLeave={leaveLayers}
          onKeyDown={event => {
            if (event.key === 'Escape' && inspectorTab === 'layers') {
              event.stopPropagation(); cancelLayerDismiss(); setInspectorOpen(false); layerTrigger.current?.focus()
            }
          }}
          hidden={(drawerOpen && !observe) || !inspectorOpen}>
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
          <div className={styles.inspectorBody} role="tabpanel" id="scene-panel-status"
            aria-labelledby="scene-tab-status" hidden={inspectorTab !== 'status'} tabIndex={0}>
          <details className={styles.statusDetails} aria-label="机器人诊断">
            <summary>控制与定位详情</summary>
            {robotStatusMessage && <div className={styles.robotNotice} role="status">
              <p className={styles.statusLine}>{robotStatusMessage}</p>
              {resumePending && <p className={styles.sourceNote}>等待机器人确认解除控制暂停</p>}
              {navigationView.control.resumeRequired && resumeError && <p className={styles.issueLine}>{resumeError}</p>}
              {!observe && navigationView.control.resumeRequired && <button className={styles.toolbarBtn}
                disabled={resumePending || !sseState.connected} onClick={handleResumeControl}>
                {resumePending ? '等待确认…' : '解除控制暂停'}
              </button>}

            </div>}
            {!observe && navigationView.motion.permission.state === 'ESTOPPED' && onResetEstop && <button
              className={styles.toolbarBtn} disabled={Boolean(resetEstopReason)} title={resetEstopReason || '解除软件急停'}
              onClick={onResetEstop}>解除急停</button>}
              {robotModel === 'go2' && <RobotJointStatus stream={sseState.jointTelemetry} />}
            <div className={styles.poseGrid}>
              <div className={styles.statItem}><span className={styles.statLabel}>X · m</span><span className={`${styles.statValue} ${styles.robotPositionValue}`}>{displayRobotX}</span></div>
              <div className={styles.statItem}><span className={styles.statLabel}>Y · m</span><span className={`${styles.statValue} ${styles.robotPositionValue}`}>{displayRobotY}</span></div>
              <div className={styles.statItem}><span className={styles.statLabel}>Z · m</span><span className={styles.statValue}>{displayRobotZ}</span></div>
            </div>
            <div className={styles.metricRow}><span>朝向</span><strong className={styles.robotYawValue}>{displayYawDeg}</strong></div>
            <div className={styles.metricRow}><span>任务</span><strong>{sseState.connected ? missionStateLabel : '等待连接'}</strong></div>
            {sseState.connected && <p className={styles.statusLine}>{plannerLabel}</p>}
            {!observe && sseState.connected && !canSendGoal && <p className={styles.statusLine}>{goalDisabledReason}</p>}
            {!sseState.connected ? <div className={styles.statusWaiting} role="status">
              <span className={styles.statusSignal} aria-hidden="true" />
              <span><strong>等待连接</strong><small>连接后显示运动和导航状态</small></span>
            </div> : <>
              <table className={styles.commandTable} aria-label="请求与输出速度">
                <thead><tr><th></th><th>前后</th><th>左右</th><th>转向</th></tr></thead>
                <tbody>
                  <tr><th scope="row">请求</th>{commandValues(nativeFresh ? request : undefined).map((v, i) => <td key={i}>{v}</td>)}</tr>
                  <tr><th scope="row">输出</th>{commandValues(nativeFresh ? finalCommand : undefined).map((v, i) => <td key={i}>{v}</td>)}</tr>
                </tbody>
              </table>
              <p className={styles.units}>平移 m/s · 转向 rad/s</p>
              <div className={styles.metricRow}><span>运动</span><strong className={styles.statusValue} data-state={navigationView.motion.observation.state}>{navigationView.motion.observation.label}</strong></div>
              <div className={styles.metricRow}><span>导航</span><strong className={styles.statusValue} data-state={navigationView.goalAdmission.state}>{navigationView.goalAdmission.label}</strong></div>
              <details className={styles.metricDetails}>
                <summary>安全细节</summary>
                <div className={styles.metricRow}><span>控制权</span><strong className={styles.statusValue} data-state={navigationView.control.state}>{navigationView.control.label}</strong></div>
                <div className={styles.metricRow}><span>运动许可</span><strong className={styles.statusValue} data-state={navigationView.motion.permission.state}>{navigationView.motion.permission.label}</strong></div>
                <div className={styles.metricRow}><span>停止确认</span><strong className={styles.statusValue} data-state={navigationView.motion.stopConfirmation.state}>{navigationView.motion.stopConfirmation.label}</strong></div>
              </details>
              {sceneDebugTools && <p className={styles.units}>{String(lastLocal?.reason ?? '—')}</p>}
            </>}
          </details>

          {sensorDiagnosticsVisible && <section className={styles.statCard} aria-label="雷达与定位">
            <div className={styles.statCardTitle}>雷达<span>{alignedScanCloud ? `${scanCloud.count.toLocaleString()} 点` : '未更新'}</span></div>
            <details className={styles.statusDetails}>
              <summary>雷达数据</summary>
            <p className={styles.sourceNote}>当前扫描，非完整碰撞栅格。</p>
            <div className={styles.metricRow}><span>定位更新</span><strong>{telemetryFresh ? formatHz(processedScanHz) : '—'}</strong></div>
            <div className={styles.metricRow}><span>雷达输入</span><strong>{telemetryFresh ? formatHz(lidarInputHz) : '—'}</strong></div>
            <div className={styles.metricRow}><span>IMU 输入</span><strong>{telemetryFresh ? formatHz(numericMetric(slamDiag, 'imu_input_hz')) : '—'}</strong></div>
            <div className={styles.metricRow}><span>局部地图点</span><strong>{cloud.count > 0 ? `${cloud.count.toLocaleString()} 点` : '当前未发布'}</strong></div>
            {sceneDebugTools && <div className={styles.metricRow}><span>后端地图点数</span><strong>{formatCount(displayedMapPoints)}</strong></div>}
            </details>
          </section>}

          </div>
          <div className={`${styles.inspectorBody} ${styles.layerSettings}`} role="tabpanel" id="scene-panel-layers"
            aria-labelledby="scene-tab-layers" hidden={inspectorTab !== 'layers'} tabIndex={0}>
              {isMappingSession && showGlobalMapping && (cloud.mappingSummary?.droppedFrames ?? 0) > 0
                ? <p className={styles.issueLine}>建图数据有缺失，请先保存原始记录</p>
                : isMappingSession && showGlobalMapping && (cloud.mappingSummary?.registrationRejections ?? 0) > 0
                    ? <p className={styles.sourceNote}>部分区域待校正</p>
                    : null}
            <section className={styles.layerSection} aria-labelledby="scene-layer-title" aria-description="只影响当前三维视图">
              <div className={styles.compactLayerHeader}>
                <strong id="scene-layer-title">显示内容</strong>
                <span title={savedMapLoadError ?? undefined}>{isMappingSession
                  ? cloud.count > 0 ? `${cloud.count.toLocaleString()} 点` : '等待点云'
                  : showNavigationMap ? navigationMap.status === 'ready' ? '导航占据地图'
                    : navigationMap.status === 'error' ? '加载失败' : '正在加载…'
                  : savedMapForScene ? `${Math.floor(savedMapForScene.points.length / 3).toLocaleString()} 点`
                    : savedMapLoadError ? '加载失败' : '正在加载…'}</span>
              </div>
              <div className={styles.layerTileGrid}>
                {isMappingSession ? <LayerButton active={mappingView === 'points'} icon={<Cloud size={13} />} label="空间点云"
                  onClick={() => { setMappingView(value => value === 'points' ? 'coverage' : 'points'); setLayers(value => ({ ...value, cloud: true })) }} />
                  : <LayerBtn k="cloud" icon={<Cloud size={13} />} label={showNavigationMap ? '导航体素' : '地图点云'} />}
                <LayerButton active={liveScanVisible} icon={<Radio size={13} />} label="实时雷达"
                  onClick={() => setLiveScanPreference(!liveScanVisible)} />
                <LayerBtn k="path" icon={<Navigation size={13} />} label="规划路径" />
                <LayerBtn k="goal" icon={<Target size={13} />} label="目标点" />
              </div>
            </section>
            {!isMappingWorkspace && !isMappingSession && showSavedMapInScene && <section className={styles.layerSection} aria-label="地图表示">
              <div className={styles.cloudColorRow}>
                <span>地图表示</span>
                <div className={styles.colorModeSwitch} role="group" aria-label="地图表示">
                  <button type="button" aria-pressed={mapRepresentation === 'points'} onClick={() => setMapRepresentation('points')}>点云</button>
                  <button type="button" aria-pressed={mapRepresentation === 'voxels'} disabled={mapEditing || stairEditing}
                    onClick={() => setMapRepresentation('voxels')}>导航体素</button>
                </div>
              </div>
              {showNavigationMap && <p className={styles.sourceNote} role="status">{navigationMap.status === 'ready'
                ? `原始 ${(navigationMap.map.resolution * 100).toFixed(0)} cm · 显示最小 ${(navigationMap.map.resolution * voxelScale * 100).toFixed(0)} cm · 共 ${(navigationMap.map.cells.length / 4).toLocaleString()} 个原始占据单元。显示附近 20 × 20 m。`
                : navigationMap.status === 'error' ? navigationMap.message
                  : layers.cloud ? '正在读取并解析导航体素…' : '地图显示已关闭'}</p>}
              {showNavigationMap && navigationMap.status === 'ready' && <>
                <div className={styles.cloudColorRow}>
                  <span>显示粒度</span>
                  <div className={`${styles.colorModeSwitch} ${styles.heightModeSwitch}`} role="group" aria-label="体素显示粒度">
                    {[1, 2, 4].map(scale => <button key={scale} type="button" aria-pressed={voxelScale === scale}
                      onClick={() => setVoxelScale(scale)}>{scale === 1 ? '原始' : `${(navigationMap.map.resolution * scale * 100).toFixed(0)} cm`}</button>)}
                  </div>
                </div>
                <p className={styles.sourceNote}>合并显示不修改导航地图或选点高度。这里只显示原始占据，未叠加推断支撑面；空白也可能是自由或未观测空间。</p>
                <button type="button" className={styles.workbenchButton} onClick={() => onOpenSavedMap(activeMapName ?? null)}>重建实际导航分辨率</button>
              </>}
            </section>}
            <section className={styles.layerSection} aria-label="地图外观">
              <div className={styles.cloudAppearance}>
                <div className={styles.cloudColorRow}>
                  <span>颜色</span>
                  <div className={styles.colorModeSwitch} role="group" aria-label="点云颜色">
                    <button type="button" aria-pressed={cloudColorMode === 'height'}
                      onClick={() => setCloudColorMode('height')}>高度</button>
                    <button type="button" aria-pressed={cloudColorMode === 'neutral'}
                      onClick={() => setCloudColorMode('neutral')}>单色</button>
                  </div>
                </div>
                {cloudColorMode === 'height' && <div className={styles.cloudHeightLegend}
                  aria-label="地图 Z 高度配色，超出色阶沿用端点颜色，不删除点">
                  <span>≤ {(colorHeightRange?.min ?? CLOUD_HEIGHT_MIN).toFixed(2)} m</span>
                  <i style={{ background: CLOUD_HEIGHT_GRADIENT }} aria-hidden="true" />
                  <span>≥ {(colorHeightRange?.max ?? CLOUD_HEIGHT_MAX).toFixed(2)} m</span>
                  <small>{cloudMinZ !== null ? '当前高度范围' : mapColorHeightRange ? '主要高度 · 2–98%' : '实时 Z · 固定色阶'}</small>
                </div>}
                {!showNavigationMap && <label className={styles.pointSizeRow}>
                  <span>大小</span>
                  <output>{pointSize.toFixed(2)}</output>
                  <input type="range" min={0.02} max={0.2} step={0.005} value={pointSize}
                    onChange={e => setPointSize(parseFloat(e.target.value))} className={styles.pointSlider} aria-label="点云大小" />
                </label>}
                <div className={styles.cloudColorRow}>
                  <span>高度视图</span>
                  <div className={`${styles.colorModeSwitch} ${styles.heightModeSwitch}`} role="group" aria-label="点云高度范围">
                    <button type="button" aria-pressed={cloudHeightMode === 'all'} title="所有高度"
                      onClick={() => setCloudHeightModePreference('all')}>全图</button>
                    <button type="button" aria-pressed={cloudHeightMode === 'follow'} title={showNavigationMap ? '跟随机器人高度，范围外隐藏' : '跟随机器人高度，远处渐隐'}
                      onClick={() => setCloudHeightModePreference('follow')}>跟随</button>
                    <button type="button" aria-pressed={cloudHeightMode === 'fixed'} title="固定高度范围"
                      onClick={() => setCloudHeightModePreference('fixed')}>固定</button>
                  </div>
                </div>
                {cloudHeightMode !== 'all' && <div className={styles.cloudHeightSettings}>
                  {cloudHeightMode === 'fixed' && cloudMinZ !== null && cloudMaxZ !== null && <HeightRangeControl
                    min={cloudMinZ} max={cloudMaxZ} domainMin={cloudHeightDomain.min} domainMax={cloudHeightDomain.max}
                    robotZ={poseAvailable ? robotZ : null} goalZ={pendingGoal?.z}
                    onChange={(min, max) => {
                      const z = (min + max) / 2
                      setFixedCloudHeight({ epoch: cloudHeightKey, z, input: z.toFixed(2), below: z - min, above: max - z })
                    }} />}
                  {cloudHeightMode === 'follow' && <><label className={styles.pointSizeRow}>
                    <span>机身上方</span>
                    <output>{cloudHeightAboveBase.toFixed(1)} m</output>
                    <input type="range" min={0.3} max={3} step={0.1} value={cloudHeightAboveBase}
                      onChange={event => setCloudHeightAboveBase(Number(event.target.value))}
                      className={styles.pointSlider} aria-label="上方显示高度" />
                  </label>
                  <label className={styles.pointSizeRow}>
                    <span>机身下方</span>
                    <output>{cloudHeightBelowBase.toFixed(1)} m</output>
                    <input type="range" min={0.3} max={3} step={0.1} value={cloudHeightBelowBase}
                      onChange={event => setCloudHeightBelowBase(Number(event.target.value))}
                      className={styles.pointSlider} aria-label="下方显示高度" />
                  </label></>}
                  <span className={styles.sourceNote}>{cloudMaxZ === null || cloudMinZ === null ? '等待定位 · 暂时显示全图'
                    : `${cloudHeightMode === 'follow' ? '清晰范围' : '切片范围'} Z ${cloudMinZ.toFixed(2)} ~ ${cloudMaxZ.toFixed(2)} m${cloudHeightMode === 'follow' ? showNavigationMap ? ' · 范围外隐藏' : ' · 远处渐隐' : ''}`}</span>
                  {cloudHeightMode === 'follow' && !poseAvailable && cloudMaxZ !== null
                    && <span className={styles.sourceNote}>定位未更新 · 保持最近范围</span>}
                </div>}
              </div>
            </section>
            <details className={styles.advancedTools}>
              <summary>更多选项</summary>
              <div className={styles.extraLayerChoices}>
                <LayerBtn k="trail" icon={<Route size={13} />} label="行走轨迹" />
                <LayerBtn k="robot" icon={<Bot size={13} />} label="机器人" />
                <LayerBtn k="grid" icon={<Grid3x3 size={13} />} label="参考网格" />
              </div>
              <details className={styles.layerDisclosure}>
                <summary>图例与说明</summary>
                <div className={styles.layerHelp}>
                  {!isMappingWorkspace && !isMappingSession && <div className={styles.layerPathLegend} aria-label="轨迹图例">
                    <span><i className={styles.routeKey} />执行轨迹</span>
                    <span><i className={styles.globalKey} />全局路径</span>
                    <span><i className={styles.trailKey} />已走轨迹</span>
                  </div>}
                  <p className={styles.sourceNote}>{observe || isMappingWorkspace ? '拖动旋转 · 滚轮缩放'
                    : navigationSelectionMode === 'browse' ? '拖动浏览 · 双指或滚轮缩放'
                    : navigationSelectionMode === 'route' ? '点击添加路线点位，确认后开始导航。'
                      : '点击选目标，自动预览路线；默认不限制终点朝向，可在“调整”中设置。'}</p>
                  <p className={styles.sourceNote}>右键拖动旋转 · 中键或 Shift＋右键拖动平移 · 滚轮缩放</p>
                  {!isMappingWorkspace && !isMappingSession && <p className={styles.sourceNote}>{showNavigationMap
                    ? '颜色表示高度；体素包括地面表面。空白未区分自由与未知，不代表可以通行。'
                    : '点云空白不代表可通行'}</p>}
                  {isMappingSession && <section aria-label="建图显示图例">
                  {showMappingObservation ? <>
                    <p className={styles.mappingTruth}>地面相对高度 · 不代表可通行</p>
                    <div className={styles.mappingObservationLegend}>
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
                    <p>二维投影不是地面高度，也不判断通行；导航使用三维地图。</p>
                    <p>点击格子可查看局部表面拟合高度、残差和细 XY 支撑数；缺失时不作推断。</p>
                    {mappingObservation.status === 'ready' && <>
                      <p>当前窗口 {(mappingObservation.layer.cols * mappingObservation.layer.resolution).toFixed(1)} × {(mappingObservation.layer.rows * mappingObservation.layer.resolution).toFixed(1)} m</p>
                    </>}
                    <p>灰格缺少依据，可换视角补扫。窗口外不表示从未建图；整图是累计点云的采样预览，不代表建图完成比例。</p>
                  </details>}
                  </section>}
                </div>
              </details>
              <details className={styles.layerDisclosure}>
                <summary>诊断与清理</summary>
                <div className={styles.extraLayerChoices}>
                  <LayerButton active={sensorDiagnosticsVisible} icon={<Radio size={13} />} label="雷达诊断"
                    onClick={() => setSensorDiagnosticsVisible(value => !value)} />
                  <LayerBtn k="elevation" icon={<MapPinned size={13} />} label="最低观测高程" />
                  {sceneDebugTools && <>
                    <LayerBtn k="nativeTraversability" icon={<Activity size={13} />} label="风险投影" />
                    <LayerBtn k="localPlanner" icon={<Radio size={13} />} label="安全范围" />
                  </>}
                </div>
                {sceneDebugTools && layers.localPlanner && <div className={styles.extraLayerChoices}>
                  <LayerButton active={safetyView === 'slice'} icon={<Grid3x3 size={13} />} label="高度切片"
                    onClick={() => { setSafetyView('slice'); scene3DRef.current?.topView() }} />
                  <LayerButton active={safetyView === 'volume'} icon={<Layers2 size={13} />} label="三维诊断"
                    onClick={() => setSafetyView('volume')} />
                </div>}
                <p className={styles.sourceNote}>仅供查看，不参与导航决策。</p>
                {!observe && <div className={styles.layerCleanup}>
                  <button onClick={handleClearTrail} aria-label="清除显示轨迹" title="仅清除画面中的行走轨迹"><Trash2 size={14} /> 清除轨迹</button>
                  <button onClick={handleClearCloud} aria-label="清除显示点云" title="仅清除显示缓存，不修改地图或定位"><Cloud size={14} /> 清除点云</button>
                </div>}
              </details>
            </details>
          </div>
          {!observe && <div className={`${styles.inspectorBody} ${styles.operationSettings}`} role="tabpanel" id="scene-panel-tools"
            aria-labelledby="scene-tab-tools" hidden={inspectorTab !== 'tools'} tabIndex={0}>
            {stairEditing && savedMapForScene?.contentEpoch != null ? <StairEditor
              key={stairMapKey} ref={stairEditorRef} map={savedMapForScene.mapName}
              epoch={savedMapForScene.contentEpoch} frame={savedMapForScene.frameId}
              onSceneChange={onStairSceneChange} onClose={() => setStairEditorOpen(false)} /> : <>
            {recordingPanelOpen && inspectorTab === 'tools' && (
              <div id="scene-recording">
                <RecordingPanel view={recordingView ?? 'record'} onViewChange={setRecordingView} embedded showToast={showToast} locale={locale}
                  status={recordingStatus} statusError={recordingStatusError}
                  refreshStatus={refreshRecordingStatus} onClose={() => setRecordingView(null)} />
              </div>
            )}
              <div className={styles.toolGroup}>
                <button onClick={() => openWorkspaceTool('maps')}><MapPinned size={14} /> 地图</button>
                <button disabled={isMappingWorkspace || !savedMapForScene?.points.length || !savedMapForScene.contentEpoch || isMappingSession}
                  onClick={openMapEditor}><Pencil size={14} />点云整理</button>
                {!isMappingWorkspace && <button onClick={() => openWorkspaceTool('locations')}><Target size={14} /> 常用位置</button>}
                {!isMappingWorkspace && <button disabled={isMappingSession} onClick={() => openWorkspaceTool('localization')}><LocateFixed size={14} /> 定位</button>}
                <button disabled={isMappingWorkspace || !savedMapForScene?.points.length || savedMapForScene.contentEpoch == null || isMappingSession}
                  title="在保存的三维地图上标注楼梯范围与入口"
                  onClick={() => { setStairEditorOpen(true); setTeleopMode(false); setFollowRobot(false); setLayers(value => ({ ...value, cloud: true })) }}>
                  <Pencil size={14} /> 楼梯标注</button>
              </div>
              <div className={styles.toolGroup}>
                <span className={styles.menuLabel}>记录</span>
                <button onClick={() => {
                  setRecordingView(view => view === 'record' ? null : 'record')
                }} aria-expanded={recordingView === 'record'} aria-controls="scene-recording" title={recordingToolbarTitle}>
                  <CircleDot size={12} /> 录制
                  {recordingToolbarState && <span className={styles.menuMeta}> · {recordingToolbarState}</span>}
                </button>
                <button onClick={() => setRecordingView(view => view === 'history' ? null : 'history')}
                  aria-expanded={recordingView === 'history'} aria-controls="scene-recording">
                  <Play size={12} /> 回看
                </button>
              </div>
            </>}
          </div>}
        </aside>
      </div>

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
        message={`${capacityNotice ? `${capacityNotice} ` : ''}先保存建图点云存档，可下载 PCD。处理成功后，同名会替换原地图；要保留原图请使用新名称。`}
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
      >
        <label style={{ display: 'flex', gap: 8, marginTop: 14, alignItems: 'flex-start' }}>
          <input type="checkbox" checked={autoProcess} onChange={event => setAutoProcess(event.target.checked)} />
          <span>保存点云后自动后台处理<br /><small>取消勾选后，点击地图库中的状态图标开始处理。</small></span>
        </label>
      </PromptModal>
    </div>
  )
}

export const SceneView = memo(SceneViewComponent)
