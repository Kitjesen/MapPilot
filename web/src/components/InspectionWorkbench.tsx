import { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import {
  AlertTriangle,
  ArrowDown,
  ArrowUp,
  Pause,
  Play,
  Plus,
  RefreshCcw,
  RotateCcw,
  Save,
  Square,
  Trash2,
  XCircle,
  MapPin,
  ClipboardCheck,
} from 'lucide-react'
import type {
  InspectionFailurePolicy,
  InspectionEvidenceSummary,
  InspectionEvidenceWorkerStatus,
  InspectionRoute,
  InspectionRoutePoint,
  InspectionTaskCommandResponse,
  InspectionTaskReportResponse,
  InspectionTaskStatusResponse,
  LocationEntry,
  LocationsResponse,
  SSEState,
} from '../types'
import { text, type Locale } from '../i18n'
import * as api from '../services/api'
import { isObservationMode } from '../services/observationMode.ts'
import styles from './InspectionWorkbench.module.css'
import MeasurementInspector from './MeasurementInspector'

interface InspectionWorkbenchProps {
  sseState: SSEState
  showToast: (message: string, type?: 'success' | 'error' | 'info') => void
  locale: Locale
  initialRouteId?: string | null
  onShowRoute?: (route: InspectionRoute) => void
}

type BusyAction = 'load' | 'save' | 'start' | 'pause' | 'resume' | 'cancel' | 'release' | 'delete' | null
type InspectionAvailability = 'checking' | 'available' | 'unavailable'

const INSPECTION_ACTION_OPTIONS = [
  '',
  'capture:overview',
  'capture:parking',
  'capture:bin_full',
  'capture:plate_ocr',
  'capture:indicator_on',
  'capture:indicator_off',
  'capture:indicator_green',
  'capture:indicator_red',
  'capture:indicator_yellow',
  'capture:door_open',
  'capture:door_closed',
]

function sessionActiveMap(sseState: SSEState): string {
  const session = sseState.session as Record<string, unknown> | null | undefined
  const active = session?.active_map ?? session?.saved_active_map
  return typeof active === 'string' ? active : ''
}

function statusText(value: unknown, fallback = '--'): string {
  if (typeof value === 'number' && Number.isFinite(value)) return String(value)
  if (typeof value === 'boolean') return value ? 'true' : 'false'
  return typeof value === 'string' && value.length > 0 ? value : fallback
}

function taskTruthLabel(task: InspectionTaskStatusResponse | null, locale: Locale): string {
  if (!task) return text(locale, 'No task selected', '未选择任务')
  if (task.state_source === 'native_task_event') {
    return text(locale, 'Confirmed', '已确认')
  }
  if (task.state_source === 'business_ack_only') {
    return text(locale, 'Waiting for robot confirmation', '等待机器人确认')
  }
  if (task.state_source === 'continuity_monitor') {
    return text(locale, 'Task event continuity interrupted', '任务事件连续性已中断')
  }
  return text(locale, 'Task state unavailable', '任务状态不可用')
}

function inspectionTaskStateLabel(state: string, locale: Locale): string {
  const labels: Record<string, [string, string]> = {
    ACCEPTED: ['Waiting to start', '等待开始'],
    PLANNING: ['Planning', '规划中'],
    EXECUTING: ['Running', '执行中'],
    RECOVERING: ['Recovering', '正在脱困'],
    WAITING_NEXT_LOOP: ['Waiting for next round', '等待下一轮'],
    RUNNING: ['Running', '执行中'],
    PAUSED: ['Paused', '已暂停'],
    SUCCEEDED: ['Completed', '已完成'],
    COMPLETED: ['Completed', '已完成'],
    SUCCESS: ['Completed', '已完成'],
    FAILED: ['Failed', '失败'],
    CANCELLED: ['Cancelled', '已取消'],
  }
  const label = labels[state]
  return label ? text(locale, label[0], label[1]) : state
}

function inspectionTaskLabel(task: InspectionTaskStatusResponse, locale: Locale): string {
  if (task.current_state === 'PAUSED' && [
    'restart_resume_required', 'restart_loop_wait_resume_required',
  ].includes(task.reason)) {
    return text(locale, 'Restarted · waiting to continue', '重启后等待继续')
  }
  if (Number(task.latest_event?.state) === 14 && task.current_state === 'EXECUTING') {
    return text(locale, 'Waiting for next round', '等待下一轮')
  }
  return inspectionTaskStateLabel(task.current_state, locale)
}

function evidenceWorkerFromStatus(status: Record<string, unknown>): InspectionEvidenceWorkerStatus {
  const worker = status.evidence_worker
  if (typeof worker === 'object' && worker !== null && 'ready' in worker) {
    const parsed = worker as InspectionEvidenceWorkerStatus
    return {
      ...parsed,
      ready: parsed.ready === true,
      supported_actions: Array.isArray(parsed.supported_actions)
        ? parsed.supported_actions.filter((item): item is string => typeof item === 'string')
        : [],
    }
  }
  return {
    ready: false,
    reason: 'unknown',
    supported_actions: [],
  }
}

function routeHasEvidenceActions(points: InspectionRoutePoint[]): boolean {
  return points.some(point => point.enabled && point.action.trim().length > 0)
}

function routePointFromLocation(location: LocationEntry): InspectionRoutePoint {
  return {
    id: location.name,
    x: location.x,
    y: location.y,
    z: location.z ?? 0,
    yaw: location.yaw ?? null,
    tolerance: 0.35,
    dwell: 3,
    action: 'capture:overview',
    enabled: true,
  }
}

function routePointKey(point: InspectionRoutePoint, index: number): string {
  return `${point.id}:${index}`
}

function standbyPointIndex(points: InspectionRoutePoint[]): number {
  const last = points.at(-1)
  return last && last.action === '' && (last.id === 'standby' || /^route-standby(?:-\d+)?$/.test(last.id))
    ? points.length - 1 : -1
}

function locationIsBoundToRoute(
  location: LocationEntry,
  mapId: string,
  mapVersion: number,
): boolean {
  return location.metadata?.binding_status === 'bound'
    && location.map_id === mapId
    && location.map_content_epoch === mapVersion
    && (location.frame_id ?? 'map') === 'map'
}

function locationBindingLabel(location: LocationEntry): string {
  const status = statusText(location.metadata?.binding_status, 'unbound')
  if (status !== 'bound') return status
  return `${location.map_id ?? '--'} · v${location.map_content_epoch ?? '--'}`
}

function nextRevision(route?: InspectionRoute | null): number {
  const current = Number(route?.revision ?? 0)
  return Number.isFinite(current) && current > 0 ? current + 1 : 1
}

function hasEvidenceArtifact(
  evidence: InspectionEvidenceSummary,
  kind: 'rgb' | 'pose' | 'detections',
): boolean {
  return evidence.artifacts.some(artifact => artifact.kind === kind)
}

function evidenceTimestamp(evidence: InspectionEvidenceSummary, locale: Locale): string {
  const timestamp = Number(evidence.request.requested_at_s ?? 0)
  if (!Number.isFinite(timestamp) || timestamp <= 0) return '--'
  return new Date(timestamp * 1000).toLocaleString(locale === 'zh' ? 'zh-CN' : 'en-US', {
    hour12: false,
  })
}

function inspectionActionLabel(action: string, locale: Locale): string {
  switch (action) {
    case 'capture:overview':
      return text(locale, 'Overview capture', '环境总览')
    case 'capture:parking':
      return text(locale, 'Parking inspection', '停车区域巡检')
    case 'capture:bin_full':
      return text(locale, 'Bin status', '料箱状态')
    case 'capture:plate_ocr':
      return text(locale, 'Identifier OCR', '标识识别')
    case 'capture:indicator_on':
      return text(locale, 'Indicator expected on', '指示灯亮')
    case 'capture:indicator_off':
      return text(locale, 'Indicator expected off', '指示灯灭')
    case 'capture:indicator_green':
      return text(locale, 'Indicator expected green', '指示灯绿')
    case 'capture:indicator_red':
      return text(locale, 'Indicator expected red', '指示灯红')
    case 'capture:indicator_yellow':
      return text(locale, 'Indicator expected yellow', '指示灯黄')
    case 'capture:door_open':
      return text(locale, 'Cabinet door expected open', '柜门打开')
    case 'capture:door_closed':
      return text(locale, 'Cabinet door expected closed', '柜门关闭')
    case '':
      return text(locale, 'None', '无')
    default:
      return action
  }
}

function inspectionVerdictLabel(verdict: string, locale: Locale): string {
  switch (verdict.toLowerCase()) {
    case 'anomaly':
      return text(locale, 'Anomaly', '异常')
    case 'needs_review':
    case 'not_evaluated':
      return text(locale, 'Needs review', '需复核')
    case 'violation':
      return text(locale, 'Needs review', '需复核')
    case 'ok':
    case 'normal':
    case 'pass':
      return text(locale, 'Normal', '正常')
    case 'inconclusive':
      return text(locale, 'Not readable', '未读到')
    case 'unknown':
      return text(locale, 'Uncertain', '不确定')
    default:
      return verdict
  }
}

function inspectionResultLabel(result: string, locale: Locale): string {
  const labels: Record<string, [string, string]> = {
    NOT_REQUIRED: ['No inspection required', '无检查动作'],
    IN_PROGRESS: ['Inspection in progress', '检查进行中'],
    NEEDS_REVIEW: ['Needs review', '需复核'],
    NORMAL: ['Normal', '正常'],
    ANOMALY: ['Anomaly found', '发现异常'],
    INCONCLUSIVE: ['Not readable', '未读到'],
    NOT_INSPECTED: ['Not inspected', '未检查'],
  }
  const label = labels[result]
  return label ? text(locale, label[0], label[1]) : text(locale, 'Awaiting result', '等待结果')
}

function inspectionAcceptanceLabel(status: string, locale: Locale): string {
  switch (status) {
    case 'ACCEPTABLE':
      return text(locale, 'Complete records', '资料完整')
    case 'REVIEW_REQUIRED':
      return text(locale, 'Review required', '需要复核')
    case 'NOT_ACCEPTABLE':
      return text(locale, 'Incomplete records', '资料未齐')
    case 'PENDING':
      return text(locale, 'Pending', '等待完成')
    default:
      return text(locale, 'Unknown', '未知')
  }
}

function inspectionPointResultLabel(status: string, locale: Locale): string {
  switch (status) {
    case 'COMPLETED':
      return text(locale, 'Complete', '已完成')
    case 'SKIPPED':
      return text(locale, 'Skipped', '已跳过')
    case 'FAILED':
      return text(locale, 'Failed', '失败')
    case 'CANCELLED':
      return text(locale, 'Cancelled', '已取消')
    case 'NOT_VISITED':
      return text(locale, 'Not visited', '未执行')
    case 'IN_PROGRESS':
      return text(locale, 'In progress', '执行中')
    case 'PENDING':
      return text(locale, 'Pending', '等待执行')
    case 'MISSING_EVIDENCE':
      return text(locale, 'Missing evidence', '缺少证据')
    case 'INVALID_EVIDENCE':
      return text(locale, 'Invalid evidence', '证据无效')
    case 'UNAVAILABLE_EVIDENCE':
      return text(locale, 'Evidence unavailable', '证据不可用')
    default:
      return text(locale, 'Unknown', '未知')
  }
}

function inspectionReportIssueLabel(
  issue: InspectionTaskReportResponse['issues'][number],
  locale: Locale,
): string {
  const point = statusText(
    issue.point_id,
    text(locale, 'an inspection point', '某个巡检点'),
  )
  switch (issue.code) {
    case 'task_history_incomplete':
      if (issue.reason === 'native_event_identity_conflict') {
        return text(
          locale,
          'A native event disagreed with the task map or route. The terminal result was rejected and this task needs review.',
          '原生事件与任务启动时的地图或路线不一致；该终态已被拒绝，本任务需要复核。',
        )
      }
      return text(
        locale,
        'Task history is incomplete, so this result cannot be accepted yet.',
        '任务历史不完整，因此当前结果暂不可验收。',
      )
    case 'missing_evidence':
      return text(locale, 'Evidence is missing at ' + point + '.', point + ' 缺少证据。')
    case 'invalid_evidence':
      return text(locale, 'Evidence is invalid at ' + point + '.', point + ' 的证据无效。')
    case 'unavailable_evidence':
      return text(locale, 'Evidence is unavailable at ' + point + '.', point + ' 的证据不可用。')
    case 'point_skipped':
      return text(locale, point + ' was skipped.', point + ' 已跳过。')
    case 'point_outcome_unknown':
      return text(locale, point + ' has no confirmed completion.', point + ' 缺少完成记录。')
    default:
      return text(locale, 'This result needs operator review.', '此结果需要人工复核。')
  }
}

function inspectionReportErrorLabel(error: unknown, locale: Locale): string {
  if (api.isGatewayApiError(error)) {
    if (error.errorCode === 'inspection_task_route_snapshot_unavailable') {
      return text(
        locale,
        'The original route requirements were not preserved, so this historical task cannot be accepted. Run a new inspection.',
        '该历史任务未保存启动时的路线要求，因此不能验收；请重新执行一次巡检。',
      )
    }
    if (error.errorCode === 'inspection_task_journal_unavailable') {
      return text(
        locale,
        'The task journal is unavailable. Restore the journal before reviewing this result.',
        '任务日志当前不可用；请先恢复任务日志，再复核本次结果。',
      )
    }
  }
  return error instanceof Error ? error.message : String(error)
}

function taskEventTimestamp(event: Record<string, unknown>, locale: Locale): string {
  const timestamp = Number(event.ts)
  if (!Number.isFinite(timestamp) || timestamp <= 0) return '--'
  return new Date(timestamp * 1000).toLocaleString(locale === 'zh' ? 'zh-CN' : 'en-US', {
    hour12: false,
  })
}

function upsertInspectionTask(
  tasks: InspectionTaskStatusResponse[],
  snapshot: InspectionTaskStatusResponse,
): InspectionTaskStatusResponse[] {
  return [snapshot, ...tasks.filter(task => task.task_id !== snapshot.task_id)].slice(0, 12)
}

export function InspectionWorkbench({ sseState, showToast, locale, initialRouteId, onShowRoute }: InspectionWorkbenchProps) {
  const observe = isObservationMode()
  const [mapId, setMapId] = useState(() => sessionActiveMap(sseState))
  const [routeId, setRouteId] = useState('route-1')
  const [routeName, setRouteName] = useState(() => text(locale, 'Inspection route', '巡检路线'))
  const [mapVersion, setMapVersion] = useState(1)
  const [revision, setRevision] = useState(1)
  const [savedRevision, setSavedRevision] = useState(0)
  const [loopCount, setLoopCount] = useState(1)
  const [loopInterval, setLoopInterval] = useState(0)
  const [failurePolicy, setFailurePolicy] = useState<InspectionFailurePolicy>('stop')
  const [maxRetries, setMaxRetries] = useState(0)
  const [points, setPoints] = useState<InspectionRoutePoint[]>([])
  const [routes, setRoutes] = useState<InspectionRoute[]>([])
  const [locations, setLocations] = useState<LocationsResponse | null>(null)
  const [selectedRouteId, setSelectedRouteId] = useState<string | null>(initialRouteId ?? null)
  const [status, setStatus] = useState<Record<string, unknown>>({})
  const [activeTaskId, setActiveTaskId] = useState<string | null>(null)
  const [taskStatus, setTaskStatus] = useState<InspectionTaskStatusResponse | null>(null)
  const [taskReport, setTaskReport] = useState<InspectionTaskReportResponse | null>(null)
  const [taskReportError, setTaskReportError] = useState<string | null>(null)
  const [inspectionTasks, setInspectionTasks] = useState<InspectionTaskStatusResponse[]>([])
  const [manualTakeoverReleasedForTask, setManualTakeoverReleasedForTask] = useState<string | null>(null)
  const [evidenceItems, setEvidenceItems] = useState<InspectionEvidenceSummary[]>([])
  const [evidenceIntegrityFailures, setEvidenceIntegrityFailures] = useState(0)
  const [reviewNotes, setReviewNotes] = useState<Record<string, string>>({})
  const [reviewErrors, setReviewErrors] = useState<Record<string, string>>({})
  const [reviewBusyId, setReviewBusyId] = useState<string | null>(null)
  const [busy, setBusy] = useState<BusyAction>(null)
  const [error, setError] = useState<string | null>(null)
  const [taskReadError, setTaskReadError] = useState<string | null>(null)
  const [statusReadError, setStatusReadError] = useState<string | null>(null)
  const [evidenceReadError, setEvidenceReadError] = useState<string | null>(null)
  const [inspectionAvailability, setInspectionAvailability] = useState<InspectionAvailability>('checking')
  const activeTaskIdRef = useRef<string | null>(null)

  useEffect(() => {
    const active = sessionActiveMap(sseState)
    if (active && !mapId) setMapId(active)
  }, [mapId, sseState])

  const selectedRoute = useMemo(
    () => routes.find(route => route.id === selectedRouteId) ?? null,
    [routes, selectedRouteId],
  )
  const routeMatchesActiveMap = Boolean(selectedRoute?.map_id)
    && selectedRoute?.map_id === sessionActiveMap(sseState)
  const editorHasUnsavedChanges = selectedRoute !== null && (
    routeId !== selectedRoute.id
    || routeName !== (selectedRoute.name || selectedRoute.id)
    || mapId !== (selectedRoute.map_id || '')
    || mapVersion !== Number(selectedRoute.map_content_epoch ?? 1)
    || revision !== nextRevision(selectedRoute)
    || loopCount !== Number(selectedRoute.loop_count ?? 1)
    || loopInterval !== Number(selectedRoute.loop_interval_s ?? 0)
    || failurePolicy !== (selectedRoute.failure_policy || 'stop')
    || maxRetries !== Number(selectedRoute.max_retries ?? 0)
    || points !== selectedRoute.points
  )

  const enabledPointCount = points.filter(point => point.enabled).length
  const standbyIndex = standbyPointIndex(points)
  const standbyPoint = standbyIndex >= 0 ? points[standbyIndex] : null
  const devicePoints = standbyPoint ? points.slice(0, -1) : points
  const standbyLocations = (locations?.locations ?? []).filter(location => locationIsBoundToRoute(location, mapId, mapVersion))
  const matchedStandbyLocation = standbyPoint && standbyLocations.find(location =>
    location.x === standbyPoint.x && location.y === standbyPoint.y && (location.z ?? 0) === standbyPoint.z,
  )
  const taskProgress = taskStatus?.progress
  const taskPointNumber = taskProgress?.current_point_number ?? null
  const progressLabel = taskReport
    ? `${taskReport.coverage.completed_points}/${taskReport.coverage.required_points}`
    : '--'
  const taskStillOpen = (activeTaskId !== null && taskStatus?.terminal !== true)
    || inspectionTasks.some(task => !task.terminal)
  const taskTimeline = useMemo(
    () => [...(taskStatus?.timeline ?? [])].slice(-8).reverse(),
    [taskStatus?.timeline],
  )
  const manualTakeoverReleaseRequired = taskStatus?.current_state === 'PAUSED'
    && statusText(taskStatus.reason, '').includes('operator_takeover')
    && manualTakeoverReleasedForTask !== activeTaskId
  const evidenceWorker = evidenceWorkerFromStatus(status)
  const evidenceWorkerReady = evidenceWorker.ready === true
  const supportedEvidenceActions = new Set(evidenceWorker.supported_actions)
  const availableActionOptions = INSPECTION_ACTION_OPTIONS.filter(
    action => action === '' || supportedEvidenceActions.has(action),
  )
  const unsupportedRouteActions = Array.from(new Set(
    points
      .filter(point => point.enabled && point.action.trim().length > 0)
      .map(point => point.action.trim())
      .filter(action => !supportedEvidenceActions.has(action)),
  ))
  const startBlockedByEvidenceWorker = routeHasEvidenceActions(points)
    && (!evidenceWorkerReady || unsupportedRouteActions.length > 0)

  const applyRouteToEditor = useCallback((route: InspectionRoute) => {
    const currentRevision = Number(route.revision ?? 0)
    setRoutes(current => current.map(item => item.id === route.id ? route : item))
    setSelectedRouteId(route.id)
    setRouteId(route.id)
    setRouteName(route.name || route.id)
    setMapId(route.map_id || mapId)
    setMapVersion(Number(route.map_content_epoch ?? 1))
    setSavedRevision(Number.isInteger(currentRevision) && currentRevision > 0 ? currentRevision : 0)
    setRevision(nextRevision(route))
    setLoopCount(Number(route.loop_count ?? 1))
    setLoopInterval(Number(route.loop_interval_s ?? 0))
    setFailurePolicy((route.failure_policy as InspectionFailurePolicy) || 'stop')
    setMaxRetries(Number(route.max_retries ?? 0))
    setPoints(route.points ?? [])
  }, [mapId])

  const selectInspectionTask = useCallback((task: InspectionTaskStatusResponse | null) => {
    const taskId = task?.task_id ?? null
    activeTaskIdRef.current = taskId
    setActiveTaskId(taskId)
    setTaskStatus(task)
    setTaskReport(null)
    setTaskReportError(null)
  }, [])

  const load = useCallback(async () => {
    setBusy('load')
    try {
      const bootstrap = await api.fetchAppBootstrap()
      const routeApiAvailable = Boolean(
        bootstrap.links?.inspection_routes
          && bootstrap.links?.inspection_status
          && bootstrap.links?.inspection_tasks
          && bootstrap.links?.inspection_task_status
          && bootstrap.links?.inspection_task_report,
      )
      if (!routeApiAvailable) {
        const locationResult = await Promise.allSettled([api.fetchLocations()])
        if (locationResult[0].status === 'fulfilled') setLocations(locationResult[0].value)
        setRoutes([])
        setStatus({})
        setInspectionTasks([])
        selectInspectionTask(null)
        setEvidenceItems([])
        setEvidenceIntegrityFailures(0)
        setInspectionAvailability('unavailable')
        setError(null)
        return
      }
      setInspectionAvailability('available')
      const [routeResult, locationResult, statusResult, taskResult, evidenceResult] = await Promise.allSettled([
        api.fetchInspectionRoutes(mapId || null),
        api.fetchLocations(),
        api.fetchInspectionStatus(),
        api.fetchInspectionTasks({ mapId: mapId || null, includeTerminal: true, limit: 12 }),
        api.fetchInspectionEvidence(12),
      ])
      if (routeResult.status === 'fulfilled') {
        const summaries = routeResult.value.routes
        setRoutes(summaries)
        const preferredRoute = summaries.find(route => route.id === selectedRouteId) ?? summaries[0]
        if (preferredRoute) {
          const detail = await api.fetchInspectionRoute(
            preferredRoute.id,
            preferredRoute.map_id || mapId || null,
          )
          applyRouteToEditor(detail.route)
        } else {
          setSelectedRouteId(null)
          setSavedRevision(0)
          setPoints([])
        }
      } else {
        throw routeResult.reason
      }
      if (locationResult.status === 'fulfilled') setLocations(locationResult.value)
      if (statusResult.status === 'fulfilled') {
        setStatus(statusResult.value.status)
        setStatusReadError(null)
      } else {
        setStatusReadError(String(statusResult.reason))
      }
      if (taskResult.status === 'fulfilled') {
        const retainedTasks = taskResult.value.tasks
        setInspectionTasks(retainedTasks)
        const currentTask = activeTaskIdRef.current
          ? retainedTasks.find(task => task.task_id === activeTaskIdRef.current) ?? null
          : retainedTasks.find(task => !task.terminal) ?? retainedTasks[0] ?? null
        selectInspectionTask(currentTask)
        setTaskReadError(null)
      } else {
        setTaskReadError(String(taskResult.reason))
      }
      if (evidenceResult.status === 'fulfilled') {
        setEvidenceItems(evidenceResult.value.evidence)
        setEvidenceIntegrityFailures(evidenceResult.value.integrity_failures)
        setEvidenceReadError(null)
      } else {
        setEvidenceReadError(String(evidenceResult.reason))
      }
      if (locationResult.status === 'rejected') throw locationResult.reason
      setError(null)
    } catch (err) {
      if (api.isGatewayApiError(err) && (err.statusCode === 404 || err.statusCode === 503)) {
        setInspectionAvailability('unavailable')
        setError(null)
        return
      }
      const message = api.isGatewayApiError(err) && err.statusCode === 502
        ? text(locale, 'Inspection service is unavailable (HTTP 502). Check Gateway and refresh.', '巡检服务暂时不可用（HTTP 502），请检查 Gateway 后刷新。')
        : err instanceof Error ? err.message : String(err)
      setError(message)
    } finally {
      setBusy(null)
    }
  }, [applyRouteToEditor, locale, mapId, selectInspectionTask, selectedRouteId])

  useEffect(() => {
    void load()
  }, [load])

  useEffect(() => {
    if (inspectionAvailability !== 'available') return
    const timer = window.setInterval(() => {
      api.fetchInspectionStatus()
        .then(result => {
          setStatus(result.status)
          setStatusReadError(null)
        })
        .catch(err => setStatusReadError(String(err)))
    }, 2000)
    return () => window.clearInterval(timer)
  }, [inspectionAvailability])

  const reviewEvidence = useCallback(async (
    evidence: InspectionEvidenceSummary,
    verdict: 'normal' | 'anomaly' | 'inconclusive',
  ) => {
    const evidenceId = evidence.evidence_id
    const selectedTaskId = activeTaskIdRef.current
    setReviewBusyId(evidenceId)
    setReviewErrors(current => ({ ...current, [evidenceId]: '' }))
    try {
      const response = await api.reviewInspectionEvidence(evidenceId, {
        verdict, note: reviewNotes[evidenceId] ?? evidence.review?.note ?? '',
      })
      setEvidenceItems(current => current.map(item => item.evidence_id === evidenceId ? response.evidence : item))
      showToast(text(locale, 'Review saved', '复核已保存'), 'success')
      const [evidenceRefresh, reportRefresh] = await Promise.allSettled([
        api.fetchInspectionEvidence(12),
        selectedTaskId ? api.fetchInspectionTaskReport(selectedTaskId) : Promise.resolve(null),
      ])
      if (evidenceRefresh.status === 'fulfilled') {
        setEvidenceItems(evidenceRefresh.value.evidence)
        setEvidenceIntegrityFailures(evidenceRefresh.value.integrity_failures)
        setEvidenceReadError(null)
      } else {
        setEvidenceReadError(String(evidenceRefresh.reason))
      }
      if (activeTaskIdRef.current === selectedTaskId) {
        if (reportRefresh.status === 'fulfilled') {
          setTaskReport(reportRefresh.value)
          setTaskReportError(null)
        } else {
          setTaskReport(null)
          setTaskReportError(inspectionReportErrorLabel(reportRefresh.reason, locale))
        }
      }
    } catch (err) {
      const message = err instanceof Error ? err.message : String(err)
      setReviewErrors(current => ({ ...current, [evidenceId]: message }))
      showToast(text(locale, 'Review was not saved: ', '复核未保存：') + message, 'error')
    } finally {
      setReviewBusyId(null)
    }
  }, [locale, reviewNotes, showToast])

  useEffect(() => {
    if (!activeTaskId) return
    const selectedTaskId = activeTaskId
    const refreshTask = () => {
      api.fetchInspectionTask(selectedTaskId)
        .then(snapshot => {
          if (activeTaskIdRef.current !== selectedTaskId) return
          setTaskStatus(snapshot)
          setTaskReadError(null)
          if (snapshot.found) {
            setInspectionTasks(current => upsertInspectionTask(current, snapshot))
          }
        })
        .catch(err => {
          if (activeTaskIdRef.current === selectedTaskId) setTaskReadError(String(err))
        })
    }
    refreshTask()
    const timer = window.setInterval(refreshTask, 2000)
    return () => window.clearInterval(timer)
  }, [activeTaskId])

  useEffect(() => {
    if (!activeTaskId) {
      setTaskReport(null)
      setTaskReportError(null)
      return
    }
    const selectedTaskId = activeTaskId
    const refreshReport = () => {
      api.fetchInspectionTaskReport(selectedTaskId)
        .then(report => {
          if (activeTaskIdRef.current !== selectedTaskId) return
          setTaskReport(report)
          setTaskReportError(null)
        })
        .catch(err => {
          if (activeTaskIdRef.current !== selectedTaskId) return
          setTaskReport(null)
          setTaskReportError(inspectionReportErrorLabel(err, locale))
        })
    }
    refreshReport()
    const timer = window.setInterval(refreshReport, 5000)
    return () => window.clearInterval(timer)
  }, [
    activeTaskId,
    locale,
    sseState.inspectionTaskEvent?.event_id,
  ])

  useEffect(() => {
    setManualTakeoverReleasedForTask(null)
  }, [activeTaskId])

  useEffect(() => {
    if (!statusText(taskStatus?.reason, '').includes('operator_takeover')) {
      setManualTakeoverReleasedForTask(null)
    }
  }, [taskStatus?.reason])

  useEffect(() => {
    const eventTaskId = sseState.inspectionTaskEvent?.data?.task_id
    if (!activeTaskId || eventTaskId !== activeTaskId) return
    api.fetchInspectionTask(activeTaskId)
      .then(snapshot => {
        if (activeTaskIdRef.current !== activeTaskId) return
        setTaskStatus(snapshot)
        setTaskReadError(null)
        if (snapshot.found) {
          setInspectionTasks(current => upsertInspectionTask(current, snapshot))
        }
      })
      .catch(err => {
        if (activeTaskIdRef.current === activeTaskId) setTaskReadError(String(err))
      })
  }, [
    activeTaskId,
    sseState.inspectionTaskEvent?.data?.task_id,
    sseState.inspectionTaskEvent?.event_id,
  ])

  useEffect(() => {
    if (inspectionAvailability !== 'available') return
    const refreshEvidence = () => {
      api.fetchInspectionEvidence(12)
        .then(result => {
          setEvidenceItems(result.evidence)
          setEvidenceIntegrityFailures(result.integrity_failures)
          setEvidenceReadError(null)
        })
        .catch(err => setEvidenceReadError(String(err)))
    }
    const timer = window.setInterval(refreshEvidence, 5000)
    return () => window.clearInterval(timer)
  }, [inspectionAvailability])

  const loadRouteIntoEditor = useCallback(async (route: InspectionRoute) => {
    setBusy('load')
    try {
      const detail = await api.fetchInspectionRoute(route.id, route.map_id || mapId || null)
      applyRouteToEditor(detail.route)
      setError(null)
    } catch (err) {
      const message = err instanceof Error ? err.message : String(err)
      setError(message)
      showToast(message, 'error')
    } finally {
      setBusy(null)
    }
  }, [applyRouteToEditor, mapId, showToast])

  const showSelectedRouteOnMap = useCallback(async () => {
    if (!selectedRoute || !onShowRoute || editorHasUnsavedChanges || !routeMatchesActiveMap) return
    setBusy('load')
    try {
      const detail = await api.fetchInspectionRoute(selectedRoute.id, selectedRoute.map_id || null)
      onShowRoute(detail.route)
      setError(null)
    } catch (err) {
      const message = err instanceof Error ? err.message : String(err)
      setError(message)
      showToast(message, 'error')
    } finally {
      setBusy(null)
    }
  }, [editorHasUnsavedChanges, onShowRoute, routeMatchesActiveMap, selectedRoute, showToast])

  const addLocation = useCallback((location: LocationEntry) => {
    setPoints(current => {
      if (current.some(point => point.id === location.name)) return current
      const standby = standbyPointIndex(current)
      return standby >= 0
        ? [...current.slice(0, standby), routePointFromLocation(location), current[standby]]
        : [...current, routePointFromLocation(location)]
    })
  }, [])

  const selectStandby = useCallback((name: string) => {
    setPoints(current => {
      const existingIndex = standbyPointIndex(current)
      const ordinary = existingIndex >= 0 ? current.slice(0, existingIndex) : current
      if (!name) return ordinary
      if (name === '__saved_standby__') return current
      const location = locations?.locations.find(item => item.name === name)
      if (!location) return current
      let id = existingIndex >= 0 ? current[existingIndex].id : 'route-standby'
      let suffix = 2
      while (ordinary.some(point => point.id === id)) id = `route-standby-${suffix++}`
      return [...ordinary, { ...routePointFromLocation(location), id, action: '', dwell: 0, enabled: true }]
    })
  }, [locations])

  const movePoint = useCallback((index: number, delta: -1 | 1) => {
    setPoints(current => {
      const nextIndex = index + delta
      const deviceCount = standbyPointIndex(current) >= 0 ? current.length - 1 : current.length
      if (nextIndex < 0 || nextIndex >= deviceCount) return current
      const next = [...current]
      const [item] = next.splice(index, 1)
      next.splice(nextIndex, 0, item)
      return next
    })
  }, [])

  const updatePoint = useCallback((index: number, patch: Partial<InspectionRoutePoint>) => {
    setPoints(current => current.map((point, i) => i === index ? { ...point, ...patch } : point))
  }, [])

  const saveRoute = useCallback(async () => {
    if (!routeId.trim() || !mapId.trim() || points.length === 0) {
      setError(text(locale, 'Route id, map id, and points are required', '需要路线 ID、地图 ID 和点位'))
      return
    }
    setBusy('save')
    try {
      const saved = await api.saveInspectionRoute({
        id: routeId.trim(),
        name: routeName.trim() || routeId.trim(),
        map_id: mapId.trim(),
        map_content_epoch: Math.max(0, Math.trunc(mapVersion)),
        revision: Math.max(1, Math.trunc(revision)),
        loop_count: Math.max(1, Math.trunc(loopCount)),
        loop_interval_s: loopInterval,
        failure_policy: failurePolicy,
        max_retries: Math.max(0, Math.trunc(maxRetries)),
        points,
      })
      setSelectedRouteId(saved.route.id)
      setSavedRevision(Number(saved.route.revision ?? 0))
      setRevision(nextRevision(saved.route))
      await load()
      showToast(text(locale, 'Inspection route saved', '巡检路线已保存'), 'success')
    } catch (err) {
      const message = err instanceof Error ? err.message : String(err)
      setError(message)
      showToast(message, 'error')
    } finally {
      setBusy(null)
    }
  }, [failurePolicy, load, locale, loopCount, loopInterval, mapId, mapVersion, maxRetries, points, revision, routeId, routeName, showToast])

  const runAction = useCallback(async (
    action: Exclude<BusyAction, 'load' | 'save' | 'delete' | null>,
    call: () => Promise<InspectionTaskCommandResponse>,
  ) => {
    setBusy(action)
    try {
      const receipt = await call()
      activeTaskIdRef.current = receipt.task_id
      setActiveTaskId(receipt.task_id)
      if (receipt.action === 'start') setManualTakeoverReleasedForTask(null)
      try {
        const snapshot = await api.fetchInspectionTask(receipt.task_id)
        if (activeTaskIdRef.current === receipt.task_id) {
          setTaskStatus(snapshot)
          if (snapshot.found) {
            setInspectionTasks(current => upsertInspectionTask(current, snapshot))
          }
        }
      } catch {
        setTaskStatus(null)
      }
      showToast(
        text(
          locale,
          'Inspection command submitted; awaiting native task confirmation',
          '巡检命令已提交，等待原生任务确认',
        ),
        'success',
      )
    } catch (err) {
      const message = err instanceof Error ? err.message : String(err)
      setError(message)
      showToast(message, 'error')
    } finally {
      setBusy(null)
    }
  }, [locale, showToast])

  const startRoute = useCallback(async () => {
    if (!selectedRouteId || savedRevision < 1) {
      throw new Error(text(locale, 'Save the selected route before starting', '请先保存所选巡检路线'))
    }
    return api.startInspectionTask(selectedRouteId, {
      map_id: selectedRoute?.map_id || mapId,
      revision: savedRevision,
    })
  }, [locale, mapId, savedRevision, selectedRoute, selectedRouteId])

  const resumeRoute = useCallback(async () => {
    if (!activeTaskId) {
      throw new Error(text(locale, 'No inspection task is selected', '未选择巡检任务'))
    }
    if (manualTakeoverReleaseRequired) {
      throw new Error(text(
        locale,
        'Release manual control before requesting this inspection task to resume',
        '请先释放人工接管，再请求恢复该巡检任务',
      ))
    }
    return api.resumeInspectionTask(activeTaskId)
  }, [activeTaskId, locale, manualTakeoverReleaseRequired])

  const releaseManualTakeover = useCallback(async () => {
    if (!activeTaskId) {
      throw new Error(text(locale, 'No inspection task is selected', '未选择巡检任务'))
    }
    setBusy('release')
    try {
      await api.resumeNavigation()
      setManualTakeoverReleasedForTask(activeTaskId)
      setError(null)
      showToast(
        text(
          locale,
          'Manual control release was submitted. It did not resume the inspection task; request task resume next.',
          '人工接管释放已提交；这不会恢复巡检任务，请再单独请求恢复任务。',
        ),
        'info',
      )
    } catch (err) {
      const message = err instanceof Error ? err.message : String(err)
      setError(message)
      showToast(message, 'error')
    } finally {
      setBusy(null)
    }
  }, [activeTaskId, locale, showToast])

  const pauseRoute = useCallback(async () => {
    if (!activeTaskId) {
      throw new Error(text(locale, 'No inspection task is selected', '未选择巡检任务'))
    }
    return api.pauseInspectionTask(activeTaskId)
  }, [activeTaskId, locale])

  const cancelRoute = useCallback(async () => {
    if (!activeTaskId) {
      throw new Error(text(locale, 'No inspection task is selected', '未选择巡检任务'))
    }
    return api.cancelInspectionTask(activeTaskId)
  }, [activeTaskId, locale])

  const deleteRoute = useCallback(async (route: InspectionRoute) => {
    setBusy('delete')
    try {
      await api.deleteInspectionRoute(route.id, route.map_id || mapId || null)
      await load()
      showToast(text(locale, 'Inspection route deleted', '巡检路线已删除'), 'success')
    } catch (err) {
      const message = err instanceof Error ? err.message : String(err)
      setError(message)
      showToast(message, 'error')
    } finally {
      setBusy(null)
    }
  }, [load, locale, mapId, showToast])

  return (
    <div className={styles.page} role="tabpanel" id="panel-inspection">
      <header className={styles.header}>
        <div>
          <span className={styles.kicker}>{text(locale, 'LINGTU / FIELD OPERATIONS', '灵途 / 现场任务')}</span>
          <h1 className={styles.title}>{text(locale, 'Inspection', '巡检任务')}</h1>
          <p className={styles.subtitle}>{text(locale,
            'Choose a route, follow the robot’s task state, and review the result.',
            '选择路线，跟随机器人的任务状态，复核巡检结果。',
          )}</p>
        </div>
        <div className={styles.actions}>
          <button type="button" className={styles.iconButton} onClick={() => void load()} disabled={busy !== null} title={text(locale, 'Refresh', '刷新')} aria-label={text(locale, 'Refresh inspection data', '刷新巡检数据')}>
            <RefreshCcw size={16} />
          </button>
        </div>
      </header>

      <MeasurementInspector locale={locale} />

      {busy === 'load' && (
        <div className={styles.readNotice} role="status">{text(locale, 'Refreshing inspection data…', '正在读取巡检数据…')}</div>
      )}
      {error && (
        <div className={styles.alert} role="status">
          <XCircle size={16} />
          <span>{error}</span>
        </div>
      )}
      {(taskReadError || statusReadError || evidenceReadError) && (
        <div className={styles.capabilityNotice} role="status">
          <AlertTriangle size={16} />
          <span>{[
            taskReadError && text(locale, `Task refresh failed; retained state may be stale: ${taskReadError}`, `任务读取失败，保留的状态可能已过期：${taskReadError}`),
            statusReadError && text(locale, `Capture readiness refresh failed: ${statusReadError}`, `取证就绪状态读取失败：${statusReadError}`),
            evidenceReadError && text(locale, `Evidence refresh failed; showing retained evidence: ${evidenceReadError}`, `证据读取失败，显示上次结果：${evidenceReadError}`),
          ].filter(Boolean).join(' · ')}</span>
        </div>
      )}

      <section className={styles.runPanel} aria-label={text(locale, 'Inspection controls', '巡检运行')}>
        <div className={styles.missionHeading}>
          <span>{text(locale, 'MISSION CONTROL', '任务控制')}</span>
          <span className={styles.missionSignal}>{activeTaskId
            ? taskReadError
              ? text(locale, 'State may be stale', '状态可能已过期')
              : taskStatus
                ? taskTruthLabel(taskStatus, locale)
                : text(locale, 'Awaiting task confirmation', '等待任务确认')
            : text(locale, 'Awaiting selection', '等待选择任务')}</span>
        </div>
        <div className={styles.routeSelection}>
          <label>
            <span>{text(locale, 'Route', '路线')}</span>
            <select aria-label={text(locale, 'Saved routes', '已保存路线')} value={selectedRouteId ?? ''}
              disabled={busy !== null || routes.length === 0}
              onChange={event => {
                const route = routes.find(item => item.id === event.target.value)
                if (route) void loadRouteIntoEditor(route)
              }}>
              {!selectedRouteId && <option value="">{text(locale, 'Choose a route', '选择路线')}</option>}
              {routes.map(route => <option key={route.id} value={route.id}>{route.name || route.id}</option>)}
            </select>
          </label>
          <div className={styles.routeContext}>
            <span className={styles.routeMeta}>{selectedRoute
              ? `${selectedRoute.map_id} · v${selectedRoute.map_content_epoch ?? '--'} · ${selectedRoute.point_count ?? selectedRoute.points.length} ${text(locale, 'points', '个点位')}`
              : observe ? text(locale, 'No saved route', '暂无已保存路线')
                : text(locale, 'Save a route in the editor below', '在下方编辑并保存路线')}</span>
            {onShowRoute && selectedRoute && <>
              <button
                type="button"
                className={styles.showRouteButton}
                onClick={() => void showSelectedRouteOnMap()}
                disabled={busy !== null || editorHasUnsavedChanges || !routeMatchesActiveMap}
                aria-describedby="inspection-route-map-note"
              >
                <MapPin size={15} aria-hidden="true" />
                {observe
                  ? text(locale, 'View on map', '在地图中查看')
                  : text(locale, 'View / edit on map', '在地图中查看/编辑')}
              </button>
              <small id="inspection-route-map-note" className={styles.routeMapNote}>
                {editorHasUnsavedChanges
                  ? text(locale, 'Save your edits below before opening this route on the map.', '请先保存下方编辑，再打开地图中的路线。')
                  : !routeMatchesActiveMap
                    ? text(locale, 'Select this route’s map in map tools first.', '请先在地图工具中切换到该路线绑定的地图。')
                    : text(locale, 'Opens the saved route without starting a task.', '打开已保存路线，不会启动任务。')}
              </small>
            </>}
          </div>
        </div>
        <div className={styles.taskSelection}>
          <label>
            <span>{text(locale, 'Task', '任务')}</span>
            <select aria-label={text(locale, 'Recent tasks', '最近任务')} value={activeTaskId ?? ''}
              disabled={busy !== null}
              onChange={event => selectInspectionTask(inspectionTasks.find(task => task.task_id === event.target.value) ?? null)}>
              <option value="">{text(locale, 'No task selected', '未选择任务')}</option>
              {activeTaskId && !inspectionTasks.some(task => task.task_id === activeTaskId) &&
                <option value={activeTaskId}>{text(locale, 'Awaiting task confirmation', '等待任务确认')}</option>}
              {inspectionTasks.map(task => <option key={task.task_id} value={task.task_id}>
                {`${routes.find(route => route.id === task.identity.route_id)?.name || task.identity.route_id || text(locale, 'Route', '路线')} · ${inspectionTaskLabel(task, locale)} · ${taskEventTimestamp({ ts: task.updated_at }, locale)}`}
              </option>)}
            </select>
          </label>
          <div className={styles.taskState}>
            <span>{text(locale, 'Task state', '任务状态')}</span>
            <strong>{taskStatus ? inspectionTaskLabel(taskStatus, locale)
              : activeTaskId || taskReadError || error || inspectionAvailability !== 'available'
                ? text(locale, 'Unconfirmed', '待确认') : text(locale, 'Not started', '未开始')}</strong>
            {activeTaskId && <small role="status">{taskReadError ? text(locale, 'State may be stale', '状态可能已过期') : taskTruthLabel(taskStatus, locale)}</small>}
          </div>
          {activeTaskId && <div className={styles.taskProgress}>
            <span>{text(locale, 'Completed points', '已完成点位')}</span>
            <strong>{progressLabel}</strong>
            {taskProgress?.known && taskPointNumber !== null && <small>
              {text(locale, 'Current point', '当前点')} · {taskPointNumber} · {statusText(taskProgress.current_point_id)}
            </small>}
            {taskProgress?.known && taskProgress.loop_number != null && <small>
              {text(locale, 'Loop', '当前循环')} · {taskProgress.loop_number}
            </small>}
          </div>}
        </div>
        {taskStatus && ['FAILED', 'PAUSED'].includes(taskStatus.current_state) && taskStatus.reason &&
          <p className={styles.runReason} role="status">{taskStatus.reason.startsWith('restart_')
            ? text(locale, 'Progress was retained. Continue when the robot is ready.', '已保留任务进度，机器人就绪后可继续。')
            : taskStatus.reason}</p>}
        {!observe && <div className={styles.runControls}>
            {!taskStillOpen && <button
              type="button"
              className={styles.primaryButton}
              onClick={() => void runAction('start', startRoute)}
              disabled={inspectionAvailability !== 'available' || busy !== null || !selectedRouteId || savedRevision < 1 || startBlockedByEvidenceWorker || taskStillOpen || Boolean(taskReadError || statusReadError)}
              title={startBlockedByEvidenceWorker
                ? (!evidenceWorkerReady
                    ? text(locale, 'Evidence capture must be ready before starting routes with capture actions.', '带取证动作的路线启动前，取证服务必须就绪。')
                    : text(
                        locale,
                        `Unsupported capture actions: ${unsupportedRouteActions.join(', ')}`,
                        `不支持的取证动作：${unsupportedRouteActions.join('、')}`,
                      ))
                : undefined}
            >
              <Play size={15} />{text(locale, 'Start inspection', '开始巡检')}
            </button>}
            {activeTaskId && taskStatus?.terminal !== true && <>
            {taskStatus?.current_state !== 'PAUSED' && <button type="button" onClick={() => void runAction('pause', pauseRoute)} disabled={inspectionAvailability !== 'available' || busy !== null || !taskStatus?.can_pause}>
              <Pause size={15} />{text(locale, 'Pause', '暂停')}
            </button>}
            {manualTakeoverReleaseRequired && (
              <button type="button" onClick={() => void releaseManualTakeover()} disabled={inspectionAvailability !== 'available' || busy !== null}>
                <Play size={15} />{text(locale, 'Release manual control', '释放人工接管')}
              </button>
            )}
            {taskStatus?.current_state === 'PAUSED' && <button type="button" onClick={() => void runAction('resume', resumeRoute)} disabled={inspectionAvailability !== 'available' || busy !== null || !taskStatus?.can_resume || manualTakeoverReleaseRequired}>
              <RotateCcw size={15} />{text(locale, 'Resume', '恢复')}
            </button>}
            <button type="button" className={styles.cancelButton} onClick={() => void runAction('cancel', cancelRoute)} disabled={inspectionAvailability !== 'available' || busy !== null || !taskStatus?.can_cancel}>
              <Square size={15} />{text(locale, 'Cancel task', '取消任务')}
            </button>
            </>}
          </div>}
        <div className={styles.missionOutcome} aria-label={text(locale, 'Task report at a glance', '任务报告概览')}>
          <div className={styles.outcomeHeading}>
            <ClipboardCheck size={17} aria-hidden="true" />
            <span>{text(locale, 'TASK REPORT', '任务报告')}</span>
          </div>
          <div className={styles.outcomeFacts}>
            <div>
              <span>{text(locale, 'Inspection result', '巡检结果')}</span>
              <strong>{!activeTaskId
                ? text(locale, 'No task selected', '未选择任务')
                : taskReportError
                ? text(locale, 'Unavailable', '暂不可用')
                : taskReport
                  ? inspectionResultLabel(taskReport.inspection_result.status, locale)
                  : text(locale, 'Awaiting report', '等待报告')}</strong>
            </div>
            <div>
              <span>{text(locale, 'Record completeness', '资料完整性')}</span>
              <strong>{!activeTaskId
                ? text(locale, 'No task selected', '未选择任务')
                : taskReportError
                ? text(locale, 'Unavailable', '暂不可用')
                : taskReport
                  ? inspectionAcceptanceLabel(taskReport.acceptance, locale)
                  : text(locale, 'Awaiting report', '等待报告')}</strong>
            </div>
            <div>
              <span>{text(locale, 'Verified evidence', '已验证证据')}</span>
              <strong>{taskReport
                ? `${taskReport.coverage.verified_evidence}/${taskReport.coverage.required_evidence}`
                : '--'}</strong>
            </div>
          </div>
          {taskReport && <div className={styles.coverageSummary} aria-label={text(locale, 'Point coverage', '点位完成情况')}>
            <span>{text(locale, 'Completed', '完成')} {taskReport.coverage.completed_points}/{taskReport.coverage.required_points}</span>
            <span>{text(locale, 'Skipped', '跳过')} {taskReport.coverage.skipped_points}</span>
            <span>{text(locale, 'Not visited', '未执行')} {taskReport.coverage.not_visited_points}</span>
            {(taskReport.coverage.failed_points > 0 || taskReport.coverage.cancelled_points > 0) &&
              <span>{text(locale, 'Interrupted', '中断')} {taskReport.coverage.failed_points + taskReport.coverage.cancelled_points}</span>}
          </div>}
          {taskReport && <div className={styles.coverageSummary} aria-label={text(locale, 'Inspection coverage', '检查完成情况')}>
            <span>{text(locale, 'Normal', '正常')} {taskReport.inspection_result.normal_points}</span>
            <span>{text(locale, 'Anomaly', '异常')} {taskReport.inspection_result.anomaly_points}</span>
            <span>{text(locale, 'Needs review', '待复核')} {taskReport.inspection_result.unreviewed_points}</span>
            <span>{text(locale, 'Not readable', '未读到')} {taskReport.inspection_result.inconclusive_points}</span>
            <span>{text(locale, 'Not inspected', '未检查')} {taskReport.inspection_result.not_inspected_points}</span>
          </div>}
        </div>
      </section>

      {activeTaskId && <details className={styles.reportPanel}>
        <summary>
          <span>{text(locale, 'Inspection result', '巡检结果')}</span>
          <span>{taskReportError ? text(locale, 'Unavailable', '暂不可用') : inspectionResultLabel(taskReport?.inspection_result.status ?? '', locale)}</span>
        </summary>
        <div className={styles.statusBand}>
          <div>
            <span>{text(locale, 'Execution outcome', '执行结果')}</span>
            <strong>{statusText(taskReport?.execution.state, taskStatus?.current_state ?? '--')}</strong>
          </div>
          <div>
            <span>{text(locale, 'Inspection result', '巡检结果')}</span>
            <strong>{inspectionResultLabel(taskReport?.inspection_result.status ?? '', locale)}</strong>
          </div>
          <div>
            <span>{text(locale, 'Record completeness', '资料完整性')}</span>
            <strong>{inspectionAcceptanceLabel(taskReport?.acceptance ?? 'UNKNOWN', locale)}</strong>
          </div>
          <div>
            <span>{text(locale, 'Verified evidence', '已验证证据')}</span>
            <strong>
              {taskReport
                ? String(taskReport.coverage.verified_evidence)
                  + '/'
                  + String(taskReport.coverage.required_evidence)
                : '--'}
            </strong>
          </div>
        </div>
        {taskReportError && (
          <div className={styles.reportError} role="status">
            <AlertTriangle size={16} />
            <span>{text(locale, 'Report unavailable: ', '报告暂不可用：') + taskReportError}</span>
          </div>
        )}
        {taskReport && taskReport.issues.length > 0 && (
          <ul className={styles.reportIssues}>
            {taskReport.issues.map((issue, index) => (
              <li key={issue.code + ':' + String(issue.loop_index ?? '') + ':' + String(issue.point_index ?? index)}>
                <AlertTriangle size={15} />
                <span>{inspectionReportIssueLabel(issue, locale)}</span>
              </li>
            ))}
          </ul>
        )}
        {taskReport && (
          <div className={styles.reportPointSection}>
            <div className={styles.reportPointHeader}>
              <strong>{text(locale, 'Point results', '点位结果')}</strong>
              <span>{taskReport.points.length}</span>
            </div>
            <ol className={styles.reportPointList}>
              {taskReport.points.map(point => (
                <li
                  key={
                    String(point.loop_index)
                    + ':'
                    + String(point.point_index)
                    + ':'
                    + point.point_id
                  }
                >
                  <div>
                    <small>
                      {text(locale, 'Loop ', '第 ')
                        + String(point.loop_index + 1)
                        + text(locale, ' · point ', ' 轮 · 点位 ')
                        + String(point.point_index + 1)}
                    </small>
                    <strong>{statusText(point.point_id)}</strong>
                  </div>
                  <span>{inspectionActionLabel(point.action, locale)}</span>
                  <em>{inspectionPointResultLabel(point.status, locale)}</em>
                  <small>{inspectionResultLabel(point.inspection_result, locale)}</small>
                  {point.evidence_id && (
                    <details className={styles.inlineDetails}>
                      <summary>{text(locale, 'Evidence ID', '证据 ID')}</summary>
                      <code>{point.evidence_id}</code>
                    </details>
                  )}
                </li>
              ))}
            </ol>
          </div>
        )}
      </details>}

      {manualTakeoverReleaseRequired && (
        <div className={styles.taskSafetyNotice} role="status">
          <AlertTriangle size={16} />
          <span>{text(
            locale,
            'Release manual control, then request task resume separately.',
            '先释放人工接管，再单独恢复任务。',
          )}</span>
        </div>
      )}

      {taskStatus?.delivery.history_complete === false && (
        <div className={styles.taskSafetyNotice} role="status">
          <AlertTriangle size={16} />
          <span>{text(
            locale,
            'Some task records are missing; the report needs review.',
            '部分历史记录缺失，报告需复核。',
          )}</span>
        </div>
      )}

      <details className={styles.evidencePanel}>
        <summary>
          <span>{text(locale, 'Recent evidence', '最近证据')}</span>
          <span>{evidenceIntegrityFailures > 0
            ? text(locale, `${evidenceIntegrityFailures} invalid hidden`, `已隐藏 ${evidenceIntegrityFailures} 条异常证据`)
            : text(locale, `${evidenceItems.length} verified`, `${evidenceItems.length} 条已验证`)}</span>
        </summary>
        <div className={styles.evidenceGrid}>
          {evidenceItems.map(evidence => {
            const request = evidence.request
            const verdict = evidence.review?.verdict
              ?? (['unavailable', 'not_applicable'].includes(evidence.analysis.support ?? '')
                ? 'needs_review' : statusText(evidence.analysis.verdict, 'needs_review'))
            const verdictLabel = inspectionVerdictLabel(verdict, locale)
            const rgbUrl = api.inspectionEvidenceArtifactUrl(evidence.evidence_id, 'rgb')
            return (
              <article key={evidence.evidence_id} className={styles.evidenceCard} aria-busy={reviewBusyId === evidence.evidence_id}>
                {hasEvidenceArtifact(evidence, 'rgb') ? (
                  <a
                    className={styles.evidencePreview}
                    href={rgbUrl}
                    target="_blank"
                    rel="noreferrer"
                    title={text(locale, 'Open full image', '打开原图')}
                  >
                    <img src={rgbUrl} alt={`${request.route_id ?? 'route'} · ${text(locale, 'Point', '点位')} ${(request.point_index ?? 0) + 1}`} loading="lazy" />
                  </a>
                ) : (
                  <div className={styles.evidencePlaceholder}>{text(locale, 'No RGB', '无 RGB')}</div>
                )}
                <div className={styles.evidenceBody}>
                  <div className={styles.evidenceIdentity}>
                    <strong>{statusText(request.route_id)}</strong>
                    <span>{text(locale, 'Point', '点位')} {(request.point_index ?? 0) + 1}</span>
                  </div>
                  <div className={styles.evidenceAction}>{inspectionActionLabel(statusText(request.action, ''), locale)}</div>
                  <div className={styles.evidenceMeta}>
                    <span className={['violation', 'anomaly'].includes(verdict) ? styles.verdictAlert : styles.verdictNeutral} title={verdict}>
                      {verdictLabel}
                    </span>
                    <time>{evidenceTimestamp(evidence, locale)}</time>
                  </div>
                  {!observe && <fieldset className={styles.evidenceReview} disabled={reviewBusyId !== null}>
                    <legend>{text(locale, 'Manual review', '人工复核')}</legend>
                    <label>
                      <span className={styles.reviewNoteLabel}>{text(locale, 'Optional note', '说明（可选）')}</span>
                      <textarea rows={2} maxLength={2000}
                        aria-label={text(locale, 'Review note for point ', '点位复核说明 ') + String((request.point_index ?? 0) + 1)}
                        placeholder={text(locale, 'Optional note', '说明（可选）')}
                        value={reviewNotes[evidence.evidence_id] ?? evidence.review?.note ?? ''}
                        onChange={event => setReviewNotes(current => ({ ...current, [evidence.evidence_id]: event.target.value }))} />
                    </label>
                    <div className={styles.reviewActions}>
                      {(['normal', 'anomaly', 'inconclusive'] as const).map(value => <button key={value} type="button"
                        aria-pressed={evidence.review?.verdict === value}
                        onClick={() => void reviewEvidence(evidence, value)}>
                        {inspectionVerdictLabel(value, locale)}
                      </button>)}
                    </div>
                    {reviewBusyId === evidence.evidence_id && <span role="status">{text(locale, 'Saving review…', '正在保存复核…')}</span>}
                    {reviewErrors[evidence.evidence_id] && <p className={styles.reviewError} role="alert">
                      {text(locale, 'Review was not saved: ', '复核未保存：') + reviewErrors[evidence.evidence_id]}
                    </p>}
                  </fieldset>}
                  {evidence.review && <small className={styles.reviewStamp}>
                    {text(locale, 'Manually reviewed · ', '人工复核 · ')}
                    {new Date(evidence.review.reviewed_at).toLocaleString(locale === 'zh' ? 'zh-CN' : 'en-US', { hour12: false })}
                  </small>}
                  <details className={styles.inlineDetails}>
                    <summary>{text(locale, 'Evidence details', '证据详情')}</summary>
                    <code>{evidence.evidence_id}</code>
                    <div className={styles.evidenceLinks}>
                      {hasEvidenceArtifact(evidence, 'pose') && (
                        <a href={api.inspectionEvidenceArtifactUrl(evidence.evidence_id, 'pose')} target="_blank" rel="noreferrer">
                          {text(locale, 'Pose', '位姿')}
                        </a>
                      )}
                      {hasEvidenceArtifact(evidence, 'detections') && (
                        <a href={api.inspectionEvidenceArtifactUrl(evidence.evidence_id, 'detections')} target="_blank" rel="noreferrer">
                          {text(locale, 'Detections', '检测')}
                        </a>
                      )}
                    </div>
                  </details>
                </div>
              </article>
            )
          })}
          {evidenceItems.length === 0 && !evidenceReadError && !error && busy !== 'load' && inspectionAvailability === 'available' && (
            <div className={styles.empty}>{text(locale, 'No verified evidence yet', '暂无已验证证据')}</div>
          )}
        </div>
      </details>

      {inspectionAvailability === 'unavailable' && (
        <div className={styles.capabilityNotice}>
          <AlertTriangle size={16} />
          <span>{text(
            locale,
            'Inspection service is unavailable; route editing and task controls are disabled.',
            '当前未提供巡检服务，暂不能编辑或运行路线。',
          )}</span>
        </div>
      )}

      {startBlockedByEvidenceWorker && (
        <div className={styles.capabilityNotice}>
          <AlertTriangle size={16} />
          <span>{!evidenceWorkerReady
            ? text(
                locale,
                `This route contains capture actions, but evidence capture is unavailable: ${statusText(evidenceWorker.reason)}`,
                `该路线包含取证动作，但取证服务不可用：${statusText(evidenceWorker.reason)}`,
              )
            : text(
                locale,
                `This route contains unsupported capture actions: ${unsupportedRouteActions.join(', ')}`,
                `该路线包含当前不支持的取证动作：${unsupportedRouteActions.join('、')}`,
              )}</span>
        </div>
      )}

      {!observe && <details className={styles.editorPane}>
        <summary>{text(locale, 'Route editor', '编辑路线')}</summary>
          <div className={styles.editorHeader}>
            <button type="button" className={styles.primaryButton} onClick={() => void saveRoute()} disabled={inspectionAvailability !== 'available' || busy !== null || points.length === 0}>
              <Save size={15} />{text(locale, 'Save route', '保存路线')}
            </button>
            <span>{text(locale, 'Planned points', '计划点位')} {enabledPointCount}/{points.length}</span>
          </div>
          <div className={styles.formGrid}>
            <label>
              <span>{text(locale, 'Name', '名称')}</span>
              <input value={routeName} onChange={event => setRouteName(event.target.value)} />
            </label>
            <label>
              <span>{text(locale, 'Loop count', '循环次数')}</span>
              <input type="number" min="1" value={loopCount} onChange={event => setLoopCount(Number(event.target.value))} />
            </label>
            <label>
              <span>{text(locale, 'Between rounds (seconds)', '循环间隔（秒）')}</span>
              <input type="number" min="0" step="1" value={loopInterval} onChange={event => setLoopInterval(Number(event.target.value))} />
            </label>
            <label>
              <span>{text(locale, 'Failure policy', '失败策略')}</span>
              <select value={failurePolicy} onChange={event => setFailurePolicy(event.target.value as InspectionFailurePolicy)}>
                <option value="stop">{text(locale, 'Stop mission', '停止任务')}</option>
                <option value="retry">{text(locale, 'Retry point', '重试点位')}</option>
                <option value="skip">{text(locale, 'Skip point', '跳过点位')}</option>
              </select>
            </label>
            <label>
              <span>{text(locale, 'Retry limit', '最大重试')}</span>
              <input type="number" min="0" value={maxRetries} onChange={event => setMaxRetries(Number(event.target.value))} />
            </label>
          </div>

          <details className={styles.details}>
            <summary>{text(locale, 'Route and map binding', '路线标识与地图绑定')}</summary>
            <div className={styles.formGrid}>
              <label>
                <span>{text(locale, 'Route ID', '路线 ID')}</span>
                <input value={routeId} onChange={event => setRouteId(event.target.value)} />
              </label>
              <label>
                <span>{text(locale, 'Map ID', '地图 ID')}</span>
                <input value={mapId} onChange={event => setMapId(event.target.value)} />
              </label>
              <label>
                <span>{text(locale, 'Map version', '地图版本')}</span>
                <input type="number" min="0" value={mapVersion} onChange={event => setMapVersion(Number(event.target.value))} />
              </label>
              <label>
                <span>{text(locale, 'Revision', '修订')}</span>
                <input type="number" min="1" value={revision} onChange={event => setRevision(Number(event.target.value))} />
              </label>
            </div>
          </details>

          <div className={styles.builder}>
            <section className={styles.locationPane}>
              <div className={styles.sectionTitle}>{text(locale, 'Inspection locations', '巡检位置')}</div>
              <div className={styles.locationList}>
                {(locations?.locations ?? []).map(location => {
                  const bound = locationIsBoundToRoute(location, mapId, mapVersion)
                  const binding = locationBindingLabel(location)
                  return (
                    <button
                      key={location.name}
                      type="button"
                      onClick={() => addLocation(location)}
                      disabled={!bound}
                      title={bound
                        ? text(locale, `Add ${location.name}`, `添加 ${location.name}`)
                        : text(
                            locale,
                            `Location is not bound to ${mapId} v${mapVersion}: ${binding}`,
                            `点位未绑定到 ${mapId} v${mapVersion}：${binding}`,
                          )}
                    >
                      <Plus size={14} />
                      <MapPin size={14} />
                      <span>{location.name}</span>
                      <small>{binding}</small>
                    </button>
                  )
                })}
                {(locations?.locations ?? []).length === 0 && <div className={styles.empty}>{text(locale, 'No saved locations', '暂无位置')}</div>}
              </div>
            </section>

            <section className={styles.pointPane}>
              <div className={styles.sectionTitle}>{text(locale, 'Device inspection points', '设备检查点')}</div>
              <div className={styles.pointRows}>
                {devicePoints.map((point, index) => (
                  <div key={routePointKey(point, index)} className={styles.pointRow}>
                    <div className={styles.orderButtons}>
                      <button type="button" onClick={() => movePoint(index, -1)} disabled={index === 0} title={text(locale, 'Move up', '上移')}><ArrowUp size={14} /></button>
                      <button type="button" onClick={() => movePoint(index, 1)} disabled={index === devicePoints.length - 1} title={text(locale, 'Move down', '下移')}><ArrowDown size={14} /></button>
                    </div>
                    <strong>{index + 1}. {point.id}</strong>
                    <label>
                      <span>{text(locale, 'Dwell (s)', '停留（秒）')}</span>
                      <input type="number" min="0" step="0.5" value={point.dwell} onChange={event => updatePoint(index, { dwell: Number(event.target.value) })} />
                    </label>
                    <label>
                      <span>{text(locale, 'Arrival radius (m)', '到点半径（米）')}</span>
                      <input type="number" min="0" step="0.05" value={point.tolerance} onChange={event => updatePoint(index, { tolerance: Number(event.target.value) })} />
                    </label>
                    <label>
                      <span>{text(locale, 'Evidence action', '取证动作')}</span>
                      {evidenceWorkerReady ? (
                        <select
                          value={point.action}
                          onChange={event => updatePoint(index, { action: event.target.value })}
                          title={text(locale, 'Evidence action at this point', '该点位的取证动作')}
                        >
                          {point.action && !supportedEvidenceActions.has(point.action) && (
                            <option value={point.action} disabled>
                              {text(
                                locale,
                                `Unsupported action: ${point.action}`,
                                `不支持的动作：${point.action}`,
                              )}
                            </option>
                          )}
                          {availableActionOptions.map(action => (
                            <option key={action || 'none'} value={action}>
                              {inspectionActionLabel(action, locale)}
                            </option>
                          ))}
                        </select>
                      ) : (
                        <input
                          value={inspectionActionLabel(point.action, locale)}
                          disabled
                          title={text(
                            locale,
                            'Capture actions are disabled until evidence capture is ready.',
                            '取证服务就绪后才能配置点位动作。',
                          )}
                        />
                      )}
                    </label>
                    <label className={styles.toggle}>
                      <input type="checkbox" checked={point.enabled} onChange={event => updatePoint(index, { enabled: event.target.checked })} />
                      <span>{text(locale, 'Enabled', '启用')}</span>
                    </label>
                    <button type="button" className={styles.iconButton} onClick={() => setPoints(current => current.filter((_, i) => i !== index))} title={text(locale, 'Remove', '移除')}>
                      <Trash2 size={15} />
                    </button>
                  </div>
                ))}
                {devicePoints.length === 0 && <div className={styles.empty}>{text(locale, 'Select locations to build a route', '选择位置来生成路线')}</div>}
              </div>
              <section className={styles.standbySection} aria-label={text(locale, 'End each round at standby', '每轮回待机')}>
                <label>
                  <span>{text(locale, 'End each round at standby', '每轮最后回待机')}</span>
                  <select aria-label={text(locale, 'Standby location', '待机点')}
                    value={standbyPoint ? matchedStandbyLocation?.name ?? '__saved_standby__' : ''}
                    onChange={event => selectStandby(event.target.value)}>
                    <option value="">{text(locale, 'No standby destination', '不设待机点')}</option>
                    {standbyPoint && !matchedStandbyLocation && <option value="__saved_standby__">{text(locale, 'Saved standby point', '已保存待机点')}</option>}
                    {standbyLocations.map(location => <option key={location.name} value={location.name}>{location.name}</option>)}
                  </select>
                </label>
                {standbyPoint && <div className={styles.standbyPoint}>
                  <strong>{points.length}. {text(locale, 'Return to standby · no capture', '回到待机 · 不拍照')}</strong>
                  <span>X {standbyPoint.x.toFixed(2)} · Y {standbyPoint.y.toFixed(2)} · Z {standbyPoint.z.toFixed(2)} m</span>
                  <label className={styles.toggle}>
                    <input type="checkbox" checked={standbyPoint.enabled} onChange={event => updatePoint(standbyIndex, { enabled: event.target.checked })} />
                    <span>{text(locale, 'Enabled', '启用')}</span>
                  </label>
                </div>}
                <small>{text(locale,
                  'Each round returns here before the interval. Pause for manual charging; continue when ready.',
                  '每轮结束先回到此处，再等待循环间隔。人工充电时暂停，完成后继续。')}</small>
              </section>
            </section>
          </div>

          {selectedRoute && (
            <button type="button" className={styles.deleteButton} onClick={() => void deleteRoute(selectedRoute)} disabled={busy !== null}>
              <Trash2 size={15} />
              {text(locale, 'Delete selected route', '删除所选路线')}
            </button>
          )}
      </details>}
      <details className={styles.details}>
        <summary>{text(locale, 'System details', '系统详情')}</summary>
      <section className={styles.statusBand}>
        <div>
          <span>{text(locale, 'Task state', '任务状态')}</span>
          <strong>{statusText(taskStatus?.current_state,
            busy === 'load'
              ? text(locale, 'Loading', '读取中')
              : taskReadError || error || inspectionAvailability !== 'available'
                ? text(locale, 'Unconfirmed', '未确认')
                : text(locale, 'No active task', '无当前任务'),
          )}</strong>
        </div>
        <div>
          <span>{text(locale, 'Task truth', '任务事实')}</span>
          <strong>{taskReadError ? text(locale, 'State may be stale', '状态可能已过期') : taskTruthLabel(taskStatus, locale)}</strong>
        </div>
        <div>
          <span>{text(locale, 'Completed points', '已完成点位')}</span>
          <strong>{progressLabel}</strong>
        </div>
        <div>
          <span>{text(locale, 'Current point', '当前点')}</span>
          <strong>{statusText(taskProgress?.current_point_id)}</strong>
        </div>
        <div className={styles.currentReason}>
          <span>{text(locale, 'Current reason', '当前原因')}</span>
          <strong>{statusText(taskStatus?.reason)}</strong>
        </div>
        <div>
          <span>{text(locale, 'Point action', '点位动作')}</span>
          <strong>{inspectionActionLabel(statusText(taskProgress?.action, ''), locale)}</strong>
        </div>
        <div>
          <span>{text(locale, 'Capture readiness', '取证就绪')}</span>
          <strong className={evidenceWorkerReady && !statusReadError ? styles.workerReady : styles.workerUnavailable}>
            {statusReadError
              ? text(locale, 'Unconfirmed', '未确认')
              : evidenceWorkerReady
              ? text(locale, 'Ready', '就绪')
              : text(locale, 'Unavailable', '不可用')}
          </strong>
        </div>
      </section>

      <details className={styles.details}>
        <summary>{text(locale, 'Task details', '任务详情')}</summary>
        <div className={styles.statusBand}>
          <div>
            <span>{text(locale, 'Task ID', '任务 ID')}</span>
            <strong>{statusText(activeTaskId)}</strong>
          </div>
          <div>
            <span>{text(locale, 'State source', '状态来源')}</span>
            <strong>{statusText(taskStatus?.state_source)}</strong>
          </div>
          <div>
            <span>{text(locale, 'Route ID', '路线 ID')}</span>
            <strong>{statusText(taskStatus?.identity.route_id ?? selectedRouteId)}</strong>
          </div>
          <div>
            <span>{text(locale, 'Route revision', '路线修订')}</span>
            <strong>{statusText(taskStatus?.identity.route_revision)}</strong>
          </div>
          <div>
            <span>{text(locale, 'Map ID', '地图 ID')}</span>
            <strong>{statusText(taskStatus?.identity.map_id)}</strong>
          </div>
          <div>
            <span>{text(locale, 'Map epoch', '地图纪元')}</span>
            <strong>{statusText(taskStatus?.identity.map_content_epoch)}</strong>
          </div>
          <div>
            <span>{text(locale, 'Latest evidence ID', '最近证据 ID')}</span>
            <strong>{statusText(taskProgress?.evidence_id)}</strong>
          </div>
        </div>
      </details>

      <details className={styles.details}>
        <summary>{text(locale, 'Native task events', '原生任务事件')} · {taskTimeline.length}</summary>
        <p className={styles.detailNote}>{text(
          locale,
          'Execution facts below come only from the native endpoint; a command receipt is not a state change.',
          '以下执行事实只来自原生端；命令回执不代表状态已改变。',
        )}</p>
        <ol className={styles.taskEventList}>
          {taskTimeline.map(event => (
            <li key={statusText(event.event_id, `${statusText(event.boot_id)}:${statusText(event.event_sequence)}`)}>
              <strong>{statusText(event.state_name)}</strong>
              <span>{statusText(event.kind_name)}</span>
              <time>{taskEventTimestamp(event, locale)}</time>
              <small>{statusText(event.reason, text(locale, 'No reason reported', '未报告原因'))}</small>
              {statusText(event.evidence_id, '') !== '' && <em>{statusText(event.evidence_id)}</em>}
            </li>
          ))}
          {taskStatus && taskTimeline.length === 0 && (
            <li className={styles.taskEventEmpty}>{text(
              locale,
              'No native event yet. Submission acceptance does not mean execution has started.',
              '尚未收到原生事件。提交已受理不代表任务已开始执行。',
            )}</li>
          )}
          {!taskStatus && (
            <li className={styles.taskEventEmpty}>{text(
              locale,
              'Select a retained task to inspect its native event history.',
              '请选择一个保留任务以查看其原生事件历史。',
            )}</li>
          )}
        </ol>
      </details>

      </details>
    </div>
  )
}
