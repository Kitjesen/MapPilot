import { useCallback, useEffect, useMemo, useState } from 'react'
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
} from 'lucide-react'
import type {
  InspectionFailurePolicy,
  InspectionEvidenceSummary,
  InspectionEvidenceWorkerStatus,
  InspectionRoute,
  InspectionRoutePoint,
  LocationEntry,
  LocationsResponse,
  SSEState,
} from '../types'
import { text, type Locale } from '../i18n'
import * as api from '../services/api'
import styles from './InspectionWorkbench.module.css'

interface InspectionWorkbenchProps {
  sseState: SSEState
  showToast: (message: string, type?: 'success' | 'error' | 'info') => void
  locale: Locale
}

type BusyAction = 'load' | 'save' | 'start' | 'pause' | 'resume' | 'cancel' | 'delete' | null
type InspectionAvailability = 'checking' | 'available' | 'unavailable'

const INSPECTION_ACTION_OPTIONS = [
  '',
  'capture:overview',
  'capture:parking',
  'capture:bin_full',
  'capture:plate_ocr',
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
    action: '',
    enabled: true,
  }
}

function routePointKey(point: InspectionRoutePoint, index: number): string {
  return `${point.id}:${index}`
}

function locationIsBoundToRoute(
  location: LocationEntry,
  mapId: string,
  mapVersion: number,
): boolean {
  return location.metadata?.binding_status === 'bound'
    && location.map_id === mapId
    && location.map_version === mapVersion
    && (location.frame_id ?? 'map') === 'map'
}

function locationBindingLabel(location: LocationEntry): string {
  const status = statusText(location.metadata?.binding_status, 'unbound')
  if (status !== 'bound') return status
  return `${location.map_id ?? '--'} · v${location.map_version ?? '--'}`
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

export function InspectionWorkbench({ sseState, showToast, locale }: InspectionWorkbenchProps) {
  const [mapId, setMapId] = useState(() => sessionActiveMap(sseState))
  const [routeId, setRouteId] = useState('route-1')
  const [routeName, setRouteName] = useState('Inspection route')
  const [mapVersion, setMapVersion] = useState(1)
  const [revision, setRevision] = useState(1)
  const [savedRevision, setSavedRevision] = useState(0)
  const [loopCount, setLoopCount] = useState(1)
  const [failurePolicy, setFailurePolicy] = useState<InspectionFailurePolicy>('stop')
  const [maxRetries, setMaxRetries] = useState(0)
  const [points, setPoints] = useState<InspectionRoutePoint[]>([])
  const [routes, setRoutes] = useState<InspectionRoute[]>([])
  const [locations, setLocations] = useState<LocationsResponse | null>(null)
  const [selectedRouteId, setSelectedRouteId] = useState<string | null>(null)
  const [status, setStatus] = useState<Record<string, unknown>>({})
  const [evidenceItems, setEvidenceItems] = useState<InspectionEvidenceSummary[]>([])
  const [evidenceIntegrityFailures, setEvidenceIntegrityFailures] = useState(0)
  const [busy, setBusy] = useState<BusyAction>(null)
  const [error, setError] = useState<string | null>(null)
  const [inspectionAvailability, setInspectionAvailability] = useState<InspectionAvailability>('checking')

  useEffect(() => {
    const active = sessionActiveMap(sseState)
    if (active && !mapId) setMapId(active)
  }, [mapId, sseState])

  const selectedRoute = useMemo(
    () => routes.find(route => route.id === selectedRouteId) ?? null,
    [routes, selectedRouteId],
  )

  const enabledPointCount = points.filter(point => point.enabled).length
  const currentPointIndex = Number(status.current_point_index ?? status.point_index ?? 0)
  const pointTotal = Number(
    status.point_count
      ?? status.total_points
      ?? selectedRoute?.point_count
      ?? points.length,
  )
  const progressLabel = Number.isFinite(currentPointIndex) && Number.isFinite(pointTotal) && pointTotal > 0
    ? `${Math.min(currentPointIndex + 1, pointTotal)}/${pointTotal}`
    : '--'
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
    setSelectedRouteId(route.id)
    setRouteId(route.id)
    setRouteName(route.name || route.id)
    setMapId(route.map_id || mapId)
    setMapVersion(Number(route.map_version ?? 1))
    setSavedRevision(Number.isInteger(currentRevision) && currentRevision > 0 ? currentRevision : 0)
    setRevision(nextRevision(route))
    setLoopCount(Number(route.loop_count ?? 1))
    setFailurePolicy((route.failure_policy as InspectionFailurePolicy) || 'stop')
    setMaxRetries(Number(route.max_retries ?? 0))
    setPoints(route.points ?? [])
  }, [mapId])

  const load = useCallback(async () => {
    setBusy('load')
    try {
      const bootstrap = await api.fetchAppBootstrap()
      const routeApiAvailable = Boolean(
        bootstrap.links?.inspection_routes && bootstrap.links?.inspection_status,
      )
      if (!routeApiAvailable) {
        const locationResult = await Promise.allSettled([api.fetchLocations()])
        if (locationResult[0].status === 'fulfilled') setLocations(locationResult[0].value)
        setRoutes([])
        setStatus({})
        setEvidenceItems([])
        setEvidenceIntegrityFailures(0)
        setInspectionAvailability('unavailable')
        setError(null)
        return
      }
      setInspectionAvailability('available')
      const [routeResult, locationResult, statusResult, evidenceResult] = await Promise.allSettled([
        api.fetchInspectionRoutes(mapId || null),
        api.fetchLocations(),
        api.fetchInspectionStatus(),
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
      if (statusResult.status === 'fulfilled') setStatus(statusResult.value.status)
      if (evidenceResult.status === 'fulfilled') {
        setEvidenceItems(evidenceResult.value.evidence)
        setEvidenceIntegrityFailures(evidenceResult.value.integrity_failures)
      }
      setError(null)
    } catch (err) {
      if (api.isGatewayApiError(err) && err.statusCode === 404) {
        setInspectionAvailability('unavailable')
        setError(null)
        return
      }
      const message = err instanceof Error ? err.message : String(err)
      setError(message)
      showToast(message, 'error')
    } finally {
      setBusy(null)
    }
  }, [applyRouteToEditor, mapId, selectedRouteId, showToast])

  useEffect(() => {
    void load()
  }, [load])

  useEffect(() => {
    if (inspectionAvailability !== 'available') return
    const timer = window.setInterval(() => {
      api.fetchInspectionStatus()
        .then(result => setStatus(result.status))
        .catch(() => undefined)
    }, 2000)
    return () => window.clearInterval(timer)
  }, [inspectionAvailability])

  useEffect(() => {
    if (inspectionAvailability !== 'available') return
    const refreshEvidence = () => {
      api.fetchInspectionEvidence(12)
        .then(result => {
          setEvidenceItems(result.evidence)
          setEvidenceIntegrityFailures(result.integrity_failures)
        })
        .catch(() => undefined)
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

  const addLocation = useCallback((location: LocationEntry) => {
    setPoints(current => {
      if (current.some(point => point.id === location.name)) return current
      return [...current, routePointFromLocation(location)]
    })
  }, [])

  const movePoint = useCallback((index: number, delta: -1 | 1) => {
    setPoints(current => {
      const nextIndex = index + delta
      if (nextIndex < 0 || nextIndex >= current.length) return current
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
        map_version: Math.max(0, Math.trunc(mapVersion)),
        revision: Math.max(1, Math.trunc(revision)),
        loop_count: Math.max(1, Math.trunc(loopCount)),
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
  }, [failurePolicy, load, locale, loopCount, mapId, mapVersion, maxRetries, points, revision, routeId, routeName, showToast])

  const runAction = useCallback(async (
    action: Exclude<BusyAction, 'load' | 'save' | 'delete' | null>,
    call: () => Promise<unknown>,
  ) => {
    setBusy(action)
    try {
      await call()
      const nextStatus = await api.fetchInspectionStatus()
      setStatus(nextStatus.status)
      showToast(text(locale, 'Inspection command accepted', '巡检命令已接受'), 'success')
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
    await api.startInspectionRoute(selectedRouteId, {
      map_id: selectedRoute?.map_id || mapId,
      revision: savedRevision,
    })
  }, [locale, mapId, savedRevision, selectedRoute, selectedRouteId])

  const resumeRoute = useCallback(async () => {
    const pauseReason = statusText(status.reason, '')
    if (pauseReason.includes('operator_takeover')) {
      await api.resumeNavigation()
    }
    await api.resumeInspectionRun(mapId)
  }, [mapId, status.reason])

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
          <div className={styles.eyebrow}>{text(locale, 'Inspection Workbench', '巡检工作台')}</div>
          <h1 className={styles.title}>{text(locale, 'Routes and Native Run Control', '路线与原生运行控制')}</h1>
        </div>
        <div className={styles.actions}>
          <button type="button" className={styles.iconButton} onClick={() => void load()} disabled={busy !== null} title="Refresh">
            <RefreshCcw size={16} />
          </button>
          <button type="button" className={styles.primaryButton} onClick={() => void saveRoute()} disabled={inspectionAvailability !== 'available' || busy !== null || points.length === 0}>
            <Save size={16} />
            {text(locale, 'Save', '保存')}
          </button>
        </div>
      </header>

      <section className={styles.statusBand}>
        <div>
          <span>{text(locale, 'Native state', '原生状态')}</span>
          <strong>{statusText(status.state ?? status.status)}</strong>
        </div>
        <div>
          <span>{text(locale, 'Route', '路线')}</span>
          <strong>{statusText(status.route_id ?? selectedRouteId)}</strong>
        </div>
        <div>
          <span>{text(locale, 'Point progress', '点位进度')}</span>
          <strong>{progressLabel}</strong>
        </div>
        <div>
          <span>{text(locale, 'Current point', '当前点')}</span>
          <strong>{statusText(status.current_point_id ?? status.point_id)}</strong>
        </div>
        <div>
          <span>{text(locale, 'Phase reason', '阶段原因')}</span>
          <strong>{statusText(status.reason)}</strong>
        </div>
        <div>
          <span>{text(locale, 'Action', '动作')}</span>
          <strong>{statusText(status.action)}</strong>
        </div>
        <div>
          <span>{text(locale, 'Evidence', '证据')}</span>
          <strong>{statusText(status.evidence_id)}</strong>
        </div>
        <div>
          <span>{text(locale, 'Evidence worker', '取证模块')}</span>
          <strong className={evidenceWorkerReady ? styles.workerReady : styles.workerUnavailable}>
            {evidenceWorkerReady
              ? text(locale, 'Ready', 'Ready')
              : text(locale, 'Unavailable', 'Unavailable')}
          </strong>
        </div>
        <div>
          <span>{text(locale, 'Enabled points', '启用点位')}</span>
          <strong>{enabledPointCount}/{points.length}</strong>
        </div>
      </section>

      <section className={styles.evidencePanel} aria-label="recent inspection evidence">
        <div className={styles.evidenceHeader}>
          <div>
            <div className={styles.sectionTitle}>{text(locale, 'Recent evidence', '最近证据')}</div>
            <p>{text(
              locale,
              'Only checksum-verified captures are shown here.',
              '这里只显示通过校验和验证的巡检取证。',
            )}</p>
          </div>
          <div className={evidenceIntegrityFailures > 0 ? styles.evidenceWarning : styles.evidenceCount}>
            {evidenceIntegrityFailures > 0
              ? text(locale, `${evidenceIntegrityFailures} invalid hidden`, `已隐藏 ${evidenceIntegrityFailures} 条异常证据`)
              : text(locale, `${evidenceItems.length} verified`, `${evidenceItems.length} 条已验证`)}
          </div>
        </div>
        <div className={styles.evidenceGrid}>
          {evidenceItems.map(evidence => {
            const request = evidence.request
            const verdict = statusText(evidence.analysis.verdict, 'inconclusive')
            const rgbUrl = api.inspectionEvidenceArtifactUrl(evidence.evidence_id, 'rgb')
            return (
              <article key={evidence.evidence_id} className={styles.evidenceCard}>
                {hasEvidenceArtifact(evidence, 'rgb') ? (
                  <a
                    className={styles.evidencePreview}
                    href={rgbUrl}
                    target="_blank"
                    rel="noreferrer"
                    title={text(locale, 'Open full image', '打开原图')}
                  >
                    <img src={rgbUrl} alt={`${request.route_id ?? 'route'} ${request.point_id ?? 'point'}`} loading="lazy" />
                  </a>
                ) : (
                  <div className={styles.evidencePlaceholder}>{text(locale, 'No RGB', '无 RGB')}</div>
                )}
                <div className={styles.evidenceBody}>
                  <div className={styles.evidenceIdentity}>
                    <strong>{statusText(request.route_id)}</strong>
                    <span>{statusText(request.point_id)}</span>
                  </div>
                  <div className={styles.evidenceAction}>{statusText(request.action)}</div>
                  <div className={styles.evidenceMeta}>
                    <span className={verdict === 'violation' ? styles.verdictAlert : styles.verdictNeutral}>
                      {verdict}
                    </span>
                    <time>{evidenceTimestamp(evidence, locale)}</time>
                  </div>
                  <div className={styles.evidenceLinks}>
                    {hasEvidenceArtifact(evidence, 'pose') && (
                      <a href={api.inspectionEvidenceArtifactUrl(evidence.evidence_id, 'pose')} target="_blank" rel="noreferrer">
                        pose
                      </a>
                    )}
                    {hasEvidenceArtifact(evidence, 'detections') && (
                      <a href={api.inspectionEvidenceArtifactUrl(evidence.evidence_id, 'detections')} target="_blank" rel="noreferrer">
                        detections
                      </a>
                    )}
                  </div>
                </div>
              </article>
            )
          })}
          {evidenceItems.length === 0 && (
            <div className={styles.empty}>{text(locale, 'No verified evidence yet', '暂无已验证证据')}</div>
          )}
        </div>
      </section>

      {inspectionAvailability === 'unavailable' && (
        <div className={styles.capabilityNotice}>
          <AlertTriangle size={16} />
          <span>{text(
            locale,
            'This workbench builds saved locations into inspection routes and controls native runs. The connected Gateway has not deployed the inspection route service, so route actions are disabled.',
            '该页面用于把保存点位编排成巡检路线，并控制原生巡检运行。当前 Gateway 尚未部署巡检路线服务，路线操作已禁用。',
          )}</span>
        </div>
      )}

      {error && (
        <div className={styles.alert}>
          <XCircle size={16} />
          <span>{error}</span>
        </div>
      )}

      {startBlockedByEvidenceWorker && (
        <div className={styles.capabilityNotice}>
          <AlertTriangle size={16} />
          <span>{!evidenceWorkerReady
            ? text(
                locale,
                `This route contains evidence actions, but the evidence worker is unavailable: ${statusText(evidenceWorker.reason)}`,
                `该路线包含取证动作，但取证模块不可用：${statusText(evidenceWorker.reason)}`,
              )
            : text(
                locale,
                `This route contains unsupported evidence actions: ${unsupportedRouteActions.join(', ')}`,
                `该路线包含当前不支持的取证动作：${unsupportedRouteActions.join('、')}`,
              )}</span>
        </div>
      )}

      <section className={styles.content}>
        <aside className={styles.routesPane}>
          <div className={styles.sectionTitle}>{text(locale, 'Saved routes', '已保存路线')}</div>
          <div className={styles.routeList}>
            {routes.map(route => (
              <button
                key={route.id}
                type="button"
                className={route.id === selectedRouteId ? styles.routeActive : styles.routeItem}
                onClick={() => void loadRouteIntoEditor(route)}
              >
                <span>{route.name || route.id}</span>
                <small>{route.point_count ?? route.points.length} pts · rev {route.revision ?? 0}</small>
              </button>
            ))}
            {routes.length === 0 && <div className={styles.empty}>{text(locale, 'No routes saved', '暂无路线')}</div>}
          </div>
          <div className={styles.runControls}>
            <button
              type="button"
              onClick={() => void runAction('start', startRoute)}
              disabled={inspectionAvailability !== 'available' || busy !== null || !selectedRouteId || savedRevision < 1 || startBlockedByEvidenceWorker}
              title={startBlockedByEvidenceWorker
                ? (!evidenceWorkerReady
                    ? text(locale, 'Evidence worker must be ready before starting routes with actions.', '带动作路线启动前需要取证模块 Ready。')
                    : text(
                        locale,
                        `Unsupported evidence actions: ${unsupportedRouteActions.join(', ')}`,
                        `不支持的取证动作：${unsupportedRouteActions.join('、')}`,
                      ))
                : undefined}
            >
              <Play size={15} />{text(locale, 'Start', '启动')}
            </button>
            <button type="button" onClick={() => void runAction('pause', () => api.pauseInspectionRun(mapId))} disabled={inspectionAvailability !== 'available' || busy !== null}>
              <Pause size={15} />{text(locale, 'Pause', '暂停')}
            </button>
            <button type="button" onClick={() => void runAction('resume', resumeRoute)} disabled={inspectionAvailability !== 'available' || busy !== null}>
              <RotateCcw size={15} />{text(locale, 'Resume', '恢复')}
            </button>
            <button type="button" onClick={() => void runAction('cancel', () => api.cancelInspectionRun(mapId))} disabled={inspectionAvailability !== 'available' || busy !== null}>
              <Square size={15} />{text(locale, 'Cancel', '取消')}
            </button>
          </div>
        </aside>

        <main className={styles.editorPane}>
          <div className={styles.formGrid}>
            <label>
              <span>{text(locale, 'Route ID', '路线 ID')}</span>
              <input value={routeId} onChange={event => setRouteId(event.target.value)} />
            </label>
            <label>
              <span>{text(locale, 'Name', '名称')}</span>
              <input value={routeName} onChange={event => setRouteName(event.target.value)} />
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
            <label>
              <span>{text(locale, 'Loop', '循环')}</span>
              <input type="number" min="1" value={loopCount} onChange={event => setLoopCount(Number(event.target.value))} />
            </label>
            <label>
              <span>{text(locale, 'Failure policy', '失败策略')}</span>
              <select value={failurePolicy} onChange={event => setFailurePolicy(event.target.value as InspectionFailurePolicy)}>
                <option value="stop">stop</option>
                <option value="retry">retry</option>
                <option value="skip">skip</option>
              </select>
            </label>
            <label>
              <span>{text(locale, 'Retry', '重试')}</span>
              <input type="number" min="0" value={maxRetries} onChange={event => setMaxRetries(Number(event.target.value))} />
            </label>
          </div>

          <div className={styles.builder}>
            <section className={styles.locationPane}>
              <div className={styles.sectionTitle}>{text(locale, 'Locations', '位置')}</div>
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
                        ? `Add ${location.name}`
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
              <div className={styles.sectionTitle}>{text(locale, 'Route points', '路线点位')}</div>
              <div className={styles.pointRows}>
                {points.map((point, index) => (
                  <div key={routePointKey(point, index)} className={styles.pointRow}>
                    <div className={styles.orderButtons}>
                      <button type="button" onClick={() => movePoint(index, -1)} disabled={index === 0} title="Move up"><ArrowUp size={14} /></button>
                      <button type="button" onClick={() => movePoint(index, 1)} disabled={index === points.length - 1} title="Move down"><ArrowDown size={14} /></button>
                    </div>
                    <strong>{index + 1}. {point.id}</strong>
                    <label>
                      <span>dwell</span>
                      <input type="number" min="0" step="0.5" value={point.dwell} onChange={event => updatePoint(index, { dwell: Number(event.target.value) })} />
                    </label>
                    <label>
                      <span>tol</span>
                      <input type="number" min="0" step="0.05" value={point.tolerance} onChange={event => updatePoint(index, { tolerance: Number(event.target.value) })} />
                    </label>
                    <label>
                      <span>action</span>
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
                              {action || text(locale, 'None', '无')}
                            </option>
                          ))}
                        </select>
                      ) : (
                        <input
                          value={point.action}
                          disabled
                          title={text(
                            locale,
                            'Evidence actions are disabled until the evidence worker is ready.',
                            '取证模块 Ready 后才能配置点位动作。',
                          )}
                        />
                      )}
                    </label>
                    <label className={styles.toggle}>
                      <input type="checkbox" checked={point.enabled} onChange={event => updatePoint(index, { enabled: event.target.checked })} />
                      <span>{text(locale, 'Enabled', '启用')}</span>
                    </label>
                    <button type="button" className={styles.iconButton} onClick={() => setPoints(current => current.filter((_, i) => i !== index))} title="Remove">
                      <Trash2 size={15} />
                    </button>
                  </div>
                ))}
                {points.length === 0 && <div className={styles.empty}>{text(locale, 'Select locations to build a route', '选择位置来生成路线')}</div>}
              </div>
            </section>
          </div>

          {selectedRoute && (
            <button type="button" className={styles.deleteButton} onClick={() => void deleteRoute(selectedRoute)} disabled={busy !== null}>
              <Trash2 size={15} />
              {text(locale, 'Delete selected route', '删除所选路线')}
            </button>
          )}
        </main>
      </section>
    </div>
  )
}
