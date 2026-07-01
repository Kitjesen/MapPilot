import { useEffect, useRef, useState, useCallback } from 'react'
import * as api from '../services/api'
import type {
  AppBootstrapResponse,
  AppCapabilitiesResponse,
  LocationsResponse,
  MissionStatusEvent,
  OdometryEvent,
  SafetyStateEvent,
  SceneGraphEvent,
  SSEState,
  SSEEvent,
  StateResponse,
} from '../types'

// Re-export types for backward compatibility
export type {
  OdometryEvent,
  MissionStatusEvent,
  SafetyStateEvent,
  SceneGraphEvent,
  HeartbeatEvent,
  PingEvent,
  SnapshotEvent,
  MissionEvent,
  SafetyEvent,
  NavigationStatusEvent,
  LeaseEvent,
  CommandAckEvent,
  EvalEvent,
  DialogueEvent,
  GnssFusionEvent,
  SlamDiagnosticEvent,
  SlamDriftEvent,
  TareStatsEvent,
  ExplorationSupervisorEvent,
  ExploringEvent,
  SlamStatusEvent,
  RobotStatusEvent,
  GlobalPathEvent,
  MapCloudEvent,
  MapLifecycleEvent,
  CostmapEvent,
  SlopeGridEvent,
  AgentMessageEvent,
  PathPoint,
  SSEEvent,
  SSEState,
} from '../types'

const INITIAL_STATE: SSEState = {
  odometry: null,
  missionStatus: null,
  safetyState: null,
  sceneGraph: null,
  slamStatus: null,
  robotStatus: null,
  globalPath: null,
  localPath: null,
  mapCloud: null,
  savedMap: null,
  mapEvent: null,
  session: null,
  navigationStatus: null,
  lease: null,
  commandAck: null,
  locations: null,
  stateSnapshot: null,
  traffic: null,
  costmap: null,
  slopeGrid: null,
  agentMessage: null,
  gnssFusion: null,
  slamDiag: null,
  slamDrift: null,
  tareStats: null,
  explorationSupervisor: null,
  exploring: null,
  evalEvent: null,
  dialogue: null,
  lastHeartbeat: null,
  lastEventId: null,
  missedEvents: 0,
  reconnects: 0,
  lastError: null,
  lastRefreshAt: null,
  lastRefreshReason: null,
  refreshError: null,
  connected: false,
  events: [],
}

const TRAFFIC_REFRESH_MS = 5000
type SnapshotRefreshMode = 'full' | 'auxiliary'

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === 'object' && value !== null
}

function finiteNumber(value: unknown, fallback = 0): number {
  return typeof value === 'number' && Number.isFinite(value) ? value : fallback
}

function optionalString(value: unknown): string | null {
  return typeof value === 'string' && value.length > 0 ? value : null
}

function sameOriginHttpPath(value: unknown): string | null {
  if (typeof value !== 'string' || value.length === 0) return null
  const origin = typeof window !== 'undefined' && window.location?.origin
    ? window.location.origin
    : 'http://localhost'
  try {
    const parsed = new URL(value, origin)
    if (parsed.origin !== origin) return null
    return `${parsed.pathname}${parsed.search}`
  } catch {
    return null
  }
}

function endpointPath(
  capabilities: AppCapabilitiesResponse | null,
  group: string,
  name: string,
): string | null {
  return sameOriginHttpPath(capabilities?.endpoints?.[group]?.[name]?.path)
}

function trafficEndpointFrom(
  bootstrap: AppBootstrapResponse | null,
  capabilities: AppCapabilitiesResponse | null,
): string | null {
  return (
    sameOriginHttpPath(bootstrap?.links?.traffic)
    ?? sameOriginHttpPath(bootstrap?.traffic?.client_policy?.traffic_endpoint)
    ?? sameOriginHttpPath(capabilities?.links?.traffic)
    ?? endpointPath(capabilities, 'app', 'traffic')
  )
}

function snapshotOdometry(snapshot: StateResponse): OdometryEvent | null {
  const raw = snapshot.odometry
  if (!isRecord(raw)) return null
  if (typeof raw.x !== 'number' || typeof raw.y !== 'number') return null
  return {
    type: 'odometry',
    x: raw.x,
    y: raw.y,
    yaw: finiteNumber(raw.yaw),
    vx: finiteNumber(raw.vx),
  }
}

function snapshotMission(snapshot: StateResponse): MissionStatusEvent | null {
  const raw = snapshot.mission
  if (!isRecord(raw)) return null
  return {
    type: 'mission_status',
    state: optionalString(raw.state) ?? 'IDLE',
    goal: optionalString(raw.goal),
    progress: finiteNumber(raw.progress),
  }
}

function snapshotSafety(snapshot: StateResponse): SafetyStateEvent | null {
  const raw = snapshot.safety
  if (!isRecord(raw)) return null
  const level = raw.level
  return {
    type: 'safety_state',
    estop: Boolean(raw.estop),
    level: typeof level === 'string' ? level : String(level ?? 'unknown'),
  }
}

function snapshotSceneGraph(scene: Awaited<ReturnType<typeof api.fetchSceneGraph>>): SceneGraphEvent {
  return {
    type: 'scene_graph',
    objects: scene.objects
      .filter(obj => typeof obj.x === 'number' && typeof obj.y === 'number')
      .map(obj => ({
        id: obj.id ?? obj.label,
        label: obj.label,
        x: obj.x as number,
        y: obj.y as number,
        confidence: typeof obj.confidence === 'number' ? obj.confidence : 0.5,
      })),
  }
}

export function useSSE(url: string = '/api/v1/events') {
  const [state, setState] = useState<SSEState>(INITIAL_STATE)
  const esRef = useRef<EventSource | null>(null)
  const reconnectTimer = useRef<ReturnType<typeof setTimeout> | null>(null)
  const refreshTimer = useRef<ReturnType<typeof setTimeout> | null>(null)
  const initialSnapshotFallbackTimer = useRef<ReturnType<typeof setTimeout> | null>(null)
  const trafficTimer = useRef<ReturnType<typeof setInterval> | null>(null)
  const refreshInFlight = useRef(false)
  const trafficInFlight = useRef(false)
  const trafficDiscoveryInFlight = useRef(false)
  const trafficEndpointRef = useRef<string | null>(null)
  const mountedRef = useRef(true)
  const everConnectedRef = useRef(false)
  const initialSnapshotSeenRef = useRef(false)
  const lastEventIdRef = useRef<number | null>(null)
  const connectRef = useRef<() => void>(() => {})
  const refreshRef = useRef<(reason: string, mode?: SnapshotRefreshMode) => void>(() => {})

  const refreshSnapshot = useCallback(async (
    reason: string,
    mode: SnapshotRefreshMode = 'full',
  ) => {
    if (refreshInFlight.current) return
    refreshInFlight.current = true
    try {
      const statePromise: Promise<StateResponse | null> = mode === 'full'
        ? api.fetchState()
        : Promise.resolve(null)
      const [stateResult, pathResult, sceneResult, locationsResult] = await Promise.allSettled([
        statePromise,
        api.fetchPath(),
        api.fetchSceneGraph(),
        api.fetchLocations(),
      ])
      if (!mountedRef.current) return

      const statePayload = stateResult.status === 'fulfilled' ? stateResult.value : null
      const pathPayload = pathResult.status === 'fulfilled' ? pathResult.value : null
      const scenePayload = sceneResult.status === 'fulfilled' ? sceneResult.value : null
      const locationsPayload: LocationsResponse | null =
        locationsResult.status === 'fulfilled' ? locationsResult.value : null

      const failures = [stateResult, pathResult, sceneResult, locationsResult]
        .filter(result => result.status === 'rejected').length
      const odometry = statePayload ? snapshotOdometry(statePayload) : null
      const mission = statePayload ? snapshotMission(statePayload) : null
      const safety = statePayload ? snapshotSafety(statePayload) : null

      setState(prev => ({
        ...prev,
        odometry: odometry ?? prev.odometry,
        missionStatus: mission ?? prev.missionStatus,
        safetyState: safety ?? prev.safetyState,
        session: (statePayload?.session as SSEState['session'] | undefined) ?? prev.session,
        navigationStatus: statePayload?.navigation ?? prev.navigationStatus,
        lease: statePayload?.lease ?? prev.lease,
        stateSnapshot: statePayload ?? prev.stateSnapshot,
        globalPath: pathPayload
          ? { type: 'global_path', points: pathPayload.path }
          : prev.globalPath,
        sceneGraph: scenePayload ? snapshotSceneGraph(scenePayload) : prev.sceneGraph,
        locations: locationsPayload ?? prev.locations,
        lastRefreshAt: Date.now(),
        lastRefreshReason: reason,
        refreshError: failures > 0 ? `${failures}_snapshot_fetch_failed` : null,
      }))
    } catch (err) {
      if (!mountedRef.current) return
      setState(prev => ({
        ...prev,
        refreshError: err instanceof Error ? err.message : String(err),
        lastRefreshReason: reason,
      }))
    } finally {
      refreshInFlight.current = false
    }
  }, [])

  const queueRefresh = useCallback((reason: string, mode: SnapshotRefreshMode = 'full') => {
    if (refreshTimer.current) return
    refreshTimer.current = setTimeout(() => {
      refreshTimer.current = null
      refreshRef.current(reason, mode)
    }, 250)
  }, [])

  const clearInitialSnapshotFallback = useCallback(() => {
    if (!initialSnapshotFallbackTimer.current) return
    clearTimeout(initialSnapshotFallbackTimer.current)
    initialSnapshotFallbackTimer.current = null
  }, [])

  const discoverTrafficEndpoint = useCallback(async (): Promise<string | null> => {
    if (trafficDiscoveryInFlight.current) return trafficEndpointRef.current
    trafficDiscoveryInFlight.current = true
    try {
      const [bootstrapResult, capabilitiesResult] = await Promise.allSettled([
        api.fetchAppBootstrap(),
        api.fetchAppCapabilities(),
      ])
      const bootstrap = bootstrapResult.status === 'fulfilled' ? bootstrapResult.value : null
      const capabilities = capabilitiesResult.status === 'fulfilled' ? capabilitiesResult.value : null
      const endpoint = trafficEndpointFrom(bootstrap, capabilities)
      trafficEndpointRef.current = endpoint
      if (!endpoint && mountedRef.current) {
        setState(prev => ({ ...prev, traffic: null }))
      }
      return endpoint
    } finally {
      trafficDiscoveryInFlight.current = false
    }
  }, [])

  const refreshTraffic = useCallback(async () => {
    const endpoint = trafficEndpointRef.current
    if (!endpoint) return
    if (trafficInFlight.current) return
    trafficInFlight.current = true
    try {
      const traffic = await api.fetchAppTraffic(endpoint)
      if (!mountedRef.current) return
      setState(prev => ({ ...prev, traffic }))
    } catch (err) {
      const message = err instanceof Error ? err.message : String(err)
      if (message.includes('HTTP 404') || message.includes('HTTP 405')) {
        trafficEndpointRef.current = null
        if (trafficTimer.current) {
          clearInterval(trafficTimer.current)
          trafficTimer.current = null
        }
        if (mountedRef.current) {
          setState(prev => ({ ...prev, traffic: null }))
        }
      }
      // Traffic is diagnostic; keep the main realtime path alive if it is unavailable.
    } finally {
      trafficInFlight.current = false
    }
  }, [])

  const connect = useCallback(() => {
    if (!mountedRef.current) return
    if (esRef.current) {
      esRef.current.close()
      esRef.current = null
    }

    const es = new EventSource(url)
    esRef.current = es

    es.onopen = () => {
      if (!mountedRef.current) return
      setState(prev => ({ ...prev, connected: true, lastError: null }))
      if (everConnectedRef.current) {
        queueRefresh('event_stream_reconnected')
      } else {
        initialSnapshotSeenRef.current = false
        clearInitialSnapshotFallback()
        initialSnapshotFallbackTimer.current = setTimeout(() => {
          initialSnapshotFallbackTimer.current = null
          if (!mountedRef.current || initialSnapshotSeenRef.current) return
          queueRefresh('event_stream_initial_snapshot_timeout')
        }, 1000)
      }
      everConnectedRef.current = true
    }

    es.onmessage = (e: MessageEvent) => {
      if (!mountedRef.current) return
      // Handle NDJSON: each line is a JSON object
      const lines = (e.data as string).split('\n').filter(l => l.trim())
      for (const line of lines) {
        try {
          const event = JSON.parse(line) as SSEEvent
          const eventId = typeof event.event_id === 'number' && Number.isFinite(event.event_id)
            ? event.event_id
            : null
          const prevEventId = lastEventIdRef.current
          const missedByEvent = eventId !== null && prevEventId !== null && eventId > prevEventId + 1
            ? eventId - prevEventId - 1
            : 0
          if (eventId !== null) lastEventIdRef.current = eventId
          if (missedByEvent > 0) queueRefresh('event_id_gap')
          setState(prev => {
            const missedEvents = prev.missedEvents + missedByEvent
            const next = {
              ...prev,
              events: [...prev.events.slice(-99), event],
              lastEventId: eventId ?? prev.lastEventId,
              missedEvents,
            }
            const evt = event as { type: string; data?: Record<string, unknown>; [k: string]: unknown }
            switch (evt.type) {
              case 'snapshot': {
                // Backend sends nested {type: snapshot, data: {odometry, safety, mission, ...}}
                const isInitialSnapshot = !initialSnapshotSeenRef.current
                initialSnapshotSeenRef.current = true
                clearInitialSnapshotFallback()
                if (isInitialSnapshot) {
                  queueRefresh('event_stream_initial_snapshot_auxiliary', 'auxiliary')
                }
                const d = evt.data || {}
                if (d.odometry) next.odometry = { type: 'odometry', ...d.odometry as object } as never
                if (d.mission)  next.missionStatus = { type: 'mission_status', ...d.mission as object } as never
                if (d.safety)   next.safetyState = { type: 'safety_state', ...d.safety as object } as never
                if (d.session)  next.session = d.session as never
                if (d.navigation) next.navigationStatus = d.navigation as never
                if (d.lease) next.lease = d.lease as never
                break
              }
              case 'odometry':
                next.odometry = { type: 'odometry', ...(evt.data as object || evt) } as never
                break
              case 'mission':
              case 'mission_status':
                next.missionStatus = { type: 'mission_status', ...(evt.data as object || evt) } as never
                break
              case 'safety':
              case 'safety_state':
                next.safetyState = { type: 'safety_state', ...(evt.data as object || evt) } as never
                break
              case 'navigation_status':
                next.navigationStatus = (evt.data ?? evt) as never
                break
              case 'lease':
                next.lease = (evt.data ?? evt) as never
                break
              case 'locations':
                next.locations = (evt.data ?? evt) as never
                break
              case 'location': {
                const data = evt.data as { locations?: unknown } | undefined
                if (data?.locations) next.locations = data.locations as never
                break
              }
              case 'command_ack':
                next.commandAck = (evt.data ?? evt) as never
                break
              case 'scene_graph':
                next.sceneGraph = event as never
                break
              case 'slam_status':
                next.slamStatus = event as never
                break
              case 'gnss_fusion':
                next.gnssFusion = event as never
                break
              case 'slam_diag':
                next.slamDiag = event as never
                break
              case 'slam_drift':
                next.slamDrift = event as never
                break
              case 'tare_stats':
                next.tareStats = event as never
                break
              case 'exploration_supervisor':
                next.explorationSupervisor = event as never
                break
              case 'exploring':
                next.exploring = event as never
                break
              case 'eval':
                next.evalEvent = event as never
                break
              case 'dialogue':
                next.dialogue = event as never
                break
              case 'robot_status':
                next.robotStatus = event as never
                break
              case 'global_path':
                next.globalPath = event as never
                break
              case 'local_path':
                next.localPath = event as never
                break
              case 'map_cloud':
                next.mapCloud = event as never
                break
              case 'saved_map':
                next.savedMap = event as never
                break
              case 'map_event':
                next.mapEvent = (evt.data ?? evt) as never
                break
              case 'session':
                next.session = (evt.data ?? evt) as never
                break
              case 'costmap':
                next.costmap = event as never
                break
              case 'slope_grid': {
                const slope = event as { grid_b64?: unknown }
                next.slopeGrid = typeof slope.grid_b64 === 'string' && slope.grid_b64.length > 0
                  ? event as never
                  : null
                break
              }
              case 'agent_message':
                next.agentMessage = event as never
                break
              case 'heartbeat':
              case 'ping':
                next.lastHeartbeat = Date.now()
                break
            }
            return next
          })
        } catch {
          // malformed line — ignore
        }
      }
    }

    es.onerror = () => {
      if (!mountedRef.current) return
      clearInitialSnapshotFallback()
      es.close()
      esRef.current = null
      setState(prev => ({
        ...prev,
        connected: false,
        reconnects: prev.reconnects + 1,
        lastError: 'event_stream_disconnected',
      }))
      queueRefresh('event_stream_disconnected')
      reconnectTimer.current = setTimeout(() => {
        if (mountedRef.current) connectRef.current()
      }, 3000)
    }
  }, [clearInitialSnapshotFallback, queueRefresh, url])

  useEffect(() => {
    connectRef.current = connect
  }, [connect])

  useEffect(() => {
    refreshRef.current = refreshSnapshot
  }, [refreshSnapshot])

  useEffect(() => {
    mountedRef.current = true
    connect()
    return () => {
      mountedRef.current = false
      if (refreshTimer.current) clearTimeout(refreshTimer.current)
      clearInitialSnapshotFallback()
      if (reconnectTimer.current) clearTimeout(reconnectTimer.current)
      if (esRef.current) {
        esRef.current.close()
        esRef.current = null
      }
    }
  }, [clearInitialSnapshotFallback, connect])

  useEffect(() => {
    let cancelled = false
    const startTrafficPolling = async () => {
      const endpoint = await discoverTrafficEndpoint()
      if (cancelled || !mountedRef.current || !endpoint) return
      await refreshTraffic()
      if (cancelled || !mountedRef.current || !trafficEndpointRef.current) return
      trafficTimer.current = setInterval(refreshTraffic, TRAFFIC_REFRESH_MS)
    }
    startTrafficPolling()
    return () => {
      cancelled = true
      if (trafficTimer.current) {
        clearInterval(trafficTimer.current)
        trafficTimer.current = null
      }
    }
  }, [discoverTrafficEndpoint, refreshTraffic])

  return state
}
