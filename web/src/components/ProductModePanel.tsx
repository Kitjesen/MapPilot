import { useCallback, useEffect, useMemo, useState } from 'react'
import type { ReactNode } from 'react'
import {
  Activity,
  AlertTriangle,
  CheckCircle2,
  Compass,
  Eye,
  Map as MapIcon,
  Play,
  Radio,
  RefreshCw,
  Route,
  Search,
  ShieldCheck,
  Square,
  Target,
} from 'lucide-react'
import * as api from '../services/api'
import type {
  MapInfo,
  ProductModeProfile,
  RuntimeSwitchRequest,
  RuntimeSwitchResponse,
  SSEState,
  ToastKind,
  VisualServoMode,
} from '../types'
import styles from './ProductModePanel.module.css'

interface ProductModePanelProps {
  sseState: SSEState
  showToast: (msg: string, kind?: ToastKind) => void
}

interface ProductModeOption {
  profile: ProductModeProfile
  label: string
  session: string
  policy: 'hot_candidate' | 'cold_restart'
  requiresMap: boolean
  summary: string
  icon: ReactNode
}

const PRODUCT_MODES: ProductModeOption[] = [
  {
    profile: 'teleop',
    label: 'Teleop',
    session: 'teleop',
    policy: 'cold_restart',
    requiresMap: false,
    summary: 'Manual velocity path through Gateway, Teleop, CmdVelMux and safety.',
    icon: <Radio size={16} />,
  },
  {
    profile: 'teleop_avoid',
    label: 'Teleop Avoid',
    session: 'teleop_avoid',
    policy: 'cold_restart',
    requiresMap: true,
    summary: 'Manual input with SLAM map and traversability cost guard.',
    icon: <ShieldCheck size={16} />,
  },
  {
    profile: 'map',
    label: 'Mapping',
    session: 'mapping',
    policy: 'cold_restart',
    requiresMap: false,
    summary: 'SLAM and map layers active; no mission ownership.',
    icon: <MapIcon size={16} />,
  },
  {
    profile: 'tracking',
    label: 'Tracking',
    session: 'tracking',
    policy: 'hot_candidate',
    requiresMap: true,
    summary: 'Goal service and mission chain without semantic planner.',
    icon: <Target size={16} />,
  },
  {
    profile: 'nav',
    label: 'Navigation',
    session: 'navigation',
    policy: 'hot_candidate',
    requiresMap: true,
    summary: 'Normal saved-map navigation and semantic goal entry.',
    icon: <Route size={16} />,
  },
  {
    profile: 'inspection',
    label: 'Inspection',
    session: 'inspection',
    policy: 'hot_candidate',
    requiresMap: true,
    summary: 'Scheduler and semantic patrol over the navigation chain.',
    icon: <Eye size={16} />,
  },
  {
    profile: 'tare_explore',
    label: 'TARE Explore',
    session: 'exploration',
    policy: 'cold_restart',
    requiresMap: false,
    summary: 'Exploration target generation feeding the normal nav mission chain.',
    icon: <Compass size={16} />,
  },
]

const ENDPOINTS = ['thunder_field', 'mujoco_live', 'replay', 'cmu_unity'] as const
const STRATEGIES: RuntimeSwitchRequest['strategy'][] = ['auto', 'hot', 'cold']

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === 'object' && value !== null
}

function sessionFallbackProfile(mode?: string | null): ProductModeProfile {
  if (mode === 'mapping') return 'map'
  if (mode === 'exploring') return 'tare_explore'
  if (mode === 'navigating') return 'nav'
  return 'teleop'
}

function mapIsNavigable(map: MapInfo): boolean {
  return map.has_pcd && (map.navigation_ready === true || map.has_octomap === true)
}

function shortBlockers(blockers?: string[]): string {
  if (!blockers || blockers.length === 0) return 'No blockers'
  const visible = blockers.slice(0, 2).join('; ')
  return blockers.length > 2 ? `${visible}; +${blockers.length - 2}` : visible
}

function lifecycleFrom(response: RuntimeSwitchResponse | null): string {
  if (!response) return '--'
  const switchPlan = isRecord(response.product_mode_switch)
    ? response.product_mode_switch
    : isRecord(response.plan?.product_mode_switch)
      ? response.plan.product_mode_switch
      : {}
  return typeof switchPlan.required_lifecycle === 'string'
    ? switchPlan.required_lifecycle
    : response.lifecycle
}

function responseTone(response: RuntimeSwitchResponse | null): 'ok' | 'warn' | 'dim' {
  if (!response) return 'dim'
  return response.ok ? 'ok' : 'warn'
}

export function ProductModePanel({ sseState, showToast }: ProductModePanelProps) {
  const [session, setSession] = useState(sseState.session)
  const [maps, setMaps] = useState<MapInfo[]>([])
  const [selectedProfile, setSelectedProfile] = useState<ProductModeProfile>('nav')
  const [endpoint, setEndpoint] = useState<(typeof ENDPOINTS)[number]>('thunder_field')
  const [strategy, setStrategy] = useState<RuntimeSwitchRequest['strategy']>('auto')
  const [selectedMap, setSelectedMap] = useState('')
  const [allowRestart, setAllowRestart] = useState(false)
  const [relocalize, setRelocalize] = useState(true)
  const [loadingRuntime, setLoadingRuntime] = useState(false)
  const [switchBusy, setSwitchBusy] = useState(false)
  const [switchResult, setSwitchResult] = useState<RuntimeSwitchResponse | null>(null)
  const [servoMode, setServoMode] = useState<VisualServoMode>('find')
  const [servoTarget, setServoTarget] = useState('person')
  const [servoBusy, setServoBusy] = useState(false)
  const [servoStatus, setServoStatus] = useState<string>('idle')

  useEffect(() => {
    if (sseState.session) setSession(sseState.session)
  }, [sseState.session])

  const refreshRuntime = useCallback(async () => {
    setLoadingRuntime(true)
    try {
      const [sessionResult, mapsResult] = await Promise.allSettled([
        api.fetchSession(),
        api.fetchMaps(),
      ])
      if (sessionResult.status === 'fulfilled') setSession(sessionResult.value)
      if (mapsResult.status === 'fulfilled') setMaps(mapsResult.value)
      if (sessionResult.status === 'rejected' && mapsResult.status === 'rejected') {
        throw sessionResult.reason
      }
    } catch (error) {
      showToast(`Runtime refresh failed: ${error instanceof Error ? error.message : String(error)}`, 'error')
    } finally {
      setLoadingRuntime(false)
    }
  }, [showToast])

  useEffect(() => {
    void refreshRuntime()
  }, [refreshRuntime])

  const navigableMaps = useMemo(() => maps.filter(mapIsNavigable), [maps])
  const selectedOption = PRODUCT_MODES.find(item => item.profile === selectedProfile) ?? PRODUCT_MODES[4]
  const currentProfile = (
    session?.product_profile
    || sessionFallbackProfile(session?.mode)
  ) as string
  const activeMap = session?.active_map ?? ''

  useEffect(() => {
    if (selectedMap) return
    if (activeMap) {
      setSelectedMap(activeMap)
      return
    }
    const first = navigableMaps[0]?.name
    if (first) setSelectedMap(first)
  }, [activeMap, navigableMaps, selectedMap])

  const missingMap = selectedOption.requiresMap && !selectedMap
  const plannedLifecycle = lifecycleFrom(switchResult)
  const restartAckRequired = plannedLifecycle === 'cold_restart'
    || (!switchResult && selectedOption.policy === 'cold_restart')
  const executeDisabled = switchBusy || missingMap || (restartAckRequired && !allowRestart)
  const executeTitle = missingMap
    ? 'Select a navigation-ready map first'
    : restartAckRequired && !allowRestart
      ? 'Cold restart requires the explicit restart toggle'
      : 'Execute product mode switch through Gateway'

  const buildSwitchRequest = useCallback((execute: boolean): RuntimeSwitchRequest => ({
    current_profile: currentProfile,
    target_profile: selectedProfile,
    target_endpoint: endpoint,
    endpoint,
    map_name: selectedOption.requiresMap ? selectedMap : null,
    relocalize,
    strategy,
    execute,
    allow_restart: execute ? allowRestart : false,
  }), [
    allowRestart,
    currentProfile,
    endpoint,
    relocalize,
    selectedMap,
    selectedOption.requiresMap,
    selectedProfile,
    strategy,
  ])

  const runPreflight = useCallback(async () => {
    setSwitchBusy(true)
    try {
      const result = await api.runRuntimeSwitch(buildSwitchRequest(false))
      setSwitchResult(result)
      showToast(
        result.ok
          ? `Switch preflight: ${lifecycleFrom(result)}`
          : `Switch blocked: ${shortBlockers(result.blockers)}`,
        result.ok ? 'success' : 'error',
      )
    } catch (error) {
      showToast(`Switch preflight failed: ${error instanceof Error ? error.message : String(error)}`, 'error')
    } finally {
      setSwitchBusy(false)
    }
  }, [buildSwitchRequest, showToast])

  const executeSwitch = useCallback(async () => {
    setSwitchBusy(true)
    try {
      const result = await api.runRuntimeSwitch(buildSwitchRequest(true))
      setSwitchResult(result)
      showToast(
        result.ok
          ? `Switch ${result.accepted ? 'accepted' : 'planned'}: ${result.status}`
          : `Switch rejected: ${shortBlockers(result.blockers)}`,
        result.ok ? 'success' : 'error',
      )
      void refreshRuntime()
    } catch (error) {
      showToast(`Switch failed: ${error instanceof Error ? error.message : String(error)}`, 'error')
    } finally {
      setSwitchBusy(false)
    }
  }, [buildSwitchRequest, refreshRuntime, showToast])

  const sendServo = useCallback(async (mode: VisualServoMode) => {
    setServoBusy(true)
    try {
      const response = await api.sendVisualServo(mode, servoTarget.trim())
      setServoMode(mode)
      setServoStatus(response.ok ? `${mode}: ${response.status}` : `${mode}: rejected`)
      showToast(response.ok ? `Visual servo ${mode}` : `Visual servo rejected`, response.ok ? 'success' : 'error')
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error)
      setServoStatus(message)
      showToast(`Visual servo failed: ${message}`, 'error')
    } finally {
      setServoBusy(false)
    }
  }, [servoTarget, showToast])

  return (
    <section className={styles.page} role="tabpanel" id="panel-runtime">
      <header className={styles.header}>
        <div>
          <p className={styles.kicker}>Runtime Control</p>
          <h1>Product mode switch</h1>
          <p>Gateway-only entry for map, nav, tracking, inspection, exploration and visual servo.</p>
        </div>
        <button className={styles.iconButton} onClick={refreshRuntime} disabled={loadingRuntime} title="Refresh session and map list">
          <RefreshCw size={16} />
          Refresh
        </button>
      </header>

      <div className={styles.layout}>
        <main className={styles.main}>
          <section className={styles.panel}>
            <div className={styles.panelTitle}>
              <Activity size={16} />
              <h2>Current Runtime</h2>
            </div>
            <div className={styles.statusGrid}>
              <div>
                <span>Profile</span>
                <strong>{currentProfile}</strong>
              </div>
              <div>
                <span>Product session</span>
                <strong>{session?.product_session ?? session?.mode ?? 'idle'}</strong>
              </div>
              <div>
                <span>Gateway session</span>
                <strong>{session?.mode ?? 'idle'}</strong>
              </div>
              <div>
                <span>Active map</span>
                <strong>{activeMap || '--'}</strong>
              </div>
              <div>
                <span>SLAM</span>
                <strong>{session?.localization_backend ?? session?.slam_profile ?? sseState.slamStatus?.mode ?? '--'}</strong>
              </div>
              <div>
                <span>Navigation</span>
                <strong>{sseState.missionStatus?.state ?? 'IDLE'}</strong>
              </div>
            </div>
          </section>

          <section className={styles.panel}>
            <div className={styles.panelTitle}>
              <Route size={16} />
              <h2>Target Product Mode</h2>
            </div>
            <div className={styles.modeGrid}>
              {PRODUCT_MODES.map(item => {
                const active = item.profile === selectedProfile
                return (
                  <button
                    type="button"
                    key={item.profile}
                    className={active ? styles.modeCardActive : styles.modeCard}
                    onClick={() => {
                      setSelectedProfile(item.profile)
                      setSwitchResult(null)
                    }}
                  >
                    <span className={styles.modeIcon}>{item.icon}</span>
                    <span className={styles.modeBody}>
                      <strong>{item.label}</strong>
                      <small>{item.summary}</small>
                    </span>
                    <span className={item.policy === 'hot_candidate' ? styles.hotBadge : styles.coldBadge}>
                      {item.policy === 'hot_candidate' ? 'HOT' : 'COLD'}
                    </span>
                  </button>
                )
              })}
            </div>
          </section>

          <section className={styles.panel}>
            <div className={styles.panelTitle}>
              <Play size={16} />
              <h2>Switch Request</h2>
            </div>
            <div className={styles.formGrid}>
              <label>
                <span>Endpoint</span>
                <select value={endpoint} onChange={event => setEndpoint(event.target.value as typeof endpoint)}>
                  {ENDPOINTS.map(item => <option key={item} value={item}>{item}</option>)}
                </select>
              </label>
              <label>
                <span>Strategy</span>
                <select value={strategy} onChange={event => setStrategy(event.target.value as RuntimeSwitchRequest['strategy'])}>
                  {STRATEGIES.map(item => <option key={item} value={item}>{item}</option>)}
                </select>
              </label>
              <label>
                <span>Map</span>
                <select
                  value={selectedMap}
                  onChange={event => setSelectedMap(event.target.value)}
                  disabled={navigableMaps.length === 0}
                >
                  <option value="">{navigableMaps.length ? 'Select map' : 'No navigation-ready map'}</option>
                  {navigableMaps.map(map => <option key={map.name} value={map.name}>{map.name}</option>)}
                </select>
              </label>
              <label className={styles.toggleRow}>
                <input
                  type="checkbox"
                  checked={relocalize}
                  onChange={event => setRelocalize(event.target.checked)}
                  disabled={!selectedOption.requiresMap}
                />
                <span>Run relocalize when a map is used</span>
              </label>
              <label className={styles.toggleRow}>
                <input
                  type="checkbox"
                  checked={allowRestart}
                  onChange={event => setAllowRestart(event.target.checked)}
                />
                <span>Allow cold restart of robot-side services</span>
              </label>
            </div>

            {missingMap && (
              <div className={styles.inlineWarn}>
                <AlertTriangle size={15} />
                <span>{selectedOption.label} requires a saved navigation-ready map.</span>
              </div>
            )}

            <div className={styles.actions}>
              <button className={styles.secondaryButton} onClick={runPreflight} disabled={switchBusy || missingMap}>
                <CheckCircle2 size={15} />
                Preflight
              </button>
              <button className={styles.primaryButton} onClick={executeSwitch} disabled={executeDisabled} title={executeTitle}>
                <Play size={15} />
                Execute Switch
              </button>
            </div>
          </section>
        </main>

        <aside className={styles.side}>
          <section className={styles.panel}>
            <div className={styles.panelTitle}>
              <ShieldCheck size={16} />
              <h2>Switch Result</h2>
            </div>
            <div className={styles.resultHead}>
              <span className={styles[responseTone(switchResult)]}>
                {switchResult ? (switchResult.ok ? 'READY' : 'BLOCKED') : 'NO PLAN'}
              </span>
              <strong>{plannedLifecycle}</strong>
            </div>
            <div className={styles.resultRows}>
              <span>target</span><strong>{selectedProfile}</strong>
              <span>session</span><strong>{selectedOption.session}</strong>
              <span>status</span><strong>{switchResult?.status ?? '--'}</strong>
              <span>accepted</span><strong>{switchResult?.accepted === true ? 'yes' : 'no'}</strong>
              <span>command</span><strong>{switchResult?.command?.length ? switchResult.command.join(' ') : '--'}</strong>
            </div>
            {switchResult?.blockers && switchResult.blockers.length > 0 && (
              <div className={styles.blockerList}>
                {switchResult.blockers.map(item => <span key={item}>{item}</span>)}
              </div>
            )}
            {switchResult?.effects && switchResult.effects.length > 0 && (
              <div className={styles.effectsList}>
                {switchResult.effects.map(item => <span key={item}>{item}</span>)}
              </div>
            )}
          </section>

          <section className={styles.panel}>
            <div className={styles.panelTitle}>
              <Target size={16} />
              <h2>Visual Servo</h2>
            </div>
            <p className={styles.sideHint}>
              Hot target switch only works when the active graph already loads VisualServoModule.
            </p>
            <label className={styles.servoTarget}>
              <span>Target</span>
              <input
                value={servoTarget}
                onChange={event => setServoTarget(event.target.value)}
                placeholder="person, dock, red chair"
              />
            </label>
            <div className={styles.servoModes}>
              <button
                className={servoMode === 'find' ? styles.servoButtonActive : styles.servoButton}
                onClick={() => sendServo('find')}
                disabled={servoBusy || !servoTarget.trim()}
                title="Find a visual target and hand the far target to navigation"
              >
                <Search size={14} />
                Find
              </button>
              <button
                className={servoMode === 'follow' ? styles.servoButtonActive : styles.servoButton}
                onClick={() => sendServo('follow')}
                disabled={servoBusy || !servoTarget.trim()}
                title="Follow a detected visual target"
              >
                <Target size={14} />
                Follow
              </button>
              <button
                className={styles.stopButton}
                onClick={() => sendServo('stop')}
                disabled={servoBusy}
                title="Release visual servo command ownership"
              >
                <Square size={14} />
                Stop
              </button>
            </div>
            <div className={styles.servoStatus}>
              <span>Status</span>
              <strong>{servoStatus}</strong>
            </div>
          </section>
        </aside>
      </div>
    </section>
  )
}
