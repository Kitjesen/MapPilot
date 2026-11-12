import { isObservationMode } from './services/observationMode.ts'
import { initialMapEntry } from './services/mapEntry.ts'
import { lazy, Suspense, useState, useEffect, useCallback, useRef, useSyncExternalStore } from 'react'
import { createPortal } from 'react-dom'
import { useSSE } from './hooks/useSSE'
import { useToast } from './hooks/useToast'
import { useProductControl } from './hooks/useProductControl'
import { Topbar } from './components/Topbar'
import { useMenuDismiss } from './components/menuEvents.ts'
import { MapSettingsProvider } from './components/MapSettings.tsx'
const MapView = lazy(() => import('./components/MapView').then(m => ({ default: m.MapView })))
import { SceneView } from './components/SceneView'
import { CurrentTaskCard } from './components/CurrentTaskCard'
import { InstructionTaskBar } from './components/InstructionTaskBar'
import { AgentConversation } from './components/AgentConversation'
import { SensorMonitorProvider } from './components/SensorMonitor'
import { SharedConversation } from './components/SharedConversation'
import { useInstructionTask } from './hooks/useInstructionTask'
import { useAgentConversation } from './hooks/useAgentConversation'
import { instructionTaskCanDismiss } from './services/instructionTask'
import { useTheme } from './components/useTheme'
import { readStoredLocale, text, writeStoredLocale, type Locale } from './i18n'

import { ToastContainer } from './components/Toast'
import * as api from './services/api'
import { currentNavigationTaskStore } from './services/currentNavigationTask'
import { liveNavigationStatus } from './services/navigationStatus'
import {
  MotionAction,
  MotionGateReason,
  evaluateMotionAction,
} from './services/motionTruth'
import type { Tab } from './types'
import './App.css'

const MOTION_TRUTH_MAX_AGE_MS = 7_000

function motionBlockedMessage(reason: string, locale: Locale): string {
  switch (reason) {
    case MotionGateReason.DISCONNECTED:
      return text(locale, 'Realtime connection is offline', '实时连接已断开')
    case MotionGateReason.INITIAL_SNAPSHOT_MISSING:
      return text(locale, 'Waiting for authoritative robot state', '正在等待机器人权威状态')
    case MotionGateReason.REFRESH_ERROR:
      return text(locale, 'Authoritative state refresh failed', '机器人权威状态刷新失败')
    case MotionGateReason.STALE_TIMESTAMP:
      return text(locale, 'Robot state is stale', '机器人状态已过期')
    case MotionGateReason.INVALID_TIMESTAMP:
      return text(locale, 'Robot state timestamp is invalid', '状态时间异常，等待更新')
    case MotionGateReason.SNAPSHOT_NOT_AUTHORITATIVE:
      return text(locale, 'Waiting for confirmed robot state', '等待机器人状态确认')
    case MotionGateReason.EMERGENCY_STOP_NOT_ACTIVE:
      return text(locale, 'Emergency stop is not active', '当前没有激活的急停锁')
    case MotionGateReason.EMERGENCY_STOP_ACTIVE:
      return text(locale, 'Emergency stop is active', '急停锁仍处于激活状态')
    default:
      return text(locale, 'Motion is not ready', '运动尚未就绪')
  }
}

function Dashboard() {
  const appRef = useRef<HTMLDivElement>(null)
  useMenuDismiss(appRef)
  const [mapEntry] = useState(() => initialMapEntry(window.location.search))
  const [taskPanelTarget, setTaskPanelTarget] = useState<HTMLDivElement | null>(null)
  const [globalTaskPanelTarget, setGlobalTaskPanelTarget] = useState<HTMLDivElement | null>(null)
  const [elevationSubscription, setElevationSubscription] = useState(false)
  const { toasts, show: showToast, dismiss } = useToast()
  const { theme, resolvedTheme, setTheme } = useTheme()
  const [locale, setLocale] = useState<Locale>(() => readStoredLocale())
  const [nowMs, setNowMs] = useState(() => Date.now())
  const [activeTab, setActiveTab] = useState<Tab>(mapEntry.tab)
  const [mapReturnTab, setMapReturnTab] = useState<'scene' | 'mapping'>('scene')
  const sceneWorkspace = activeTab === 'scene' || activeTab === 'mapping'
  const currentTaskPanelTarget = sceneWorkspace ? taskPanelTarget : globalTaskPanelTarget
  const [selectedSavedMap, setSelectedSavedMap] = useState<string | null>(mapEntry.map)
  const mapEditorExitGuard = useRef<((proceed: () => void) => void) | null>(null)
  const handleMapEditorExitGuardChange = useCallback((guard: ((proceed: () => void) => void) | null) => {
    mapEditorExitGuard.current = guard
  }, [])
  const leaveMapEditor = useCallback((proceed: () => void) => {
    if (mapEditorExitGuard.current) mapEditorExitGuard.current(proceed)
    else proceed()
  }, [])
  const handleProductChanged = useCallback((product: 'map' | 'nav' | 'inspection', restoring: boolean) => {
    setActiveTab(current => {
      if (restoring && current !== 'scene') return current
      return product === 'map' ? 'mapping' : 'scene'
    })
  }, [])
  const productControl = useProductControl(handleProductChanged)
  const standby = productControl.available && productControl.current?.status === 'stopped' && !productControl.busy
  const sseState = useSSE(
    elevationSubscription ? '/api/v1/events?include_elevation=1' : '/api/v1/events',
    !standby,
  )
  const liveNavigation = liveNavigationStatus(sseState, nowMs / 1000)
  const activeTaskId = liveNavigation?.task.task_id
  const handleOpenSavedMap = useCallback((name: string | null) => {
    leaveMapEditor(() => {
      setMapReturnTab(activeTab === 'mapping' ? 'mapping' : 'scene')
      setSelectedSavedMap(name)
      setActiveTab('map')
    })
  }, [activeTab, leaveMapEditor])
  const handleTabChange = useCallback((tab: Tab) => {
    const proceed = () => {
      if (tab === 'map') setMapReturnTab(activeTab === 'mapping' ? 'mapping' : 'scene')
      setSelectedSavedMap(null)
      setActiveTab(tab)
    }
    if (tab === activeTab) proceed()
    else leaveMapEditor(proceed)
  }, [activeTab, leaveMapEditor])
  const [estopResetIssuedAt, setEstopResetIssuedAt] = useState<number | null>(null)
  const estop = sseState.safetyState?.estop ?? false
  const motionTruth = {
    connected: sseState.connected,
    refreshError: sseState.truthError,
    snapshot: sseState.lastTruthAt === null
      ? null
      : {
          authoritative: sseState.authoritativeStateSeen,
          timestampMs: sseState.lastTruthAt,
          emergencyStopActive: estop,
        },
  }
  // Both values are browser receipt times; a new receipt may precede the next
  // UI timer tick. Use the latest observed local clock, not the older tick.
  const freshness = { nowMs: Math.max(nowMs, sseState.lastTruthAt ?? 0), maxAgeMs: MOTION_TRUTH_MAX_AGE_MS }
  const motionStartGate = evaluateMotionAction(MotionAction.START, motionTruth, freshness)
  const estopResetGate = evaluateMotionAction(
    MotionAction.RESET_EMERGENCY_STOP,
    motionTruth,
    freshness,
  )
  const motionResumeGate = evaluateMotionAction(MotionAction.RESUME, motionTruth, freshness)
  const motionStartBlockedReason = motionStartGate.allowed
    ? ''
    : motionBlockedMessage(motionStartGate.reason, locale)
  const motionResumeBlockedReason = motionResumeGate.allowed
    ? ''
    : motionBlockedMessage(motionResumeGate.reason, locale)
  const estopResetBlockedReason = estopResetGate.allowed
    ? ''
    : motionBlockedMessage(estopResetGate.reason, locale)
  const estopResetBusy = estopResetIssuedAt !== null && estop
  const instructionTask = useInstructionTask({
    enabled: !standby,
    motionStartAllowed: motionStartGate.allowed,
    motionStartBlockedReason,
  })
  const agentConversation = useAgentConversation(instructionTask.adopt, !standby, sseState.agentMessage)
  const nativeTaskIdentity = useSyncExternalStore(
    currentNavigationTaskStore.subscribe,
    currentNavigationTaskStore.getSnapshot,
  )
  const ownedNavigation = instructionTask.task?.status.navigation
  const instructionOwnsNativeTask = ownedNavigation?.owned === true
    && Boolean(nativeTaskIdentity?.task_id)
    && ownedNavigation.task_id === nativeTaskIdentity?.task_id
  const instructionActive = Boolean(instructionTask.task && !instructionTaskCanDismiss(instructionTask.task))

  useEffect(() => {
    const t = setInterval(() => {
      setNowMs(Date.now())
    }, 1000)
    return () => clearInterval(t)
  }, [])

  const handleStop = useCallback(async () => {
    try {
      const response = await api.sendStop()
      showToast(
        text(
          locale,
          response.ok ? 'Emergency stop submitted' : 'Emergency stop was not accepted',
          api.formatCommandAck(response, '紧急停止'),
        ),
        response.ok ? 'info' : 'error',
      )
    } catch (error: unknown) {
      showToast(
        api.formatCommandError(error, text(locale, 'Emergency stop failed', '紧急停止失败')),
        'error',
      )
    }
  }, [locale, showToast])

  const handleResetEstop = useCallback(async () => {
    if (!estopResetGate.allowed) {
      showToast(estopResetBlockedReason, 'error')
      return
    }
    const confirmed = window.confirm(text(
      locale,
      'Release the software emergency-stop latch? The robot remains stopped and the old task will not resume automatically.',
      '解除软件急停锁？机器人仍会保持停止，旧任务不会自动恢复。',
    ))
    if (!confirmed) return

    setEstopResetIssuedAt(Date.now())
    try {
      const response = await api.resetEstop()
      if (!response.ok) {
        setEstopResetIssuedAt(null)
        showToast(api.formatCommandAck(response, text(locale, 'E-stop reset', '解除急停')), 'error')
        return
      }
      showToast(
        text(locale, 'Reset accepted; waiting for fresh safety confirmation', '解除请求已受理，等待新的安全状态确认'),
        'info',
      )
    } catch (error: unknown) {
      setEstopResetIssuedAt(null)
      showToast(
        api.formatCommandError(error, text(locale, 'E-stop reset failed', '解除急停失败')),
        'error',
      )
    }
  }, [estopResetBlockedReason, estopResetGate.allowed, locale, showToast])

  const handleLocaleChange = useCallback((nextLocale: Locale) => {
    setLocale(nextLocale)
    writeStoredLocale(nextLocale)
    document.documentElement.lang = nextLocale === 'zh' ? 'zh-CN' : 'en'
  }, [])

  useEffect(() => {
    document.documentElement.lang = locale === 'zh' ? 'zh-CN' : 'en'
  }, [locale])

  useEffect(() => {
    currentNavigationTaskStore.adoptAuthoritative({
      task_id: activeTaskId,
    })
  }, [activeTaskId])

  useEffect(() => {
    if (estopResetIssuedAt === null || estop || !motionStartGate.allowed) return undefined
    const confirmationTimer = window.setTimeout(() => {
      setEstopResetIssuedAt(null)
      showToast(
        text(locale, 'Emergency-stop release confirmed; robot remains stopped', '急停锁已确认解除；机器人仍保持停止'),
        'success',
      )
    }, 0)
    return () => window.clearTimeout(confirmationTimer)
  }, [estop, estopResetIssuedAt, locale, motionStartGate.allowed, showToast])

  return (
    <SensorMonitorProvider sseState={sseState} localNowS={nowMs / 1000} locale={locale} onAnalyze={(id, start, end) => {
      const prompt = `请读取 IMU 记录 ${id} 中 ${start.toFixed(3)} 到 ${end.toFixed(3)} 秒的观测，并用 Python 分析角速度和加速度。说明缺失数据与时间间断。`
      agentConversation.setDraft(agentConversation.draft.trim() ? `${agentConversation.draft}\n\n${prompt}` : prompt)
      agentConversation.setOpen(true)
    }}>
    <div ref={appRef} className="app">
      <Topbar
        sseState={sseState}
        standby={standby}
        localNowS={nowMs / 1000}
        activeTab={activeTab === 'map' ? mapReturnTab : activeTab}
        onTabChange={handleTabChange}
        theme={theme}
        resolvedTheme={resolvedTheme}
        onThemeChange={setTheme}
        locale={locale}
        onLocaleChange={handleLocaleChange}
        onStop={handleStop}
        showToast={showToast}
      />

      <main className="main-content" data-scene={sceneWorkspace} key={activeTab}>
        {(productControl.message || standby) && <div className="product-transition" role="status">
          <span>{productControl.message ? productControl.message : text(locale, 'Standby. Ready to start mapping.', '机器人待机中，可以开始建图。')}</span>
          {standby && <button disabled={!productControl.allowed} onClick={() => void productControl.switchProduct('map')}>{text(locale, 'Start mapping', '开始建图')}</button>}
          {!standby && !productControl.busy && <button onClick={productControl.dismiss} aria-label="关闭模式切换提示">关闭</button>}
        </div>}
        {!standby && <Suspense fallback={<div className="view-loading" role="status">{text(locale, 'Loading view…', '正在打开视图…')}</div>}>
        {sceneWorkspace && (
          <>
          <SceneView
            workspace={activeTab === 'mapping' ? 'mapping' : 'navigation'}
            instructionStatus={instructionTask.task?.status ?? null}
            onResetEstop={handleResetEstop}
            resetEstopReason={estopResetBusy ? text(locale, 'Waiting for confirmation', '等待解除确认') : estopResetBlockedReason}
            onTaskPanelMount={setTaskPanelTarget}
            sseState={sseState}
            showToast={showToast}
            locale={locale}
            instructionActive={instructionActive}
            motionStartAllowed={motionStartGate.allowed && !instructionActive}
            motionStartBlockedReason={instructionActive
              ? text(locale, 'End the current instruction before starting a map goal', '请先结束当前指令任务，再开始地图导航')
              : motionStartBlockedReason}
            onElevationSubscriptionChange={setElevationSubscription}
            onOpenSavedMap={handleOpenSavedMap}
            onMapEditorExitGuardChange={handleMapEditorExitGuardChange}
            onStartMapping={() => void productControl.switchProduct('map')}
            productSwitchAllowed={productControl.allowed}
            productSwitchReason={productControl.reason}
          />
          </>
        )}
        {activeTab === 'map' && (
          <MapView
            initialSelectedMap={selectedSavedMap}
            onMapEditorExitGuardChange={handleMapEditorExitGuardChange}
            onUseMap={(name, pose) => void productControl.switchProduct('nav', name, pose)}
            productSwitchAllowed={productControl.allowed}
            productSwitchReason={productControl.reason}
            onReturnLive={() => handleTabChange(mapReturnTab)}
            returnLabel={mapReturnTab === 'mapping' ? text(locale, 'Mapping', '建图') : text(locale, 'Navigation', '导航')}
            session={sseState.session}
            slamDiag={sseState.slamDiag?.data ?? null}
            showToast={showToast}
            locale={locale}
          />
        )}
        </Suspense>}
      </main>

      <div data-chat-slot="global" className="global-task-slot" ref={setGlobalTaskPanelTarget} />
      {!standby && !isObservationMode() && currentTaskPanelTarget && createPortal(
        <>
        <InstructionTaskBar
          controller={instructionTask}
          showComposer={false}
          locale={locale}
          motionStartAllowed={motionStartGate.allowed}
          motionStartBlockedReason={sseState.connected ? motionStartBlockedReason : ''}
          hasNativeTask={Boolean(nativeTaskIdentity) && !instructionOwnsNativeTask}
        />
        <AgentConversation locale={locale} controller={agentConversation} />
        </>,
        currentTaskPanelTarget,
      )}
      {!standby && !isObservationMode() && !instructionOwnsNativeTask && <CurrentTaskCard
        container={currentTaskPanelTarget}
        locale={locale}
        showToast={showToast}
        resumeAllowed={motionResumeGate.allowed}
        resumeBlockedReason={motionResumeBlockedReason}
      />
      }
      <ToastContainer toasts={toasts} dismiss={dismiss} />
    </div>
    </SensorMonitorProvider>
  )
}

function App() {
  const shareToken = new URLSearchParams(window.location.search).get('conversation-share')
  if (shareToken) return <SharedConversation token={shareToken} locale={readStoredLocale()} />
  return <MapSettingsProvider><Dashboard /></MapSettingsProvider>
}

export default App
