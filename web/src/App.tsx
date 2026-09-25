import { isObservationMode } from './services/observationMode.ts'
import { lazy, Suspense, useState, useEffect, useCallback } from 'react'
import { useSSE } from './hooks/useSSE'
import { useToast } from './hooks/useToast'
import { useProductControl } from './hooks/useProductControl'
import { Topbar } from './components/Topbar'
import { CameraFeed } from './components/CameraFeed'
import { ChatPanel } from './components/ChatPanel'
import { LocalizationCard } from './components/LocalizationCard'
import { StatusBar } from './components/StatusBar'
const MapView = lazy(() => import('./components/MapView').then(m => ({ default: m.MapView })))
const SlamStatusPanel = lazy(() => import('./components/SlamStatusPanel').then(m => ({ default: m.SlamStatusPanel })))
import { SceneView } from './components/SceneView'
const PlannerTuning = lazy(() => import('./components/PlannerTuning').then(m => ({ default: m.PlannerTuning })))
import { RobotStatusPanel } from './components/RobotStatusPanel'
const RuntimeDataflowView = lazy(() => import('./components/RuntimeDataflowView').then(m => ({ default: m.RuntimeDataflowView })))
const InspectionWorkbench = lazy(() => import('./components/InspectionWorkbench').then(m => ({ default: m.InspectionWorkbench })))
import { CurrentTaskCard } from './components/CurrentTaskCard'
import { useTheme } from './components/useTheme'
import { readStoredLocale, text, writeStoredLocale, type Locale } from './i18n'

import { ToastContainer } from './components/Toast'
import * as api from './services/api'
import { currentNavigationTaskStore } from './services/currentNavigationTask'
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
  const [elevationSubscription, setElevationSubscription] = useState(false)
  const sseState = useSSE(
    elevationSubscription ? '/api/v1/events?include_elevation=1' : '/api/v1/events',
  )
  const activeTaskId = sseState.navigationStatus?.task.task_id
  const { toasts, show: showToast, dismiss } = useToast()
  const { theme, resolvedTheme, setTheme } = useTheme()
  const [locale, setLocale] = useState<Locale>(() => readStoredLocale())
  const [uptimeSeconds, setUptimeSeconds] = useState(0)
  const [nowMs, setNowMs] = useState(() => Date.now())
  const [activeTab, setActiveTab] = useState<Tab>('scene')
  const [selectedSavedMap, setSelectedSavedMap] = useState<string | null>(null)
  const handleProductChanged = useCallback(() => setActiveTab('scene'), [])
  const productControl = useProductControl(handleProductChanged)
  const handleOpenSavedMap = useCallback((name: string | null) => {
    setSelectedSavedMap(name)
    setActiveTab('map')
  }, [])
  const handleTabChange = useCallback((tab: Tab) => {
    setSelectedSavedMap(null)
    setActiveTab(tab)
  }, [])
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

  useEffect(() => {
    const t = setInterval(() => {
      setUptimeSeconds(s => s + 1)
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
    <div className="app">
      <Topbar
        sseState={sseState}
        activeTab={activeTab}
        onTabChange={handleTabChange}
        theme={theme}
        resolvedTheme={resolvedTheme}
        onThemeChange={setTheme}
        locale={locale}
        onLocaleChange={handleLocaleChange}
        onStop={handleStop}
      />

      <main className="main-content" key={activeTab}>
        <Suspense fallback={<div className="view-loading" role="status">{text(locale, 'Loading view…', '正在打开视图…')}</div>}>
        {activeTab === 'console' && (
          <div className="console-canvas" role="tabpanel" id="panel-console">
            <div className="console-grid">
              <section className="console-camera" aria-label="camera feed">
                <CameraFeed
                  onResetEstop={handleResetEstop}
                  estop={estop}
                  resetBusy={estopResetBusy}
                  resetAllowed={estopResetGate.allowed}
                  resetBlockedReason={estopResetBlockedReason}
                  sseState={sseState}
                  locale={locale}
                />
              </section>
              <section className="console-localization" aria-label="localization">
                <LocalizationCard sseState={sseState} />
              </section>
              <section className="console-mode" aria-label="robot status">
                <RobotStatusPanel
                  sseState={sseState}
                  showToast={showToast}
                  locale={locale}
                  motionStartAllowed={motionStartGate.allowed}
                  motionStartBlockedReason={motionStartBlockedReason}
                />
              </section>
              <section className="console-chat" aria-label="assistant">
                <ChatPanel
                  sseState={sseState}
                  motionStartAllowed={motionStartGate.allowed}
                  motionStartBlockedReason={motionStartBlockedReason}
                />
              </section>
            </div>
          </div>
        )}
        {activeTab === 'scene' && (
          <>
          {productControl.message && <div className="product-transition" role="status">
            <span>{productControl.message}</span>
            {!productControl.busy && <button onClick={productControl.dismiss}>关闭</button>}
          </div>}
          <SceneView
            sseState={sseState}
            showToast={showToast}
            locale={locale}
            motionStartAllowed={motionStartGate.allowed}
            motionStartBlockedReason={motionStartBlockedReason}
            onElevationSubscriptionChange={setElevationSubscription}
            onOpenSavedMap={handleOpenSavedMap}
            onStartMapping={() => void productControl.switchProduct('map')}
            productSwitchAllowed={productControl.allowed}
            productSwitchReason={productControl.reason}
          />
          </>
        )}
        {activeTab === 'map' && (
          <MapView
            initialSelectedMap={selectedSavedMap}
            onUseMap={(name, pose) => void productControl.switchProduct('nav', name, pose)}
            productSwitchAllowed={productControl.allowed}
            productSwitchReason={productControl.reason}
            productSwitchMessage={productControl.message}
            onReturnLive={() => handleTabChange('scene')}
            session={sseState.session}
            showToast={showToast}
            locale={locale}
            motionStartAllowed={motionStartGate.allowed}
            motionStartBlockedReason={motionStartBlockedReason}
          />
        )}
        {activeTab === 'slam' && <SlamStatusPanel sseState={sseState} showToast={showToast} locale={locale} />}
        {activeTab === 'dataflow' && <RuntimeDataflowView sseState={sseState} />}
        {activeTab === 'inspection' && (
          <InspectionWorkbench sseState={sseState} showToast={showToast} locale={locale} />
        )}
        {activeTab === 'planner' && <PlannerTuning showToast={showToast} />}

        </Suspense>
      </main>

      {!isObservationMode() && <CurrentTaskCard
        locale={locale}
        showToast={showToast}
        resumeAllowed={motionResumeGate.allowed}
        resumeBlockedReason={motionResumeBlockedReason}
      />
      }
      {activeTab !== 'scene' && activeTab !== 'map' && activeTab !== 'inspection' && <StatusBar sseState={sseState} uptimeSeconds={uptimeSeconds} locale={locale} />}
      <ToastContainer toasts={toasts} dismiss={dismiss} />
    </div>
  )
}

function App() {
  return <Dashboard />
}

export default App
