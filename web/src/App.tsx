import { useState, useEffect, useCallback } from 'react'
import { useSSE } from './hooks/useSSE'
import { useToast } from './hooks/useToast'
import { Topbar } from './components/Topbar'
import { CameraFeed } from './components/CameraFeed'
import { ChatPanel } from './components/ChatPanel'
import { LocalizationCard } from './components/LocalizationCard'
import { StatusBar } from './components/StatusBar'
import { MapView } from './components/MapView'
import { SlamPanel } from './components/SlamPanel'
import { SceneView } from './components/SceneView'
import { PlannerTuning } from './components/PlannerTuning'
import { ProductModePanel } from './components/ProductModePanel'
import { RuntimeDataflowView } from './components/RuntimeDataflowView'
import { InspectionWorkbench } from './components/InspectionWorkbench'
import { CurrentTaskCard } from './components/CurrentTaskCard'
import { useTheme } from './components/useTheme'
import { readStoredLocale, text, writeStoredLocale, type Locale } from './i18n'

import { ToastContainer } from './components/Toast'
import { LoginPage } from './components/LoginPage'
import { Landing } from './components/Landing'
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
    case MotionGateReason.EMERGENCY_STOP_NOT_ACTIVE:
      return text(locale, 'Emergency stop is not active', '当前没有激活的急停锁')
    case MotionGateReason.EMERGENCY_STOP_ACTIVE:
      return text(locale, 'Emergency stop is active', '急停锁仍处于激活状态')
    default:
      return text(locale, 'Robot state cannot safely authorize motion', '当前状态不足以安全授权运动')
  }
}

function Dashboard() {
  const sseState = useSSE('/api/v1/events')
  const authoritativeMission = sseState.navigationStatus?.mission.raw
  const activeTaskId = authoritativeMission?.active_task_id
  const activeRequestId = authoritativeMission?.active_request_id
  const { toasts, show: showToast, dismiss } = useToast()
  const { theme, resolvedTheme, setTheme } = useTheme()
  const [locale, setLocale] = useState<Locale>(() => readStoredLocale())
  const [uptimeSeconds, setUptimeSeconds] = useState(0)
  const [nowMs, setNowMs] = useState(() => Date.now())
  const [activeTab, setActiveTab] = useState<Tab>('console')
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
  const freshness = { nowMs, maxAgeMs: MOTION_TRUTH_MAX_AGE_MS }
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
      request_id: activeRequestId,
    })
  }, [activeRequestId, activeTaskId])

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
        onTabChange={setActiveTab}
        theme={theme}
        resolvedTheme={resolvedTheme}
        onThemeChange={setTheme}
        locale={locale}
        onLocaleChange={handleLocaleChange}
      />

      <main className="main-content" key={activeTab}>
        {activeTab === 'console' && (
          <div className="console-canvas" role="tabpanel" id="panel-console">
            <div className="console-grid">
              <section className="console-camera" aria-label="camera feed">
                <CameraFeed
                  onStop={handleStop}
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
              <section className="console-mode" aria-label="product mode">
                <ProductModePanel sseState={sseState} showToast={showToast} locale={locale} />
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
          <SceneView
            sseState={sseState}
            showToast={showToast}
            locale={locale}
            motionStartAllowed={motionStartGate.allowed}
            motionStartBlockedReason={motionStartBlockedReason}
          />
        )}
        {activeTab === 'map' && (
          <MapView
            showToast={showToast}
            locale={locale}
            motionStartAllowed={motionStartGate.allowed}
            motionStartBlockedReason={motionStartBlockedReason}
          />
        )}
        {activeTab === 'slam' && <SlamPanel sseState={sseState} showToast={showToast} locale={locale} />}
        {activeTab === 'dataflow' && <RuntimeDataflowView sseState={sseState} />}
        {activeTab === 'inspection' && (
          <InspectionWorkbench sseState={sseState} showToast={showToast} locale={locale} />
        )}
        {activeTab === 'planner' && <PlannerTuning showToast={showToast} />}

      </main>

      <CurrentTaskCard
        locale={locale}
        showToast={showToast}
        resumeAllowed={motionResumeGate.allowed}
        resumeBlockedReason={motionResumeBlockedReason}
      />
      <StatusBar sseState={sseState} uptimeSeconds={uptimeSeconds} locale={locale} />
      <ToastContainer toasts={toasts} dismiss={dismiss} />
    </div>
  )
}

function App() {
  // Landing page: ?landing bypasses auth and shows the marketing page
  const isLanding = typeof window !== 'undefined' &&
    new URLSearchParams(window.location.search).has('landing')

  // Dev preview: ?login forces the login page to render
  const forceLogin = typeof window !== 'undefined' &&
    new URLSearchParams(window.location.search).has('login')
  const bypassAuth = isLanding || forceLogin

  const [authChecked, setAuthChecked] = useState(bypassAuth)
  const [loggedIn, setLoggedIn] = useState(false)

  useEffect(() => {
    // Landing page needs scroll — remove dashboard overflow:hidden from html/body
    if (isLanding) {
      document.documentElement.style.overflow = 'auto'
      document.documentElement.style.height = 'auto'
      document.body.style.overflow = 'auto'
      document.body.style.height = 'auto'
      const root = document.getElementById('root')
      if (root) { root.style.overflow = 'auto'; root.style.height = 'auto' }
    }
    return () => {
      if (isLanding) {
        document.documentElement.style.overflow = ''
        document.documentElement.style.height = ''
        document.body.style.overflow = ''
        document.body.style.height = ''
        const root = document.getElementById('root')
        if (root) { root.style.overflow = ''; root.style.height = '' }
      }
    }
  }, [isLanding])

  useEffect(() => {
    if (bypassAuth) return
    api.checkAuth()
      .then(data => {
        setLoggedIn(!data.auth_required)
        setAuthChecked(true)
      })
      .catch(() => {
        // Backend offline — skip auth
        setLoggedIn(true)
        setAuthChecked(true)
      })
  }, [bypassAuth])

  if (isLanding) return <Landing />
  if (!authChecked) return null
  if (!loggedIn) return <LoginPage onLogin={() => setLoggedIn(true)} />
  return <Dashboard />
}

export default App
