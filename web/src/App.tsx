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
import { useTheme } from './components/useTheme'
import { readStoredLocale, writeStoredLocale, type Locale } from './i18n'

import { ToastContainer } from './components/Toast'
import { LoginPage } from './components/LoginPage'
import { Landing } from './components/Landing'
import * as api from './services/api'
import type { Tab } from './types'
import './App.css'

function Dashboard() {
  const sseState = useSSE('/api/v1/events')
  const { toasts, show: showToast, dismiss } = useToast()
  const { theme, resolvedTheme, setTheme } = useTheme()
  const [locale, setLocale] = useState<Locale>(() => readStoredLocale())
  const [uptimeSeconds, setUptimeSeconds] = useState(0)
  const [activeTab, setActiveTab] = useState<Tab>('console')

  useEffect(() => {
    const t = setInterval(() => setUptimeSeconds(s => s + 1), 1000)
    return () => clearInterval(t)
  }, [])

  const handleStop = useCallback(async () => {
    try {
      await api.sendStop()
    } catch { /* best-effort */ }
  }, [])

  const handleLocaleChange = useCallback((nextLocale: Locale) => {
    setLocale(nextLocale)
    writeStoredLocale(nextLocale)
    document.documentElement.lang = nextLocale === 'zh' ? 'zh-CN' : 'en'
  }, [])

  useEffect(() => {
    document.documentElement.lang = locale === 'zh' ? 'zh-CN' : 'en'
  }, [locale])

  const estop = sseState.safetyState?.estop ?? false

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
                <CameraFeed onStop={handleStop} estop={estop} sseState={sseState} />
              </section>
              <section className="console-localization" aria-label="localization">
                <LocalizationCard sseState={sseState} />
              </section>
              <section className="console-mode" aria-label="product mode">
                <ProductModePanel sseState={sseState} showToast={showToast} locale={locale} />
              </section>
              <section className="console-chat" aria-label="assistant">
                <ChatPanel sseState={sseState} />
              </section>
            </div>
          </div>
        )}
        {activeTab === 'scene' && <SceneView sseState={sseState} showToast={showToast} locale={locale} />}
        {activeTab === 'map' && <MapView showToast={showToast} locale={locale} />}
        {activeTab === 'slam' && <SlamPanel sseState={sseState} showToast={showToast} locale={locale} />}
        {activeTab === 'dataflow' && <RuntimeDataflowView sseState={sseState} />}
        {activeTab === 'inspection' && (
          <InspectionWorkbench sseState={sseState} showToast={showToast} locale={locale} />
        )}
        {activeTab === 'planner' && <PlannerTuning showToast={showToast} />}

      </main>

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
