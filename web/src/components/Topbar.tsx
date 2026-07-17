import { useState } from 'react'
import { Settings } from 'lucide-react'
import type { SSEState, Tab } from '../types'
import { SettingsMenu } from './SettingsMenu'
import type { ResolvedTheme, Theme } from './useTheme'
import { text, type Locale } from '../i18n'
import styles from './Topbar.module.css'

const PRODUCT_TABS: { key: Tab; en: string; zh: string }[] = [
  { key: 'console', en: 'Console', zh: '控制台' },
  { key: 'scene', en: 'Scene', zh: '场景' },
  { key: 'map', en: 'Maps', zh: '地图' },
  { key: 'slam', en: 'Localization', zh: '定位' },
  { key: 'inspection', en: 'Inspect', zh: '巡检' },
]

const INTERNAL_TABS: { key: Tab; en: string; zh: string }[] = [
  { key: 'dataflow', en: 'Data', zh: '数据' },
  { key: 'planner', en: 'Planning', zh: '规划' },
]

interface TopbarProps {
  sseState: SSEState
  activeTab: Tab
  onTabChange: (tab: Tab) => void
  theme: Theme
  resolvedTheme: ResolvedTheme
  onThemeChange: (theme: Theme) => void
  locale: Locale
  onLocaleChange: (locale: Locale) => void
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

function showInternalTabs(): boolean {
  if (typeof window === 'undefined') return false
  const params = new URLSearchParams(window.location.search)
  return params.has('debug_nav')
}

export function Topbar({
  sseState,
  activeTab,
  onTabChange,
  theme,
  resolvedTheme,
  onThemeChange,
  locale,
  onLocaleChange,
}: TopbarProps) {
  const [settingsOpen, setSettingsOpen] = useState(false)
  const tabs = showInternalTabs() ? [...PRODUCT_TABS, ...INTERNAL_TABS] : PRODUCT_TABS
  const slamDiag = sseState.slamDiag?.data ?? {}
  const scanHz = numericMetric(slamDiag, 'processed_scan_hz') ?? sseState.slamStatus?.slam_hz ?? 0
  const lidarHz = numericMetric(slamDiag, 'lidar_input_hz')
  const hasCloud = !!sseState.mapCloud
  const slamMode = sseState.slamStatus?.mode ?? null
  const slamPipClass = scanHz > 3 ? styles.pipGood : scanHz > 0 ? styles.pipWarn : styles.pipBad
  const cloudPipClass = hasCloud ? styles.pipGood : styles.pipBad

  const handleKeyDown = (e: React.KeyboardEvent, idx: number) => {
    if (e.key !== 'ArrowLeft' && e.key !== 'ArrowRight') return
    e.preventDefault()
    const delta = e.key === 'ArrowRight' ? 1 : -1
    const next = (idx + delta + tabs.length) % tabs.length
    onTabChange(tabs[next].key)
  }

  return (
    <header className={styles.topbar}>
      <span className={styles.logo}>
        <span className={styles.logoIcon} aria-hidden="true">
          <svg width="15" height="15" viewBox="0 0 32 32" fill="none" xmlns="http://www.w3.org/2000/svg">
            <circle cx="16" cy="16" r="11.5" stroke="url(#tbRing)" strokeWidth="1.5" fill="none" strokeDasharray="7 3.5" strokeLinecap="round" />
            <path d="M16 5.5 L23.5 21 L16 17.8 L8.5 21 Z" fill="url(#tbArrow)" />
            <path d="M16 10.5 L20.5 19.5 L16 17.8 L11.5 19.5 Z" fill="rgba(14,16,32,0.85)" />
            <circle cx="16" cy="25.5" r="1.8" fill="#22d3ee" opacity="0.9" />
            <circle cx="16" cy="25.5" r="1" fill="rgba(14,16,32,0.85)" />
            <defs>
              <linearGradient id="tbArrow" x1="8" y1="5" x2="24" y2="24" gradientUnits="userSpaceOnUse">
                <stop offset="0" stopColor="#818cf8" />
                <stop offset="1" stopColor="#22d3ee" />
              </linearGradient>
              <linearGradient id="tbRing" x1="5" y1="5" x2="27" y2="27" gradientUnits="userSpaceOnUse">
                <stop offset="0" stopColor="#6366f1" stopOpacity="0.8" />
                <stop offset="1" stopColor="#06b6d4" stopOpacity="0.8" />
              </linearGradient>
            </defs>
          </svg>
        </span>
        LingTu
      </span>

      <div className={styles.middle}>
        <nav className={styles.tabs} role="tablist" aria-label={text(locale, 'Main view switcher', '主视图切换')}>
          {tabs.map((tab, i) => (
            <button
              key={tab.key}
              role="tab"
              tabIndex={activeTab === tab.key ? 0 : -1}
              aria-selected={activeTab === tab.key}
              aria-controls={`panel-${tab.key}`}
              className={activeTab === tab.key ? styles.tabActive : styles.tab}
              onClick={() => onTabChange(tab.key)}
              onKeyDown={(e) => handleKeyDown(e, i)}
            >
              {text(locale, tab.en, tab.zh)}
            </button>
          ))}
        </nav>
      </div>

      <div className={styles.right}>
        <span
          className={sseState.connected ? styles.badgeOnline : styles.badgeOffline}
          title={text(
            locale,
            `Gateway SSE ${sseState.connected ? 'connected' : 'disconnected'}`,
            `网关 SSE ${sseState.connected ? '已连接' : '已断开'}`,
          )}
        >
          {sseState.connected ? text(locale, 'Online', '在线') : text(locale, 'Offline', '离线')}
        </span>
        <span
          className={`${styles.miniPip} ${slamPipClass}`}
          title={text(
            locale,
            `Localization processing ${scanHz.toFixed(1)} Hz${lidarHz ? ` / LiDAR input ${lidarHz.toFixed(1)} Hz` : ''} / ${slamMode ?? '--'}`,
            `定位处理 ${scanHz.toFixed(1)} Hz${lidarHz ? ` / 雷达输入 ${lidarHz.toFixed(1)} Hz` : ''} / ${slamMode ?? '--'}`,
          )}
        >
          {text(locale, 'Localization', '定位')}
        </span>
        <span
          className={`${styles.miniPip} ${cloudPipClass}`}
          title={text(locale, `Point cloud ${hasCloud ? 'active' : 'no data'}`, `点云 ${hasCloud ? '活跃' : '无数据'}`)}
        >
          {text(locale, 'Environment', '环境')}
        </span>
        {sseState.safetyState?.estop && <span className={styles.estop}>{text(locale, 'E-STOP', '急停')}</span>}
        <button
          className={styles.btnIcon}
          aria-label={text(locale, 'Settings', '设置')}
          aria-expanded={settingsOpen}
          onClick={() => setSettingsOpen(v => !v)}
        >
          <Settings size={16} />
        </button>
      </div>

      <SettingsMenu
        open={settingsOpen}
        onClose={() => setSettingsOpen(false)}
        theme={theme}
        resolvedTheme={resolvedTheme}
        onThemeChange={onThemeChange}
        locale={locale}
        onLocaleChange={onLocaleChange}
        onNavigateTab={onTabChange}
      />
    </header>
  )
}
