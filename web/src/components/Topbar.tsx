import { useState } from 'react'
import { Settings, Radio, RotateCcw } from 'lucide-react'
import { resetAllLayouts } from './floatingWidgetLayout'
import type { SSEState, Tab } from '../types'
import { BagRecorder } from './BagRecorder'
import { SettingsMenu } from './SettingsMenu'
import styles from './Topbar.module.css'

const SLAM_MODE_ZH: Record<string, string> = {
  fastlio2:  '建图',
  localizer: '导航',
  stop:      '停止',
}

const NAV_STATE_ZH: Record<string, string> = {
  IDLE: '空闲',
  EXECUTING: '执行中',
  PLANNING: '规划中',
  ARRIVED: '已到达',
  FAILED: '失败',
  CANCELLED: '已取消',
}

const NAV_STATE_CLASS: Record<string, string> = {
  executing: styles.navExecuting,
  failed:    styles.navFailed,
  planning:  styles.navPlanning,
}

const SLAM_MODE_CLASS: Record<string, string> = {
  fastlio2:  styles.slamFastlio2,
  localizer: styles.slamLocalizer,
  stop:      styles.slamStop,
}

const TABS: { key: Tab; label: string }[] = [
  { key: 'console', label: '控制台' },
  { key: 'scene',   label: '场景' },
  { key: 'map',     label: '地图' },
  { key: 'slam',    label: 'SLAM' },
]

interface TopbarProps {
  sseState:    SSEState
  activeTab:   Tab
  onTabChange: (tab: Tab) => void
}

const SESSION_MODE_ZH: Record<string, string> = {
  idle:       '空闲',
  mapping:    '建图',
  navigating: '巡航',
}

export function Topbar({ sseState, activeTab, onTabChange }: TopbarProps) {
  const [settingsOpen, setSettingsOpen] = useState(false)
  const estop = sseState.safetyState?.estop ?? false
  const odom = sseState.odometry
  const posLabel =
    typeof odom?.x === 'number' && typeof odom?.y === 'number'
      ? `(${odom.x.toFixed(1)}, ${odom.y.toFixed(1)})`
      : '--'
  const navState = sseState.missionStatus?.state ?? 'IDLE'
  const navStateZh = NAV_STATE_ZH[navState] ?? navState
  const slamMode = sseState.slamStatus?.mode ?? null
  const slamModeZh = slamMode ? (SLAM_MODE_ZH[slamMode] ?? slamMode) : null

  // Health 四件套来自 session state
  const session = sseState.session
  const sessMode = session?.mode ?? 'idle'
  const sessModeZh = SESSION_MODE_ZH[sessMode] ?? sessMode
  const activeMap = session?.active_map ?? '--'
  const icpQ = session?.icp_quality ?? 0
  const icpLabel = icpQ > 0 ? icpQ.toFixed(3) : '--'
  // ICP health thresholds:
  //   <0.15 healthy (green)
  //   0.15-0.3 warning (yellow)
  //   >=0.3 critical (red + pulse) — drift watchdog will auto-recover soon
  const icpClass =
    sessMode !== 'navigating' ? styles.navPlanning
    : icpQ <= 0 ? styles.navPlanning
    : icpQ < 0.15 ? '' // healthy default
    : icpQ < 0.3 ? styles.navPlanning
    : `${styles.navFailed} ${styles.icpPulse}`
  const locReady = !!session?.localizer_ready
  const locLabel = sessMode !== 'navigating' ? '--' : locReady ? '就绪' : '对齐中'
  const sessionModeClass =
    sessMode === 'navigating' ? styles.slamLocalizer
    : sessMode === 'mapping' ? styles.slamFastlio2
    : styles.slamStop
  const slamHz = sseState.slamStatus?.slam_hz ?? 0
  const hasCloud = !!sseState.mapCloud
  const slamPipClass = slamHz > 3 ? styles.pipGood : slamHz > 0 ? styles.pipWarn : styles.pipBad
  const cloudPipClass = hasCloud ? styles.pipGood : styles.pipBad
  const traffic = sseState.traffic
  const trafficWarn = traffic?.status === 'degraded' || (traffic?.warnings?.length ?? 0) > 0
  const trafficPipClass = !traffic ? styles.pipWarn : trafficWarn ? styles.pipWarn : styles.pipGood
  const trafficTitle = traffic
    ? [
        `Traffic ${traffic.status}`,
        `SSE clients ${traffic.sse.clients}, drops ${traffic.sse.dropped_events}, depth ${traffic.sse.max_depth_seen}`,
        `Cloud clients ${traffic.cloud.clients}, drops ${traffic.cloud.dropped_frames}, depth ${traffic.cloud.max_depth_seen}`,
      ].join(' | ')
    : 'Traffic stats pending'

  const handleKeyDown = (e: React.KeyboardEvent, idx: number) => {
    if (e.key !== 'ArrowLeft' && e.key !== 'ArrowRight') return
    e.preventDefault()
    const delta = e.key === 'ArrowRight' ? 1 : -1
    const next = (idx + delta + TABS.length) % TABS.length
    onTabChange(TABS[next].key)
  }

  return (
    <header className={styles.topbar}>
      <div className={styles.left}>
        {/* Logo */}
        <span className={styles.logo}>
          <span className={styles.logoIcon} aria-hidden="true">
            <svg width="15" height="15" viewBox="0 0 32 32" fill="none" xmlns="http://www.w3.org/2000/svg">
              <circle cx="16" cy="16" r="11.5" stroke="url(#tbRing)" strokeWidth="1.5" fill="none" strokeDasharray="7 3.5" strokeLinecap="round"/>
              <path d="M16 5.5 L23.5 21 L16 17.8 L8.5 21 Z" fill="url(#tbArrow)"/>
              <path d="M16 10.5 L20.5 19.5 L16 17.8 L11.5 19.5 Z" fill="rgba(14,16,32,0.85)"/>
              <circle cx="16" cy="25.5" r="1.8" fill="#22d3ee" opacity="0.9"/>
              <circle cx="16" cy="25.5" r="1" fill="rgba(14,16,32,0.85)"/>
              <defs>
                <linearGradient id="tbArrow" x1="8" y1="5" x2="24" y2="24" gradientUnits="userSpaceOnUse">
                  <stop offset="0" stopColor="#818cf8"/>
                  <stop offset="1" stopColor="#22d3ee"/>
                </linearGradient>
                <linearGradient id="tbRing" x1="5" y1="5" x2="27" y2="27" gradientUnits="userSpaceOnUse">
                  <stop offset="0" stopColor="#6366f1" stopOpacity="0.8"/>
                  <stop offset="1" stopColor="#06b6d4" stopOpacity="0.8"/>
                </linearGradient>
              </defs>
            </svg>
          </span>
          LingTu
        </span>
        <span
          className={sseState.connected ? styles.badgeOnline : styles.badgeOffline}
          title={`Gateway SSE · ${sseState.connected ? '已连' : '断开'}`}
        >
          {sseState.connected ? '在线' : '离线'}
        </span>
        <span
          className={`${styles.miniPip} ${slamPipClass}`}
          title={`SLAM ${slamHz.toFixed(1)} Hz · ${slamMode ?? '—'}`}
        >
          SLAM
        </span>
        <span
          className={`${styles.miniPip} ${cloudPipClass}`}
          title={`点云 ${hasCloud ? '活跃' : '无数据'}`}
        >
          点云
        </span>
        <span
          className={`${styles.miniPip} ${trafficPipClass}`}
          title={trafficTitle}
        >
          Traffic
        </span>

        <span className={styles.sep} aria-hidden="true" />

        {/* Tabs inline */}
        <nav className={styles.tabs} role="tablist" aria-label="主视图切换">
          {TABS.map((tab, i) => (
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
              {tab.label}
            </button>
          ))}
        </nav>
      </div>

      <div className={styles.center}>
        {/* Health 四件套：session 模式 / active map / ICP 质量 / Localizer */}
        <span className={styles.stat} title="当前会话模式（来自 /api/v1/session）">
          <span className={styles.statLabel}>模式</span>
          <span className={`${styles.statValue} ${sessionModeClass}`}>{sessModeZh}</span>
        </span>
        <span className={styles.divider} />
        <span className={styles.stat} title={`激活地图: ${activeMap}`}>
          <span className={styles.statLabel}>地图</span>
          <span className={styles.statValue} style={{ maxWidth: 140, overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap' }}>{activeMap}</span>
        </span>
        <span className={styles.divider} />
        <span className={styles.stat} title="ICP 对齐分数（低=好；>0.3 告警）">
          <span className={styles.statLabel}>ICP</span>
          <span className={`${styles.statValue} ${icpClass}`}>{icpLabel}</span>
        </span>
        <span className={styles.divider} />
        <span className={styles.stat} title="Localizer 运行状态">
          <span className={styles.statLabel}>定位</span>
          <span className={`${styles.statValue} ${sessMode === 'navigating' && locReady ? '' : styles.navPlanning}`}>{locLabel}</span>
        </span>
        <span className={styles.divider} />
        <span className={styles.stat} title="里程计位置 (x, y)">
          <span className={styles.statLabel}>位置</span>
          <span className={styles.statValue}>{posLabel}</span>
        </span>
        <span className={styles.divider} />
        <span className={styles.stat}>
          <span className={styles.statLabel}>导航</span>
          <span className={`${styles.statValue} ${NAV_STATE_CLASS[navState.toLowerCase()] ?? ''}`}>{navStateZh}</span>
        </span>
        {slamModeZh && sessMode === 'idle' && (
          <>
            <span className={styles.divider} />
            <span className={styles.stat} title="底层 SLAM service 状态">
              <span className={styles.statLabel}><Radio size={10} style={{ display: 'inline', verticalAlign: 'middle', marginRight: 2 }} />SLAM</span>
              <span className={`${styles.statValue} ${SLAM_MODE_CLASS[slamMode!] ?? ''}`}>{slamModeZh}</span>
            </span>
          </>
        )}
        {estop && (
          <>
            <span className={styles.divider} />
            <span className={styles.estop}>急停</span>
          </>
        )}
      </div>

      <div className={styles.right}>
        <BagRecorder />
        {activeTab === 'console' && (
          <button
            className={styles.btnIcon}
            aria-label="重置控制台布局"
            title="重置控制台布局（清除拖拽保存的位置/尺寸）"
            onClick={() => {
              if (confirm('重置控制台的 FloatingWidget 位置和尺寸？页面将刷新。')) {
                resetAllLayouts()
              }
            }}
          >
            <RotateCcw size={16} />
          </button>
        )}
        <button
          className={styles.btnIcon}
          aria-label="设置"
          aria-expanded={settingsOpen}
          onClick={() => setSettingsOpen(v => !v)}
        >
          <Settings size={16} />
        </button>
      </div>

      <SettingsMenu open={settingsOpen} onClose={() => setSettingsOpen(false)} />
    </header>
  )
}
