import { useCallback, useEffect, useRef, useState } from 'react'
import { Activity, Box, Check, ChevronDown, Gamepad2, ListChecks, Map, Navigation, Settings, SlidersHorizontal, Square, Workflow } from 'lucide-react'
import type { SSEState, Tab } from '../types'
import { isObservationMode } from '../services/observationMode.ts'
import { SettingsMenu } from './SettingsMenu'
import type { ResolvedTheme, Theme } from './useTheme'
import { text, type Locale } from '../i18n'
import styles from './Topbar.module.css'

const WORKSPACE_PAGES = [
  { key: 'scene', en: 'Scene', zh: '现场', icon: Box },
  { key: 'console', en: 'Control', zh: '控制台', icon: Gamepad2 },
  { key: 'map', en: 'Maps', zh: '地图', icon: Map },
  { key: 'inspection', en: 'Tasks', zh: '巡检任务', icon: ListChecks },
  { key: 'slam', en: 'Localization', zh: '定位诊断', icon: Activity },
  { key: 'dataflow', en: 'Data', zh: '数据诊断', icon: Workflow },
  { key: 'planner', en: 'Planning', zh: '规划参数', icon: SlidersHorizontal },
] as const
const PRIMARY_PAGES = ['scene', 'map', 'inspection']

interface TopbarProps {
  sseState: SSEState
  activeTab: Tab
  onTabChange: (tab: Tab) => void
  theme: Theme
  resolvedTheme: ResolvedTheme
  onThemeChange: (theme: Theme) => void
  locale: Locale
  onLocaleChange: (locale: Locale) => void
  onStop: () => void
}
export function Topbar(props: TopbarProps) {
  const { sseState, activeTab, onTabChange, locale } = props
  const [settingsOpen, setSettingsOpen] = useState(false)
  const workspaceMenu = useRef<HTMLDetailsElement>(null)
  const closeSettings = useCallback(() => setSettingsOpen(false), [])
  const observe = isObservationMode()
  const currentPage = WORKSPACE_PAGES.find(page => page.key === activeTab) ?? WORKSPACE_PAGES[0]

  useEffect(() => {
    const dismissOutside = (event: PointerEvent) => {
      const menu = workspaceMenu.current
      if (menu?.open && !menu.contains(event.target as Node)) menu.open = false
    }
    const dismissEscape = (event: KeyboardEvent) => {
      const menu = workspaceMenu.current
      if (event.key === 'Escape' && menu?.open) {
        menu.open = false
        menu.querySelector('summary')?.focus()
      }
    }
    document.addEventListener('pointerdown', dismissOutside)
    document.addEventListener('keydown', dismissEscape)
    return () => {
      document.removeEventListener('pointerdown', dismissOutside)
      document.removeEventListener('keydown', dismissEscape)
    }
  }, [])

  return (
    <header className={styles.topbar}>
      <button className={styles.logo} onClick={() => onTabChange('scene')}
        aria-label={text(locale, 'LingTu — return to scene', 'LingTu · 返回现场')}
        title={text(locale, 'Return to scene', '返回现场')}>
        <Navigation size={24} strokeWidth={1.8} aria-hidden="true" />
      </button>
      <nav className={styles.workspace} aria-label={text(locale, 'Workspace', '工作区')}>
        {observe ? <span className={styles.viewLabel}>{text(locale, 'Scene', '现场')}</span> : (<>
          <div className={styles.primaryPages}>
            {WORKSPACE_PAGES.filter(page => PRIMARY_PAGES.includes(page.key)).map(page => (
              <button key={page.key} className={styles.pageButton}
                aria-current={activeTab === page.key ? 'page' : undefined}
                onClick={() => onTabChange(page.key)}>
                <page.icon size={16} strokeWidth={1.7} />
                {text(locale, page.en, page.zh)}
              </button>
            ))}
          </div>
          <details ref={workspaceMenu} className={styles.workspaceMenu} name="lingtu-menu"
            onBlur={event => {
              if (event.relatedTarget && !event.currentTarget.contains(event.relatedTarget)) event.currentTarget.open = false
            }}
            onKeyDown={event => {
              const menu = event.currentTarget
              const buttons = [...menu.querySelectorAll<HTMLButtonElement>('[role="menuitem"]')]
              const index = buttons.indexOf(document.activeElement as HTMLButtonElement)
              if (['ArrowDown', 'ArrowUp', 'Home', 'End'].includes(event.key)) {
                event.preventDefault()
                menu.open = true
                const next = event.key === 'Home' ? 0 : event.key === 'End' ? buttons.length - 1
                  : index < 0 ? event.key === 'ArrowDown' ? 0 : buttons.length - 1
                    : (index + (event.key === 'ArrowDown' ? 1 : -1) + buttons.length) % buttons.length
                buttons[next]?.focus()
              }
            }}>
            <summary className={styles.workspaceTrigger} aria-haspopup="menu"
              aria-label={text(locale, `Switch workspace: ${currentPage.en}`, `切换工作区：${currentPage.zh}`)}>
              {PRIMARY_PAGES.includes(activeTab) ? text(locale, 'More', '更多') : text(locale, currentPage.en, currentPage.zh)}<ChevronDown size={13} />
            </summary>
            <div className={styles.workspacePanel} role="menu" aria-label={text(locale, 'Choose workspace', '选择工作区')}>
              {WORKSPACE_PAGES.filter(page => !PRIMARY_PAGES.includes(page.key)).map((page, index) => <div key={page.key}>
                {index === 1 && <div className={styles.groupLabel}>{text(locale, 'Diagnostics', '诊断与调试')}</div>}
                <button role="menuitem" aria-current={activeTab === page.key ? 'page' : undefined}
                  onClick={() => {
                    onTabChange(page.key)
                    if (workspaceMenu.current) {
                      workspaceMenu.current.open = false
                      workspaceMenu.current.querySelector('summary')?.focus()
                    }
                  }}>
                  <page.icon size={16} strokeWidth={1.6} />
                  <span>{text(locale, page.en, page.zh)}</span>
                  {activeTab === page.key && <Check size={14} />}
                </button>
              </div>)}
            </div>
          </details>
        </>)}
      </nav>
      <div className={styles.right}>
        <span className={sseState.connected ? styles.online : styles.offline} role="status"
          title={text(locale, 'Realtime connection', '实时连接状态')}>
          {sseState.connected ? text(locale, 'Connected', '已连接') : text(locale, 'Disconnected', '未连接')}
        </span>
        {observe ? <a className={styles.pageButton} href="/" title={text(locale, 'Open goal selection; no motion until you confirm a goal', '打开选点工作区，确认目标后才会运动')}>
          <Navigation size={16} />{text(locale, 'Navigate', '进入导航')}
        </a> : (
          <button className={styles.stop} onClick={props.onStop} title={text(locale, 'Emergency stop', '紧急停止机器人')}>
            <Square size={11} fill="currentColor" />{text(locale, 'Stop', '停止')}
          </button>
        )}
        {sseState.safetyState?.estop && <span className={styles.estop}>{text(locale, 'E-stop active', '急停中')}</span>}
        <button className={styles.iconButton} aria-label={text(locale, 'Settings', '设置')}
          aria-expanded={settingsOpen} onClick={() => setSettingsOpen(value => !value)}><Settings size={18} strokeWidth={1.7} /></button>
      </div>
      <SettingsMenu open={settingsOpen} onClose={closeSettings}
        theme={props.theme} resolvedTheme={props.resolvedTheme} onThemeChange={props.onThemeChange}
        locale={locale} onLocaleChange={props.onLocaleChange} />
    </header>
  )
}
