import { createPortal } from 'react-dom'
import { useCallback, useEffect, useRef, useState, type PointerEvent as ReactPointerEvent } from 'react'
import { Activity, Box, Check, ChevronDown, ChevronUp, Gamepad2, GripVertical, ListChecks, Map, Navigation, Settings, SlidersHorizontal, Square, Workflow } from 'lucide-react'
import type { SSEState, Tab } from '../types'
import { isObservationMode } from '../services/observationMode.ts'
import { mapLocalizationState, shortMapLabel } from '../services/mapLabel.ts'
import { SettingsMenu } from './SettingsMenu'
import { loadLayouts, saveLayouts } from './floatingWidgetLayout'
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
const OPEN_DOCK_WIDTH = 920
const COLLAPSED_DOCK_WIDTH = 380

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
  // Keep the scene primary. The compact identity pill expands only on an explicit click.
  const [collapsed, setCollapsed] = useState(true)
  const [dragging, setDragging] = useState(false)
  const [collapsedWidth, setCollapsedWidth] = useState(COLLAPSED_DOCK_WIDTH)
  const dockRef = useRef<HTMLElement>(null)
  const [now, setNow] = useState(() => Date.now())
  const [position, setPosition] = useState(() => {
    const layout = loadLayouts().operatorDock
    return { x: layout?.x ?? 16, y: layout?.y ?? 14 }
  })
  const dragRef = useRef<{ startX: number; startY: number; originX: number; originY: number; moved: boolean } | null>(null)
  const suppressClickRef = useRef(false)
  const workspaceMenu = useRef<HTMLDetailsElement>(null)
  const closeSettings = useCallback(() => setSettingsOpen(false), [])
  const observe = isObservationMode()
  const currentPage = WORKSPACE_PAGES.find(page => page.key === activeTab) ?? WORKSPACE_PAGES[0]
  const activeMap = sseState.session?.active_map?.trim() || null
  const mapLabel = shortMapLabel(activeMap) ?? text(locale, 'No active map', '未激活地图')
  const localizationState = mapLocalizationState(sseState, now / 1000)
  const localizationLabel = {
    disconnected: text(locale, 'Disconnected', '连接断开'),
    waiting: text(locale, 'Waiting for data', '等待数据'),
    stale: text(locale, 'Data stale', '数据已过期'),
    tracking: text(locale, 'Tracking', '定位跟踪中'),
    initializing: text(locale, 'Initializing', '初始化中'),
    relocalizing: text(locale, 'Relocalizing', '重定位中'),
    degraded: text(locale, 'Degraded', '定位退化'),
    lost: text(locale, 'Lost', '定位丢失'),
    no_odometry: text(locale, 'Waiting for odometry', '等待里程计'),
    unknown: text(locale, 'Unknown', '未知'),
  }[localizationState]
  const sessionMode = sseState.session?.mode
    ? text(locale, {
      idle: 'Idle',
      mapping: 'Mapping',
      navigating: 'Navigation',
      exploring: 'Exploration',
    }[sseState.session.mode], {
      idle: '待机',
      mapping: '建图',
      navigating: '导航',
      exploring: '探索',
    }[sseState.session.mode])
    : null
  const compactContext = [activeMap ?? mapLabel, sessionMode].filter(Boolean).join(', ')
  const connectionNotice = sseState.connected ? '' : sseState.lastTruthAt
    ? text(locale, 'Connection lost · reconnecting', '连接中断，正在重连')
    : text(locale, 'Waiting for robot connection', '等待机器人连接')

  useEffect(() => {
    const timer = window.setInterval(() => setNow(Date.now()), 1000)
    return () => window.clearInterval(timer)
  }, [])

  useEffect(() => {
    const element = dockRef.current
    if (!collapsed || !element) return
    const observer = new ResizeObserver(() => setCollapsedWidth(element.getBoundingClientRect().width))
    observer.observe(element)
    return () => observer.disconnect()
  }, [collapsed])

  const dockWidth = useCallback((isCollapsed: boolean) => {
    const available = Math.max(0, window.innerWidth - 16)
    return Math.min(isCollapsed ? collapsedWidth : OPEN_DOCK_WIDTH, available)
  }, [collapsedWidth])
  const dockHeight = useCallback((isCollapsed: boolean) => {
    if (isCollapsed) return 56
    return window.innerWidth <= 700 ? 370 : 320
  }, [])

  const clampPosition = useCallback((x: number, y: number) => {
    const width = dockWidth(collapsed)
    const height = dockHeight(collapsed)
    return {
      x: Math.max(8, Math.min(Math.max(8, window.innerWidth - width - 8), x)),
      y: Math.max(8, Math.min(Math.max(8, window.innerHeight - height - 8), y)),
    }
  }, [collapsed, dockHeight, dockWidth])

  const beginDrag = useCallback((event: ReactPointerEvent<HTMLElement>) => {
    const target = event.target as HTMLElement
    const handle = target.closest<HTMLElement>('[data-dock-drag]')
    if (event.button !== 0 || !handle) return
    dragRef.current = {
      startX: event.clientX,
      startY: event.clientY,
      originX: position.x,
      originY: position.y,
      moved: false,
    }
    handle.setPointerCapture?.(event.pointerId)
    setDragging(true)
  }, [position.x, position.y])

  const moveDrag = useCallback((event: ReactPointerEvent<HTMLElement>) => {
    const drag = dragRef.current
    if (!drag) return
    const dx = event.clientX - drag.startX
    const dy = event.clientY - drag.startY
    if (Math.hypot(dx, dy) > 3) drag.moved = true
    if (drag.moved) setPosition(clampPosition(drag.originX + dx, drag.originY + dy))
  }, [clampPosition])

  const endDrag = useCallback(() => {
    const drag = dragRef.current
    if (!drag) return
    if (drag?.moved) suppressClickRef.current = true
    dragRef.current = null
    setDragging(false)
  }, [])

  useEffect(() => {
    const layouts = loadLayouts()
    const savedPosition = clampPosition(position.x, position.y)
    saveLayouts({
      ...layouts,
      operatorDock: {
        x: savedPosition.x,
        y: savedPosition.y,
        w: dockWidth(collapsed),
        h: dockHeight(collapsed),
        z: 20,
      },
    })
  }, [clampPosition, collapsed, dockHeight, dockWidth, position])

  useEffect(() => {
    const onResize = () => setPosition(value => clampPosition(value.x, value.y))
    window.addEventListener('resize', onResize)
    return () => window.removeEventListener('resize', onResize)
  }, [clampPosition])

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

  const dockPosition = clampPosition(position.x, position.y)
  const dockStyle = { left: dockPosition.x, top: dockPosition.y }

  return (
    <>
    <header ref={dockRef} className={`${styles.topbar} ${collapsed ? styles.collapsed : ''} ${dragging ? styles.dragging : ''}`} style={dockStyle}
      onPointerDown={beginDrag} onPointerMove={moveDrag} onPointerUp={endDrag} onPointerCancel={endDrag}>
      <button className={styles.collapseButton} data-dock-drag
        aria-label={`${text(locale, 'Expand controls', '展开操作坞')}${compactContext ? ` · ${compactContext}` : ''}`}
        aria-describedby={connectionNotice ? 'operator-connection-notice' : undefined}
        title={`${text(locale, 'Expand controls', '展开操作坞')}${compactContext ? ` · ${compactContext}` : ''}`}
        onClick={() => {
          if (suppressClickRef.current) {
            suppressClickRef.current = false
            return
          }
          setCollapsed(false)
        }}>
        <Navigation className={styles.compactMark} size={22} strokeWidth={1.9} aria-hidden="true" />
        <span className={styles.compactIdentity}>
          <span className={styles.compactMapName} aria-hidden="true">{mapLabel}</span>
          {connectionNotice && <span id="operator-connection-notice" className={styles.compactConnection} role="status">{connectionNotice}</span>}
        </span>
        <ChevronDown className={styles.compactChevron} size={18} aria-hidden="true" />
      </button>
      <div className={styles.dockContent}>
        <span className={styles.dragHandle} data-dock-drag title={text(locale, 'Drag controls', '拖动操作坞')} aria-hidden="true">
          <GripVertical size={13} />
        </span>
        <button className={styles.logo} onClick={() => {
          if (collapsed) setCollapsed(false)
          else if (activeTab === 'scene') setCollapsed(true)
          else onTabChange('scene')
        }}
          aria-label={collapsed ? text(locale, 'Expand controls', '展开操作坞') : text(locale, activeTab === 'scene' ? 'LingTu — collapse controls' : 'LingTu — return to scene', activeTab === 'scene' ? 'LingTu · 收起操作坞' : 'LingTu · 返回现场')}
          title={collapsed ? text(locale, 'Expand controls', '展开操作坞') : text(locale, activeTab === 'scene' ? 'Collapse controls' : 'Return to scene', activeTab === 'scene' ? '收起操作坞' : '返回现场')}>
          <Navigation size={24} strokeWidth={1.8} aria-hidden="true" />
        </button>
        <nav className={styles.workspace} aria-label={text(locale, 'Workspace', '工作区')}>
        {observe ? <span className={styles.viewLabel}>{text(locale, 'Scene', '现场')}</span> : (<>
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
              <currentPage.icon size={15} strokeWidth={1.7} />
              <span>{text(locale, currentPage.en, currentPage.zh)}</span>
              <ChevronDown size={13} />
            </summary>
            <div className={styles.workspacePanel} role="menu" aria-label={text(locale, 'Choose workspace', '选择工作区')}>
              {WORKSPACE_PAGES.map((page, index) => <div key={page.key}>
                {index === 4 && <div className={styles.groupLabel}>{text(locale, 'Diagnostics', '诊断与调试')}</div>}
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
        </a> : null}
        {sseState.safetyState?.estop && <span className={styles.estop}>{text(locale, 'E-stop active', '急停中')}</span>}
        <button className={styles.iconButton} aria-label={text(locale, 'Settings', '设置')}
          aria-expanded={settingsOpen} onClick={() => setSettingsOpen(value => !value)}><Settings size={18} strokeWidth={1.7} /></button>
        <button className={styles.iconButton} aria-label={text(locale, 'Collapse controls', '收起操作坞')}
          title={text(locale, 'Collapse controls', '收起操作坞')}
          onClick={() => setCollapsed(true)}><ChevronUp size={18} strokeWidth={1.7} /></button>
        </div>
        <SettingsMenu open={settingsOpen} onClose={closeSettings}
          theme={props.theme} resolvedTheme={props.resolvedTheme} onThemeChange={props.onThemeChange}
          locale={locale} onLocaleChange={props.onLocaleChange} />
      </div>
      <div id="scene-actions-slot" className={styles.sceneActionsSlot}
        aria-label={text(locale, 'Scene actions', '现场操作')} />
      {!collapsed && <section className={styles.mapContext} aria-label={text(locale, 'Current map', '当前地图')}>
        <div className={styles.contextHeading}>
          <span className={styles.contextBrand}>{'LingTu'}</span>
          <span className={styles.contextLabel}>{text(locale, 'Current map', '当前地图')}</span>
        </div>
        <strong className={styles.mapName} title={activeMap ?? undefined}>{mapLabel}</strong>
        <dl className={styles.mapFacts}>
          {sessionMode && <div><dt>{text(locale, 'Mode', '模式')}</dt><dd>{sessionMode}</dd></div>}
          <div><dt>{text(locale, 'Connection', '连接')}</dt><dd>{sseState.connected ? text(locale, 'Connected', '已连接') : text(locale, 'Disconnected', '未连接')}</dd></div>
          <div><dt>{text(locale, 'Localization', '定位')}</dt><dd data-attention={['stale', 'degraded', 'lost'].includes(localizationState) || undefined}>{localizationLabel}</dd></div>
        </dl>
        {activeMap && <details className={styles.mapIdentity}>
          <summary>{text(locale, 'Map identifier', '地图标识')}</summary>
          <code>{activeMap}</code>
        </details>}
        {!observe && <button className={styles.mapAction} onClick={() => {
          onTabChange('map')
          setCollapsed(true)
        }}>
          <Map size={17} strokeWidth={1.7} aria-hidden="true" />
          {text(locale, 'Open maps', '打开地图库')}<ChevronDown size={14} aria-hidden="true" />
        </button>}
      </section>}
    </header>
      {!observe && createPortal(<button className={styles.emergencyStop} onClick={props.onStop}
        aria-label={text(locale, 'Emergency stop', '紧急停止机器人')}
        title={text(locale, 'Emergency stop', '紧急停止机器人')}>
        <Square size={16} fill="currentColor" aria-hidden="true" />{text(locale, 'Stop', '停止')}
      </button>, document.body)}
    </>
  )
}
