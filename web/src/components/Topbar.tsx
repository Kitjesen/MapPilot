import { createPortal } from 'react-dom'
import { useCallback, useEffect, useRef, useState, type PointerEvent as ReactPointerEvent } from 'react'
import { Check, ChevronDown, ChevronUp, GripVertical, Map, MapPinned, Navigation, Settings, Square, X } from 'lucide-react'
import type { SSEState, Tab, ToastKind } from '../types'
import { isObservationMode } from '../services/observationMode.ts'
import { mapLocalizationLabel, mapLocalizationState, shortMapLabel } from '../services/mapLabel.ts'
import { SettingsMenu } from './SettingsMenu'
import { GeographicPlannerButton } from './GeographicPlanner'
import { useMapSettings } from '../hooks/useMapSettings.ts'
import { loadLayouts, saveLayouts } from './floatingWidgetLayout'
import type { ResolvedTheme, Theme } from './useTheme'
import { text, type Locale } from '../i18n'
import styles from './Topbar.module.css'

const WORKSPACE_PAGES = [
  { key: 'mapping', en: 'Mapping', zh: '建图', icon: MapPinned },
  { key: 'scene', en: 'Navigation', zh: '导航', icon: Navigation },
] as const
const INITIAL_DOCK_WIDTH = 380

interface TopbarProps {
  sseState: SSEState
  standby?: boolean
  localNowS: number
  activeTab: Tab
  onTabChange: (tab: Tab) => void
  theme: Theme
  resolvedTheme: ResolvedTheme
  onThemeChange: (theme: Theme) => void
  locale: Locale
  onLocaleChange: (locale: Locale) => void
  onStop: () => void
  showToast: (message: string, kind?: ToastKind) => void
}
export function Topbar(props: TopbarProps) {
  const { sseState, activeTab, onTabChange, locale, standby = false } = props
  const [localSettingsOpen, setSettingsOpen] = useState(false)
  const [closedSettingsRequest, setClosedSettingsRequest] = useState(0)
  const mapSettings = useMapSettings()
  const settingsOpen = localSettingsOpen || (mapSettings?.request ?? 0) > closedSettingsRequest
  // Mouse hover reveals the dock; touch and keyboard retain the explicit toggle.
  const [collapsed, setCollapsed] = useState(true)
  const [dragging, setDragging] = useState(false)
  const [dockWidth, setDockWidth] = useState(INITIAL_DOCK_WIDTH)
  const dockRef = useRef<HTMLElement>(null)
  const [position, setPosition] = useState(() => {
    const layout = loadLayouts().operatorDock
    return { x: layout?.x ?? 16, y: layout?.y ?? 14 }
  })
  const dragRef = useRef<{ startX: number; startY: number; originX: number; originY: number; moved: boolean } | null>(null)
  const suppressClickRef = useRef(false)
  const workspaceMenu = useRef<HTMLDetailsElement>(null)
  const mapInfoMenu = useRef<HTMLDetailsElement>(null)
  const closeSettings = useCallback(() => {
    setSettingsOpen(false); setClosedSettingsRequest(mapSettings?.request ?? 0)
  }, [mapSettings?.request])
  const observe = isObservationMode()
  const currentPage = WORKSPACE_PAGES.find(page => page.key === activeTab) ?? WORKSPACE_PAGES[1]
  const activeMap = sseState.session?.active_map?.trim() || null
  const mapLabel = shortMapLabel(activeMap) ?? text(locale, 'No active map', '未激活地图')
  const localizationState = mapLocalizationState(sseState, props.localNowS)
  const localizationLabel = mapLocalizationLabel(localizationState, locale)
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
  const compactContext = [text(locale, currentPage.en, currentPage.zh), activeMap ?? mapLabel, sessionMode].filter(Boolean).join(', ')
  const connectionLabel = standby ? text(locale, 'Standby', '待机')
    : sseState.connected ? text(locale, 'Realtime data connected', '实时数据已连接')
      : text(locale, 'Realtime data disconnected', '实时数据未连接')
  const connectionNotice = standby || sseState.connected ? '' : sseState.lastTruthAt
    ? text(locale, 'Realtime data connection lost · reconnecting', '实时数据连接中断，正在重连')
    : text(locale, 'Waiting for runtime data', '等待运行服务数据')

  useEffect(() => {
    const element = dockRef.current
    if (!element) return
    if (collapsed) for (const menu of element.querySelectorAll<HTMLDetailsElement>('details[data-hover-menu]')) menu.open = false
    const observer = new ResizeObserver(() => setDockWidth(element.getBoundingClientRect().width))
    observer.observe(element)
    return () => observer.disconnect()
  }, [collapsed])

  const dockHeight = useCallback((isCollapsed: boolean) => {
    if (isCollapsed) return 56
    return window.innerWidth <= 700 ? 370 : 320
  }, [])

  const clampPosition = useCallback((x: number, y: number) => {
    const width = Math.min(dockWidth, Math.max(0, window.innerWidth - 16))
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
        w: dockWidth,
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

  const dockPosition = clampPosition(position.x, position.y)
  const dockStyle = { left: dockPosition.x, top: dockPosition.y }

  useEffect(() => {
    const dock = dockRef.current
    if (!dock) return
    let opening: ReturnType<typeof setTimeout>
    let closing: ReturnType<typeof setTimeout>
    const clear = () => { clearTimeout(opening); clearTimeout(closing) }
    const enter = (event: PointerEvent) => {
      if (event.pointerType !== 'mouse') return
      clear()
      opening = setTimeout(() => { if (!dragRef.current) setCollapsed(false) }, 220)
    }
    const leave = (event: PointerEvent) => {
      if (event.pointerType !== 'mouse') return
      clear()
      closing = setTimeout(() => {
        if (dragRef.current || (dock.contains(document.activeElement) && document.activeElement?.matches(':focus-visible'))) return
        setCollapsed(true)
      }, 180)
    }
    const key = (event: KeyboardEvent) => {
      if (event.key !== 'Escape' || event.defaultPrevented || dock.querySelector('details[open]')) return
      clear()
      setCollapsed(true)
      requestAnimationFrame(() => dock.querySelector<HTMLButtonElement>('button[data-dock-drag]')?.focus())
    }
    dock.addEventListener('pointerenter', enter)
    dock.addEventListener('pointerleave', leave)
    dock.addEventListener('pointerdown', clear)
    dock.addEventListener('keydown', key)
    return () => {
      clear()
      dock.removeEventListener('pointerenter', enter)
      dock.removeEventListener('pointerleave', leave)
      dock.removeEventListener('pointerdown', clear)
      dock.removeEventListener('keydown', key)
    }
  }, [])

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
          <span className={styles.compactMapName} aria-hidden="true">{text(locale, currentPage.en, currentPage.zh)}{activeMap ? ` · ${mapLabel}` : ''}</span>
          {connectionNotice && <span id="operator-connection-notice" className={styles.compactConnection} role="status">{connectionNotice}</span>}
        </span>
        <ChevronDown className={styles.compactChevron} size={18} aria-hidden="true" />
      </button>
      <div className={styles.dockContent}>
        <span className={styles.dragHandle} data-dock-drag title={text(locale, 'Drag controls', '拖动操作坞')} aria-hidden="true">
          <GripVertical size={13} />
        </span>
        <button className={styles.logo} onClick={() => setCollapsed(value => !value)}
          aria-label={collapsed ? text(locale, 'Expand controls', '展开操作坞') : text(locale, 'Collapse controls', '收起操作坞')}
          title={collapsed ? text(locale, 'Expand controls', '展开操作坞') : text(locale, 'Collapse controls', '收起操作坞')}>
          <Navigation size={24} strokeWidth={1.8} aria-hidden="true" />
        </button>
        <nav className={styles.workspace} aria-label={text(locale, 'Workspace', '工作区')}>
        {observe ? <span className={styles.viewLabel}>{text(locale, 'Observation', '只读监控')}</span> : (<>
          <details ref={workspaceMenu} className={styles.workspaceMenu} name="lingtu-menu"
            data-hover-menu
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
              {WORKSPACE_PAGES.map(page => <div key={page.key}>
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
        {!observe && <GeographicPlannerButton locale={locale} className={styles.pageButton} />}
        </nav>
        <div className={styles.right}>
        <details ref={mapInfoMenu} className={styles.mapInfoMenu} name="lingtu-menu"
          data-hover-menu
          onBlur={event => {
            if (event.relatedTarget && !event.currentTarget.contains(event.relatedTarget)) event.currentTarget.open = false
          }}>
          <summary className={styles.connectionTrigger} aria-label={text(locale, 'Map and connection info', '地图与连接信息')}
            title={text(locale, 'Map and connection info', '地图与连接信息')}>
            <span className={standby || sseState.connected ? styles.online : styles.offline} role="status">
              {connectionLabel}
            </span>
            <ChevronDown size={12} aria-hidden="true" />
          </summary>
          <section className={styles.mapContext} aria-label={text(locale, 'Current map', '当前地图')}>
            <div className={styles.contextHeading}>
              <span className={styles.contextBrand}>{'LingTu'}</span>
              <span className={styles.contextLabel}>{text(locale, 'Current map', '当前地图')}</span>
              <button className={styles.iconButton} aria-label={text(locale, 'Close map info', '关闭地图信息')}
                onClick={() => {
                  if (mapInfoMenu.current) {
                    mapInfoMenu.current.open = false
                    mapInfoMenu.current.querySelector('summary')?.focus()
                  }
                }}><X size={17} aria-hidden="true" /></button>
            </div>
            <strong className={styles.mapName} title={activeMap ?? undefined}>{mapLabel}</strong>
            <dl className={styles.mapFacts}>
              {sessionMode && <div><dt>{text(locale, 'Mode', '模式')}</dt><dd>{sessionMode}</dd></div>}
              <div><dt>{text(locale, 'Realtime data', '实时数据')}</dt><dd>{connectionLabel}</dd></div>
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
          </section>
        </details>
        {observe ? <a className={styles.pageButton} href="/" title={text(locale, 'Open goal selection; no motion until you confirm a goal', '打开选点工作区，确认目标后才会运动')}>
          <Navigation size={16} />{text(locale, 'Navigate', '进入导航')}
        </a> : null}
        {sseState.safetyState?.estop && <span className={styles.estop}>{text(locale, 'E-stop active', '急停中')}</span>}
        <button className={styles.iconButton} aria-label={text(locale, 'Settings', '设置')}
          aria-expanded={settingsOpen} onClick={() => settingsOpen ? closeSettings() : setSettingsOpen(true)}><Settings size={18} strokeWidth={1.7} /></button>
        <button className={styles.iconButton} aria-label={text(locale, 'Collapse controls', '收起操作坞')}
          title={text(locale, 'Collapse controls', '收起操作坞')}
          onClick={() => setCollapsed(true)}><ChevronUp size={18} strokeWidth={1.7} /></button>
        </div>
        <SettingsMenu open={settingsOpen} onClose={closeSettings} sseState={sseState} localNowS={props.localNowS} showToast={props.showToast}
          theme={props.theme} resolvedTheme={props.resolvedTheme} onThemeChange={props.onThemeChange}
          locale={locale} onLocaleChange={props.onLocaleChange} />
      </div>
      <div id="scene-actions-slot" className={styles.sceneActionsSlot}
        aria-label={text(locale, 'Workspace actions', '当前业务操作')} />
    </header>
      {!observe && createPortal(<button className={styles.emergencyStop} onClick={props.onStop}
        aria-label={text(locale, 'Emergency stop', '紧急停止机器人')}
        title={text(locale, 'Emergency stop', '紧急停止机器人')}>
        <Square size={16} fill="currentColor" aria-hidden="true" />{text(locale, 'Stop', '停止')}
      </button>, document.body)}
    </>
  )
}
