import { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import type { ReactNode } from 'react'
import { createPortal } from 'react-dom'
import {
  AlertTriangle,
  Check,
  CheckCircle2,
  ChevronDown,
  Compass,
  Eye,
  LocateFixed,
  Map as MapIcon,
  Play,
  Radio,
  RefreshCw,
  Route,
  ShieldCheck,
  Target,
} from 'lucide-react'
import * as api from '../services/api'
import { mapIsNavigationReady, waitForProductProfileReady } from '../services/mapReadiness'
import type {
  MapInfo,
  ProductModeProfile,
  RuntimeSwitchRequest,
  RuntimeSwitchResponse,
  SSEState,
  ToastKind,
} from '../types'
import { text, type Locale } from '../i18n'
import styles from './ProductModePanel.module.css'

interface ProductModePanelProps {
  sseState: SSEState
  showToast: (msg: string, kind?: ToastKind) => void
  locale?: Locale
}

interface ProductModeOption {
  profile: ProductModeProfile
  session: string
  labelEn: string
  labelZh: string
  summaryEn: string
  summaryZh: string
  policy: 'hot_candidate' | 'cold_restart'
  requiresSavedMap: boolean
  requiresLiveMap: boolean
  icon: ReactNode
}

const PRODUCT_MODES: ProductModeOption[] = [
  {
    profile: 'teleop',
    session: 'teleop',
    labelEn: 'Teleop',
    labelZh: '手动',
    summaryEn: 'Manual velocity control with safety retained.',
    summaryZh: '人工速度控制，保留安全链路。',
    policy: 'cold_restart',
    requiresSavedMap: false,
    requiresLiveMap: false,
    icon: <Radio size={16} />,
  },
  {
    profile: 'teleop_avoid',
    session: 'teleop_avoid',
    labelEn: 'Teleop Avoid',
    labelZh: '手动避障',
    summaryEn: 'Manual control with live localization guard.',
    summaryZh: '人工控制叠加实时定位保护。',
    policy: 'cold_restart',
    requiresSavedMap: false,
    requiresLiveMap: true,
    icon: <ShieldCheck size={16} />,
  },
  {
    profile: 'map',
    session: 'mapping',
    labelEn: 'Mapping',
    labelZh: '建图',
    summaryEn: 'Builds a live map from the current scan stream.',
    summaryZh: '使用当前扫描流生成实时地图。',
    policy: 'cold_restart',
    requiresSavedMap: false,
    requiresLiveMap: true,
    icon: <MapIcon size={16} />,
  },
  {
    profile: 'tracking',
    session: 'tracking',
    labelEn: 'Tracking',
    labelZh: '跟踪',
    summaryEn: 'Runs target tracking on top of localization.',
    summaryZh: '在定位链路上执行目标跟踪。',
    policy: 'hot_candidate',
    requiresSavedMap: true,
    requiresLiveMap: true,
    icon: <Target size={16} />,
  },
  {
    profile: 'nav',
    session: 'navigation',
    labelEn: 'Navigation',
    labelZh: '导航',
    summaryEn: 'Navigates against a saved map.',
    summaryZh: '基于保存地图执行导航。',
    policy: 'hot_candidate',
    requiresSavedMap: true,
    requiresLiveMap: true,
    icon: <Route size={16} />,
  },
  {
    profile: 'inspection',
    session: 'inspection',
    labelEn: 'Inspection',
    labelZh: '巡检',
    summaryEn: 'Runs scheduled semantic inspection.',
    summaryZh: '按计划执行语义巡检。',
    policy: 'hot_candidate',
    requiresSavedMap: true,
    requiresLiveMap: true,
    icon: <Eye size={16} />,
  },
  {
    profile: 'tare_explore',
    session: 'exploration',
    labelEn: 'Explore',
    labelZh: '探索',
    summaryEn: 'Generates exploration targets for navigation.',
    summaryZh: '生成探索目标并接入导航。',
    policy: 'cold_restart',
    requiresSavedMap: false,
    requiresLiveMap: true,
    icon: <Compass size={16} />,
  },
]

const STRATEGIES: Array<NonNullable<RuntimeSwitchRequest['strategy']>> = ['auto', 'hot', 'cold']

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === 'object' && value !== null
}

function sessionFallbackProfile(mode?: string | null): ProductModeProfile {
  if (mode === 'mapping') return 'map'
  if (mode === 'exploring') return 'tare_explore'
  if (mode === 'navigating') return 'nav'
  return 'teleop'
}

function sessionProductProfile(session?: SSEState['session']): ProductModeProfile {
  const profile = session?.product_profile
  if (PRODUCT_MODES.some(item => item.profile === profile)) {
    return profile as ProductModeProfile
  }
  return sessionFallbackProfile(session?.mode)
}

function productSessionForProfile(profile: string, fallbackMode?: string | null): string {
  const option = PRODUCT_MODES.find(item => item.profile === profile)
  if (option) return option.session
  if (fallbackMode === 'mapping') return 'mapping'
  if (fallbackMode === 'exploring') return 'exploration'
  if (fallbackMode === 'navigating') return 'navigation'
  return 'teleop'
}

function labelForOption(option: ProductModeOption, locale: Locale): string {
  return text(locale, option.labelEn, option.labelZh)
}

function summaryForOption(option: ProductModeOption, locale: Locale): string {
  return text(locale, option.summaryEn, option.summaryZh)
}

function productLabelFor(profileOrSession: string, locale: Locale): string {
  const option = PRODUCT_MODES.find(item => item.profile === profileOrSession || item.session === profileOrSession)
  return option ? labelForOption(option, locale) : profileOrSession
}

function policyLabel(policy: ProductModeOption['policy'], locale: Locale): string {
  return policy === 'hot_candidate'
    ? text(locale, 'Hot', '热切换')
    : text(locale, 'Restart', '重启')
}

function strategyLabel(strategy: RuntimeSwitchRequest['strategy'], locale: Locale): string {
  if (strategy === 'hot') return text(locale, 'Hot switch', '热切换')
  if (strategy === 'cold') return text(locale, 'Restart services', '重启服务')
  if (strategy === 'warm') return text(locale, 'Warm switch', '温切换')
  return text(locale, 'Auto', '自动')
}

function compactNumber(value: number | undefined | null): string {
  if (typeof value !== 'number' || !Number.isFinite(value)) return '--'
  if (value >= 1_000_000) return `${(value / 1_000_000).toFixed(1)}M`
  if (value >= 10_000) return `${Math.round(value / 1_000)}k`
  return Math.round(value).toLocaleString()
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

function shortBlockers(blockers: string[] | undefined, locale: Locale): string {
  if (!blockers || blockers.length === 0) return text(locale, 'No blockers', '无阻断项')
  const visible = blockers.slice(0, 2).join('; ')
  return blockers.length > 2 ? `${visible}; +${blockers.length - 2}` : visible
}

function responseTone(response: RuntimeSwitchResponse | null): 'ok' | 'warn' | 'dim' {
  if (!response) return 'dim'
  return response.ok ? 'ok' : 'warn'
}

function backendLabel(value: string, locale: Locale): string {
  if (!value || value === '--') return text(locale, 'Disconnected', '未连接')
  if (value === 'native_dds' || value === 'cpp_slam_status') return text(locale, 'Native localization', '原生定位')
  if (value.includes('ros2')) return text(locale, 'Compatibility bridge', '兼容桥')
  if (value === 'fastlio2') return text(locale, 'Mapping SLAM', '建图 SLAM')
  if (value === 'localizer') return text(locale, 'Map localization', '地图定位')
  return value
}

function diagText(data: Record<string, unknown>, key: string): string | undefined {
  const value = data[key]
  return typeof value === 'string' && value.length > 0 ? value : undefined
}

interface ModePickerProps {
  value: ProductModeProfile
  locale: Locale
  onChange: (profile: ProductModeProfile) => void
}

interface FloatingPickerConfig {
  itemCount: number
  selectedIndex: number
  minWidth: number
  rowHeight: number
}

function useFloatingPicker({ itemCount, selectedIndex, minWidth, rowHeight }: FloatingPickerConfig) {
  const [open, setOpen] = useState(false)
  const [highlightedIndex, setHighlightedIndex] = useState(0)
  const [menuPosition, setMenuPosition] = useState({
    left: 0,
    top: 0,
    width: minWidth,
    maxHeight: 360,
  })
  const triggerRef = useRef<HTMLButtonElement>(null)
  const menuRef = useRef<HTMLDivElement>(null)

  const updateMenuPosition = useCallback(() => {
    const trigger = triggerRef.current
    if (!trigger) return

    const rect = trigger.getBoundingClientRect()
    const viewportMargin = 10
    const gap = 8
    const preferredHeight = Math.min(itemCount * rowHeight + 16, 404)
    const spaceBelow = window.innerHeight - rect.bottom - viewportMargin - gap
    const spaceAbove = rect.top - viewportMargin - gap
    const placeAbove = spaceBelow < 280 && spaceAbove > spaceBelow
    const availableHeight = placeAbove ? spaceAbove : spaceBelow
    const maxHeight = Math.max(180, Math.min(preferredHeight, availableHeight))
    const width = Math.min(Math.max(rect.width, minWidth), window.innerWidth - viewportMargin * 2)
    const left = Math.min(
      Math.max(viewportMargin, rect.left),
      window.innerWidth - width - viewportMargin,
    )
    const top = placeAbove
      ? Math.max(viewportMargin, rect.top - maxHeight - gap)
      : rect.bottom + gap

    setMenuPosition({ left, top, width, maxHeight })
  }, [itemCount, minWidth, rowHeight])

  useEffect(() => {
    if (!open) return

    updateMenuPosition()
    const focusFrame = window.requestAnimationFrame(() => menuRef.current?.focus())
    const closeOnOutsidePointer = (event: PointerEvent) => {
      const target = event.target as Node
      if (!triggerRef.current?.contains(target) && !menuRef.current?.contains(target)) {
        setOpen(false)
      }
    }
    const closeOnViewportChange = () => updateMenuPosition()

    document.addEventListener('pointerdown', closeOnOutsidePointer)
    window.addEventListener('resize', closeOnViewportChange)
    window.addEventListener('scroll', closeOnViewportChange, true)
    return () => {
      window.cancelAnimationFrame(focusFrame)
      document.removeEventListener('pointerdown', closeOnOutsidePointer)
      window.removeEventListener('resize', closeOnViewportChange)
      window.removeEventListener('scroll', closeOnViewportChange, true)
    }
  }, [open, updateMenuPosition])

  const openMenu = (index = selectedIndex) => {
    setHighlightedIndex(index)
    setOpen(true)
  }

  const closeMenu = () => {
    setOpen(false)
    window.requestAnimationFrame(() => triggerRef.current?.focus())
  }

  const moveHighlight = (delta: number) => {
    setHighlightedIndex(current => (
      (current + delta + itemCount) % itemCount
    ))
  }

  return {
    open,
    setOpen,
    highlightedIndex,
    setHighlightedIndex,
    menuPosition,
    triggerRef,
    menuRef,
    openMenu,
    closeMenu,
    moveHighlight,
  }
}

function ModePicker({ value, locale, onChange }: ModePickerProps) {
  const selectedIndex = Math.max(0, PRODUCT_MODES.findIndex(item => item.profile === value))
  const selected = PRODUCT_MODES[selectedIndex] ?? PRODUCT_MODES[0]
  const {
    open,
    setOpen,
    highlightedIndex,
    setHighlightedIndex,
    menuPosition,
    triggerRef,
    menuRef,
    openMenu,
    closeMenu,
    moveHighlight,
  } = useFloatingPicker({
    itemCount: PRODUCT_MODES.length,
    selectedIndex,
    minWidth: 360,
    rowHeight: 58,
  })

  const chooseMode = (profile: ProductModeProfile) => {
    onChange(profile)
    closeMenu()
  }

  const handleTriggerKeyDown = (event: React.KeyboardEvent<HTMLButtonElement>) => {
    if (event.key === 'ArrowDown') {
      event.preventDefault()
      openMenu((selectedIndex + 1) % PRODUCT_MODES.length)
    } else if (event.key === 'ArrowUp') {
      event.preventDefault()
      openMenu((selectedIndex - 1 + PRODUCT_MODES.length) % PRODUCT_MODES.length)
    }
  }

  const handleMenuKeyDown = (event: React.KeyboardEvent<HTMLDivElement>) => {
    if (event.key === 'ArrowDown') {
      event.preventDefault()
      moveHighlight(1)
    } else if (event.key === 'ArrowUp') {
      event.preventDefault()
      moveHighlight(-1)
    } else if (event.key === 'Home') {
      event.preventDefault()
      setHighlightedIndex(0)
    } else if (event.key === 'End') {
      event.preventDefault()
      setHighlightedIndex(PRODUCT_MODES.length - 1)
    } else if (event.key === 'Enter' || event.key === ' ') {
      event.preventDefault()
      chooseMode(PRODUCT_MODES[highlightedIndex].profile)
    } else if (event.key === 'Escape') {
      event.preventDefault()
      closeMenu()
    } else if (event.key === 'Tab') {
      setOpen(false)
    }
  }

  return (
    <div className={styles.modePicker}>
      <span className={styles.modePickerLabel}>{text(locale, 'Mode', '模式')}</span>
      <button
        ref={triggerRef}
        type="button"
        className={styles.modePickerTrigger}
        aria-haspopup="listbox"
        aria-expanded={open}
        aria-label={text(locale, 'Choose target mode', '选择目标模式')}
        onClick={() => (open ? closeMenu() : openMenu())}
        onKeyDown={handleTriggerKeyDown}
      >
        <span className={styles.modePickerIcon}>{selected.icon}</span>
        <span className={styles.modePickerText}>
          <strong>{labelForOption(selected, locale)}</strong>
          <small>{summaryForOption(selected, locale)}</small>
        </span>
        <span className={styles.modePickerMeta}>
          <span className={selected.policy === 'hot_candidate' ? styles.hotBadge : styles.coldBadge}>
            {policyLabel(selected.policy, locale)}
          </span>
          <ChevronDown className={open ? styles.modePickerChevronOpen : styles.modePickerChevron} size={16} />
        </span>
      </button>

      {open && createPortal(
        <div
          ref={menuRef}
          className={styles.modeMenu}
          style={menuPosition}
          role="listbox"
          tabIndex={-1}
          aria-label={text(locale, 'Target mode', '目标模式')}
          onKeyDown={handleMenuKeyDown}
        >
          {PRODUCT_MODES.map((item, index) => {
            const isSelected = item.profile === value
            const isHighlighted = index === highlightedIndex
            return (
              <button
                type="button"
                key={item.profile}
                role="option"
                aria-selected={isSelected}
                className={[
                  styles.modeMenuOption,
                  isSelected ? styles.modeMenuOptionSelected : '',
                  isHighlighted ? styles.modeMenuOptionHighlighted : '',
                ].filter(Boolean).join(' ')}
                onMouseEnter={() => setHighlightedIndex(index)}
                onClick={() => chooseMode(item.profile)}
              >
                <span className={styles.modeMenuIcon}>{item.icon}</span>
                <span className={styles.modeMenuText}>
                  <strong>{labelForOption(item, locale)}</strong>
                  <small>{summaryForOption(item, locale)}</small>
                </span>
                <span className={styles.modeMenuAside}>
                  <span className={item.policy === 'hot_candidate' ? styles.hotBadge : styles.coldBadge}>
                    {policyLabel(item.policy, locale)}
                  </span>
                  <Check className={isSelected ? styles.modeMenuCheckVisible : styles.modeMenuCheck} size={15} />
                </span>
              </button>
            )
          })}
        </div>,
        document.body,
      )}
    </div>
  )
}

interface SimplePickerOption<T extends string> {
  value: T
  label: string
}

interface SimplePickerProps<T extends string> {
  label: string
  ariaLabel: string
  value: T
  options: SimplePickerOption<T>[]
  onChange: (value: T) => void
  disabled?: boolean
}

function SimplePicker<T extends string>({
  label,
  ariaLabel,
  value,
  options,
  onChange,
  disabled = false,
}: SimplePickerProps<T>) {
  const selectedIndex = Math.max(0, options.findIndex(item => item.value === value))
  const selected = options[selectedIndex]
  const {
    open,
    setOpen,
    highlightedIndex,
    setHighlightedIndex,
    menuPosition,
    triggerRef,
    menuRef,
    openMenu,
    closeMenu,
    moveHighlight,
  } = useFloatingPicker({
    itemCount: options.length,
    selectedIndex,
    minWidth: 220,
    rowHeight: 42,
  })

  const chooseOption = (nextValue: T) => {
    onChange(nextValue)
    closeMenu()
  }

  const handleTriggerKeyDown = (event: React.KeyboardEvent<HTMLButtonElement>) => {
    if (disabled) return
    if (event.key === 'ArrowDown') {
      event.preventDefault()
      openMenu((selectedIndex + 1) % options.length)
    } else if (event.key === 'ArrowUp') {
      event.preventDefault()
      openMenu((selectedIndex - 1 + options.length) % options.length)
    }
  }

  const handleMenuKeyDown = (event: React.KeyboardEvent<HTMLDivElement>) => {
    if (event.key === 'ArrowDown') {
      event.preventDefault()
      moveHighlight(1)
    } else if (event.key === 'ArrowUp') {
      event.preventDefault()
      moveHighlight(-1)
    } else if (event.key === 'Home') {
      event.preventDefault()
      setHighlightedIndex(0)
    } else if (event.key === 'End') {
      event.preventDefault()
      setHighlightedIndex(options.length - 1)
    } else if (event.key === 'Enter' || event.key === ' ') {
      event.preventDefault()
      chooseOption(options[highlightedIndex].value)
    } else if (event.key === 'Escape') {
      event.preventDefault()
      closeMenu()
    } else if (event.key === 'Tab') {
      setOpen(false)
    }
  }

  return (
    <div className={styles.compactField}>
      <span>{label}</span>
      <button
        ref={triggerRef}
        type="button"
        className={styles.simplePickerTrigger}
        aria-haspopup="listbox"
        aria-expanded={open}
        aria-label={ariaLabel}
        disabled={disabled}
        onClick={() => (open ? closeMenu() : openMenu())}
        onKeyDown={handleTriggerKeyDown}
      >
        <span className={styles.simplePickerValue}>{selected?.label ?? value}</span>
        <ChevronDown className={open ? styles.modePickerChevronOpen : styles.modePickerChevron} size={16} />
      </button>

      {open && createPortal(
        <div
          ref={menuRef}
          className={`${styles.modeMenu} ${styles.simplePickerMenu}`}
          style={menuPosition}
          role="listbox"
          tabIndex={-1}
          aria-label={ariaLabel}
          onKeyDown={handleMenuKeyDown}
        >
          {options.map((item, index) => {
            const isSelected = item.value === value
            const isHighlighted = index === highlightedIndex
            return (
              <button
                type="button"
                key={item.value}
                role="option"
                aria-selected={isSelected}
                className={[
                  styles.simplePickerOption,
                  isSelected ? styles.modeMenuOptionSelected : '',
                  isHighlighted ? styles.modeMenuOptionHighlighted : '',
                ].filter(Boolean).join(' ')}
                onMouseEnter={() => setHighlightedIndex(index)}
                onClick={() => chooseOption(item.value)}
              >
                <span>{item.label}</span>
                <Check className={isSelected ? styles.modeMenuCheckVisible : styles.modeMenuCheck} size={15} />
              </button>
            )
          })}
        </div>,
        document.body,
      )}
    </div>
  )
}

export function ProductModePanel({
  sseState,
  showToast,
  locale = 'en',
}: ProductModePanelProps) {
  const [session, setSession] = useState(sseState.session)
  const [maps, setMaps] = useState<MapInfo[]>([])
  const [selectedProfile, setSelectedProfile] = useState<ProductModeProfile>('teleop')
  const endpoint = 'thunder_field'
  const [strategy, setStrategy] = useState<NonNullable<RuntimeSwitchRequest['strategy']>>('auto')
  const [selectedMap, setSelectedMap] = useState('')
  const [relocalize, setRelocalize] = useState(true)
  const [allowRestart, setAllowRestart] = useState(false)
  const [loadingRuntime, setLoadingRuntime] = useState(false)
  const [switchBusy, setSwitchBusy] = useState(false)
  const [switchWaiting, setSwitchWaiting] = useState(false)
  const [switchResult, setSwitchResult] = useState<RuntimeSwitchResponse | null>(null)
  const [recoveryBusy, setRecoveryBusy] = useState(false)
  const profileSelectionTouched = useRef(false)

  useEffect(() => {
    if (sseState.session) setSession(sseState.session)
  }, [sseState.session])

  useEffect(() => {
    if (session && !profileSelectionTouched.current) {
      setSelectedProfile(sessionProductProfile(session))
    }
  }, [session])

  const refreshRuntime = useCallback(async (notifyOnError = true) => {
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
      if (notifyOnError) {
        showToast(`${text(locale, 'Runtime refresh failed', '运行状态刷新失败')}: ${error instanceof Error ? error.message : String(error)}`, 'error')
      }
    } finally {
      setLoadingRuntime(false)
    }
  }, [locale, showToast])

  useEffect(() => {
    void refreshRuntime(false)
  }, [refreshRuntime])

  const navigableMaps = useMemo(() => maps.filter(mapIsNavigationReady), [maps])
  const selectedOption = PRODUCT_MODES.find(item => item.profile === selectedProfile) ?? PRODUCT_MODES[0]
  const strategyOptions = useMemo(
    () => STRATEGIES.map(item => ({ value: item, label: strategyLabel(item, locale) })),
    [locale],
  )
  const mapOptions = useMemo(
    () => [
      {
        value: '',
        label: selectedOption.requiresSavedMap
          ? text(locale, 'Choose map', '选择地图')
          : text(locale, 'Not required', '不需要'),
      },
      ...navigableMaps.map(map => ({ value: map.name, label: map.name })),
    ],
    [locale, navigableMaps, selectedOption.requiresSavedMap],
  )
  const currentProfile = (session?.product_profile || sessionFallbackProfile(session?.mode)) as string
  const currentProductSession = session?.product_session && session.product_session !== 'idle'
    ? session.product_session
    : productSessionForProfile(currentProfile, session?.mode)
  const currentProductLabel = productLabelFor(currentProductSession || currentProfile, locale)
  const switchPending = switchWaiting
  const activeMap = session?.active_map ?? ''
  const activeMapInfo = maps.find(map => map.name === activeMap)
  const selectedMapInfo = maps.find(map => map.name === selectedMap)
  const activeMapReady = activeMapInfo ? mapIsNavigationReady(activeMapInfo) : Boolean(session?.map_has_pcd)
  const selectedMapReady = selectedMapInfo ? mapIsNavigationReady(selectedMapInfo) : !selectedOption.requiresSavedMap
  const mapPoints = sseState.mapCloud?.count
  const savedPoints = sseState.savedMap?.count
  const slamDiag = sseState.slamDiag?.data ?? {}
  const sceneMode = diagText(slamDiag, 'scene_mode') ?? '--'
  const localizationBackend = session?.localization_backend ?? session?.slam_profile ?? sseState.slamStatus?.mode ?? '--'
  const relocalizeSupported =
    session?.saved_map_relocalization_supported ??
    session?.relocalization_supported ??
    false
  const savedMapAligned =
    !selectedOption.requiresSavedMap ||
    (Boolean(activeMap) && activeMap === selectedMap && activeMapReady && selectedMapReady)
  const liveMapReady = !selectedOption.requiresLiveMap || Boolean(mapPoints && mapPoints > 0)
  const mapChainOk = savedMapAligned && liveMapReady
  const mapUsageLabel = selectedOption.requiresSavedMap
    ? selectedMap || text(locale, 'Choose a saved map', '选择保存地图')
    : selectedOption.requiresLiveMap
      ? text(locale, 'Live map stream', '实时地图流')
      : text(locale, 'No map required', '不需要地图')
  const activeMapLabel = activeMap || (
    selectedOption.requiresSavedMap
      ? text(locale, 'Not loaded', '未加载')
      : text(locale, 'Not required', '不需要')
  )
  const mapChainStatus = selectedOption.requiresSavedMap
    ? (savedMapAligned ? text(locale, 'Saved map aligned', '保存地图已对齐') : text(locale, 'Saved map mismatch', '保存地图不一致'))
    : selectedOption.requiresLiveMap
      ? (liveMapReady ? text(locale, 'Live map ready', '实时地图就绪') : text(locale, 'Waiting for live map', '等待实时地图'))
      : text(locale, 'No map required', '不需要地图')
  const missingMap = selectedOption.requiresSavedMap && !selectedMap
  const plannedLifecycle = lifecycleFrom(switchResult)
  const restartAckRequired = plannedLifecycle === 'cold_restart'
    || (!switchResult && selectedOption.policy === 'cold_restart')
  const executeDisabled = switchBusy
    || switchPending
    || missingMap
    || (restartAckRequired && !allowRestart)
  const executeTitle = missingMap
    ? text(locale, 'Choose a navigation-ready map first', '请先选择可导航地图')
    : switchPending
      ? text(locale, 'Waiting for restarted runtime readiness', '等待重启后的运行时就绪')
    : restartAckRequired && !allowRestart
      ? text(locale, 'Enable service restart first', '请先允许重启服务')
      : text(locale, 'Switch product mode', '切换产品模式')
  const switchResultText = switchResult
    ? (switchResult.accepted
        ? switchPending
          ? text(locale, 'Accepted; waiting for runtime', '已受理；等待运行时')
          : text(locale, 'Runtime ready', '运行时已就绪')
        : switchResult.ok
          ? text(locale, 'Preflight passed', '预检通过')
          : text(locale, 'Blocked', '受阻'))
    : plannedLifecycle

  useEffect(() => {
    if (selectedMap) return
    if (activeMap) {
      setSelectedMap(activeMap)
      return
    }
    const first = navigableMaps[0]?.name
    if (first) setSelectedMap(first)
  }, [activeMap, navigableMaps, selectedMap])

  const buildSwitchRequest = useCallback((execute: boolean): RuntimeSwitchRequest => ({
    current_profile: currentProfile,
    target_profile: selectedProfile,
    target_endpoint: endpoint,
    endpoint,
    map_name: selectedOption.requiresSavedMap ? selectedMap : null,
    relocalize,
    initial_pose: null,
    strategy,
    execute,
    allow_restart: execute ? allowRestart : false,
  }), [
    allowRestart,
    currentProfile,
    endpoint,
    relocalize,
    selectedMap,
    selectedOption.requiresSavedMap,
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
          ? `${text(locale, 'Preflight passed', '预检通过')}: ${lifecycleFrom(result)}`
          : `${text(locale, 'Preflight blocked', '预检受阻')}: ${shortBlockers(result.blockers, locale)}`,
        result.ok ? 'success' : 'error',
      )
    } catch (error) {
      showToast(`${text(locale, 'Preflight failed', '预检失败')}: ${error instanceof Error ? error.message : String(error)}`, 'error')
    } finally {
      setSwitchBusy(false)
    }
  }, [buildSwitchRequest, locale, showToast])

  const executeSwitch = useCallback(async () => {
    setSwitchBusy(true)
    setSwitchWaiting(false)
    try {
      const targetProfile = selectedProfile
      const targetMap = selectedOption.requiresSavedMap ? selectedMap : null
      const result = await api.runRuntimeSwitch(buildSwitchRequest(true))
      setSwitchResult(result)
      showToast(
        result.ok
          ? `${text(locale, result.accepted ? 'Switch accepted' : 'Switch planned', result.accepted ? '切换已接受' : '切换计划已生成')}: ${result.status}`
          : `${text(locale, 'Switch rejected', '切换被拒绝')}: ${shortBlockers(result.blockers, locale)}`,
        result.accepted ? 'info' : result.ok ? 'success' : 'error',
      )
      if (result.accepted) {
        setSwitchWaiting(true)
        const readySession = await waitForProductProfileReady(targetProfile, targetMap, {
          fetchSession: api.fetchSession,
          fetchNavigation: api.fetchNavigationStatus,
        })
        setSession(readySession)
        showToast(
          `${text(locale, 'Product mode ready', '产品模式已就绪')}: ${productLabelFor(targetProfile, locale)}`,
          'success',
        )
      }
      await refreshRuntime(false)
    } catch (error) {
      showToast(`${text(locale, 'Product mode switch failed', '产品模式切换失败')}: ${error instanceof Error ? error.message : String(error)}`, 'error')
    } finally {
      setSwitchWaiting(false)
      setSwitchBusy(false)
    }
  }, [buildSwitchRequest, locale, refreshRuntime, selectedMap, selectedOption.requiresSavedMap, selectedProfile, showToast])

  const runAutoRelocalize = useCallback(async () => {
    setRecoveryBusy(true)
    try {
      const response = await api.autoRelocalize()
      const statusText = typeof response.status === 'string' ? response.status : ''
      const message = response.message || statusText || (response.ok ? text(locale, 'Accepted', '已接受') : text(locale, 'Rejected', '已拒绝'))
      showToast(
        response.ok
          ? `${text(locale, 'Relocalization accepted', '重定位已接受')}: ${message}`
          : `${text(locale, 'Relocalization rejected', '重定位被拒绝')}: ${message}`,
        response.ok ? 'success' : 'error',
      )
      void refreshRuntime(false)
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error)
      showToast(`${text(locale, 'Relocalization failed', '重定位失败')}: ${message}`, 'error')
    } finally {
      setRecoveryBusy(false)
    }
  }, [locale, refreshRuntime, showToast])

  const modeSelect = (
    <ModePicker
      value={selectedProfile}
      locale={locale}
      onChange={profile => {
        profileSelectionTouched.current = true
        setSelectedProfile(profile)
        setSwitchResult(null)
      }}
    />
  )

  const mapSelect = (
    <SimplePicker
      label={text(locale, 'Map', '地图')}
      ariaLabel={text(locale, 'Choose map', '选择地图')}
      value={selectedMap}
      options={mapOptions}
      onChange={setSelectedMap}
      disabled={navigableMaps.length === 0 || !selectedOption.requiresSavedMap}
    />
  )

  const compactBody = (
    <>
      <div className={styles.compactStatus}>
        <div>
          <span>{text(locale, 'Current', '当前')}</span>
          <strong title={currentProductSession}>{currentProductLabel}</strong>
        </div>
        <div>
          <span>{text(locale, 'Target', '目标')}</span>
          <strong title={selectedOption.session}>{labelForOption(selectedOption, locale)}</strong>
        </div>
        <div>
          <span>{text(locale, 'Result', '结果')}</span>
          <strong className={styles[responseTone(switchResult)]}>{switchResultText}</strong>
        </div>
      </div>

      {modeSelect}

      <div className={styles.compactSplit}>
        <SimplePicker
          label={text(locale, 'Strategy', '策略')}
          ariaLabel={text(locale, 'Choose switch strategy', '选择切换策略')}
          value={strategy}
          options={strategyOptions}
          onChange={setStrategy}
        />
        {mapSelect}
      </div>

      <div className={styles.compactToggles}>
        <label>
          <input
            type="checkbox"
            checked={relocalize}
            onChange={event => setRelocalize(event.target.checked)}
            disabled={!selectedOption.requiresSavedMap}
          />
          <span>{text(locale, 'Relocalize', '重定位')}</span>
        </label>
        <label>
          <input
            type="checkbox"
            checked={allowRestart}
            onChange={event => setAllowRestart(event.target.checked)}
          />
          <span>{text(locale, 'Allow restart', '允许重启')}</span>
        </label>
      </div>

      <div className={mapChainOk ? styles.mapChain : styles.mapChainWarn}>
        <div>
          <span>{text(locale, 'Active map', '当前地图')}</span>
          <strong>{activeMapLabel}</strong>
        </div>
        <div>
          <span>{text(locale, 'Mode map', '模式用图')}</span>
          <strong>{mapUsageLabel}</strong>
        </div>
        <div>
          <span>{text(locale, 'Backend', '后端')}</span>
          <strong title={localizationBackend}>{backendLabel(localizationBackend, locale)}</strong>
        </div>
        <div>
          <span>{text(locale, 'Cloud', '点云')}</span>
          <strong>{compactNumber(mapPoints)}</strong>
        </div>
      </div>

      <div className={styles.mapChainMeta}>
        <span>{mapChainStatus}</span>
        <span>{text(locale, 'Saved', '保存')} {compactNumber(savedPoints)}</span>
        <span>{text(locale, 'Scene', '场景')} {sceneMode}</span>
      </div>

      {missingMap && (
        <div className={styles.inlineWarn}>
          <AlertTriangle size={14} />
          <span>{text(locale, 'This mode needs a navigation-ready saved map.', '这个模式需要可导航的保存地图。')}</span>
        </div>
      )}
      <div className={styles.compactActions}>
        <button className={styles.secondaryButton} onClick={runPreflight} disabled={switchBusy || switchPending || missingMap}>
          <CheckCircle2 size={15} />
          {text(locale, 'Check', '预检')}
        </button>
        <button className={styles.primaryButton} onClick={executeSwitch} disabled={executeDisabled} title={executeTitle}>
          <Play size={15} />
          {text(locale, 'Switch', '切换')}
        </button>
        <button
          className={styles.secondaryButton}
          onClick={runAutoRelocalize}
          disabled={recoveryBusy || !activeMap || !relocalizeSupported}
          title={!activeMap
            ? text(locale, 'Load a saved map first', '请先加载保存地图')
            : text(locale, 'Run global relocalization', '执行全局重定位')}
        >
          <LocateFixed size={15} />
          {text(locale, 'Relocalize', '重定位')}
        </button>
      </div>

      {switchResult?.blockers && switchResult.blockers.length > 0 && (
        <div className={styles.compactBlocker}>{shortBlockers(switchResult.blockers, locale)}</div>
      )}
    </>
  )

  return (
    <section className={`${styles.panel} ${styles.compactCard}`}>
      <div className={styles.compactHeader}>
        <div className={styles.panelTitle}>
          <Route size={16} />
          <h2>{text(locale, 'Mode Setup', '模式配置')}</h2>
        </div>
        <button
          className={styles.iconOnlyButton}
          onClick={() => refreshRuntime()}
          disabled={loadingRuntime}
          title={text(locale, 'Refresh runtime and maps', '刷新运行状态和地图')}
        >
          <RefreshCw size={15} />
        </button>
      </div>
      {compactBody}
    </section>
  )
}
