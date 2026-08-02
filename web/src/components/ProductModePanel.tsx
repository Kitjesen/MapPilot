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
  Copy,
  Radio,
  RefreshCw,
  Route,
  ShieldCheck,
  Target,
} from 'lucide-react'
import * as api from '../services/api'
import { mapIsNavigationReady } from '../services/mapReadiness'
import type {
  MapInfo,
  ProductName,
  RuntimeSwitchPlanRequest,
  RuntimeSwitchPlanResponse,
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
  product: ProductName
  session: string
  labelEn: string
  labelZh: string
  summaryEn: string
  summaryZh: string
  requiresSavedMap: boolean
  requiresLiveMap: boolean
  icon: ReactNode
}

const PRODUCT_MODES: ProductModeOption[] = [
  {
    product: 'teleop',
    session: 'teleop',
    labelEn: 'Teleop',
    labelZh: '手动',
    summaryEn: 'Manual velocity control with safety retained.',
    summaryZh: '人工速度控制，保留安全链路。',
    requiresSavedMap: false,
    requiresLiveMap: false,
    icon: <Radio size={16} />,
  },
  {
    product: 'teleop_avoid',
    session: 'teleop_avoid',
    labelEn: 'Teleop Avoid',
    labelZh: '手动避障',
    summaryEn: 'Manual control with live localization guard.',
    summaryZh: '人工控制叠加实时定位保护。',
    requiresSavedMap: false,
    requiresLiveMap: true,
    icon: <ShieldCheck size={16} />,
  },
  {
    product: 'map',
    session: 'mapping',
    labelEn: 'Mapping',
    labelZh: '建图',
    summaryEn: 'Builds a live map from the current scan stream.',
    summaryZh: '使用当前扫描流生成实时地图。',
    requiresSavedMap: false,
    requiresLiveMap: true,
    icon: <MapIcon size={16} />,
  },
  {
    product: 'explore',
    session: 'exploration',
    labelEn: 'Explore',
    labelZh: '前沿探索',
    summaryEn: 'Explores reachable frontiers with the navigation stack.',
    summaryZh: '通过导航栈探索可达前沿。',
    requiresSavedMap: false,
    requiresLiveMap: true,
    icon: <Compass size={16} />,
  },
  {
    product: 'nav',
    session: 'navigation',
    labelEn: 'Navigation',
    labelZh: '导航',
    summaryEn: 'Navigates against a saved map.',
    summaryZh: '基于保存地图执行导航。',
    requiresSavedMap: true,
    requiresLiveMap: true,
    icon: <Route size={16} />,
  },
  {
    product: 'tracking',
    session: 'tracking',
    labelEn: 'Tracking',
    labelZh: '跟踪',
    summaryEn: 'Runs target tracking on top of localization.',
    summaryZh: '在定位链路上执行目标跟踪。',
    requiresSavedMap: true,
    requiresLiveMap: true,
    icon: <Target size={16} />,
  },
  {
    product: 'inspection',
    session: 'inspection',
    labelEn: 'Inspection',
    labelZh: '巡检',
    summaryEn: 'Repeats fixed routes and points, then archives verified evidence.',
    summaryZh: '按固定路线与点位复拍，归档可验证证据。',
    requiresSavedMap: true,
    requiresLiveMap: true,
    icon: <Eye size={16} />,
  },
]

function sessionProduct(session?: SSEState['session']): ProductName | null {
  const product = session?.product
  if (PRODUCT_MODES.some(item => item.product === product)) {
    return product as ProductName
  }
  return null
}

function productSessionForProduct(product: ProductName): string {
  const option = PRODUCT_MODES.find(item => item.product === product)
  if (option) return option.session
  return 'unknown'
}

function labelForOption(option: ProductModeOption, locale: Locale): string {
  return text(locale, option.labelEn, option.labelZh)
}

function summaryForOption(option: ProductModeOption, locale: Locale): string {
  return text(locale, option.summaryEn, option.summaryZh)
}

function productLabelFor(productOrSession: string, locale: Locale): string {
  const option = PRODUCT_MODES.find(item => item.product === productOrSession || item.session === productOrSession)
  return option ? labelForOption(option, locale) : productOrSession
}

function compactNumber(value: number | undefined | null): string {
  if (typeof value !== 'number' || !Number.isFinite(value)) return '--'
  if (value >= 1_000_000) return `${(value / 1_000_000).toFixed(1)}M`
  if (value >= 10_000) return `${Math.round(value / 1_000)}k`
  return Math.round(value).toLocaleString()
}

function shortBlockers(blockers: string[] | undefined, locale: Locale): string {
  if (!blockers || blockers.length === 0) return text(locale, 'No blockers', '无阻断项')
  const visible = blockers.slice(0, 2).join('; ')
  return blockers.length > 2 ? `${visible}; +${blockers.length - 2}` : visible
}

function responseTone(response: RuntimeSwitchPlanResponse | null): 'ok' | 'warn' | 'dim' {
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
  value: ProductName | null
  locale: Locale
  onChange: (product: ProductName) => void
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
  const selectedIndex = PRODUCT_MODES.findIndex(item => item.product === value)
  const activeIndex = selectedIndex >= 0 ? selectedIndex : 0
  const selected = selectedIndex >= 0 ? PRODUCT_MODES[selectedIndex] : null
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
    selectedIndex: activeIndex,
    minWidth: 360,
    rowHeight: 58,
  })

  const chooseMode = (product: ProductName) => {
    onChange(product)
    closeMenu()
  }

  const handleTriggerKeyDown = (event: React.KeyboardEvent<HTMLButtonElement>) => {
    if (event.key === 'ArrowDown') {
      event.preventDefault()
      openMenu((activeIndex + 1) % PRODUCT_MODES.length)
    } else if (event.key === 'ArrowUp') {
      event.preventDefault()
      openMenu((activeIndex - 1 + PRODUCT_MODES.length) % PRODUCT_MODES.length)
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
      chooseMode(PRODUCT_MODES[highlightedIndex].product)
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
        <span className={styles.modePickerIcon}>
          {selected ? selected.icon : <AlertTriangle size={16} />}
        </span>
        <span className={styles.modePickerText}>
          <strong>
            {selected ? labelForOption(selected, locale) : text(locale, 'Choose Product', '选择 Product')}
          </strong>
          <small>
            {selected
              ? summaryForOption(selected, locale)
              : text(locale, 'Runtime Product is unknown.', '运行时 Product 未知。')}
          </small>
        </span>
        <span className={styles.modePickerMeta}>
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
            const isSelected = item.product === value
            const isHighlighted = index === highlightedIndex
            return (
              <button
                type="button"
                key={item.product}
                role="option"
                aria-selected={isSelected}
                className={[
                  styles.modeMenuOption,
                  isSelected ? styles.modeMenuOptionSelected : '',
                  isHighlighted ? styles.modeMenuOptionHighlighted : '',
                ].filter(Boolean).join(' ')}
                onMouseEnter={() => setHighlightedIndex(index)}
                onClick={() => chooseMode(item.product)}
              >
                <span className={styles.modeMenuIcon}>{item.icon}</span>
                <span className={styles.modeMenuText}>
                  <strong>{labelForOption(item, locale)}</strong>
                  <small>{summaryForOption(item, locale)}</small>
                </span>
                <span className={styles.modeMenuAside}>
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
  const [selectedProduct, setSelectedProduct] = useState<ProductName | null>(null)
  const [selectedMap, setSelectedMap] = useState('')
  const [relocalize, setRelocalize] = useState(true)
  const [loadingRuntime, setLoadingRuntime] = useState(false)
  const [switchBusy, setSwitchBusy] = useState(false)
  const [switchResult, setSwitchResult] = useState<RuntimeSwitchPlanResponse | null>(null)
  const [recoveryBusy, setRecoveryBusy] = useState(false)
  const productSelectionTouched = useRef(false)

  useEffect(() => {
    if (sseState.session) setSession(sseState.session)
  }, [sseState.session])

  useEffect(() => {
    if (session && !productSelectionTouched.current) {
      const product = sessionProduct(session)
      setSelectedProduct(product)
      if (product === 'explore') setSelectedMap(session.active_map ?? '')
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
  const selectedOption = PRODUCT_MODES.find(item => item.product === selectedProduct)
  const targetProductKnown = selectedProduct !== null && selectedOption !== undefined
  const requiresSavedMap = selectedOption?.requiresSavedMap === true
  const allowsSavedMap = requiresSavedMap || selectedProduct === 'explore'
  const usesSavedMap = allowsSavedMap && Boolean(selectedMap)
  const requiresLiveMap = selectedOption?.requiresLiveMap === true
  const mapOptions = useMemo(
    () => [
      {
        value: '',
        label: !selectedOption
          ? text(locale, 'Choose Product first', '请先选择 Product')
          : requiresSavedMap
            ? text(locale, 'Choose map', '选择地图')
            : selectedProduct === 'explore'
              ? text(locale, 'No saved map (live exploration)', '不使用保存地图（实时探索）')
              : text(locale, 'Not required', '不需要'),
      },
      ...navigableMaps.map(map => ({ value: map.name, label: map.name })),
    ],
    [locale, navigableMaps, requiresSavedMap, selectedOption, selectedProduct],
  )
  const currentProduct = sessionProduct(session)
  const currentProductSession = currentProduct
    ? session?.product_session && session.product_session !== 'idle'
      ? session.product_session
      : productSessionForProduct(currentProduct)
    : 'unknown'
  const currentProductLabel = currentProduct
    ? productLabelFor(currentProductSession || currentProduct, locale)
    : text(locale, 'Unknown Product', 'Product 未知')
  const activeMap = session?.active_map ?? ''
  const activeMapInfo = maps.find(map => map.name === activeMap)
  const selectedMapInfo = maps.find(map => map.name === selectedMap)
  const activeMapReady = activeMapInfo ? mapIsNavigationReady(activeMapInfo) : Boolean(session?.map_has_pcd)
  const selectedMapReady = targetProductKnown && (
    selectedMap
      ? Boolean(selectedMapInfo && mapIsNavigationReady(selectedMapInfo))
      : !requiresSavedMap
  )
  const mapPoints = sseState.mapCloud?.count
  const savedPoints = sseState.savedMap?.count
  const slamDiag = sseState.slamDiag?.data ?? {}
  const sceneMode = diagText(slamDiag, 'scene_mode') ?? '--'
  const localizationBackend = session?.localization_backend ?? session?.slam_profile ?? sseState.slamStatus?.mode ?? '--'
  const relocalizeSupported =
    session?.saved_map_relocalization_supported ??
    session?.relocalization_supported ??
    false
  const savedMapAligned = targetProductKnown && (
    !usesSavedMap ||
    (Boolean(activeMap) && activeMap === selectedMap && activeMapReady && selectedMapReady)
  )
  const liveMapReady = targetProductKnown && (
    !requiresLiveMap || Boolean(mapPoints && mapPoints > 0)
  )
  const mapChainOk = savedMapAligned && liveMapReady
  const mapUsageLabel = !selectedOption
    ? text(locale, 'Choose Product first', '请先选择 Product')
    : usesSavedMap
      ? selectedMap
      : selectedProduct === 'explore'
        ? text(locale, 'Live exploration', '实时探索')
        : requiresSavedMap
          ? text(locale, 'Choose a saved map', '选择保存地图')
      : requiresLiveMap
      ? text(locale, 'Live map stream', '实时地图流')
      : text(locale, 'No map required', '不需要地图')
  const activeMapLabel = activeMap || (
    usesSavedMap
      ? text(locale, 'Not loaded', '未加载')
      : selectedProduct === 'explore'
        ? text(locale, 'Live exploration', '实时探索')
        : text(locale, 'Not required', '不需要')
  )
  const mapChainStatus = !selectedOption
    ? text(locale, 'Product selection required', '需要选择 Product')
    : usesSavedMap
      ? (savedMapAligned ? text(locale, 'Saved map aligned', '保存地图已对齐') : text(locale, 'Saved map mismatch', '保存地图不一致'))
      : requiresLiveMap
      ? (liveMapReady ? text(locale, 'Live map ready', '实时地图就绪') : text(locale, 'Waiting for live map', '等待实时地图'))
      : text(locale, 'No map required', '不需要地图')
  const missingMap = requiresSavedMap && !selectedMap
  const selectedSavedMapUnavailable = usesSavedMap && !selectedMapReady
  const commandDisabled = switchBusy
    || !targetProductKnown
    || missingMap
    || selectedSavedMapUnavailable
  const commandTitle = !targetProductKnown
    ? text(locale, 'Choose a Product first', '请先选择 Product')
    : missingMap
    ? text(locale, 'Choose a navigation-ready map first', '请先选择可导航地图')
    : selectedSavedMapUnavailable
      ? text(locale, 'The selected map is not navigation-ready', '所选地图尚未达到可导航状态')
    : text(locale, 'Copy ProductControl command', '复制 ProductControl 命令')
  const switchResultText = switchResult
    ? (switchResult.ok ? text(locale, 'Preflight passed', '预检通过') : text(locale, 'Blocked', '受阻'))
    : '--'

  useEffect(() => {
    if (selectedMap || !requiresSavedMap) return
    if (activeMap) {
      setSelectedMap(activeMap)
      return
    }
    const first = navigableMaps[0]?.name
    if (first) setSelectedMap(first)
  }, [activeMap, navigableMaps, requiresSavedMap, selectedMap])

  const buildSwitchRequest = useCallback((): RuntimeSwitchPlanRequest => {
    if (!selectedProduct || !selectedOption) {
      throw new Error('Product selection required')
    }
    return {
      current_product: currentProduct,
      target_product: selectedProduct,
      map_name: usesSavedMap ? selectedMap : null,
      relocalize: usesSavedMap && relocalize,
      initial_pose: null,
    }
  }, [
    currentProduct,
    relocalize,
    selectedMap,
    usesSavedMap,
    selectedOption,
    selectedProduct,
  ])

  const runPreflight = useCallback(async () => {
    setSwitchBusy(true)
    try {
      const result = await api.runRuntimeSwitchPlan(buildSwitchRequest())
      setSwitchResult(result)
      showToast(
        result.ok
          ? text(locale, 'Preflight passed', '预检通过')
          : `${text(locale, 'Preflight blocked', '预检受阻')}: ${shortBlockers(result.blockers, locale)}`,
        result.ok ? 'success' : 'error',
      )
    } catch (error) {
      showToast(`${text(locale, 'Preflight failed', '预检失败')}: ${error instanceof Error ? error.message : String(error)}`, 'error')
    } finally {
      setSwitchBusy(false)
    }
  }, [buildSwitchRequest, locale, showToast])

  const copySwitchCommand = useCallback(async () => {
    setSwitchBusy(true)
    try {
      if (!selectedProduct || !selectedOption) {
        throw new Error('Product selection required')
      }
      const handoff = await api.copyProductSwitchCommand(selectedProduct, {
        currentProduct,
        mapName: usesSavedMap ? selectedMap : null,
        relocalize: usesSavedMap && relocalize,
      })
      setSwitchResult(handoff.plan)
      showToast(
        `${text(locale, 'ProductControl command copied', 'ProductControl 命令已复制')}: ${handoff.command}`,
        'info',
      )
    } catch (error) {
      showToast(`${text(locale, 'Command handoff failed', '命令交接失败')}: ${error instanceof Error ? error.message : String(error)}`, 'error')
    } finally {
      setSwitchBusy(false)
    }
  }, [currentProduct, locale, relocalize, selectedMap, selectedOption, selectedProduct, showToast, usesSavedMap])

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
      value={selectedProduct}
      locale={locale}
      onChange={product => {
        productSelectionTouched.current = true
        setSelectedProduct(product)
        if (product === 'explore' && selectedProduct !== 'explore') setSelectedMap('')
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
      disabled={!targetProductKnown || navigableMaps.length === 0 || !allowsSavedMap}
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
          <strong title={selectedOption?.session ?? 'unknown'}>
            {selectedOption
              ? labelForOption(selectedOption, locale)
              : text(locale, 'Unknown Product', 'Product 未知')}
          </strong>
        </div>
        <div>
          <span>{text(locale, 'Result', '结果')}</span>
          <strong className={styles[responseTone(switchResult)]}>{switchResultText}</strong>
        </div>
      </div>

      {modeSelect}

      <div className={styles.compactSplit}>{mapSelect}</div>

      <div className={styles.compactToggles}>
        <label>
          <input
            type="checkbox"
            checked={relocalize}
            onChange={event => setRelocalize(event.target.checked)}
            disabled={!usesSavedMap}
          />
          <span>{text(locale, 'Relocalize', '重定位')}</span>
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
      {selectedSavedMapUnavailable && (
        <div className={styles.inlineWarn}>
          <AlertTriangle size={14} />
          <span>{text(locale, 'The selected saved map is not navigation-ready.', '所选保存地图尚未达到可导航状态。')}</span>
        </div>
      )}
      <div className={styles.compactActions}>
        <button className={styles.secondaryButton} onClick={runPreflight} disabled={commandDisabled}>
          <CheckCircle2 size={15} />
          {text(locale, 'Check', '预检')}
        </button>
        <button className={styles.primaryButton} onClick={copySwitchCommand} disabled={commandDisabled} title={commandTitle}>
          <Copy size={15} />
          {text(locale, 'Copy command', '复制命令')}
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
