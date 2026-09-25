import { useCallback, useEffect, useMemo, useRef, useState, type SyntheticEvent } from 'react'
import { ChevronDown, Gamepad2, Keyboard, RotateCcw, X } from 'lucide-react'
import type { AppBootstrapResponse, ProductName, SSEState, ToastKind } from '../types'
import * as api from '../services/api'
import {
  TeleopWsClient,
  type TeleopAck,
  type TeleopConnectionState,
} from '../services/teleopWsClient'
import styles from './TeleopPanel.module.css'

interface TeleopPanelProps {
  sseState: SSEState
  onExit: () => void
  showToast: (msg: string, kind?: ToastKind) => void
}

const TELEOP_PRODUCTS = new Set<ProductName>(['teleop', 'teleop_avoid', 'map'])
const SEND_INTERVAL_MS = 20
const BOOTSTRAP_RETRY_MS = 1000
const PRECISION_SCALE = 0.4
const DEFAULT_TELEOP_LIMITS = { linearMps: 0.5, yawRadS: 1.0 }

interface TeleopLimits {
  linearMps: number
  yawRadS: number
}

function currentProduct(sseState: SSEState): ProductName | null {
  const product = sseState.session?.product
  return product && TELEOP_PRODUCTS.has(product) ? product : null
}

function teleopPathFromBootstrap(bootstrap: AppBootstrapResponse | null): string | null {
  const path = bootstrap?.media?.teleop_ws
  if (typeof path === 'string' && path.length > 0) return path
  return null
}

function teleopLimitsFromBootstrap(bootstrap: AppBootstrapResponse | null): TeleopLimits {
  const teleop = bootstrap?.control?.teleop
  const limits = teleop && typeof teleop === 'object'
    ? (teleop as Record<string, unknown>).limits
    : null
  const values = limits && typeof limits === 'object'
    ? limits as Record<string, unknown>
    : {}
  const linearMps = Number(values.linear_mps)
  const yawRadS = Number(values.yaw_rad_s)
  return {
    linearMps: Number.isFinite(linearMps) && linearMps > 0
      ? linearMps
      : DEFAULT_TELEOP_LIMITS.linearMps,
    yawRadS: Number.isFinite(yawRadS) && yawRadS > 0
      ? yawRadS
      : DEFAULT_TELEOP_LIMITS.yawRadS,
  }
}

function blocksTeleopKeyboard(target: EventTarget | null, teleopRoot: HTMLElement | null): boolean {
  if (!(target instanceof Element)) return false
  if (target.closest('input, textarea, select, summary, details[open]') || (target instanceof HTMLElement && target.isContentEditable)) return true
  if (teleopRoot?.contains(target)) return false
  return target.closest('button, a[href], summary, [role="button"], [role="tab"], [role="dialog"], [role="menu"], [role="listbox"], details[open]') !== null
}

function teleopKey(event: KeyboardEvent): string {
  // Key values can change between down/up under an IME; physical codes do not.
  if (/^Key[WASDQEM]$/.test(event.code)) return event.code.slice(3).toLowerCase()
  if (event.code === 'ShiftLeft' || event.code === 'ShiftRight') return 'shift'
  return event.key.toLowerCase()
}

function commandFromKeys(
  keys: Set<string>,
  limits: TeleopLimits,
  scale = 1,
): { vxMps: number; vyMps: number; yawRps: number } {
  const forward = (keys.has('w') ? 1 : 0) + (keys.has('s') ? -1 : 0)
  const lateral = (keys.has('a') ? 1 : 0) + (keys.has('d') ? -1 : 0)
  const yaw = (keys.has('q') ? 1 : 0) + (keys.has('e') ? -1 : 0)
  return {
    vxMps: forward * limits.linearMps * scale,
    vyMps: lateral * limits.linearMps * scale,
    yawRps: yaw * limits.yawRadS * scale,
  }
}

function rejectionMessage(ack: TeleopAck): string {
  if (ack.error === 'control_in_use') return '另一个控制端正在使用机器人'
  if (ack.error === 'safety_stop') return '安全停车仍在生效'
  if (ack.error === 'hold_unconfirmed') return '机器人没有确认保持指令'
  if (ack.error === 'connection_unavailable') return '遥控连接暂时不可用'
  if (ack.error === 'input_expired' || ack.error === 'input_timeout') return '遥控响应超时，已停止输入；连接稳定后请重新按键'
  return ack.message ?? '遥控暂时不可用'
}

export function TeleopPanel({ sseState, showToast, onExit }: TeleopPanelProps) {
  const [bootstrap, setBootstrap] = useState<AppBootstrapResponse | null>(null)
  const [bootstrapError, setBootstrapError] = useState<string | null>(null)
  const [openState, setOpenState] = useState<TeleopConnectionState>('idle')
  const [sessionActive, setSessionActive] = useState(false)
  const [lastAck, setLastAck] = useState<TeleopAck | null>(null)
  const [resumePending, setResumePending] = useState(false)
  const [precisionMode, setPrecisionMode] = useState(false)
  const [speedLimit, setSpeedLimit] = useState(0.5)
  const panelRef = useRef<HTMLDivElement>(null)
  const keysRef = useRef<Set<string>>(new Set())
  const blockedKeysRef = useRef<Set<string>>(new Set())
  const clientRef = useRef<TeleopWsClient | null>(null)
  const inputActiveRef = useRef(false)
  const manualModeRef = useRef(false)

  const clearInputIntent = useCallback(() => {
    for (const key of keysRef.current) blockedKeysRef.current.add(key)
    keysRef.current.clear()
    inputActiveRef.current = false
    manualModeRef.current = false
    setPrecisionMode(false)
  }, [])

  const product = currentProduct(sseState)
  const teleopPath = teleopPathFromBootstrap(bootstrap)
  const backendLimits = useMemo(() => teleopLimitsFromBootstrap(bootstrap), [bootstrap])
  const teleopLimits = useMemo(() => ({ ...backendLimits,
    linearMps: Math.min(speedLimit, backendLimits.linearMps),
  }), [backendLimits, speedLimit])
  const resumeRequired = sseState.navigationStatus?.control.resume_required === true
  const enabled = Boolean(sseState.connected && product && teleopPath)
  const connectionReady = openState === 'open'

  useEffect(() => {
    if (!product) return undefined
    let cancelled = false
    let retryTimer: number | null = null
    const loadBootstrap = () => {
      api.fetchAppBootstrap().then(next => {
        if (cancelled) return
        setBootstrap(next)
        setBootstrapError(null)
        if (!teleopPathFromBootstrap(next)) {
          retryTimer = window.setTimeout(loadBootstrap, BOOTSTRAP_RETRY_MS)
        }
      })
      .catch((cause: unknown) => {
        if (cancelled) return
        setBootstrapError(cause instanceof Error ? cause.message : String(cause))
        retryTimer = window.setTimeout(loadBootstrap, BOOTSTRAP_RETRY_MS)
      })
    }
    loadBootstrap()
    return () => {
      cancelled = true
      if (retryTimer !== null) window.clearTimeout(retryTimer)
    }
  }, [product])

  const closeClient = useCallback(() => {
    clearInputIntent()
    clientRef.current?.disconnect()
    clientRef.current = null
    setSessionActive(false)
    setOpenState('closed')
  }, [clearInputIntent])

  const connectClient = useCallback(() => {
    if (!teleopPath) return
    if (clientRef.current) {
      clientRef.current.connect()
      return
    }
    const client = new TeleopWsClient({
      url: teleopPath,
      clientId: `web-scene-${Math.random().toString(36).slice(2, 8)}`,
      onState: state => {
        setOpenState(state)
        if (state === 'open') setLastAck(null)
        if (state === 'closed' || state === 'error') clearInputIntent()
      },
      onAck: ack => {
        setLastAck(ack)
        if (ack.type === 'control_rejected') {
          clearInputIntent()
          if (['control_unavailable', 'safety_stop'].includes(ack.error ?? '')) {
            client.hold('rejected_input')
          }
          showToast(rejectionMessage(ack), 'error')
        }
      },
    })
    clientRef.current = client
    setSessionActive(true)
    client.connect()
  }, [clearInputIntent, showToast, teleopPath])

  useEffect(() => {
    if (enabled) return undefined
    const timer = window.setTimeout(() => closeClient(), 0)
    return () => window.clearTimeout(timer)
  }, [closeClient, enabled])

  useEffect(() => {
    panelRef.current?.focus({ preventScroll: true })
    return () => closeClient()
  }, [closeClient])

  const sendHold = useCallback(() => {
    clientRef.current?.hold()
    inputActiveRef.current = false
  }, [])

  const setManualEscape = useCallback((active: boolean) => {
    if (active && enabled && !connectionReady) connectClient()
    const next = active && product === 'teleop_avoid' && enabled && !resumeRequired
    manualModeRef.current = next
  }, [connectClient, connectionReady, enabled, product, resumeRequired])

  const quiesceInput = useCallback(() => {
    clearInputIntent()
    sendHold()
  }, [clearInputIntent, sendHold])

  const onShortcutToggle = (event: SyntheticEvent<HTMLDetailsElement>) => {
    const menu = event.currentTarget
    if (!menu.open && menu.contains(document.activeElement)) {
      panelRef.current?.focus({ preventScroll: true })
    }
  }

  useEffect(() => {
    if (resumeRequired) quiesceInput()
  }, [quiesceInput, resumeRequired])

  const resumeControl = useCallback(async () => {
    quiesceInput()
    setResumePending(true)
    try {
      const response = await api.resumeNavigation()
      if (!response.ok) throw new Error(response.status || 'navigation_resume_rejected')
      showToast('控制保护已恢复；请重新按下方向键发送新指令', 'success')
    } catch (cause: unknown) {
      showToast(`恢复控制失败：${cause instanceof Error ? cause.message : String(cause)}`, 'error')
    } finally {
      setResumePending(false)
    }
  }, [quiesceInput, showToast])

  useEffect(() => {
    const stopForWindowLoss = () => quiesceInput()
    const onVisibility = () => {
      if (document.hidden) quiesceInput()
    }
    window.addEventListener('blur', stopForWindowLoss)
    window.addEventListener('pagehide', stopForWindowLoss)
    document.addEventListener('visibilitychange', onVisibility)
    return () => {
      window.removeEventListener('blur', stopForWindowLoss)
      window.removeEventListener('pagehide', stopForWindowLoss)
      document.removeEventListener('visibilitychange', onVisibility)
    }
  }, [quiesceInput])

  useEffect(() => {
    const quiesceForInteraction = (event: Event) => {
      if (blocksTeleopKeyboard(event.target, panelRef.current)
        && (keysRef.current.size > 0 || inputActiveRef.current || manualModeRef.current)) {
        quiesceInput()
      }
    }
    const onKeyDown = (event: KeyboardEvent) => {
      const key = teleopKey(event)
      if (event.ctrlKey || event.metaKey || event.altKey || event.isComposing) {
        if (keysRef.current.size > 0 || inputActiveRef.current || manualModeRef.current) quiesceInput()
        return
      }
      if (blocksTeleopKeyboard(event.target, panelRef.current)) {
        if (['w', 'a', 's', 'd', 'q', 'e', 'm', 'shift'].includes(key)) blockedKeysRef.current.add(key)
        quiesceForInteraction(event)
        return
      }
      if (key === 'escape') {
        quiesceInput()
        onExit()
        event.preventDefault()
        return
      }
      if (blockedKeysRef.current.has(key)) {
        event.preventDefault()
        return
      }
      if (event.repeat && (key === 'm' || key === 'shift')) return
      if (key === 'shift') {
        setPrecisionMode(true)
        return
      }
      if (key === 'm' && product === 'teleop_avoid' && connectionReady && !resumeRequired) {
        setManualEscape(true)
        event.preventDefault()
        return
      }
      if (event.code === 'Space' && connectionReady) {
        event.preventDefault()
        quiesceInput()
        return
      }
      if (enabled && !resumeRequired && ['w', 'a', 's', 'd', 'q', 'e'].includes(key)) {
        if (event.repeat && !keysRef.current.has(key)) return
        if (!connectionReady) {
          connectClient()
          event.preventDefault()
          return
        }
        keysRef.current.add(key)
        event.preventDefault()
      }
    }
    const onKeyUp = (event: KeyboardEvent) => {
      const key = teleopKey(event)
      const wasBlocked = blockedKeysRef.current.delete(key)
      const inputBlocked = blocksTeleopKeyboard(event.target, panelRef.current)
      if (key === 'shift') {
        setPrecisionMode(false)
        return
      }
      if (key === 'm') {
        setManualEscape(false)
        if (!inputBlocked) event.preventDefault()
        return
      }
      if (event.code === 'Space' && connectionReady && !inputBlocked) {
        event.preventDefault()
        return
      }
      if (['w', 'a', 's', 'd', 'q', 'e'].includes(key)) {
        const tracked = keysRef.current.delete(key)
        if ((tracked || wasBlocked) && !inputBlocked) event.preventDefault()
        if (tracked && keysRef.current.size === 0) sendHold()
      }
    }
    window.addEventListener('keydown', onKeyDown)
    window.addEventListener('keyup', onKeyUp, true)
    document.addEventListener('focusin', quiesceForInteraction)
    document.addEventListener('pointerdown', quiesceForInteraction, true)
    return () => {
      window.removeEventListener('keydown', onKeyDown)
      window.removeEventListener('keyup', onKeyUp, true)
      document.removeEventListener('focusin', quiesceForInteraction)
      document.removeEventListener('pointerdown', quiesceForInteraction, true)
    }
  }, [connectClient, connectionReady, enabled, onExit, product, quiesceInput, resumeRequired, sendHold, setManualEscape])

  useEffect(() => {
    if (!enabled || !connectionReady || resumeRequired) return
    const timer = window.setInterval(() => {
      const fromKeys = commandFromKeys(
        keysRef.current,
        teleopLimits,
        precisionMode ? PRECISION_SCALE : 1,
      )
      const { vxMps, vyMps, yawRps } = fromKeys
      const deadman = keysRef.current.size > 0
      if (deadman) {
        clientRef.current?.move({
          vxMps,
          vyMps,
          yawRps,
          deadman: true,
          manualMode: manualModeRef.current,
        })
        inputActiveRef.current = true
      } else if (inputActiveRef.current) {
        sendHold()
      }
    }, SEND_INTERVAL_MS)
    return () => window.clearInterval(timer)
  }, [connectionReady, enabled, precisionMode, resumeRequired, sendHold, teleopLimits])

  const controlDisabledReason = useMemo(() => {
    if (!sseState.connected) return '实时连接已断开，恢复后请重新按键'
    if (!product) return '当前模式不支持网页遥控'
    if (bootstrapError) return '无法获取遥控配置，正在重试'
    if (!teleopPath) return '正在准备遥控'
    if (resumeRequired) return '请恢复控制后重新按键'
    if (!connectionReady) return sessionActive ? '正在连接遥控，连接后请重新按键' : '按住方向键开始，松键停车'
    return ''
  }, [bootstrapError, connectionReady, product, resumeRequired, sessionActive, sseState.connected, teleopPath])

  const controlStatus = !sseState.connected ? '连接已断开'
    : resumeRequired ? '需恢复控制' : bootstrapError ? '连接失败'
    : !enabled ? '准备中' : connectionReady ? (precisionMode ? '精细模式' : '遥控已连接')
    : sessionActive ? (openState === 'error' || openState === 'closed' ? '连接已断开' : '连接中') : '按键开始'

  return (
    <div ref={panelRef} className={styles.modeBar} tabIndex={-1} aria-label="遥控模式">
      <span className={styles.modeLabel} title={product === 'teleop_avoid' ? '辅助避障遥控 · 松键停车' : '直接遥控 · 松键停车 · 需要主动避让障碍'}>
        <Gamepad2 size={16} aria-hidden="true" />遥控
      </span>
      <label className={styles.speedLimit} title="遥控限速">
        <select aria-label="遥控限速" disabled={!enabled} value={teleopLimits.linearMps} onChange={event => {
          quiesceInput()
          setSpeedLimit(Number(event.target.value))
          panelRef.current?.focus({ preventScroll: true })
        }}>
          {[...new Set([0.1, 0.2, 0.3, 0.5, backendLimits.linearMps])]
            .filter(value => value <= backendLimits.linearMps && value <= 0.5)
            .sort((a, b) => a - b)
            .map(value => <option key={value} value={value}>{value.toFixed(2)} m/s</option>)}
        </select>
        <ChevronDown size={12} aria-hidden="true" />
      </label>
      <details className={styles.shortcutHelp} name="lingtu-menu" onToggle={onShortcutToggle} onKeyDown={event => {
        if (event.key !== 'Escape') return
        event.preventDefault()
        event.stopPropagation()
        event.currentTarget.open = false
        panelRef.current?.focus({ preventScroll: true })
      }}>
        <summary aria-label="遥控快捷键" title="查看快捷键"><Keyboard size={16} aria-hidden="true" /></summary>
        <div className={styles.shortcutContent}>
          <div className={styles.shortcutHeading}><strong>键盘遥控</strong><span>松键停车</span></div>
          <dl>
            <div><dt><kbd>W</kbd><kbd>S</kbd></dt><dd>前进 / 后退</dd></div>
            <div><dt><kbd>A</kbd><kbd>D</kbd></dt><dd>向左 / 向右侧移</dd></div>
            <div><dt><kbd>Q</kbd><kbd>E</kbd></dt><dd>向左 / 向右转向</dd></div>
            <div><dt><kbd>Shift</kbd></dt><dd>按住慢行（40%）</dd></div>
            <div><dt><kbd>Space</kbd></dt><dd>停止</dd></div>
            <div><dt><kbd>Esc</kbd></dt><dd>退出遥控</dd></div>
          </dl>
          <p>{product === 'map' ? '建图直接遥控，请主动避让障碍。' : product === 'teleop_avoid'
            ? '按住 M 加方向键可临时脱困。' : '直接遥控，请主动避让障碍。'}</p>
          <p>松键停车。操作菜单时暂停，点击地图后重新按键。</p>
        </div>
      </details>
      <button className={styles.exitBtn} aria-label="退出遥控" title="退出遥控 · Esc" onClick={() => {
        quiesceInput()
        onExit()
      }}><X size={15} aria-hidden="true" /></button>
      <span className={styles.connection} role="status" title={controlDisabledReason || controlStatus}
        data-connected={connectionReady && enabled && !resumeRequired}
        data-attention={!sseState.connected || resumeRequired || Boolean(bootstrapError) || openState === 'error' || (openState === 'closed' && sessionActive)}>
        <i aria-hidden="true" /><span>{controlStatus}</span>
      </span>
      {resumeRequired && <button className={styles.actionBtn} disabled={resumePending || !sseState.connected} onClick={resumeControl}>
        <RotateCcw size={13} /> {resumePending ? '恢复中' : '恢复控制'}
      </button>}
      {lastAck?.type === 'control_rejected' && sseState.connected &&
        <span className={styles.rejection} role="status">{rejectionMessage(lastAck)}</span>}
    </div>
  )
}
