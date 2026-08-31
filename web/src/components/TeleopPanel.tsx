import { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import { Gamepad2, PauseCircle, PlayCircle, RotateCcw, StopCircle } from 'lucide-react'
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
  showToast: (msg: string, kind?: ToastKind) => void
}

type Direction = 'forward' | 'back' | 'left' | 'right' | 'turnLeft' | 'turnRight'

const TELEOP_PRODUCTS = new Set<ProductName>(['teleop', 'teleop_avoid'])
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

function isEditableTarget(target: EventTarget | null): boolean {
  if (!(target instanceof HTMLElement)) return false
  return target.isContentEditable || ['INPUT', 'TEXTAREA', 'SELECT'].includes(target.tagName)
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

function commandFromDirections(
  directions: Set<Direction>,
  limits: TeleopLimits,
): { vxMps: number; vyMps: number; yawRps: number } {
  const forward = (directions.has('forward') ? 1 : 0) + (directions.has('back') ? -1 : 0)
  const lateral = (directions.has('left') ? 1 : 0) + (directions.has('right') ? -1 : 0)
  const yaw = (directions.has('turnLeft') ? 1 : 0) + (directions.has('turnRight') ? -1 : 0)
  return {
    vxMps: forward * limits.linearMps,
    vyMps: lateral * limits.linearMps,
    yawRps: yaw * limits.yawRadS,
  }
}

function ackSummary(ack: TeleopAck | null): string {
  if (!ack) return '尚未收到 Gateway 回执'
  const reason = ack.error ?? ack.message ?? ack.stage ?? ''
  const bits = [
    ack.type,
    ack.action,
    ack.request_id ? `request=${ack.request_id}` : null,
    ack.final_cmd_vel_confirmed === true ? '最终零速已确认' : null,
    ack.motor_confirmed === true ? '电机已确认' : ack.motor_confirmed === false ? '非电机确认' : null,
    reason,
  ].filter(Boolean)
  return bits.join(' · ')
}

function rejectionMessage(ack: TeleopAck): string {
  if (ack.error === 'control_in_use') return '另一个控制端正在使用机器人'
  if (ack.error === 'safety_stop') return '安全停车仍在生效'
  if (ack.error === 'hold_unconfirmed') return '机器人没有确认保持指令'
  if (ack.error === 'connection_unavailable') return '遥控连接暂时不可用'
  return ack.message ?? '遥控暂时不可用'
}

export function TeleopPanel({ sseState, showToast }: TeleopPanelProps) {
  const [bootstrap, setBootstrap] = useState<AppBootstrapResponse | null>(null)
  const [bootstrapError, setBootstrapError] = useState<string | null>(null)
  const [openState, setOpenState] = useState<TeleopConnectionState>('idle')
  const [sessionActive, setSessionActive] = useState(false)
  const [lastAck, setLastAck] = useState<TeleopAck | null>(null)
  const [resumePending, setResumePending] = useState(false)
  const [precisionMode, setPrecisionMode] = useState(false)
  const [manualMode, setManualMode] = useState(false)
  const [activeDirections, setActiveDirections] = useState<Set<Direction>>(() => new Set())
  const keysRef = useRef<Set<string>>(new Set())
  const blockedKeysRef = useRef<Set<string>>(new Set())
  const directionsRef = useRef<Set<Direction>>(new Set())
  const clientRef = useRef<TeleopWsClient | null>(null)
  const inputActiveRef = useRef(false)
  const manualModeRef = useRef(false)

  const clearInputIntent = useCallback(() => {
    for (const key of keysRef.current) blockedKeysRef.current.add(key)
    keysRef.current.clear()
    directionsRef.current.clear()
    inputActiveRef.current = false
    manualModeRef.current = false
    setPrecisionMode(false)
    setManualMode(false)
    setActiveDirections(new Set())
  }, [])

  const product = currentProduct(sseState)
  const teleopPath = teleopPathFromBootstrap(bootstrap)
  const teleopLimits = useMemo(() => teleopLimitsFromBootstrap(bootstrap), [bootstrap])
  const resumeRequired = sseState.navigationStatus?.control.resume_required === true
  const enabled = Boolean(product && teleopPath)
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

  useEffect(() => () => closeClient(), [closeClient])

  const sendHold = useCallback(() => {
    clientRef.current?.hold()
    inputActiveRef.current = false
  }, [])

  const setManualEscape = useCallback((active: boolean) => {
    if (active && enabled && !connectionReady) connectClient()
    const next = active && product === 'teleop_avoid' && enabled && !resumeRequired
    manualModeRef.current = next
    setManualMode(next)
  }, [connectClient, connectionReady, enabled, product, resumeRequired])

  const quiesceInput = useCallback(() => {
    clearInputIntent()
    sendHold()
  }, [clearInputIntent, sendHold])

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
    const onKeyDown = (event: KeyboardEvent) => {
      if (isEditableTarget(event.target)) return
      const key = event.key.toLowerCase()
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
        if (!connectionReady) connectClient()
        if (blockedKeysRef.current.has(key)) {
          event.preventDefault()
          return
        }
        keysRef.current.add(key)
        event.preventDefault()
      }
    }
    const onKeyUp = (event: KeyboardEvent) => {
      const key = event.key.toLowerCase()
      if (key === 'shift') {
        setPrecisionMode(false)
        return
      }
      if (key === 'm') {
        setManualEscape(false)
        event.preventDefault()
        return
      }
      if (event.code === 'Space' && connectionReady) {
        event.preventDefault()
        return
      }
      if (['w', 'a', 's', 'd', 'q', 'e'].includes(key)) {
        const wasBlocked = blockedKeysRef.current.delete(key)
        const tracked = keysRef.current.delete(key)
        if (tracked || wasBlocked) event.preventDefault()
        if (tracked && keysRef.current.size === 0 && directionsRef.current.size === 0) sendHold()
      }
    }
    window.addEventListener('keydown', onKeyDown)
    window.addEventListener('keyup', onKeyUp)
    return () => {
      window.removeEventListener('keydown', onKeyDown)
      window.removeEventListener('keyup', onKeyUp)
    }
  }, [connectClient, connectionReady, enabled, product, quiesceInput, resumeRequired, sendHold, setManualEscape])

  useEffect(() => {
    if (!connectionReady || resumeRequired) return
    const timer = window.setInterval(() => {
      const fromKeys = commandFromKeys(
        keysRef.current,
        teleopLimits,
        precisionMode ? PRECISION_SCALE : 1,
      )
      const fromButtons = commandFromDirections(directionsRef.current, teleopLimits)
      const vxMps = fromKeys.vxMps + fromButtons.vxMps
      const vyMps = fromKeys.vyMps + fromButtons.vyMps
      const yawRps = fromKeys.yawRps + fromButtons.yawRps
      const deadman = keysRef.current.size > 0 || directionsRef.current.size > 0
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
  }, [connectionReady, precisionMode, resumeRequired, sendHold, teleopLimits])

  const setDirection = useCallback((direction: Direction, active: boolean) => {
    if (resumeRequired) return
    const wasActive = directionsRef.current.has(direction)
    const next = new Set(directionsRef.current)
    if (active) next.add(direction)
    else next.delete(direction)
    directionsRef.current = next
    setActiveDirections(next)
    if (!active && wasActive && next.size === 0 && keysRef.current.size === 0) sendHold()
  }, [resumeRequired, sendHold])

  const startDirection = useCallback((direction: Direction) => {
    if (!enabled || resumeRequired) return
    if (!connectionReady) connectClient()
    setDirection(direction, true)
  }, [connectClient, connectionReady, enabled, resumeRequired, setDirection])

  const controlDisabledReason = useMemo(() => {
    if (!product) return '当前 Product 不是 teleop/teleop_avoid'
    if (bootstrapError) return `bootstrap 失败：${bootstrapError}`
    if (!teleopPath) return 'bootstrap 未声明 teleop_ws'
    if (!connectionReady) return 'WebSocket 未连接'
    if (resumeRequired) return '运行时保护已锁存，需要显式恢复后重新发送方向指令'
    return ''
  }, [bootstrapError, connectionReady, product, resumeRequired, teleopPath])

  const holdLike = lastAck?.type === 'control_ack' && lastAck.action === 'hold'

  return (
    <div className={styles.panel} aria-label="Web teleop control">
      <div className={styles.header}>
        <div className={styles.title}><Gamepad2 size={15} /> 遥控</div>
        <span className={styles.badge}>{product ?? '非遥控模式'} · {openState}</span>
      </div>
      <div className={styles.body}>
        <div className={styles.status}>
          <span className={enabled ? styles.ok : styles.warn}>
            {enabled ? '可连接：仅提交操作员意图，不代表电机已执行' : controlDisabledReason}
          </span>
          <span>键盘：直接按住 W/S 前后、A/D 横移、Q/E 旋转；Shift 为 40% 精细模式，Space 立即保持。速度单位为 m/s、rad/s。</span>
          {product === 'teleop_avoid' && (
            <span>脱困：按住 M 或“按住脱困”并同时给方向；按住期间局部避障关闭，松开立即恢复 CMU。急停、deadman、驱动与速度限制始终有效。</span>
          )}
          <span>当前上限：平移 {teleopLimits.linearMps.toFixed(2)} m/s，旋转 {teleopLimits.yawRadS.toFixed(2)} rad/s。</span>
          {holdLike && <span className={styles.warn}>机器人已保持；再次按方向键即可继续。</span>}
          {resumeRequired && <span className={styles.warn}>安全保护要求恢复；旧指令不会自动重放。</span>}
        </div>

        <div className={styles.controls}>
          <span />
          <button
            className={styles.controlBtn}
            disabled={!enabled || resumeRequired}
            onPointerDown={() => startDirection('forward')}
            onPointerUp={() => setDirection('forward', false)}
            onPointerLeave={() => setDirection('forward', false)}
            onPointerCancel={() => setDirection('forward', false)}
          >前</button>
          <span />
          <button
            className={styles.controlBtn}
            disabled={!enabled || resumeRequired}
            onPointerDown={() => startDirection('left')}
            onPointerUp={() => setDirection('left', false)}
            onPointerLeave={() => setDirection('left', false)}
            onPointerCancel={() => setDirection('left', false)}
          >左</button>
          <button
            className={styles.controlBtn}
            disabled={!enabled || resumeRequired}
            onPointerDown={() => startDirection('back')}
            onPointerUp={() => setDirection('back', false)}
            onPointerLeave={() => setDirection('back', false)}
            onPointerCancel={() => setDirection('back', false)}
          >后</button>
          <button
            className={styles.controlBtn}
            disabled={!enabled || resumeRequired}
            onPointerDown={() => startDirection('right')}
            onPointerUp={() => setDirection('right', false)}
            onPointerLeave={() => setDirection('right', false)}
            onPointerCancel={() => setDirection('right', false)}
          >右</button>
          <button
            className={styles.controlBtn}
            disabled={!enabled || resumeRequired}
            onPointerDown={() => startDirection('turnLeft')}
            onPointerUp={() => setDirection('turnLeft', false)}
            onPointerLeave={() => setDirection('turnLeft', false)}
            onPointerCancel={() => setDirection('turnLeft', false)}
          >左转</button>
          <span className={activeDirections.size > 0 ? styles.ok : styles.badge}>按住运行</span>
          <button
            className={styles.controlBtn}
            disabled={!enabled || resumeRequired}
            onPointerDown={() => startDirection('turnRight')}
            onPointerUp={() => setDirection('turnRight', false)}
            onPointerLeave={() => setDirection('turnRight', false)}
            onPointerCancel={() => setDirection('turnRight', false)}
          >右转</button>
        </div>

        <div className={styles.actions}>
          <button className={styles.actionBtn} disabled={!enabled || connectionReady} onClick={connectClient}>
            <PlayCircle size={13} /> 连接
          </button>
          <button className={styles.actionBtn} disabled={!connectionReady} onClick={quiesceInput}>
            <PauseCircle size={13} /> 保持
          </button>
          {product === 'teleop_avoid' && (
            <button
              className={manualMode ? styles.dangerBtn : styles.actionBtn}
              disabled={!enabled || resumeRequired}
              onPointerDown={() => setManualEscape(true)}
              onPointerUp={() => setManualEscape(false)}
              onPointerLeave={() => setManualEscape(false)}
              onPointerCancel={() => setManualEscape(false)}
            >
              <Gamepad2 size={13} /> {manualMode ? '脱困中' : '按住脱困'}
            </button>
          )}
          {resumeRequired && (
            <button className={styles.actionBtn} disabled={resumePending} onClick={resumeControl}>
              <RotateCcw size={13} /> {resumePending ? '恢复中' : '恢复控制'}
            </button>
          )}
          <button className={styles.dangerBtn} disabled={!sessionActive} onClick={closeClient}>
            <StopCircle size={13} /> 断开
          </button>
        </div>

        <div className={styles.ack} aria-live="polite">
          <strong>最新回执</strong>
          {ackSummary(lastAck)}
        </div>
      </div>
    </div>
  )
}
