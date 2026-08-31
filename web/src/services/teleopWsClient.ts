export type TeleopAckType = 'ingress_ack' | 'control_ack' | 'control_rejected' | 'unknown'

export interface TeleopAck {
  type: TeleopAckType
  action?: string
  accepted?: boolean
  request_id?: string
  stage?: string
  error?: string
  message?: string
  final_cmd_vel_confirmed?: boolean
  motor_confirmed?: boolean
}

export interface TeleopVelocityCommand {
  vxMps?: number
  vyMps?: number
  yawRps?: number
  deadman: boolean
  manualMode?: boolean
  requestId?: string
}

type WebSocketLike = {
  readyState: number
  send(data: string): void
  close(): void
  onopen: ((event: Event) => void) | null
  onmessage: ((event: MessageEvent) => void) | null
  onclose: ((event: CloseEvent) => void) | null
  onerror: ((event: Event) => void) | null
}

export type TeleopSocketFactory = (url: string) => WebSocketLike

export interface TeleopWsClientOptions {
  url: string
  clientId?: string
  socketFactory?: TeleopSocketFactory
  onAck?: (ack: TeleopAck) => void
  onState?: (state: TeleopConnectionState) => void
  reconnectDelayMs?: number
}

export type TeleopConnectionState = 'idle' | 'connecting' | 'open' | 'closed' | 'error'

const WS_OPEN = 1
const WS_CONNECTING = 0

function finiteOrZero(value: number): number {
  if (!Number.isFinite(value)) return 0
  return value
}

function requestId(prefix: string, sequence: number): string {
  return `web-${prefix}-${Date.now().toString(36)}-${sequence}`
}

function parseAck(data: unknown): TeleopAck {
  let raw: Record<string, unknown>
  if (typeof data === 'string') {
    try {
      const parsed = JSON.parse(data) as unknown
      raw = parsed && typeof parsed === 'object' && !Array.isArray(parsed)
        ? parsed as Record<string, unknown>
        : { value: parsed }
    } catch {
      raw = { value: data }
    }
  } else if (data && typeof data === 'object' && !Array.isArray(data)) {
    raw = data as Record<string, unknown>
  } else {
    raw = { value: data }
  }

  const rawType = typeof raw.type === 'string' ? raw.type : 'unknown'
  const type: TeleopAckType = rawType === 'ingress_ack'
    || rawType === 'control_ack'
    || rawType === 'control_rejected'
    ? rawType
    : 'unknown'

  return {
    type,
    action: typeof raw.action === 'string' ? raw.action : undefined,
    accepted: typeof raw.accepted === 'boolean' ? raw.accepted : undefined,
    request_id: typeof raw.request_id === 'string' ? raw.request_id : undefined,
    stage: typeof raw.stage === 'string' ? raw.stage : undefined,
    error: typeof raw.error === 'string' ? raw.error : undefined,
    message: typeof raw.message === 'string' ? raw.message : undefined,
    final_cmd_vel_confirmed: typeof raw.final_cmd_vel_confirmed === 'boolean'
      ? raw.final_cmd_vel_confirmed
      : undefined,
    motor_confirmed: typeof raw.motor_confirmed === 'boolean' ? raw.motor_confirmed : undefined,
  }
}

export function resolveTeleopWsUrl(path: string, clientId: string): string {
  const suffix = `client_id=${encodeURIComponent(clientId)}`
  if (/^wss?:\/\//i.test(path)) {
    const url = new URL(path)
    url.searchParams.set('client_id', clientId)
    return url.toString()
  }
  const base = typeof window !== 'undefined' && window.location
    ? `${window.location.protocol === 'https:' ? 'wss:' : 'ws:'}//${window.location.host}`
    : 'ws://127.0.0.1'
  const url = new URL(path || '/ws/teleop', base)
  url.searchParams.set('client_id', clientId)
  if (!url.searchParams.has('source')) url.searchParams.set('source', 'web_scene')
  return `${url.toString()}${url.search ? '' : `?${suffix}`}`
}

export class TeleopWsClient {
  private readonly url: string
  private readonly socketFactory: TeleopSocketFactory
  private readonly onAck?: (ack: TeleopAck) => void
  private readonly onState?: (state: TeleopConnectionState) => void
  private readonly reconnectDelayMs: number
  private socket: WebSocketLike | null = null
  private sequence = 0
  private reconnectEnabled = false
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null

  constructor(options: TeleopWsClientOptions) {
    this.url = resolveTeleopWsUrl(options.url, options.clientId ?? 'web-operator')
    this.socketFactory = options.socketFactory ?? ((url: string) => new WebSocket(url))
    this.onAck = options.onAck
    this.onState = options.onState
    this.reconnectDelayMs = Math.max(0, options.reconnectDelayMs ?? 1000)
  }

  connect(): void {
    this.reconnectEnabled = true
    if (this.socket && [WS_CONNECTING, WS_OPEN].includes(this.socket.readyState)) return
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer)
      this.reconnectTimer = null
    }
    this.onState?.('connecting')
    const socket = this.socketFactory(this.url)
    this.socket = socket
    socket.onopen = () => this.onState?.('open')
    socket.onmessage = event => {
      const ack = parseAck(event.data)
      if (
        ack.type === 'control_rejected'
        && ack.error === 'control_in_use'
        && !ack.request_id
      ) {
        this.reconnectEnabled = false
      }
      this.onAck?.(ack)
    }
    socket.onerror = event => {
      this.onState?.('error')
      this.onAck?.(parseAck({
        type: 'control_rejected',
        error: 'connection_unavailable',
        message: String(event.type),
      }))
      if (this.socket === socket) socket.close()
    }
    socket.onclose = () => {
      if (this.socket !== socket) return
      this.onState?.('closed')
      this.socket = null
      if (this.reconnectEnabled) {
        this.reconnectTimer = setTimeout(() => {
          this.reconnectTimer = null
          this.connect()
        }, this.reconnectDelayMs)
      }
    }
  }

  isOpen(): boolean {
    return this.socket?.readyState === WS_OPEN
  }

  move(command: TeleopVelocityCommand): string | null {
    if (!this.isOpen()) return null
    const sequence = ++this.sequence
    const vxMps = finiteOrZero(command.vxMps ?? 0)
    const vyMps = finiteOrZero(command.vyMps ?? 0)
    const yawRps = finiteOrZero(command.yawRps ?? 0)
    const id = command.requestId ?? requestId(command.deadman ? 'velocity' : 'hold', sequence)
    this.socket?.send(JSON.stringify({
      type: 'velocity',
      vx_mps: vxMps,
      vy_mps: vyMps,
      yaw_rps: yawRps,
      deadman: command.deadman,
      manual_mode: command.manualMode === true,
      request_id: id,
    }))
    return id
  }

  hold(reason = 'web_operator_hold'): string | null {
    return this.move({
      vxMps: 0,
      vyMps: 0,
      yawRps: 0,
      deadman: false,
      requestId: requestId(reason, this.sequence + 1),
    })
  }

  disconnect(): void {
    this.reconnectEnabled = false
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer)
      this.reconnectTimer = null
    }
    if (this.socket) {
      if (this.socket.readyState === WS_OPEN) this.hold('web_close')
      this.socket.close()
    }
    this.socket = null
  }
}
