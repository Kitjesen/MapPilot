/**
 * Pull a WebRTC video stream through a WHEP endpoint.
 *
 * The endpoint is intentionally generic. Gateway may proxy go2rtc or a native
 * RTC publisher. Each manager owns one source generation so a late offer,
 * track event, or reconnect timer cannot modify the current view.
 */
import { useCallback, useEffect, useRef, useState } from 'react'

export type WHEPError = null | 'unavailable' | 'signalling' | 'ice'

export interface WHEPState {
  stream: MediaStream | null
  connected: boolean
  error: WHEPError
  rotationDegrees: number
  reconnect: () => void
}

export interface WHEPConnectionCallbacks {
  onConnecting: () => void
  onStream: (stream: MediaStream) => void
  onConnected: () => void
  onError: (error: Exclude<WHEPError, null>) => void
}

export interface WHEPRuntime {
  createPeerConnection: (configuration: RTCConfiguration) => RTCPeerConnection
  fetch: typeof fetch
  setTimeout: (callback: () => void, delay: number) => ReturnType<typeof setTimeout>
  clearTimeout: (handle: ReturnType<typeof setTimeout>) => void
}

interface WHEPConnectionManagerOptions {
  whepPath: string
  streamSrc: string
  callbacks: WHEPConnectionCallbacks
  iceServers?: RTCIceServer[]
  runtime?: WHEPRuntime
  iceGatheringTimeoutMs?: number
  postTimeoutMs?: number
  retryMinMs?: number
  retryMaxMs?: number
}

interface WHEPSessionCallbacks {
  onStream: (stream: MediaStream) => void
  onConnected: () => void
  onTerminal: (error: Exclude<WHEPError, null>) => void
}

interface WHEPStatus {
  available?: boolean
  ice_servers?: RTCIceServer[]
  rotation_degrees?: number
}

const defaultRuntime: WHEPRuntime = {
  createPeerConnection: configuration => new RTCPeerConnection(configuration),
  fetch: (input, init) => fetch(input, init),
  setTimeout: (callback, delay) => setTimeout(callback, delay),
  clearTimeout: handle => clearTimeout(handle),
}

function endpointForSource(whepPath: string, streamSrc: string): string {
  if (!streamSrc) return whepPath
  const separator = whepPath.includes('?') ? '&' : '?'
  return `${whepPath}${separator}src=${encodeURIComponent(streamSrc)}`
}

function resolveResourceLocation(location: string, requestUrl: string): string {
  if (/^https?:\/\//i.test(location) || location.startsWith('/')) return location
  const path = requestUrl.split('?')[0]
  return `${path.slice(0, path.lastIndexOf('/') + 1)}${location}`
}

class WHEPSession {
  private readonly abort = new AbortController()
  private readonly pc: RTCPeerConnection
  private active = true
  private resourceLocation: string | null = null
  private resourceEtag: string | null = null
  private deletePromise: Promise<void> | null = null
  private postInFlight = false
  private postCleanupTimer: ReturnType<typeof setTimeout> | null = null
  private iceTimer: ReturnType<typeof setTimeout> | null = null
  private iceListener: (() => void) | null = null
  private readonly trackListener: (event: RTCTrackEvent) => void
  private readonly connectionListener: () => void
  private readonly whepPath: string
  private readonly streamSrc: string
  private readonly runtime: WHEPRuntime
  private readonly callbacks: WHEPSessionCallbacks
  private readonly iceGatheringTimeoutMs: number
  private readonly postTimeoutMs: number
  readonly done: Promise<void>

  constructor(
    whepPath: string,
    streamSrc: string,
    runtime: WHEPRuntime,
    callbacks: WHEPSessionCallbacks,
    iceGatheringTimeoutMs: number,
    postTimeoutMs: number,
    iceServers: RTCIceServer[],
  ) {
    this.whepPath = whepPath
    this.streamSrc = streamSrc
    this.runtime = runtime
    this.callbacks = callbacks
    this.iceGatheringTimeoutMs = iceGatheringTimeoutMs
    this.postTimeoutMs = postTimeoutMs
    this.pc = runtime.createPeerConnection({ iceServers })
    this.trackListener = event => {
      if (!this.active) return
      this.callbacks.onStream(event.streams[0] ?? new MediaStream([event.track]))
    }
    this.connectionListener = () => {
      if (!this.active) return
      const state = this.pc.connectionState
      if (state === 'connected') this.callbacks.onConnected()
      if (state === 'failed' || state === 'disconnected') this.fail('ice')
    }
    this.pc.addEventListener('track', this.trackListener)
    this.pc.addEventListener('connectionstatechange', this.connectionListener)
    this.pc.addTransceiver('video', { direction: 'recvonly' })
    this.done = this.negotiate()
  }

  private isActive(): boolean {
    return this.active && !this.abort.signal.aborted
  }

  private async waitForIceGathering(): Promise<void> {
    if (this.pc.iceGatheringState === 'complete' || !this.isActive()) return
    await new Promise<void>(resolve => {
      let resolved = false
      const finish = () => {
        if (resolved) return
        resolved = true
        if (this.iceListener) {
          this.pc.removeEventListener('icegatheringstatechange', this.iceListener)
          this.iceListener = null
        }
        if (this.iceTimer) {
          this.runtime.clearTimeout(this.iceTimer)
          this.iceTimer = null
        }
        this.abort.signal.removeEventListener('abort', finish)
        resolve()
      }
      this.iceListener = () => {
        if (this.pc.iceGatheringState === 'complete') finish()
      }
      this.pc.addEventListener('icegatheringstatechange', this.iceListener)
      this.abort.signal.addEventListener('abort', finish, { once: true })
      this.iceTimer = this.runtime.setTimeout(finish, this.iceGatheringTimeoutMs)
    })
  }

  private async negotiate(): Promise<void> {
    try {
      const offer = await this.pc.createOffer()
      if (!this.isActive()) return
      await this.pc.setLocalDescription(offer)
      if (!this.isActive()) return
      await this.waitForIceGathering()
      if (!this.isActive()) return
      const local = this.pc.localDescription
      if (!local?.sdp) throw new Error('WHEP offer has no local SDP')

      const requestUrl = endpointForSource(this.whepPath, this.streamSrc)
      this.postInFlight = true
      this.postCleanupTimer = this.runtime.setTimeout(() => {
        this.postCleanupTimer = null
        this.postInFlight = false
        this.abort.abort()
        if (this.active) this.fail('signalling')
      }, this.postTimeoutMs)
      let response: Response
      try {
        response = await this.runtime.fetch(requestUrl, {
          method: 'POST',
          headers: { 'content-type': 'application/sdp' },
          body: local.sdp,
          signal: this.abort.signal,
        })
      } finally {
        this.postInFlight = false
        if (this.postCleanupTimer) {
          this.runtime.clearTimeout(this.postCleanupTimer)
          this.postCleanupTimer = null
        }
      }
      const location = response.headers.get('location')
      if (location) this.resourceLocation = resolveResourceLocation(location, requestUrl)
      this.resourceEtag = response.headers.get('etag')
      if (!this.isActive()) {
        await this.deleteResource()
        return
      }
      if (!response.ok) {
        this.fail(response.status === 503 ? 'unavailable' : 'signalling')
        return
      }

      const answerSdp = await response.text()
      if (!this.isActive()) return
      await this.pc.setRemoteDescription({ type: 'answer', sdp: answerSdp })
    } catch (error) {
      if (!this.isActive() || (error as { name?: string }).name === 'AbortError') return
      this.fail('signalling')
    }
  }

  private fail(error: Exclude<WHEPError, null>): void {
    if (!this.active) return
    void this.close()
    this.callbacks.onTerminal(error)
  }

  private deleteResource(): Promise<void> {
    if (!this.resourceLocation) return Promise.resolve()
    if (!this.deletePromise) {
      const resource = this.resourceLocation
      const etag = this.resourceEtag
      this.resourceLocation = null
      this.resourceEtag = null
      this.deletePromise = this.runtime.fetch(resource, {
        method: 'DELETE',
        headers: etag ? { 'if-match': etag } : undefined,
      })
        .then(() => undefined)
        .catch(() => undefined)
    }
    return this.deletePromise
  }

  async close(): Promise<void> {
    if (this.active) {
      this.active = false
      // Once the POST body is sent, keep waiting for a bounded upstream HTTP
      // response so its Location resource can be deleted. Aborting here can
      // orphan a session that the media server already created.
      if (!this.postInFlight) this.abort.abort()
      else if (!this.postCleanupTimer) {
        this.postCleanupTimer = this.runtime.setTimeout(() => {
          this.postCleanupTimer = null
          this.abort.abort()
        }, this.postTimeoutMs)
      }
      if (this.iceListener) {
        this.pc.removeEventListener('icegatheringstatechange', this.iceListener)
        this.iceListener = null
      }
      if (this.iceTimer) {
        this.runtime.clearTimeout(this.iceTimer)
        this.iceTimer = null
      }
      this.pc.removeEventListener('track', this.trackListener)
      this.pc.removeEventListener('connectionstatechange', this.connectionListener)
      for (const receiver of this.pc.getReceivers()) {
        try { receiver.track.stop() } catch { /* best-effort browser cleanup */ }
      }
      try { this.pc.close() } catch { /* already closed */ }
    }
    await this.deleteResource()
  }
}

/** Owns reconnect policy for one endpoint and one named video source. */
export class WHEPConnectionManager {
  private readonly runtime: WHEPRuntime
  private readonly iceGatheringTimeoutMs: number
  private readonly postTimeoutMs: number
  private readonly retryMinMs: number
  private readonly retryMaxMs: number
  private session: WHEPSession | null = null
  private retryTimer: ReturnType<typeof setTimeout> | null = null
  private retryAttempt = 0
  private generation = 0
  private active = false
  private readonly options: WHEPConnectionManagerOptions

  constructor(options: WHEPConnectionManagerOptions) {
    this.options = options
    this.runtime = options.runtime ?? defaultRuntime
    this.iceGatheringTimeoutMs = options.iceGatheringTimeoutMs ?? 1500
    // Native WHEP gathers ICE for up to 30 seconds; Gateway has a 35-second budget.
    this.postTimeoutMs = options.postTimeoutMs ?? 40000
    this.retryMinMs = Math.max(1, options.retryMinMs ?? 1000)
    this.retryMaxMs = Math.max(this.retryMinMs, options.retryMaxMs ?? 8000)
  }

  start(): void {
    if (this.active) return
    this.active = true
    this.retryAttempt = 0
    this.connect(++this.generation)
  }

  reconnect(): void {
    if (!this.active) return
    this.clearRetry()
    void this.session?.close()
    this.session = null
    this.retryAttempt = 0
    this.connect(++this.generation)
  }

  async stop(): Promise<void> {
    this.active = false
    ++this.generation
    this.clearRetry()
    const session = this.session
    this.session = null
    await session?.close()
  }

  async settled(): Promise<void> {
    await this.session?.done
  }

  private connect(generation: number): void {
    if (!this.active || generation !== this.generation) return
    this.options.callbacks.onConnecting()
    const session = new WHEPSession(
      this.options.whepPath,
      this.options.streamSrc,
      this.runtime,
      {
        onStream: stream => {
          if (this.isCurrent(session, generation)) this.options.callbacks.onStream(stream)
        },
        onConnected: () => {
          if (!this.isCurrent(session, generation)) return
          this.retryAttempt = 0
          this.options.callbacks.onConnected()
        },
        onTerminal: error => {
          if (!this.isCurrent(session, generation)) return
          this.session = null
          this.options.callbacks.onError(error)
          this.scheduleReconnect(generation)
        },
      },
      this.iceGatheringTimeoutMs,
      this.postTimeoutMs,
      this.options.iceServers ?? [],
    )
    this.session = session
  }

  private isCurrent(session: WHEPSession, generation: number): boolean {
    return this.active && generation === this.generation && this.session === session
  }

  private scheduleReconnect(generation: number): void {
    if (!this.active || generation !== this.generation || this.retryTimer) return
    const delay = Math.min(this.retryMaxMs, this.retryMinMs * 2 ** this.retryAttempt)
    this.retryAttempt += 1
    this.retryTimer = this.runtime.setTimeout(() => {
      this.retryTimer = null
      if (!this.active || generation !== this.generation) return
      this.connect(generation)
    }, delay)
  }

  private clearRetry(): void {
    if (!this.retryTimer) return
    this.runtime.clearTimeout(this.retryTimer)
    this.retryTimer = null
  }
}

export function useWHEP(
  /** Same-origin Gateway endpoint or another WHEP endpoint. */
  whepPath = '/api/v1/webrtc/whep',
  /** Optional source selector understood by the endpoint. */
  streamSrc = 'cam',
  /** Hidden camera views set this false to avoid opening media resources. */
  enabled = true,
): WHEPState {
  const [stream, setStream] = useState<MediaStream | null>(null)
  const [connected, setConnected] = useState(false)
  const [error, setError] = useState<WHEPError>(null)
  const [rotationDegrees, setRotationDegrees] = useState(0)
  const [reconnectNonce, setReconnectNonce] = useState(0)
  const managerRef = useRef<WHEPConnectionManager | null>(null)

  useEffect(() => {
    let cancelled = false
    let statusRetry: ReturnType<typeof setTimeout> | null = null
    let statusAttempt = 0
    const statusAbort = new AbortController()
    setStream(null)
    setConnected(false)
    setError(null)
    if (!enabled || !whepPath) return

    let manager: WHEPConnectionManager | null = null
    const start = async () => {
      let iceServers: RTCIceServer[] = []
      let rotationDegrees = 0
      try {
        const response = await fetch('/api/v1/webrtc/status', { signal: statusAbort.signal })
        if (!response.ok) throw new Error('WebRTC status unavailable')
        const status = await response.json() as WHEPStatus
        if (!status.available) throw new Error('WebRTC unavailable')
        iceServers = status.ice_servers ?? []
        rotationDegrees = Number.isFinite(status.rotation_degrees) ? status.rotation_degrees! : 0
      } catch {
        if (cancelled) return
        setError('unavailable')
        const delay = Math.min(8000, 1000 * 2 ** statusAttempt)
        statusAttempt += 1
        statusRetry = setTimeout(() => {
          statusRetry = null
          void start()
        }, delay)
        return
      }
      if (cancelled) return
      setRotationDegrees(rotationDegrees)
      manager = new WHEPConnectionManager({
        whepPath,
        streamSrc,
        iceServers,
        callbacks: {
          onConnecting: () => {
            setStream(null)
            setConnected(false)
            setError(null)
          },
          onStream: nextStream => setStream(nextStream),
          onConnected: () => {
            setConnected(true)
            setError(null)
          },
          onError: nextError => {
            setStream(null)
            setConnected(false)
            setError(nextError)
          },
        },
      })
      managerRef.current = manager
      manager.start()
    }
    void start()
    return () => {
      cancelled = true
      statusAbort.abort()
      if (statusRetry) clearTimeout(statusRetry)
      if (managerRef.current === manager) managerRef.current = null
      void manager?.stop()
    }
  }, [enabled, reconnectNonce, streamSrc, whepPath])

  const reconnect = useCallback(() => {
    if (managerRef.current) managerRef.current.reconnect()
    else setReconnectNonce(value => value + 1)
  }, [])
  return { stream, connected, error, rotationDegrees, reconnect }
}
