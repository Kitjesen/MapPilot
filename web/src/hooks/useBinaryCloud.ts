/**
 * useBinaryCloud opens the live point-cloud stream and exposes typed arrays
 * for Scene3D. WebSocket is the primary path; HTTP polling is a fallback for
 * dev proxies or deployments where /ws/cloud is unavailable.
 */
import { useEffect, useRef, useState } from 'react'

import CloudDecoderWorker from '../workers/cloudDecoder.ts?worker'
import {
  parseHttpCloudMetadata,
  validateCloudFrameContract,
  type CloudEndpointKind,
  type CloudFrameMetadata,
  type CloudSequenceCursor,
  type CloudStreamKind,
} from '../workers/cloudDecoderCore.ts'

export interface BinaryCloud extends CloudFrameMetadata {
  positions: Float32Array
  colors: Float32Array
  count: number
  seq: number
  connected: boolean
  transport: 'none' | 'ws' | 'http'
  lastFrameAt: number | null
  error: string | null
}

const EMPTY: BinaryCloud = {
  positions: new Float32Array(0),
  colors: new Float32Array(0),
  count: 0,
  seq: 0,
  protocolVersion: null,
  frameId: null,
  epoch: null,
  stampS: null,
  sequence: null,
  streamKind: null,
  connected: false,
  transport: 'none',
  lastFrameAt: null,
  error: null,
}

// Live SLAM clouds can arrive in odom/map frames with a vertical offset before
// relocalization settles. Keep the display permissive and let Scene3D styling
// handle readability; a tight floor/ceiling window can hide every point.
const Z_FLOOR = -20
const Z_CEIL = 20
const COLOR_Z_MIN = -1.0
const COLOR_Z_SPAN = 3.5

function turboColor(t: number, out: Float32Array, off: number) {
  const x = t < 0 ? 0 : t > 1 ? 1 : t
  if (x < 0.5) {
    const u = x * 2
    out[off] = 0.18 + (0.55 - 0.18) * u
    out[off + 1] = 0.55
    out[off + 2] = 0.50 + (0.55 - 0.50) * u
  } else {
    const u = (x - 0.5) * 2
    out[off] = 0.55 + (0.78 - 0.55) * u
    out[off + 1] = 0.55 + (0.60 - 0.55) * u
    out[off + 2] = 0.55 + (0.35 - 0.55) * u
  }
}

function finitePoint(x: unknown, y: unknown, z: unknown): [number, number, number] | null {
  if (typeof x !== 'number' || typeof y !== 'number' || typeof z !== 'number') return null
  if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) return null
  if (z < Z_FLOOR || z > Z_CEIL) return null
  return [x, y, z]
}

function httpCloud(
  positions: Float32Array,
  colors: Float32Array,
  count: number,
  seq: number,
  metadata: CloudFrameMetadata,
): BinaryCloud {
  return {
    positions,
    colors,
    count,
    seq,
    ...metadata,
    connected: true,
    transport: 'http',
    lastFrameAt: Date.now(),
    error: null,
  }
}

function decodeHttpPoints(
  raw: unknown,
  seq: number,
  metadata: CloudFrameMetadata,
): BinaryCloud | null {
  if (!Array.isArray(raw)) return null

  const points: Array<[number, number, number]> = []
  const first = raw[0]
  if (Array.isArray(first)) {
    for (const row of raw) {
      if (!Array.isArray(row)) continue
      const point = finitePoint(row[0], row[1], row[2])
      if (point) points.push(point)
    }
  } else {
    for (let i = 0; i + 2 < raw.length; i += 3) {
      const point = finitePoint(raw[i], raw[i + 1], raw[i + 2])
      if (point) points.push(point)
    }
  }

  if (points.length === 0) {
    return httpCloud(new Float32Array(0), new Float32Array(0), 0, seq, metadata)
  }

  const positions = new Float32Array(points.length * 3)
  const colors = new Float32Array(points.length * 3)
  for (let i = 0; i < points.length; i++) {
    const [x, y, z] = points[i]
    const off = i * 3
    positions[off] = x
    positions[off + 1] = z
    positions[off + 2] = -y
    turboColor((z - COLOR_Z_MIN) / COLOR_Z_SPAN, colors, off)
  }
  return httpCloud(positions, colors, points.length, seq, metadata)
}

export function useBinaryCloud(
  path: string | null = '/ws/cloud',
  fallbackUrl: string | null = '/api/v1/map/points?max_points=60000',
  maxHz?: number,
): BinaryCloud {
  const [cloud, setCloud] = useState<BinaryCloud>(EMPTY)
  const wsRef = useRef<WebSocket | null>(null)
  const workerRef = useRef<Worker | null>(null)
  const reconnectTimer = useRef<ReturnType<typeof setTimeout> | null>(null)
  const fallbackTimer = useRef<ReturnType<typeof setTimeout> | null>(null)
  const noFrameTimer = useRef<ReturnType<typeof setTimeout> | null>(null)

  useEffect(() => {
    let cancelled = false
    if (path == null) {
      setCloud(prev => ({
        ...EMPTY,
        seq: prev.seq + 1,
      }))
      return () => {
        cancelled = true
      }
    }

    const worker = new CloudDecoderWorker()
    workerRef.current = worker
    const endpoint: CloudEndpointKind = path.includes('/scan') ? 'scan' : 'cloud'
    const effectiveMaxHz = maxHz ?? (path.includes('/scan') ? 10 : 2)
    const minFrameMs = effectiveMaxHz > 0 ? 1000 / effectiveMaxHz : 0
    let fallbackSeq = 0
    let httpFallbackActive = false
    let httpRequestGeneration = 0
    let decodeBusy = false
    let pendingWsFrame: {
      frame: ArrayBuffer
      connectionGeneration: number
    } | null = null
    let decodeTimer: ReturnType<typeof setTimeout> | null = null
    let lastDecodeStartedAt = 0
    let connectionGeneration = 0
    let activeConnectionGeneration = 0
    let acceptedCursor: CloudSequenceCursor = { epoch: null, sequence: null }
    let sawDecodedFrame = false

    const resetCloudState = (overrides: {
      connected?: boolean
      transport?: BinaryCloud['transport']
      error?: string | null
    } = {}) => {
      if (cancelled) return
      setCloud(prev => ({
        ...EMPTY,
        connected: overrides.connected ?? prev.connected,
        transport: overrides.transport ?? prev.transport,
        seq: prev.seq + 1,
        error: overrides.error ?? null,
      }))
    }

    const isActiveGeneration = (generation: number): boolean => (
      !cancelled && generation === activeConnectionGeneration
    )

    const clearDecodeTimer = () => {
      if (decodeTimer) {
        clearTimeout(decodeTimer)
        decodeTimer = null
      }
    }

    const flushPendingWsFrame = () => {
      decodeTimer = null
      if (cancelled || decodeBusy || !pendingWsFrame) return
      const now = Date.now()
      const waitMs = Math.max(0, minFrameMs - (now - lastDecodeStartedAt))
      if (waitMs > 1) {
        decodeTimer = setTimeout(flushPendingWsFrame, waitMs)
        return
      }
      const pending = pendingWsFrame
      pendingWsFrame = null
      decodeBusy = true
      lastDecodeStartedAt = now
      worker.postMessage(
        {
          buffer: pending.frame,
          connectionGeneration: pending.connectionGeneration,
        },
        [pending.frame],
      )
    }

    const enqueueWsFrame = (frame: ArrayBuffer, generation: number) => {
      pendingWsFrame = { frame, connectionGeneration: generation }
      flushPendingWsFrame()
    }

    const stopHttpFallback = () => {
      httpFallbackActive = false
      httpRequestGeneration++
      if (fallbackTimer.current) {
        clearTimeout(fallbackTimer.current)
        fallbackTimer.current = null
      }
    }

    async function pollHttpPoints() {
      if (cancelled || !httpFallbackActive || fallbackUrl == null) return
      const pollGeneration = activeConnectionGeneration
      const requestGeneration = httpRequestGeneration
      try {
        const res = await fetch(fallbackUrl, { cache: 'no-store' })
        if (!res.ok) throw new Error(`HTTP ${res.status}`)
        const payload = await res.json() as Record<string, unknown>
        const decoded = decodeHttpPoints(
          payload.points,
          ++fallbackSeq,
          parseHttpCloudMetadata(payload),
        )
        if (
          decoded
          && !cancelled
          && httpFallbackActive
          && pollGeneration === activeConnectionGeneration
          && requestGeneration === httpRequestGeneration
        ) {
          const declaredCount = typeof payload.count === 'number'
            && Number.isInteger(payload.count)
            && payload.count >= 0
            ? payload.count
            : decoded.count
          validateCloudFrameContract(
            decoded,
            'cloud',
            { epoch: null, sequence: null },
          )
          validateCloudFrameContract(
            { ...decoded, count: declaredCount },
            'cloud',
            { epoch: null, sequence: null },
          )
          setCloud(decoded)
        }
      } catch (error) {
        if (
          !cancelled
          && httpFallbackActive
          && pollGeneration === activeConnectionGeneration
          && requestGeneration === httpRequestGeneration
        ) {
          resetCloudState({
            connected: false,
            transport: 'http',
            error: error instanceof Error ? error.message : 'http_cloud_failed',
          })
        }
      } finally {
        if (!cancelled && httpFallbackActive) {
          fallbackTimer.current = setTimeout(pollHttpPoints, 1000)
        }
      }
    }

    function startHttpFallback(reason: string) {
      if (cancelled) return
      if (httpFallbackActive) {
        setCloud(prev => ({
          ...prev,
          connected: false,
          transport: 'http',
          error: reason,
        }))
        return
      }
      if (fallbackUrl == null) {
        resetCloudState({ connected: false, transport: 'none', error: reason })
        return
      }
      httpFallbackActive = true
      httpRequestGeneration++
      setCloud(prev => ({
        ...prev,
        connected: false,
        transport: 'http',
        error: reason,
      }))
      void pollHttpPoints()
    }

    const handleInvalidFrame = (reason: string, generation: number) => {
      if (!isActiveGeneration(generation)) return
      resetCloudState({
        connected: wsRef.current?.readyState === WebSocket.OPEN,
        transport: 'ws',
        error: `cloud_decode_failed: ${reason}`,
      })
      if (fallbackUrl != null) {
        startHttpFallback(`cloud_decode_failed: ${reason}`)
        return
      }
      pendingWsFrame = null
      const ws = wsRef.current
      if (ws && ws.readyState < WebSocket.CLOSING) {
        ws.close(1002, 'invalid point-cloud frame')
      }
    }

    worker.onmessage = (e: MessageEvent) => {
      const m = e.data as ({
        type: 'cloud'
        positions: Float32Array
        colors: Float32Array
        count: number
        seq: number
        protocolVersion: 1 | 2
        frameId: string | null
        epoch: number | null
        stampS: number | null
        sequence: number | null
        streamKind: CloudStreamKind | null
        connectionGeneration: number
      } | {
        type: 'error'
        error: string
        connectionGeneration: number
      })
      decodeBusy = false
      if (cancelled || !isActiveGeneration(m.connectionGeneration)) {
        flushPendingWsFrame()
        return
      }
      if (m.type === 'error') {
        handleInvalidFrame(m.error, m.connectionGeneration)
        flushPendingWsFrame()
        return
      }
      try {
        acceptedCursor = validateCloudFrameContract(m, endpoint, acceptedCursor)
      } catch (error) {
        handleInvalidFrame(
          error instanceof Error ? error.message : 'invalid point-cloud contract',
          m.connectionGeneration,
        )
        flushPendingWsFrame()
        return
      }
      sawDecodedFrame = true
      if (noFrameTimer.current) {
        clearTimeout(noFrameTimer.current)
        noFrameTimer.current = null
      }
      stopHttpFallback()
      setCloud(prev => ({
        positions: m.positions,
        colors: m.colors,
        count: m.count,
        seq: m.seq,
        protocolVersion: m.protocolVersion,
        frameId: m.frameId,
        epoch: m.epoch,
        stampS: m.stampS,
        sequence: m.sequence,
        streamKind: m.streamKind,
        connected: prev.connected,
        transport: 'ws',
        lastFrameAt: Date.now(),
        error: null,
      }))
      flushPendingWsFrame()
    }
    worker.onerror = (event) => {
      decodeBusy = false
      handleInvalidFrame(
        `cloud_decoder_worker_failed: ${event.message || 'unknown worker error'}`,
        activeConnectionGeneration,
      )
      flushPendingWsFrame()
    }

    const clearCloud = () => {
      httpRequestGeneration++
      resetCloudState()
    }
    window.addEventListener('lingtu:cloud-reset', clearCloud)

    function connect() {
      if (cancelled) return
      reconnectTimer.current = null
      const generation = ++connectionGeneration
      activeConnectionGeneration = generation
      acceptedCursor = { epoch: null, sequence: null }
      sawDecodedFrame = false
      pendingWsFrame = null
      resetCloudState({ connected: false, transport: 'none' })
      if (typeof WebSocket !== 'function') {
        startHttpFallback('websocket_unavailable')
        return
      }
      const proto = window.location.protocol === 'https:' ? 'wss:' : 'ws:'
      const ws = new WebSocket(`${proto}//${window.location.host}${path}`)
      ws.binaryType = 'arraybuffer'
      wsRef.current = ws

      ws.onopen = () => {
        if (!isActiveGeneration(generation) || wsRef.current !== ws) return
        resetCloudState({ connected: true, transport: 'ws' })
        if (noFrameTimer.current) clearTimeout(noFrameTimer.current)
        noFrameTimer.current = setTimeout(() => {
          if (isActiveGeneration(generation) && !sawDecodedFrame) {
            startHttpFallback('cloud_ws_no_valid_frame')
          }
        }, 2500)
      }

      ws.onmessage = (e: MessageEvent) => {
        if (!isActiveGeneration(generation) || wsRef.current !== ws) return
        if (!(e.data instanceof ArrayBuffer)) return
        enqueueWsFrame(e.data, generation)
      }

      const reopen = () => {
        if (!isActiveGeneration(generation) || wsRef.current !== ws) return
        activeConnectionGeneration = ++connectionGeneration
        wsRef.current = null
        pendingWsFrame = null
        acceptedCursor = { epoch: null, sequence: null }
        if (noFrameTimer.current) {
          clearTimeout(noFrameTimer.current)
          noFrameTimer.current = null
        }
        resetCloudState({ connected: false, transport: 'none', error: 'cloud_ws_closed' })
        startHttpFallback(sawDecodedFrame ? 'cloud_ws_closed' : 'cloud_ws_unavailable')
        reconnectTimer.current = setTimeout(connect, 3000)
      }
      ws.onclose = reopen
      ws.onerror = () => ws.close()
    }
    connect()

    return () => {
      cancelled = true
      activeConnectionGeneration = ++connectionGeneration
      if (reconnectTimer.current) clearTimeout(reconnectTimer.current)
      clearDecodeTimer()
      stopHttpFallback()
      if (noFrameTimer.current) clearTimeout(noFrameTimer.current)
      if (wsRef.current) {
        wsRef.current.close()
        wsRef.current = null
      }
      window.removeEventListener('lingtu:cloud-reset', clearCloud)
      worker.terminate()
      workerRef.current = null
    }
  }, [path, fallbackUrl, maxHz])

  return cloud
}
