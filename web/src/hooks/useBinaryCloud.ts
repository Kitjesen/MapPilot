/**
 * useBinaryCloud — opens /ws/cloud, decodes frames in a WebWorker, and
 * exposes the latest Float32Array buffers for Scene3D to consume.
 *
 * Three properties to keep the main thread quiet:
 *   1. WebSocket binaryType='arraybuffer' so the worker receives the raw
 *      bytes without going through Blob → ArrayBuffer round trips.
 *   2. The worker is a single long-lived instance (via vite worker import)
 *      so we don't pay for spin-up per frame.
 *   3. Worker outputs are Transferable, so the main thread receives the
 *      ArrayBuffer at zero copy cost.
 */
import { useEffect, useRef, useState } from 'react'

import CloudDecoderWorker from '../workers/cloudDecoder.ts?worker'

export interface BinaryCloud {
  positions: Float32Array
  colors: Float32Array
  count: number
  seq: number
  connected: boolean
}

const EMPTY: BinaryCloud = {
  positions: new Float32Array(0),
  colors: new Float32Array(0),
  count: 0,
  seq: 0,
  connected: false,
}

const Z_FLOOR = -0.5
const Z_CEIL = 2.8

function turboColor(t: number, out: Float32Array, off: number) {
  const x = t < 0 ? 0 : t > 1 ? 1 : t
  if (x < 0.5) {
    const u = x * 2
    out[off]     = 0.18 + (0.55 - 0.18) * u
    out[off + 1] = 0.55
    out[off + 2] = 0.50 + (0.55 - 0.50) * u
  } else {
    const u = (x - 0.5) * 2
    out[off]     = 0.55 + (0.78 - 0.55) * u
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

function decodeHttpPoints(raw: unknown, seq: number): BinaryCloud | null {
  if (!Array.isArray(raw) || raw.length === 0) return null

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
    return { positions: new Float32Array(0), colors: new Float32Array(0), count: 0, seq, connected: true }
  }

  let zMin = Infinity
  let zMax = -Infinity
  for (const [, , z] of points) {
    if (z < zMin) zMin = z
    if (z > zMax) zMax = z
  }
  const zSpan = Math.max(zMax - zMin, 0.1)
  const positions = new Float32Array(points.length * 3)
  const colors = new Float32Array(points.length * 3)
  for (let i = 0; i < points.length; i++) {
    const [x, y, z] = points[i]
    const off = i * 3
    positions[off] = x
    positions[off + 1] = z
    positions[off + 2] = -y
    turboColor((z - zMin) / zSpan, colors, off)
  }
  return { positions, colors, count: points.length, seq, connected: true }
}

export function useBinaryCloud(path: string = '/ws/cloud'): BinaryCloud {
  const [cloud, setCloud] = useState<BinaryCloud>(EMPTY)
  const wsRef = useRef<WebSocket | null>(null)
  const workerRef = useRef<Worker | null>(null)
  const reconnectTimer = useRef<ReturnType<typeof setTimeout> | null>(null)
  const fallbackTimer = useRef<ReturnType<typeof setTimeout> | null>(null)
  const mountedRef = useRef(true)

  useEffect(() => {
    mountedRef.current = true
    const worker = new CloudDecoderWorker()
    workerRef.current = worker
    let fallbackSeq = 0

    worker.onmessage = (e: MessageEvent) => {
      if (!mountedRef.current) return
      const m = e.data as {
        type: string
        positions: Float32Array
        colors: Float32Array
        count: number
        seq: number
      }
      if (m.type !== 'cloud') return
      setCloud(prev => ({
        positions: m.positions,
        colors: m.colors,
        count: m.count,
        seq: m.seq,
        connected: prev.connected,
      }))
    }

    const connect = () => {
      if (!mountedRef.current) return
      if (typeof WebSocket !== 'function') {
        const poll = async () => {
          if (!mountedRef.current) return
          try {
            const res = await fetch('/api/v1/map/points?max_points=60000', { cache: 'no-store' })
            if (!res.ok) throw new Error(`HTTP ${res.status}`)
            const payload = await res.json() as { points?: unknown }
            const decoded = decodeHttpPoints(payload.points, ++fallbackSeq)
            if (decoded && mountedRef.current) setCloud(decoded)
          } catch {
            if (mountedRef.current) setCloud(prev => ({ ...prev, connected: false }))
          } finally {
            if (mountedRef.current) fallbackTimer.current = setTimeout(poll, 1000)
          }
        }
        void poll()
        return
      }
      const proto = window.location.protocol === 'https:' ? 'wss:' : 'ws:'
      const ws = new WebSocket(`${proto}//${window.location.host}${path}`)
      ws.binaryType = 'arraybuffer'
      wsRef.current = ws

      ws.onopen = () => {
        if (!mountedRef.current) return
        setCloud(prev => ({ ...prev, connected: true }))
      }

      ws.onmessage = (e: MessageEvent) => {
        if (!mountedRef.current) return
        if (!(e.data instanceof ArrayBuffer)) return
        // Transfer the raw bytes to the worker — main thread keeps no copy.
        worker.postMessage(e.data, [e.data])
      }

      const reopen = () => {
        if (!mountedRef.current) return
        setCloud(prev => ({ ...prev, connected: false }))
        reconnectTimer.current = setTimeout(connect, 3000)
      }
      ws.onclose = reopen
      ws.onerror = () => ws.close()
    }
    connect()

    return () => {
      mountedRef.current = false
      if (reconnectTimer.current) clearTimeout(reconnectTimer.current)
      if (fallbackTimer.current) clearTimeout(fallbackTimer.current)
      if (wsRef.current) {
        wsRef.current.close()
        wsRef.current = null
      }
      worker.terminate()
      workerRef.current = null
    }
  }, [path])

  return cloud
}
