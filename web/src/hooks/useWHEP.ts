/**
 * useWHEP — pull a WebRTC stream via the WHEP signalling protocol (RFC 9725).
 *
 * Targets the go2rtc sidecar behind the FastAPI gateway's
 * /api/v1/webrtc/whep reverse-proxy endpoint.  Why go2rtc? It handles the
 * media hot path natively in Go and keeps Python out of SRTP/RTP processing.
 *
 * Returns ``stream`` (null until connected), ``connected``, ``error``,
 * and a ``reconnect()`` trigger.  On any failure the consumer should
 * fall through directly to useCamera (JPEG).
 */
import { useCallback, useEffect, useRef, useState } from 'react'

export type WHEPError = null | 'unavailable' | 'signalling' | 'ice'

export interface WHEPState {
  stream:    MediaStream | null
  connected: boolean
  error:     WHEPError
  reconnect: () => void
}

export function useWHEP(
  /** Endpoint of the gateway WHEP proxy. */
  whepPath:  string = '/api/v1/webrtc/whep',
  /** Stream name to request — matches a key under `streams:` in go2rtc.yaml. */
  streamSrc: string = 'cam',
): WHEPState {
  const [stream, setStream]       = useState<MediaStream | null>(null)
  const [connected, setConnected] = useState(false)
  const [error, setError]         = useState<WHEPError>(null)
  const [nonce, setNonce]         = useState(0)
  const pcRef     = useRef<RTCPeerConnection | null>(null)
  const abortRef  = useRef<AbortController | null>(null)
  const mountedRef = useRef(true)

  const teardown = useCallback(() => {
    abortRef.current?.abort()
    abortRef.current = null
    const pc = pcRef.current
    pcRef.current = null
    if (pc) {
      pc.getReceivers().forEach(r => { try { r.track.stop() } catch { /* noop */ } })
      try { pc.close() } catch { /* noop */ }
    }
  }, [])

  const reconnect = useCallback(() => {
    teardown()
    setConnected(false)
    setError(null)
    setStream(null)
    setNonce(n => n + 1)
  }, [teardown])

  useEffect(() => {
    mountedRef.current = true
    const abort = new AbortController()
    abortRef.current = abort

    // Probe first: if go2rtc isn't running, don't even open a
    // PeerConnection — bail fast so the caller can fall through to
    // JPEG without eating a 10s ICE timeout.
    ;(async () => {
      try {
        const status = await fetch('/api/v1/webrtc/go2rtc/status', {
          signal: abort.signal,
        })
        const data = await status.json() as { available?: boolean }
        if (!data.available) {
          setError('unavailable')
          return
        }
      } catch {
        setError('unavailable')
        return
      }

      const pc = new RTCPeerConnection({ iceServers: [] })
      pcRef.current = pc
      pc.addTransceiver('video', { direction: 'recvonly' })

      pc.addEventListener('track', ev => {
        if (!mountedRef.current) return
        setStream(ev.streams[0] ?? new MediaStream([ev.track]))
      })
      pc.addEventListener('connectionstatechange', () => {
        if (!mountedRef.current) return
        const s = pc.connectionState
        if (s === 'connected') setConnected(true)
        if (s === 'failed' || s === 'closed' || s === 'disconnected') {
          setConnected(false)
          if (s === 'failed') setError('ice')
        }
      })

      try {
        const offer = await pc.createOffer()
        await pc.setLocalDescription(offer)
        // Wait for ICE gathering to complete so the POST body carries
        // every candidate — WHEP is single-shot, no trickle channel.
        if (pc.iceGatheringState !== 'complete') {
          await new Promise<void>(resolve => {
            const onState = () => {
              if (pc.iceGatheringState === 'complete') {
                pc.removeEventListener('icegatheringstatechange', onState)
                resolve()
              }
            }
            pc.addEventListener('icegatheringstatechange', onState)
            setTimeout(() => {
              pc.removeEventListener('icegatheringstatechange', onState)
              resolve()
            }, 1500)
          })
        }
        const local = pc.localDescription
        if (!local) throw new Error('no local SDP')

        // WHEP uses raw application/sdp bodies, not JSON.  The gateway
        // proxies the bytes straight through to go2rtc.
        const resp = await fetch(`${whepPath}?src=${encodeURIComponent(streamSrc)}`, {
          method: 'POST',
          headers: { 'content-type': 'application/sdp' },
          body:    local.sdp,
          signal:  abort.signal,
        })
        if (resp.status === 503) {
          setError('unavailable')
          return
        }
        if (!resp.ok) {
          setError('signalling')
          return
        }
        const answerSdp = await resp.text()
        if (!mountedRef.current) return
        await pc.setRemoteDescription({ type: 'answer', sdp: answerSdp })
      } catch (e) {
        if ((e as { name?: string }).name === 'AbortError') return
        setError('signalling')
      }
    })()

    return () => {
      mountedRef.current = false
      teardown()
    }
  }, [whepPath, streamSrc, nonce, teardown])

  return { stream, connected, error, reconnect }
}
