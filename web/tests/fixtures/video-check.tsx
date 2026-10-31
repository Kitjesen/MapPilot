import { useEffect, useState } from 'react'
import { createRoot } from 'react-dom/client'
import { CameraFeed } from '../../src/components/CameraFeed'
import type { SSEState } from '../../src/types'
import '../../src/index.css'

const peers: RTCPeerConnection[] = []
const OriginalPeer = window.RTCPeerConnection
window.RTCPeerConnection = class extends OriginalPeer {
  constructor(configuration?: RTCConfiguration) {
    super(configuration)
    peers.push(this)
  }
}

const negotiations: {
  attempt: number
  duration_ms: number | null
  status: number | null
  offer_candidates: string[]
  answer_candidates: string[]
  error?: string
}[] = []
const originalFetch = window.fetch.bind(window)
window.fetch = async (input, init) => {
  const url = typeof input === 'string' ? input : input instanceof URL ? input.href : input.url
  const method = init?.method ?? (input instanceof Request ? input.method : 'GET')
  if (method.toUpperCase() !== 'POST' || new URL(url, location.href).pathname !== '/api/v1/webrtc/whep') {
    return originalFetch(input, init)
  }
  const started = performance.now()
  const offer = typeof init?.body === 'string' ? init.body : input instanceof Request ? await input.clone().text() : ''
  const negotiation: typeof negotiations[number] = {
    attempt: negotiations.length + 1,
    duration_ms: null,
    status: null,
    offer_candidates: offer.split(/\r?\n/).filter(line => line.startsWith('a=candidate:')),
    answer_candidates: [],
  }
  negotiations.push(negotiation)
  try {
    const response = await originalFetch(input, init)
    negotiation.duration_ms = Math.round(performance.now() - started)
    negotiation.status = response.status
    if (!response.ok) negotiation.error = `HTTP ${response.status} ${response.statusText}`
    void response.clone().text().then(answer => {
      negotiation.answer_candidates = answer.split(/\r?\n/).filter(line => line.startsWith('a=candidate:'))
    }).catch(error => {
      negotiation.error = String(error)
    })
    return response
  } catch (error) {
    negotiation.duration_ms = Math.round(performance.now() - started)
    negotiation.error = String(error)
    throw error
  }
}

export function Check() {
  const [enabled, setEnabled] = useState(true)
  const [evidence, setEvidence] = useState('Waiting for browser decode')
  useEffect(() => {
    const timer = setInterval(async () => {
      const video = document.querySelector('video')
      const quality = video?.getVideoPlaybackQuality()
      const pc = peers.at(-1)
      const stats = pc && pc.connectionState !== 'closed' ? await pc.getStats() : null
      const inbound: unknown[] = []
      const candidatePairs: unknown[] = []
      const candidates: unknown[] = []
      stats?.forEach(report => {
        if (report.type === 'inbound-rtp') inbound.push({
          frames_decoded: report.framesDecoded, frames_dropped: report.framesDropped,
          fps: report.framesPerSecond, keyframes: report.keyFramesDecoded,
          packets_lost: report.packetsLost,
        })
        if (report.type === 'candidate-pair') candidatePairs.push({
          id: report.id, state: report.state, nominated: report.nominated,
          requests_sent: report.requestsSent, responses_received: report.responsesReceived,
          local_candidate_id: report.localCandidateId, remote_candidate_id: report.remoteCandidateId,
          round_trip_s: report.currentRoundTripTime,
        })
        if (report.type === 'local-candidate' || report.type === 'remote-candidate') candidates.push({
          id: report.id, type: report.type, address: report.address, port: report.port,
          protocol: report.protocol, candidate_type: report.candidateType,
        })
      })
      setEvidence(JSON.stringify({
        source: 'Continuous synthetic color SHM, explicit software x264enc',
        mounted: enabled,
        video_frames_total: quality?.totalVideoFrames ?? 0,
        video_frames_dropped: quality?.droppedVideoFrames ?? 0,
        media_time_s: video?.currentTime ?? 0,
        dimensions: video ? `${video.videoWidth} x ${video.videoHeight}` : null,
        ready_state: video?.readyState ?? null,
        connections_created: peers.length,
        connection: pc?.connectionState,
        ice: pc?.iceConnectionState,
        negotiation_attempts: negotiations.length,
        recent_negotiations: negotiations.slice(-8),
        candidate_pairs: candidatePairs,
        candidates,
        inbound,
      }, null, 2))
    }, 1000)
    return () => clearInterval(timer)
  }, [enabled])
  return <main style={{ maxWidth: 960, margin: '40px auto' }}>
    <h1 style={{ fontSize: 24 }}>Camera → native encoder → WebRTC → browser</h1>
    <p>Local software integration fixture. This does not measure the robot camera or hardware encoder.</p>
    <button onClick={() => setEnabled(value => !value)} style={{ padding: '10px 20px', marginBottom: 16 }}>
      {enabled ? 'Stop video' : 'Start video'}
    </button>
    {enabled && <div style={{ height: 360, background: '#14191c' }}><CameraFeed
      onResetEstop={() => undefined} estop={false} resetBusy={false}
      resetAllowed={false} resetBlockedReason="" sseState={{} as SSEState} locale="zh" /></div>}
    <pre style={{ padding: 20, background: '#fff', borderRadius: 8 }}>{evidence}</pre>
  </main>
}

const fixtureRoot = createRoot(document.getElementById('root')!)
fixtureRoot.render(<Check />)

if (import.meta.hot) {
  import.meta.hot.dispose(() => {
    fixtureRoot.unmount()
    window.fetch = originalFetch
    window.RTCPeerConnection = OriginalPeer
  })
}
