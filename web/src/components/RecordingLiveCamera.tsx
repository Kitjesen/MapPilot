import { useEffect, useRef, useState } from 'react'
import { Camera, RefreshCw } from 'lucide-react'
import { text, type Locale } from '../i18n'
import type { CameraMediaStatus } from '../types'
import { fetchCameraStatus } from '../services/api'
import { recordingLiveCameraState } from '../services/recordingLiveCamera'
import { useCamera } from '../hooks/useCamera'
import { useWHEP } from '../hooks/useWHEP'
import { useVideoFrame } from '../hooks/useVideoFrame'
import { useRotatedVideoLayout } from '../hooks/useRotatedVideoLayout'
import styles from './RecordingLiveCamera.module.css'

export function RecordingLiveCamera({ locale, embedded = false }: { locale: Locale; embedded?: boolean }) {
  const [status, setStatus] = useState<CameraMediaStatus | null>(null)
  const [statusError, setStatusError] = useState<string | null>(null)
  const [refresh, setRefresh] = useState(0)
  const [now, setNow] = useState<number | null>(null)
  useEffect(() => {
    let current = true
    let timer: ReturnType<typeof setTimeout>
    const poll = async () => {
      try {
        const next = await fetchCameraStatus()
        if (current) { setStatus(next); setStatusError(null) }
      } catch (cause: unknown) {
        if (current) setStatusError(cause instanceof Error ? cause.message : String(cause))
      } finally { if (current) timer = setTimeout(() => void poll(), 3000) }
    }
    void poll()
    return () => { current = false; clearTimeout(timer) }
  }, [refresh])
  useEffect(() => {
    const timer = window.setInterval(() => setNow(Date.now()), 1000)
    return () => window.clearInterval(timer)
  }, [])

  const enabled = status !== null && status.status !== 'not_loaded'
  const whep = useWHEP('/api/v1/webrtc/whep', 'cam', enabled)
  const viewportRef = useRef<HTMLDivElement>(null)
  const videoRef = useRef<HTMLVideoElement>(null)
  const videoFrame = useVideoFrame(videoRef, whep.stream, enabled && whep.connected)
  const videoLayout = useRotatedVideoLayout(viewportRef, videoRef, whep.rotationDegrees, whep.stream !== null, embedded ? 'cover' : 'contain')
  const useWebRTC = whep.connected && videoFrame.hasFrame && videoLayout.ready
  const jpeg = useCamera(enabled && !useWebRTC ? '/ws/camera' : '')
  const jpegFresh = jpeg.connected && jpeg.imgSrc !== null && jpeg.lastFrameAt !== null
    && (now === null || now - jpeg.lastFrameAt <= 3000)
  const lastFrameAt = useWebRTC ? videoFrame.lastFrameAt : jpeg.lastFrameAt
  const state = recordingLiveCameraState({ hasFrame: useWebRTC || jpegFresh, lastFrameAt, now, status, statusError: !!statusError, stalled: videoFrame.stalled })
  const reconnectWhep = whep.reconnect
  useEffect(() => { if (videoFrame.stalled && whep.connected) reconnectWhep() }, [videoFrame.stalled, whep.connected, reconnectWhep])
  const live = state === 'live'
  const labels = {
    live: text(locale, 'Live', '实时'),
    loading: text(locale, 'Waiting for video', '等待画面'),
    not_enabled: text(locale, 'Disconnected', '未连接'),
    waiting: text(locale, 'Waiting for video', '等待画面'),
    paused: text(locale, 'Video interrupted', '画面中断'),
    error: text(locale, 'Connection failed', '连接失败'),
  }
  const reconnect = () => { setRefresh(value => value + 1); if (enabled) { whep.reconnect(); jpeg.reconnect() } }

  return <section className={`${styles.preview} ${embedded ? styles.embedded : ''}`} aria-label={text(locale, 'Live camera video', '实时相机视频')}>
    {(!embedded || !live && state !== 'loading') && <header className={styles.heading}>
      {!embedded && <h2><Camera size={15} strokeWidth={1.5} />{text(locale, 'Video', '视频')}</h2>}
      {!live && state !== 'loading' && <button className={styles.retry} onClick={reconnect} aria-label={text(locale, 'Retry video connection', '重试视频连接')} title={text(locale, 'Retry video connection', '重试视频连接')}><RefreshCw size={15} strokeWidth={1.5} /></button>}
    </header>}
    <div ref={viewportRef} className={styles.viewport} data-state={state} aria-busy={state === 'loading' || state === 'waiting'}>
      {whep.stream && <video ref={videoRef} className={styles.media} style={{ ...videoLayout.style, opacity: live && useWebRTC ? 1 : 0 }} autoPlay muted playsInline aria-label={text(locale, 'Current robot camera video', '机器人当前相机视频')} />}
      {live && !useWebRTC && jpeg.imgSrc && <img className={styles.media} src={jpeg.imgSrc} alt={text(locale, 'Current robot camera frame', '机器人当前相机画面')} />}
      {!live && <div className={styles.empty}><Camera size={20} strokeWidth={1.5} /><span role="status">{labels[state]}</span></div>}
      {live && <span className={styles.live} role="status" data-live title={`${useWebRTC ? 'WebRTC' : 'JPEG'} · ${lastFrameAt !== null ? new Date(lastFrameAt).toLocaleTimeString(locale === 'zh' ? 'zh-CN' : 'en-US') : ''}`}>{labels.live}</span>}
    </div>
    {statusError && !live && <details className={styles.detail}><summary>{text(locale, 'Details', '详情')}</summary><code>{statusError}</code></details>}
  </section>
}
