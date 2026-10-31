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

export function RecordingLiveCamera({ locale }: { locale: Locale }) {
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
  const videoLayout = useRotatedVideoLayout(viewportRef, videoRef, whep.rotationDegrees, whep.stream !== null)
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
    live: text(locale, 'Receiving live video', '正在接收实时视频'),
    loading: text(locale, 'Checking the current camera source…', '正在检查当前相机源…'),
    not_enabled: text(locale, 'This service has no camera connected', '当前服务未接入相机'),
    waiting: text(locale, 'Waiting for live camera frames', '等待实时相机帧'),
    paused: text(locale, 'Live frames have stopped updating', '实时画面已停止更新'),
    error: text(locale, 'Live camera is unavailable', '实时相机暂不可用'),
  }
  const reconnect = () => { setRefresh(value => value + 1); if (enabled) { whep.reconnect(); jpeg.reconnect() } }

  return <section className={styles.preview} aria-label={text(locale, 'Live camera video', '实时相机视频')}>
    <header className={styles.heading}><strong><Camera size={14} />{text(locale, 'Live video', '实时视频')}</strong><span data-live={live}>{live ? text(locale, 'Live source', '当前视频源') : text(locale, 'No live image received', '尚未收到实时画面')}</span></header>
    <div ref={viewportRef} className={styles.viewport} data-state={state} aria-busy={state === 'loading' || state === 'waiting'}>
      {whep.stream && <video ref={videoRef} className={styles.media} style={{ ...videoLayout.style, opacity: live && useWebRTC ? 1 : 0 }} autoPlay muted playsInline aria-label={text(locale, 'Current robot camera video', '机器人当前相机视频')} />}
      {live && !useWebRTC && jpeg.imgSrc && <img className={styles.media} src={jpeg.imgSrc} alt={text(locale, 'Current robot camera frame', '机器人当前相机画面')} />}
      {!live && <div className={styles.empty}><Camera size={21} strokeWidth={1.3} /><span role="status">{labels[state]}</span>{state !== 'loading' && <button onClick={reconnect}><RefreshCw size={12} />{text(locale, 'Check again', '重新检查')}</button>}</div>}
    </div>
    <footer className={styles.footer}><span>{live && lastFrameAt !== null ? `${text(locale, 'Browser last received', '浏览器最近收帧')} ${new Date(lastFrameAt).toLocaleTimeString(locale === 'zh' ? 'zh-CN' : 'en-US')}` : text(locale, 'Connects only to the current camera stream.', '此处仅连接当前相机视频流。')}</span>{live && <span>{useWebRTC ? 'WebRTC' : 'JPEG'}</span>}</footer>
    <p className={styles.note}>{text(locale, 'Use History for saved recordings. Opening this view does not start recording or robot motion.', '保存的视频请到“历史回看”查看。打开此页不会开始录制或机器人运动。')}</p>
    {statusError && !live && <details className={styles.detail}><summary>{text(locale, 'Connection information', '连接信息')}</summary><code>{statusError}</code></details>}
  </section>
}
