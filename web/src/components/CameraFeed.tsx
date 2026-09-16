import { useEffect, useRef } from 'react'
import { Camera, ChevronDown, RefreshCw, LockOpen } from 'lucide-react'
import { useCamera } from '../hooks/useCamera'
import { useWHEP } from '../hooks/useWHEP'
import { CameraHud } from './CameraHud'
import type { SSEState } from '../types'
import { text, type Locale } from '../i18n'
import styles from './CameraFeed.module.css'

interface CameraFeedProps {
  onResetEstop: () => void
  estop:    boolean
  resetBusy: boolean
  resetAllowed: boolean
  resetBlockedReason: string
  sseState: SSEState
  locale: Locale
}

type Source = 'whep' | 'jpeg'
type CameraHealth = 'live' | 'connecting' | 'offline'

export function CameraFeed({
  onResetEstop,
  estop,
  resetBusy,
  resetAllowed,
  resetBlockedReason,
  sseState,
  locale,
}: CameraFeedProps) {
  // Show the native JPEG stream while WHEP negotiates. An absent optional
  // go2rtc service must not delay an already available camera by eight seconds.
  const whep = useWHEP()
  const jpeg = useCamera(whep.connected ? '' : '/ws/camera')
  const source: Source = whep.connected ? 'whep' : 'jpeg'

  const videoRef = useRef<HTMLVideoElement>(null)
  const liveStream = source === 'whep' ? whep.stream : null
  useEffect(() => {
    if (!videoRef.current) return
    if (liveStream && videoRef.current.srcObject !== liveStream) {
      videoRef.current.srcObject = liveStream
    }
  }, [liveStream])

  const hasVideo =
    source === 'whep' ? whep.connected :
    jpeg.imgSrc != null
  const isConnected =
    source === 'whep' ? whep.connected :
    jpeg.connected
  const onReconnect =
    source === 'whep' ? whep.reconnect :
    jpeg.reconnect

  const cameraHealth: CameraHealth = hasVideo
    ? 'live'
    : (isConnected ? 'connecting' : 'offline')
  const cameraHealthClass =
    cameraHealth === 'live' ? styles.camBadgeLive :
    cameraHealth === 'connecting' ? styles.camBadgeConnecting :
    styles.camBadgeOff
  const cameraHealthLabel =
    cameraHealth === 'live' ? '实时画面' :
    cameraHealth === 'connecting' ? '图传恢复中' :
    '图传离线'

  let sourceLabel: string
  if (source === 'jpeg') {
    sourceLabel = jpeg.imgSrc ? '相机图传 · 已连接' : '相机图传 · 等待画面'
  } else {
    sourceLabel = whep.connected ? '图传 · Go2RTC · H.264' : '图传 · Go2RTC (建立中…)'
  }

  return (
    <div className={styles.cameraFeed}>
      <div className={styles.viewport}>
        {source === 'whep' ? (
          <video
            ref={videoRef}
            className={`${styles.img} ${styles.imgWhep}`}
            autoPlay
            muted
            playsInline
          />
        ) : jpeg.imgSrc ? (
          <img className={styles.img} alt="机器人相机画面" src={jpeg.imgSrc} />
        ) : null}

        {!hasVideo && (
          <div className={styles.placeholder}>
            <Camera size={48} strokeWidth={1} className={styles.placeholderIcon} />
            <span className={styles.placeholderLabel}>
              {isConnected ? '等待图传画面帧…' : '无相机画面'}
            </span>
            {!isConnected && (
              <button className={styles.reconnectBtn} onClick={onReconnect}>
                <RefreshCw size={14} />
                重新连接
              </button>
            )}
          </div>
        )}

        <div className={cameraHealthClass}>
          <span className={styles.camBadgeDot} />
          {cameraHealthLabel}
        </div>

        <CameraHud sseState={sseState} />

        {estop && <div className={styles.estopOverlay}>急停激活</div>}
      </div>

      <div className={styles.controls}>
        {estop && (
          <button
            className={styles.btnReset}
            onClick={onResetEstop}
            disabled={resetBusy || !resetAllowed}
            title={resetAllowed
              ? text(locale, 'Release the software E-stop latch', '解除软件急停锁')
              : resetBlockedReason}
            aria-label={text(locale, 'Reset emergency stop', '解除急停')}
          >
            <LockOpen size={17} />
            {resetBusy
              ? text(locale, 'Confirming…', '确认解除中…')
              : text(locale, 'Reset E-stop', '解除急停')}
          </button>
        )}
        {estop && (
          <span className={styles.resetNotice}>
            {text(
              locale,
              'Reset keeps the robot stopped; the old task will not resume automatically.',
              '解除后机器人仍保持停止，旧任务不会自动恢复。',
            )}
          </span>
        )}
        <details className={styles.streamInfo}>
          <summary>{text(locale, 'Camera information', '相机信息')}<ChevronDown size={13} /></summary>
          <p>{sourceLabel}</p>
        </details>
      </div>
    </div>
  )
}
