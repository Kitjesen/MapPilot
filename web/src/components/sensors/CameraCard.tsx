import { useRef, useState } from 'react'
import { Pause, Play } from 'lucide-react'
import { SensorMark } from './SensorMark.tsx'
import { useCamera } from '../../hooks/useCamera.ts'
import styles from '../SensorMonitor.module.css'

export function CameraCard() {
  const [paused, setPaused] = useState(false)
  const camera = useCamera(paused ? '' : '/ws/camera')
  const [last, setLast] = useState<{ url: string; received: number | null } | null>(null)
  // Freeze a data URL because useCamera releases its blob URL when paused.
  const image = useRef<HTMLImageElement>(null)
  return <>
    <div className={styles.toolbar}><span className={styles.status} data-live={!paused && !!camera.imgSrc}>{paused ? '画面已暂停' : camera.imgSrc ? '已收到画面' : camera.connected ? '等待画面' : '图传未连接'}</span><button type="button" onClick={() => {
      if (!paused && image.current?.naturalWidth) {
        const canvas = document.createElement('canvas')
        canvas.width = image.current.naturalWidth; canvas.height = image.current.naturalHeight
        canvas.getContext('2d')?.drawImage(image.current, 0, 0)
        setLast({ url: canvas.toDataURL('image/jpeg'), received: camera.lastFrameAt })
      }
      setPaused(value => !value)
    }}>{paused ? <Play size={14} /> : <Pause size={14} />}{paused ? '继续' : '暂停'}</button></div>
    <div className={styles.media}>{paused ? last && <img className={styles.camera} src={last.url} alt="暂停的机器人相机画面" /> : camera.imgSrc && <img ref={image} className={styles.camera} src={camera.imgSrc} alt="机器人实时相机" />}{!(paused ? last : camera.imgSrc) && <div className={styles.mediaEmpty}><SensorMark sensor="camera" size={36} /><p>等待相机画面</p><small>连接后会自动显示</small></div>}</div>
    <p className={styles.note}>最后接收：{(paused ? last?.received : camera.lastFrameAt) ? new Date((paused ? last!.received : camera.lastFrameAt)!).toLocaleTimeString() : '—'} · 接收时间不等于拍摄时间</p>
    {camera.error && <p role="status" className={styles.error}>{camera.error}</p>}
  </>
}

