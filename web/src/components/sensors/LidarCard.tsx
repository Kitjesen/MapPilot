import { lazy, useEffect, useMemo, useState } from 'react'
import { Pause, Play } from 'lucide-react'
import { SensorMark } from './SensorMark.tsx'
import { useBinaryCloud } from '../../hooks/useBinaryCloud.ts'
import { readDisplay } from '../../services/sensors.ts'
import styles from '../SensorMonitor.module.css'
const Scene = lazy(() => import('../MapGalleryScene.tsx').then(module => ({ default: module.MapGalleryScene })))

export function LidarCard() {
  const [now, setNow] = useState(Date.now)
  useEffect(() => { const timer = setInterval(() => setNow(Date.now()), 1000); return () => clearInterval(timer) }, [])
  const [paused, setPaused] = useState(false)
  const [settings] = useState(readDisplay)
  const scan = useBinaryCloud(paused ? null : '/ws/scan', null, 2)
  const [frozen, setFrozen] = useState<typeof scan | null>(null)
  const cloud = paused ? frozen : scan
  const points = useMemo(() => cloud ? Array.from(cloud.positions) : [], [cloud])
  return <>
    <div className={styles.toolbar}><span className={styles.status} data-live={!paused && !!scan.lastFrameAt && now - scan.lastFrameAt < 3000}>{paused ? '画面已暂停' : scan.lastFrameAt && now - scan.lastFrameAt < 3000 ? '已收到扫描' : scan.connected ? '等待新扫描' : '扫描未连接'}</span><button type="button" onClick={() => { if (!paused) setFrozen(scan); setPaused(!paused) }}>{paused ? <Play size={14} /> : <Pause size={14} />}{paused ? '继续' : '暂停'}</button></div>
    {cloud && cloud.count > 0 ? <div className={styles.viewer}><Scene name="实时 LiDAR 扫描" points={points} voxelScale={1} pointSize={settings.pointSize} opacity={1} ceiling={null} grid gridStep={1} /></div> : <div className={styles.media}><div className={styles.mediaEmpty}><SensorMark sensor="lidar" size={36} /><p>等待三维扫描</p><small>连接后显示点云与坐标系</small></div></div>}
    <p className={styles.note}>{cloud?.count ?? '—'} 点 · {cloud?.frameId || '坐标系未报告'} · 来源时间 {cloud?.stampS ?? '—'}</p>
    {scan.error && <p role="status" className={styles.error}>{scan.error}</p>}
    <details className={styles.details}><summary>扫描说明</summary><p className={styles.note}>显示最多 2 帧／秒，不修改 LiDAR 采样。扫描用于观察，碰撞和制动仍由原生导航判断。</p></details>
  </>
}

