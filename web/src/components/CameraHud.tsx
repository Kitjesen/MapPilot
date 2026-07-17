import { useEffect, useState } from 'react'
import type { SSEState } from '../types'
import styles from './CameraHud.module.css'

interface CameraHudProps {
  sseState: SSEState
}

function fmt(v: number | undefined, dec: number, unit: string, fallback = '--') {
  return typeof v === 'number' ? `${v.toFixed(dec)} ${unit}` : fallback
}

function numericMetric(data: Record<string, unknown> | undefined, key: string): number | undefined {
  const value = data?.[key]
  if (typeof value === 'number' && Number.isFinite(value)) return value
  if (typeof value === 'string') {
    const parsed = Number(value)
    return Number.isFinite(parsed) ? parsed : undefined
  }
  return undefined
}

export function CameraHud({ sseState }: CameraHudProps) {
  const slam = sseState.slamStatus
  const slamDiag = sseState.slamDiag?.data
  const robot = sseState.robotStatus
  const odom = sseState.odometry
  const [now, setNow] = useState(0)

  useEffect(() => {
    const t = setInterval(() => setNow(Date.now()), 1000)
    return () => clearInterval(t)
  }, [])

  // Latency from last heartbeat
  const latencyMs = sseState.lastHeartbeat && now > 0
    ? Math.max(0, now - sseState.lastHeartbeat)
    : null

  const displayScanHz = numericMetric(slamDiag, 'processed_scan_hz') ?? slam?.slam_hz
  const scanHz = typeof displayScanHz === 'number' ? `${displayScanHz.toFixed(0)} Hz` : '--'
  const battery = typeof robot?.battery === 'number'
    ? `${robot.battery.toFixed(0)}%`
    : '--'
  const temp = fmt(robot?.temperature, 1, '°C')
  const latency = latencyMs != null ? `${latencyMs} ms` : '--'
  const speed = fmt(odom?.vx, 2, 'm/s')

  const batteryLow = typeof robot?.battery === 'number' && robot.battery < 20
  const tempHigh = typeof robot?.temperature === 'number' && robot.temperature > 60

  return (
    <div className={styles.hud}>
      <div className={styles.row}>
        <span className={styles.key}>定位</span>
        <span className={styles.val}>{scanHz}</span>
      </div>
      <div className={styles.row}>
        <span className={styles.key}>速度</span>
        <span className={styles.val}>{speed}</span>
      </div>
      <div className={styles.row}>
        <span className={styles.key}>电量</span>
        <span className={batteryLow ? styles.valWarn : styles.val}>{battery}</span>
      </div>
      <div className={styles.row}>
        <span className={styles.key}>温度</span>
        <span className={tempHigh ? styles.valWarn : styles.val}>{temp}</span>
      </div>
      <div className={styles.row}>
        <span className={styles.key}>延迟</span>
        <span className={styles.val}>{latency}</span>
      </div>
    </div>
  )
}
