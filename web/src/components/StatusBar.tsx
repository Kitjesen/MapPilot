import { Activity } from 'lucide-react'
import { useEffect, useState } from 'react'
import type { SSEState } from '../types'
import { text, type Locale } from '../i18n'
import { liveNavigationStatus, presentNavigationStatus } from '../services/navigationStatus'
import styles from './StatusBar.module.css'

interface StatusBarProps {
  sseState: SSEState
  uptimeSeconds: number
  locale: Locale
}

function formatUptime(s: number) {
  const h = Math.floor(s / 3600)
  const m = Math.floor((s % 3600) / 60)
  const sec = s % 60
  if (h > 0) return `${h}h${String(m).padStart(2, '0')}m`
  if (m > 0) return `${m}m${String(sec).padStart(2, '0')}s`
  return `${sec}s`
}

function normalizeDisplayZero(value: number, digits: number): number {
  const threshold = 0.5 * 10 ** -digits
  return Math.abs(value) < threshold ? 0 : value
}

function rad2deg(r: number | undefined) {
  if (typeof r !== 'number') return '--'
  const degrees = (r * 180) / Math.PI
  return normalizeDisplayZero(degrees, 1).toFixed(1)
}

function num(v: unknown, digits = 2): string {
  return typeof v === 'number' && isFinite(v)
    ? normalizeDisplayZero(v, digits).toFixed(digits)
    : '--'
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

export function StatusBar({ sseState, uptimeSeconds, locale }: StatusBarProps) {
  const odom = sseState.odometry
  const safety = sseState.safetyState
  const [now, setNow] = useState(() => Date.now())
  const navigation = liveNavigationStatus(sseState, now / 1000)

  useEffect(() => {
    const t = setInterval(() => setNow(Date.now()), 1000)
    return () => clearInterval(t)
  }, [])

  const x = num(odom?.x)
  const y = num(odom?.y)
  const yaw = odom ? rad2deg(odom.yaw) + '°' : '--'
  const vx = num(odom?.vx) + (typeof odom?.vx === 'number' ? ' m/s' : '')
  const navigationView = presentNavigationStatus(navigation, locale)
  const navState = navigationView.task.state
  const estopActive = navigationView.motion.permission.state === 'ESTOPPED' || (safety?.estop ?? false)
  const navStateText = navigationView.task.label
  const goalReady = navigationView.goalAdmission.state === 'ACCEPTING'
  const goalReadinessTitle = goalReady
    ? text(locale, 'Ready to accept navigation goals', '可以接收导航目标')
    : text(locale, 'Navigation is not accepting goals', '导航暂不接收目标')
  const slamDiag = sseState.slamDiag?.data
  const displaySlamHz = numericMetric(slamDiag, 'processed_scan_hz') ?? sseState.slamStatus?.slam_hz
  const lidarHz = numericMetric(slamDiag, 'lidar_input_hz')
  const heartbeatAlive = sseState.lastHeartbeat && now > 0 && now - sseState.lastHeartbeat < 5000
  const statusTitle = [
    `${text(locale, 'Uptime', '运行时长')} ${formatUptime(uptimeSeconds)}`,
    typeof displaySlamHz === 'number' ? `${text(locale, 'Localization', '定位处理')} ${displaySlamHz.toFixed(1)} Hz` : null,
    typeof lidarHz === 'number' ? `${text(locale, 'LiDAR input', '雷达输入')} ${lidarHz.toFixed(1)} Hz` : null,
    sseState.slamStatus ? `${text(locale, 'Degeneracy', '退化')} ${sseState.slamStatus.degeneracy_count}` : null,
    sseState.robotStatus ? `${text(locale, 'Battery', '电量')} ${sseState.robotStatus.battery.toFixed(0)}%` : null,
    `${text(locale, 'Version', '版本')} ${import.meta.env.VITE_APP_VERSION ?? '--'}`,
  ].filter(Boolean).join(' · ')

  return (
    <div className={styles.statusBar} title={statusTitle}>
      <Activity size={13} className={styles.icon} />

      <span className={styles.item}>
        <span className={styles.label}>{text(locale, 'Pose', '位置')}</span>
        <span className={`${styles.value} ${styles.poseValue}`}>({x}, {y})</span>
      </span>

      <span className={styles.sep}>·</span>

      <span className={styles.item}>
        <span className={styles.label}>{text(locale, 'Yaw', '航向')}</span>
        <span className={`${styles.value} ${styles.yawValue}`}>{yaw}</span>
      </span>

      <span className={styles.sep}>·</span>

      <span className={styles.item}>
        <span className={styles.label}>{text(locale, 'Speed', '速度')}</span>
        <span className={`${styles.value} ${styles.speedValue}`}>{vx}</span>
      </span>

      <span className={styles.sep}>·</span>

      <span className={styles.item}>
        <span className={styles.label}>{text(locale, 'Nav', '导航')}</span>
        <span className={`${styles.value} ${navState === 'EXECUTING' ? styles.navActive : navState === 'FAILED' ? styles.navFail : ''}`}>
          {navStateText}
        </span>
      </span>

      <span className={styles.sep}>·</span>

      <span className={styles.item} title={goalReadinessTitle}>
        <span className={styles.label}>{text(locale, 'Goal', '任务')}</span>
        <span className={`${styles.value} ${goalReady ? styles.ready : navigationView.goalAdmission.state === 'UNKNOWN' ? styles.warn : styles.navFail}`}>
          {navigationView.goalAdmission.label}
        </span>
      </span>

      <span className={styles.sep}>·</span>

      <span
        className={styles.item}
        title={navigationView.control.resumeRequired
          ? text(locale, 'Resume is required before autonomy can continue', '自主导航继续前需要恢复任务')
          : navigationView.control.label}
      >
        <span className={styles.label}>{text(locale, 'Control', '控制')}</span>
        <span className={`${styles.value} ${navigationView.control.state === 'AUTONOMY' ? styles.ready : styles.warn}`}>
          {navigationView.control.label}
        </span>
      </span>

      <span className={styles.sep}>·</span>

      <span className={styles.item} title={navigationView.motion.observation.label}>
        <span className={styles.label}>{text(locale, 'Motion', '运动')}</span>
        <span className={`${styles.value} ${navigationView.motion.permission.state === 'CLEAR' ? styles.ready : navigationView.motion.permission.state === 'ESTOPPED' ? styles.navFail : styles.warn}`}>
          {navigationView.motion.permission.label}
        </span>
      </span>

      <span className={styles.sep}>·</span>

      <span className={styles.item} title={navigationView.motion.stopConfirmation.label}>
        <span className={styles.label}>{text(locale, 'Stop', '停稳')}</span>
        <span className={`${styles.value} ${navigationView.motion.stopConfirmation.state === 'CONFIRMED' ? styles.ready : navigationView.motion.stopConfirmation.state === 'FAILED' ? styles.navFail : styles.warn}`}>
          {navigationView.motion.stopConfirmation.label}
        </span>
      </span>

      {estopActive && (
        <>
          <span className={`${styles.item} ${styles.estop}`}>{text(locale, 'E-STOP', '急停')}</span>
          <span className={styles.sep}>·</span>
        </>
      )}

      <span className={styles.right}>
        <span className={heartbeatAlive ? styles.hbDotAlive : styles.hbDot} title={heartbeatAlive ? text(locale, 'Data online', '数据在线') : text(locale, 'Waiting for data', '等待数据')} />
      </span>
    </div>
  )
}
