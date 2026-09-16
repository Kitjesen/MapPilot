import { useEffect, useState, useSyncExternalStore } from 'react'
import { GO2_JOINT_SOURCE_MAX_AGE_S, go2JointSampleStatus } from '../services/robotJointState.ts'
import { EMPTY_JOINT_TELEMETRY, type JointTelemetryStream } from '../services/jointTelemetryStream.ts'
import styles from './SceneView.module.css'

const subscribeEmpty = () => () => {}
const emptySnapshot = () => EMPTY_JOINT_TELEMETRY

export function RobotJointStatus({ stream }: { stream?: JointTelemetryStream }) {
  const { sample, connected } = useSyncExternalStore(stream?.subscribe ?? subscribeEmpty,
    stream?.getSnapshot ?? emptySnapshot, emptySnapshot)
  const [nowMs, setNowMs] = useState(() => performance.now())
  useEffect(() => {
    if (!sample) return
    const remainingMs = (GO2_JOINT_SOURCE_MAX_AGE_S - sample.sourceAgeS) * 1000
      - (performance.now() - sample.receivedAtMs)
    const timer = window.setTimeout(() => setNowMs(performance.now()), Math.max(0, remainingMs) + 1)
    return () => window.clearTimeout(timer)
  }, [sample])
  const status = go2JointSampleStatus(sample, Math.max(nowMs, sample?.receivedAtMs ?? 0))
  const label = !sample ? '展示站姿 · 等待数据'
    : !connected || status === 'stale' ? '数据过期 · 保留姿态' : '实测 · 12 个关节'
  return <div className={styles.metricRow} title="Go2 实际关节角，来自只读遥测">
    <span>关节姿态</span><strong>{label}</strong>
  </div>
}
