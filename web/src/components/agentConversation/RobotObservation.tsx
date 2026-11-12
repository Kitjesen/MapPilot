import { useEffect, useState } from 'react'
import { fetchState } from '../../services/api'
import { liveNavigationStatus, presentNavigationStatus, operatorReasonMessage } from '../../services/navigationStatus'
import type { StateResponse } from '../../types'
import styles from './LiveCameraCard.module.css'
export default function RobotObservation({ navigation }: { navigation: boolean }) {
  const [snapshot, setSnapshot] = useState<StateResponse | null>(null)
  const [received, setReceived] = useState(0)
  const [error, setError] = useState('')
  const [now, setNow] = useState(Date.now)
  useEffect(() => { const timer = setInterval(() => setNow(Date.now()), 1000); return () => clearInterval(timer) }, [])
  useEffect(() => {
    let active = true
    let timer: ReturnType<typeof setTimeout>
    async function poll() {
      try {
        const next = await fetchState()
        if (active) { setSnapshot(next); setReceived(Date.now()); setError('') }
      } catch (cause) { if (active) { setSnapshot(null); setError(String(cause)) } }
      finally { if (active) timer = setTimeout(() => void poll(), 2000) }
    }
    void poll()
    return () => { active = false; clearTimeout(timer) }
  }, [])
  const connected = !!snapshot && !error && now - received < 7000
  const status = liveNavigationStatus({ connected, navigationStatus: snapshot?.navigation ?? null, stateSnapshot: snapshot, stateSnapshotReceivedAt: received }, now / 1000)
  const shown = presentNavigationStatus(status, 'zh')
  return <>
    <dl className={styles.facts}>
      {!navigation && <><div><dt>连接</dt><dd>{error ? '断开' : connected ? '已收到状态' : snapshot ? '数据过期' : '等待'}</dd></div><div><dt>运行模式</dt><dd>{snapshot?.session?.product === 'nav' ? '导航' : String(snapshot?.session?.product ?? '未知')}</dd></div></>}
      <div><dt>任务</dt><dd>{shown.task.label}</dd></div>
      <div><dt>运动</dt><dd>{shown.motion.observation.label}</dd></div>
      <div><dt>控制权</dt><dd>{shown.control.label}</dd></div>
      <div><dt>停止确认</dt><dd>{shown.motion.stopConfirmation.label}</dd></div>
    </dl>
    {status?.task.reason && <p role="status">{operatorReasonMessage(status.task.reason, 'zh')}</p>}
    {(shown.motion.message || shown.goalAdmission.message) && <p role="status">{shown.motion.message || shown.goalAdmission.message}</p>}
    {error && <p role="status">读取失败</p>}
    <details><summary>来源与详情</summary><pre className={styles.raw}>{error || JSON.stringify(snapshot, null, 2)}</pre></details>
  </>
}
