import { lazy, Suspense, useState } from 'react'
import { Activity, ChevronDown, Cpu, Route } from 'lucide-react'
import styles from './LiveCameraCard.module.css'
const NumericCard = lazy(() => import('../sensors/NumericCard').then(m => ({ default: m.NumericCard })))
const ImuPanel = lazy(() => import('../ImuMonitor').then(m => ({ default: m.ImuPanel })))
const RobotObservation = lazy(() => import('./RobotObservation'))
export type ObservationView = 'robot' | 'joints' | 'imu' | 'navigation'
const titles = { robot: '机器人状态', joints: '关节与故障', imu: 'IMU 与记录', navigation: '导航任务' }
export function ObservationCard({ view }: { view: ObservationView }) {
  const [open, setOpen] = useState(false)
  const [sensor, setSensor] = useState<'joints' | 'motors'>('joints')
  const Icon = view === 'navigation' ? Route : view === 'joints' ? Cpu : Activity
  return <section className={styles.card} aria-label={titles[view] + '卡片'}>
    <button type="button" className={styles.toggle} aria-expanded={open} onClick={() => setOpen(value => !value)}>
      <span className={styles.icon}><Icon size={17} /></span><span className={styles.title}>{titles[view]}</span>
      <span className={styles.action}>{open ? '收起' : '展开'}</span><ChevronDown size={14} className={styles.chevron} />
    </button>
    <div className={styles.disclosure} data-open={open} inert={!open} aria-hidden={!open}><div className={styles.body}>
      {open && <div className={styles.content}><Suspense fallback={<p role="status">正在加载…</p>}>
        {view === 'joints' ? <><div className={styles.tabs}>{(['joints', 'motors'] as const).map(id => <button type="button" key={id} aria-pressed={sensor === id} onClick={() => setSensor(id)}>{id === 'joints' ? '关节' : '电机与故障'}</button>)}</div><NumericCard key={sensor} sensor={sensor} /></>
          : view === 'imu' ? <ImuPanel /> : <RobotObservation navigation={view === 'navigation'} />}
      </Suspense></div>}
    </div></div>
  </section>
}
