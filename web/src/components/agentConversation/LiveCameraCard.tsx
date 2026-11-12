import { useEffect, useId, useRef, useState } from 'react'
import { Camera, ChevronDown } from 'lucide-react'
import { RecordingLiveCamera } from '../RecordingLiveCamera'
import styles from './LiveCameraCard.module.css'

export function LiveCameraCard() {
  const region = useId()
  const root = useRef<HTMLElement>(null)
  const [open, setOpen] = useState(false)
  const [visible, setVisible] = useState(false)
  const [foreground, setForeground] = useState(() => typeof document === 'undefined' || document.visibilityState === 'visible')
  useEffect(() => {
    const observer = new IntersectionObserver(([entry]) => setVisible(entry.isIntersecting))
    if (root.current) observer.observe(root.current)
    const update = () => setForeground(document.visibilityState === 'visible')
    document.addEventListener('visibilitychange', update)
    return () => { observer.disconnect(); document.removeEventListener('visibilitychange', update) }
  }, [])
  const active = open && visible && foreground
  return <section ref={root} className={styles.card} aria-label="实时相机卡片">
    <button type="button" className={styles.toggle} aria-expanded={open} aria-controls={region} onClick={() => setOpen(value => !value)}>
      <span className={styles.icon}><Camera size={17} /></span><span className={styles.title}>实时相机<small>仅观看</small></span>
      <span className={styles.action}>{open ? '收起' : '展开观看'}</span><ChevronDown size={14} className={styles.chevron} />
    </button>
    <div id={region} className={styles.disclosure} data-open={open} inert={!open} aria-hidden={!open}>
      <div className={styles.body}>
        {active ? <RecordingLiveCamera locale="zh" embedded /> : <div className={styles.placeholder}>{open ? '画面已暂停' : ''}</div>}
      </div>
    </div>
  </section>
}
