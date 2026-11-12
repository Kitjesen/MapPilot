import { useEffect, useState } from 'react'
import styles from './MapEditor.module.css'

export function MapProgress({ stage, completed }: { stage: string; completed?: string }) {
  const [seconds, setSeconds] = useState(0)
  useEffect(() => {
    const start = Date.now()
    const timer = window.setInterval(() => setSeconds(Math.floor((Date.now() - start) / 1000)), 1000)
    return () => window.clearInterval(timer)
  }, [stage])
  return <div className={styles.progress} role="status">
    <div><strong>{stage}</strong><span>已用 {seconds < 60 ? `${seconds} 秒` : `${Math.floor(seconds / 60)} 分 ${seconds % 60} 秒`}</span></div>
    <progress aria-label={stage} />
    <small>{completed ? `${completed}；` : ''}服务正在处理，暂未提供完成百分比。</small>
  </div>
}
