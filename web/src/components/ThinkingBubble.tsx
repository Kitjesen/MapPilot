import { useEffect, useState, useRef } from 'react'
import styles from './ThinkingBubble.module.css'

// Braille dots spinner — classic Claude Code / ora library style
const SPINNER = ['⠋', '⠙', '⠹', '⠸', '⠼', '⠴', '⠦', '⠧', '⠇', '⠏']

interface Props {
  hint?: string          // optional override text (e.g. current phase)
  startedAt?: number     // ms timestamp to show elapsed seconds
}

export function ThinkingBubble({ hint, startedAt }: Props) {
  const [frame, setFrame] = useState(0)
  const [elapsed, setElapsed] = useState(0)
  const startRef = useRef(startedAt ?? 0)

  useEffect(() => {
    if (startRef.current <= 0) startRef.current = Date.now()
  }, [])

  // Fast spinner (80ms) — matches Claude Code cadence
  useEffect(() => {
    const t = setInterval(() => setFrame(f => (f + 1) % SPINNER.length), 80)
    return () => clearInterval(t)
  }, [])

  // Elapsed counter (1s tick)
  useEffect(() => {
    const t = setInterval(() => setElapsed(Math.floor((Date.now() - startRef.current) / 1000)), 1000)
    return () => clearInterval(t)
  }, [])

  const label = hint || '等待任务反馈'

  return (
    <div className={styles.thinking} aria-live="polite">
      <span className={styles.spinner}>{SPINNER[frame]}</span>
      <span className={styles.label}>{label}…</span>
      {elapsed > 0 && <span className={styles.elapsed}>{elapsed}s</span>}
    </div>
  )
}
