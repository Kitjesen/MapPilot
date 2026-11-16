import { useState } from 'react'
import { AlertTriangle, CheckCircle2 } from 'lucide-react'
import styles from './DecisionInbox.module.css'

export interface DecisionInboxItem {
  id: string
  title: string
  detail?: string
  primary: string
  secondary: string
}

export function DecisionInbox({ decisions, onDecide }: {
  decisions: DecisionInboxItem[]
  onDecide?: (decision: DecisionInboxItem, answer: 'primary' | 'secondary') => void
}) {
  const [state, setState] = useState<Record<string, { open: boolean; answered?: string; gone?: boolean }>>(
    () => Object.fromEntries(decisions.map(decision => [decision.id, { open: true }]))
  )
  const open = decisions.filter(decision => !state[decision.id]?.answered && !state[decision.id]?.gone).length
  const decide = (decision: DecisionInboxItem, answer: 'primary' | 'secondary') => {
    setState(current => ({ ...current, [decision.id]: { ...current[decision.id], answered: answer === 'primary' ? decision.primary : decision.secondary } }))
    onDecide?.(decision, answer)
    window.setTimeout(() => setState(current => ({ ...current, [decision.id]: { ...current[decision.id], gone: true } })), 560)
  }
  return <section className={styles.inbox} data-clear={open === 0 || undefined} aria-label="决策收件箱">
    <header className={styles.header} aria-live="polite">
      {open > 0 ? <><AlertTriangle size={16} aria-hidden="true" /><span>{open} 项决策等待确认</span></>
        : <><CheckCircle2 size={16} aria-hidden="true" /><span>决策已处理完</span></>}
    </header>
    <div className={styles.card}>
      {decisions.map(decision => {
        const current = state[decision.id] ?? { open: true }
        return <div key={decision.id} className={styles.row} data-gone={current.gone || undefined}>
          <div className={styles.rowInner}>{current.open ? <div className={styles.item}>
            <p className={styles.title}>{decision.title}</p>
            {decision.detail && <p className={styles.detail}>{decision.detail}</p>}
            <div className={styles.actions}>{current.answered
              ? <span className={styles.done} role="status"><CheckCircle2 size={14} aria-hidden="true" />{current.answered}</span>
              : <><button type="button" className={styles.primary} onClick={() => decide(decision, 'primary')}>{decision.primary}</button>
                <button type="button" className={styles.secondary} onClick={() => decide(decision, 'secondary')}>{decision.secondary}</button></>}</div>
          </div> : <button type="button" className={styles.folded} onClick={() => setState(currentState => ({ ...currentState, [decision.id]: { ...currentState[decision.id], open: true } }))}>{decision.title}</button>}</div>
        </div>
      })}
      {open === 0 && <p className={styles.empty}>暂时没有等待你处理的决策。</p>}
    </div>
  </section>
}
