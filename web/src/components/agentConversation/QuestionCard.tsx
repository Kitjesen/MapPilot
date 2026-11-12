import { useState } from 'react'
import { ArrowUp, Check } from 'lucide-react'
import type { ConversationMessage } from '../../services/agentConversation.ts'
import styles from '../AgentConversation.module.css'

export function QuestionCard({ message, disabled, answer, onAnswer }: {
  message: ConversationMessage; disabled: boolean; answer?: string
  onAnswer(text: string): void
}) {
  const [other, setOther] = useState(false)
  const [text, setText] = useState('')
  return <section className={styles.questionCard} aria-label="选择回复">
    <p>{message.text}</p>
    <div className={styles.questionChoices}>
      {message.metadata?.choices?.map((choice, index) => <button type="button" key={choice}
        disabled={disabled} aria-pressed={answer === choice} onClick={() => onAnswer(choice)}>
        <span className={styles.choiceKey}>{String.fromCharCode(65 + index)}</span>
        <span>{choice}</span>{answer === choice && <Check size={14} />}
      </button>)}
      {!answer && <button type="button" disabled={disabled} aria-expanded={other} onClick={() => setOther(!other)}>
        <span className={styles.choiceKey}>…</span><span>其他，自己说</span>
      </button>}
    </div>
    {other && !answer && <form className={styles.otherReply} onSubmit={event => {
      event.preventDefault()
      if (text.trim() && !disabled) onAnswer(text.trim())
    }}>
      <input autoFocus value={text} maxLength={1000} aria-label="其他回复" placeholder="输入你的想法…"
        disabled={disabled} onChange={event => setText(event.target.value)}
        onKeyDown={event => { if (event.key === 'Enter' && event.nativeEvent.isComposing) event.preventDefault() }} />
      <button type="submit" disabled={disabled || !text.trim()} aria-label="发送其他回复"><ArrowUp size={16} /></button>
    </form>}
  </section>
}
