import { useEffect, useLayoutEffect, useRef, useState } from 'react'
import { ArrowUp, Check, ChevronDown, Clock3, LoaderCircle, Pencil, X } from 'lucide-react'
import type { ConversationMessage, ConversationTurn } from '../../services/agentConversation'
import styles from './QueuedMessages.module.css'

export function QueuedMessages({ turns, messages, onChange, onRecover, draftEmpty, editing, setEditing }: {
  turns: ConversationTurn[]
  messages: ConversationMessage[]
  onChange: (id: string, text: string | null, promote?: boolean) => Promise<void>
  onRecover: (text: string) => void
  draftEmpty: boolean
  editing: { id: string; text: string } | null
  setEditing: (edit: { id: string; text: string } | null) => void
}) {
  const [expanded, setExpanded] = useState(false)
  const [busy, setBusy] = useState<string | null>(null)
  const [error, setError] = useState('')
  const root = useRef<HTMLElement>(null)
  const positions = useRef(new Map<string, number>())
  const animations = useRef(new Map<string, Animation>())
  useLayoutEffect(() => {
    const previous = positions.current
    const next = new Map<string, number>()
    const reduced = window.matchMedia('(prefers-reduced-motion: reduce)').matches
    for (const row of root.current?.querySelectorAll<HTMLElement>('[data-queue-id]') ?? []) {
      const id = row.dataset.queueId!
      const top = row.offsetTop
      next.set(id, top)
      const oldTop = previous.get(id)
      if (reduced || oldTop === top || previous.size === 0) continue
      const transform = getComputedStyle(row).transform
      const offset = transform === 'none' ? 0 : new DOMMatrixReadOnly(transform).m42
      animations.current.get(id)?.cancel()
      animations.current.set(id, row.animate(oldTop == null
        ? [{ opacity: 0, transform: 'translateY(6px)' }, { opacity: 1, transform: 'translateY(0)' }]
        : [{ transform: `translateY(${oldTop + offset - top}px)` }, { transform: 'translateY(0)' }],
      { duration: 300, easing: 'cubic-bezier(.45,0,.1,1)' }))
    }
    for (const [id, animation] of animations.current) {
      if (!next.has(id) || reduced) { animation.cancel(); animations.current.delete(id) }
    }
    positions.current = next
  })
  useEffect(() => {
    const active = animations.current
    const media = window.matchMedia('(prefers-reduced-motion: reduce)')
    const stop = () => { for (const animation of active.values()) animation.cancel(); active.clear() }
    const changed = () => { if (media.matches) stop() }
    media.addEventListener('change', changed)
    return () => { media.removeEventListener('change', changed); stop() }
  }, [])
  const queued = turns.filter(turn => turn.state === 'queued').sort((a, b) => (a.queue_position ?? 0) - (b.queue_position ?? 0))
  const editLive = queued.some(turn => turn.turn_id === editing?.id)
  if (!queued.length && !editing) return null
  async function change(id: string, text: string | null, promote = false) {
    setBusy(id)
    setError('')
    try {
      await onChange(id, text, promote)
      if (!promote && editing?.id === id) setEditing(null)
    } catch (failure) {
      setError(failure instanceof Error ? failure.message : String(failure))
    } finally { setBusy(null) }
  }
  return <section ref={root} className={styles.queue} aria-label="待执行消息">
    <header><Clock3 size={13} /><span>待执行 {queued.length}</span></header>
    {(expanded ? queued : queued.slice(0, 3)).map((turn, index) => {
      const message = messages.find(item => item.id === turn.message_id)
      return <div className={styles.row} key={turn.turn_id} data-queue-id={turn.turn_id} aria-busy={busy === turn.turn_id}>
        <span title={message?.text}>{message?.text || message?.attachments?.map(item => item.name).join('、') || '附件消息'}</span>
        {index > 0 && <button type="button" className={styles.promote} aria-label="设为下一条" title="设为下一条" disabled={Boolean(busy)}
          onClick={() => void change(turn.turn_id, null, true)}><ArrowUp size={13} /></button>}
        <button type="button" className={styles.edit} aria-label="编辑待执行消息" disabled={Boolean(busy)} onClick={() => {
          setEditing({ id: turn.turn_id, text: message?.text ?? '' }); setError('')
        }}>{busy === turn.turn_id ? <LoaderCircle size={13} className={styles.spinner} /> : <Pencil size={13} />}</button>
        <button type="button" className={styles.cancel} aria-label="取消待执行消息" disabled={Boolean(busy)} onClick={() => void change(turn.turn_id, null)}><X size={14} /></button>
      </div>
    })}
    {queued.length > 3 && <button type="button" className={styles.more} onClick={() => setExpanded(!expanded)}>
      {expanded ? '收起' : `还有 ${queued.length - 3} 条`}<ChevronDown size={12} />
    </button>}
    {editing && <div className={styles.editor}>
      <textarea aria-label="修改待执行消息" value={editing.text} maxLength={8000} rows={2}
        disabled={Boolean(busy)} onChange={event => setEditing({ ...editing, text: event.target.value })} />
      {!editLive && <span role="status">已离开队列，修改尚未发送</span>}
      <div>
        <button type="button" disabled={Boolean(busy)} onClick={() => { setEditing(null); setError('') }}>放弃修改</button>
        {editLive ? <button type="button" disabled={Boolean(busy) || !editing.text.trim()} onClick={() => void change(editing.id, editing.text)}><Check size={13} />保存</button>
          : <button type="button" disabled={!draftEmpty} title={!draftEmpty ? '请先处理输入框中的草稿' : undefined}
            onClick={() => { onRecover(editing.text); setEditing(null); setError('') }}>移到草稿</button>}
      </div>
    </div>}
    {error && <p role="alert">{error}</p>}
  </section>
}
