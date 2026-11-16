import { useEffect, useRef, useState, type CSSProperties } from 'react'
import { X } from 'lucide-react'
import { ModalShell } from '../Modal'
import { Bot, type BotState } from './Bot'
import { BOT_SHAPES, BOT_COLORS, type BotProfile } from '../../services/botProfile'
import type { BotShape } from '../../services/botShapes.ts'
import type { ConversationPermission } from '../../services/agentConversation'
import styles from './BotStudio.module.css'

const colors = Object.keys(BOT_COLORS) as BotProfile['color'][]
const colorRows = [colors.slice(0, 8), colors.slice(8)]
// Keep one animation family in the picker so eyes, blink timing, and motion stay consistent.
const shapeOptions: BotShape[] = [
  'grok-blob', 'grok-pebble', 'grok-bean', 'grok-egg', 'grok-squircle', 'grok-tablet', 'grok-capsule',
  'grok-cylinder', 'grok-hex', 'grok-gem', 'grok-crystal', 'grok-wedge', 'grok-shield', 'grok-dome',
  'grok-arch', 'grok-cloud', 'grok-teardrop', 'grok-leaf', 'grok-square', 'grok-sparkle', 'grok-clover',
  'grok-flower', 'grok-heart', 'grok-house', 'grok-star', 'grok-pumpkin',
]
const shapeScales: Partial<Record<BotShape, [number, number]>> = {
  'grok-blob': [1.38, 1.38], 'grok-pebble': [1.38, 1.38], 'grok-bean': [1.38, 1.38], 'grok-egg': [1.38, 1.38],
  'grok-squircle': [1.38, 1.38], 'grok-tablet': [1.34, 1.34], 'grok-capsule': [1.34, 1.44], 'grok-cylinder': [1.38, 1.38],
  'grok-hex': [1.38, 1.38], 'grok-gem': [1.38, 1.38], 'grok-crystal': [1.34, 1.38], 'grok-wedge': [1.38, 1.38],
  'grok-shield': [1.38, 1.38], 'grok-dome': [1.38, 1.38], 'grok-arch': [1.38, 1.38], 'grok-cloud': [1.34, 1.34],
  'grok-teardrop': [1.38, 1.38], 'grok-leaf': [1.38, 1.38], 'grok-square': [1.38, 1.38], 'grok-sparkle': [1.38, 1.38],
  'grok-clover': [1.38, 1.38], 'grok-flower': [1.38, 1.38], 'grok-heart': [1.38, 1.38], 'grok-house': [1.38, 1.38],
  'grok-star': [1.38, 1.38], 'grok-pumpkin': [1.38, 1.38],
}

export function BotStudio({ profile, state, permission, locked, onSave, onClose }: {
  profile: BotProfile; state: BotState; permission: ConversationPermission; locked: boolean; permissionsAvailable?: boolean
  onSave: (profile: BotProfile, permission: ConversationPermission) => boolean; onClose: () => void
}) {
  const [draft, setDraft] = useState({ ...profile })
  const [mode] = useState(permission)
  const [look, setLook] = useState({ x: 0, y: 0 })
  const [greeting, setGreeting] = useState(false)
  const [error, setError] = useState('')
  const timer = useRef<ReturnType<typeof setTimeout> | undefined>(undefined)
  useEffect(() => () => clearTimeout(timer.current), [])
  return <ModalShell open title="我的 Bot" onClose={onClose} className={styles.dialog}
    headerAction={<button className={styles.close} data-modal-cancel onClick={onClose} aria-label="关闭 Bot 面板"><X size={18} /></button>}>
    <form className={styles.layout} onSubmit={event => {
      event.preventDefault()
      if (locked) return
      try { if (onSave(draft, mode)) onClose(); else setError('保存失败，请重试') }
      catch (reason) { setError(reason instanceof Error ? reason.message : '保存失败') }
    }}>
      <section className={styles.preview} aria-label="Bot 预览">
        <button type="button" className={styles.stage} aria-label="和 Bot 打招呼" title="移动鼠标看向你，点击或按回车打招呼"
          onPointerMove={event => {
            if (event.pointerType !== 'mouse') return
            const bounds = event.currentTarget.getBoundingClientRect()
            setLook({ x: (event.clientX - bounds.left) / bounds.width * 2 - 1, y: (event.clientY - bounds.top) / bounds.height * 2 - 1 })
          }} onPointerLeave={() => setLook({ x: 0, y: 0 })}
          onClick={() => { clearTimeout(timer.current); setGreeting(true); timer.current = setTimeout(() => setGreeting(false), 700) }}>
          <Bot state={state} shape={draft.shape} color={draft.color} size={64} animated preview look={look} greeting={greeting} followPointer={false} />
        </button>
      </section>
      <section className={styles.settings} aria-label="Bot 外观设置">
        <label className={styles.nameField}>
          <input aria-label="Bot 名字" placeholder="给 Bot 起个名字" maxLength={32} required value={draft.name} disabled={locked}
            onChange={event => setDraft({ ...draft, name: event.target.value })} />
        </label>
        <fieldset className={styles.shapeField} disabled={locked} aria-label="形象库">
          <div className={styles.sectionHeading}><strong>选择形象</strong></div>
          <div className={styles.shapeLibrary}>
            <div className={styles.shapes}>{shapeOptions.map(shape => <button key={shape} type="button"
              aria-label={`立体形象：${BOT_SHAPES[shape]}`} aria-pressed={draft.shape === shape} data-shape-option={shape}
              title={BOT_SHAPES[shape]} onClick={() => setDraft({ ...draft, shape })}>
              <span className={styles.shapeThumb} style={{ '--shape-scale-x': shapeScales[shape]?.[0] ?? 1, '--shape-scale-y': shapeScales[shape]?.[1] ?? 1 } as CSSProperties}><Bot state="idle" shape={shape} color={draft.color} size={48} animated={false} /></span>
            </button>)}</div>
          </div>
        </fieldset>
        <fieldset disabled={locked} aria-label="颜色">
          <div className={styles.sectionHeading}><strong>颜色</strong></div>
          <div className={styles.colors}>{colorRows.map((row, rowIndex) => <div className={styles.colorRow} key={rowIndex}>{row.map(color => <button key={color} type="button" data-color={color} style={{ backgroundColor: BOT_COLORS[color].fill }} aria-label={BOT_COLORS[color].label} title={BOT_COLORS[color].label} aria-pressed={draft.color === color} onClick={() => setDraft({ ...draft, color })} />)}</div>)}</div>
        </fieldset>
        {error && <p className={styles.error} role="alert">{error}</p>}
        <footer><button type="button" onClick={onClose}>取消</button><button className={styles.save} type="submit" disabled={locked}>保存</button></footer>
        {locked && <span className={styles.note}>本轮结束后可修改</span>}
      </section>
    </form>
  </ModalShell>
}


