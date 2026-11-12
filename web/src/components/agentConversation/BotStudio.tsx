import { useEffect, useRef, useState } from 'react'
import { RotateCcw, X } from 'lucide-react'
import { ModalShell } from '../Modal'
import { Bot, type BotState } from './Bot'
import { BOT_SHAPES, BOT_COLORS, DEFAULT_BOT_PROFILE, type BotProfile } from '../../services/botProfile'
import { BOT_SHAPE_GROUPS } from '../../services/botShapes.ts'
import { BOT_ACTIVITIES, BOT_EXPRESSIONS, type BotActivity, type BotExpression } from './botPresentation.ts'
import type { ConversationPermission } from '../../services/agentConversation'
import styles from './BotStudio.module.css'

const colors = Object.keys(BOT_COLORS) as BotProfile['color'][]
const motionStates: Record<BotState, string> = { idle: '待机', processing: '处理中', working: '工具运行', writing: '回复输出', waiting: '等待确认', done: '完成', error: '失败', unavailable: '未连接' }

export function BotStudio({ profile, state, permission, locked, permissionsAvailable, onSave, onClose }: {
  profile: BotProfile; state: BotState; permission: ConversationPermission; locked: boolean; permissionsAvailable: boolean
  onSave: (profile: BotProfile, permission: ConversationPermission) => boolean; onClose: () => void
}) {
  const [draft, setDraft] = useState({ ...profile })
  const [mode, setMode] = useState(permission)
  const [tab, setTab] = useState<'appearance' | 'role'>('appearance')
  const [look, setLook] = useState({ x: 0, y: 0 })
  const [greeting, setGreeting] = useState(false)
  const [previewState, setPreviewState] = useState<BotState | 'live'>('live')
  const [expression, setExpression] = useState<BotExpression | 'auto'>('auto')
  const [activity, setActivity] = useState<BotActivity | 'auto'>('auto')
  const [error, setError] = useState('')
  const timer = useRef<ReturnType<typeof setTimeout> | undefined>(undefined)
  useEffect(() => () => clearTimeout(timer.current), [])
  return <ModalShell open title="我的 Bot" onClose={onClose} className={styles.dialog}
    headerAction={<button className={styles.close} data-modal-cancel onClick={onClose} aria-label="关闭 Bot 面板"><X size={18} /></button>}>
    <div className={styles.layout}>
      <section className={styles.preview} aria-label="互动角色">
        <button className={styles.stage} aria-label="和 Bot 打招呼" title="移动鼠标看向你，点击或按回车打招呼"
          onPointerMove={event => {
            if (event.pointerType !== 'mouse') return
            const bounds = event.currentTarget.getBoundingClientRect()
            setLook({ x: (event.clientX - bounds.left) / bounds.width * 2 - 1, y: (event.clientY - bounds.top) / bounds.height * 2 - 1 })
          }} onPointerLeave={() => setLook({ x: 0, y: 0 })}
          onClick={() => { clearTimeout(timer.current); setGreeting(true); timer.current = setTimeout(() => setGreeting(false), 700) }}>
          <Bot state={previewState === 'live' ? state : previewState} shape={draft.shape} color={draft.color} size={draft.size}
            preview look={look} greeting={greeting} expression={expression === 'auto' ? undefined : expression} activity={activity === 'auto' ? undefined : activity} />
        </button>
        <label className={styles.motionPicker}>动作预览
          <select aria-label="预览 Bot 动作，不改变工作状态" value={previewState} onChange={event => setPreviewState(event.target.value as BotState | 'live')}>
            <option value="live">跟随实时状态</option>
            {(Object.keys(motionStates) as BotState[]).map(value => <option key={value} value={value}>{motionStates[value]}</option>)}
          </select>
        </label>
        <label className={styles.motionPicker}>表情
          <select aria-label="预览 Bot 表情" value={expression} onChange={event => setExpression(event.target.value as BotExpression | 'auto')}>
            <option value="auto">跟随状态</option>
            {(Object.keys(BOT_EXPRESSIONS) as BotExpression[]).map(value => <option key={value} value={value}>{BOT_EXPRESSIONS[value]}</option>)}
          </select>
        </label>
        <label className={styles.motionPicker}>活动
          <select aria-label="预览 Bot 活动" value={activity} onChange={event => setActivity(event.target.value as BotActivity | 'auto')}>
            <option value="auto">跟随状态</option>
            {(Object.keys(BOT_ACTIVITIES) as BotActivity[]).map(value => <option key={value} value={value}>{BOT_ACTIVITIES[value]}</option>)}
          </select>
        </label>
        <span className={styles.previewNote}>2× 预览 · 不改变工作状态</span>
      </section>
      <form className={styles.form} onSubmit={event => {
        event.preventDefault()
        if (locked) return
        try { if (onSave(draft, mode)) onClose(); else setError('保存失败，请重试') }
        catch (reason) { setError(reason instanceof Error ? reason.message : '保存失败') }
      }}>
        <div className={styles.tabs} role="tablist" aria-label="Bot 配置">
          <button type="button" role="tab" aria-selected={tab === 'appearance'} onClick={() => setTab('appearance')}>形象</button>
          <button type="button" role="tab" aria-selected={tab === 'role'} onClick={() => setTab('role')}>角色与权限</button>
        </div>
        <label className={styles.nameField}><span>名字</span><input aria-label="Bot 名字" placeholder="给 Bot 起个名字" maxLength={32} required value={draft.name} disabled={locked} onChange={e => setDraft({ ...draft, name: e.target.value })} /></label>
        {tab === 'appearance' ? <>
          <fieldset className={styles.shapeField} disabled={locked} aria-label="形象库">
            <div className={styles.shapeLibrary}>
              {BOT_SHAPE_GROUPS.map(group => <section key={group.label} aria-label={group.label}>
                <h3>{group.label}<span>{group.shapes.length}</span></h3>
                <div className={styles.shapes}>{group.shapes.map(shape => <button key={shape} type="button"
                  aria-label={`${group.label}：${BOT_SHAPES[shape]}`} aria-pressed={draft.shape === shape} data-shape-option={shape}
                  title={BOT_SHAPES[shape]} onClick={() => setDraft({ ...draft, shape })}>
                  <Bot state="idle" shape={shape} color={draft.color} size={32} animated={false} /><span>{BOT_SHAPES[shape]}</span>
                </button>)}</div>
              </section>)}
            </div>
          </fieldset>
          <fieldset disabled={locked} aria-label="显示大小"><span className={styles.optionLabel}>大小</span><div className={styles.sizes}>
            {([48, 64] as const).map(size => <button key={size} type="button" aria-pressed={draft.size === size}
              title={size === 48 ? '标准' : '放大'} onClick={() => setDraft({ ...draft, size })}>{size} px</button>)}
          </div></fieldset>
          <fieldset disabled={locked} aria-label="颜色"><span className={styles.optionLabel}>色调</span><div className={styles.colors}>
            {colors.map(color => <button key={color} type="button" data-color={color} style={{ backgroundColor: BOT_COLORS[color].fill }}
              aria-label={BOT_COLORS[color].label} title={BOT_COLORS[color].label} aria-pressed={draft.color === color} onClick={() => setDraft({ ...draft, color })} />)}
          </div></fieldset>
        </> : <>
          <label>角色说明<textarea aria-label="Bot 角色说明" maxLength={1600} rows={4} disabled={locked}
            placeholder="例如：巡检助手，中文简洁回复，先列出发现的问题。" value={draft.instructions} onChange={e => setDraft({ ...draft, instructions: e.target.value })} /></label>
          <label>工具权限<select aria-label="Bot 工具权限" disabled={locked || !permissionsAvailable} value={mode} onChange={e => setMode(e.target.value as ConversationPermission)}>
            <option value="read_only">只读查询</option><option value="default">写入与运动前确认</option><option value="full_access">允许直接执行</option>
          </select></label>
          <p className={styles.note}>{mode === 'full_access' ? '仍须通过机器人端运动准入。' : mode === 'read_only' ? '查询信息，不发起运动或持久写入。' : '涉及地图写入或启动运动时请求确认。'}</p>
        </>}
        {error && <p className={styles.error} role="alert">{error}</p>}
        <footer><button className={styles.reset} type="button" disabled={locked} onClick={() => { setDraft({ ...DEFAULT_BOT_PROFILE }); setMode('default') }}><RotateCcw size={13} />重置</button>
          <button type="button" onClick={onClose}>取消</button>
          <button className={styles.save} type="submit" disabled={locked}>保存</button>
        </footer>
        <span className={styles.note}>{locked ? '本轮结束后可修改'
          : tab === 'appearance' ? '此浏览器保存 · 外观立即更新' : '此浏览器保存 · 新消息生效'}</span>
      </form>
    </div>
  </ModalShell>
}
