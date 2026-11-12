import { useEffect, useRef, useState } from 'react'
import { createRoot } from 'react-dom/client'
import { Bot, type BotState } from '../src/components/agentConversation/Bot'
import { BotStudio } from '../src/components/agentConversation/BotStudio'
import { BOT_SHAPES, DEFAULT_BOT_PROFILE, type BotProfile } from '../src/services/botProfile'
import { BOT_SHAPE_GROUPS } from '../src/services/botShapes.ts'
import { BOT_EXPRESSIONS, BOT_ACTIVITIES, type BotExpression, type BotActivity } from '../src/components/agentConversation/botPresentation.ts'
import '../src/App.css'
import './bot-motion-preview.css'

const states: { state: BotState; label: string; detail: string }[] = [
  { state: 'idle', label: '待机', detail: '轻呼吸，偶尔看看周围' },
  { state: 'processing', label: '处理中', detail: '稍稍偏头，安静等待结果' },
  { state: 'working', label: '工具运行', detail: '保持专注，轻微观察' },
  { state: 'writing', label: '回复输出', detail: '视线轻移，正在组织文字' },
  { state: 'waiting', label: '等待确认', detail: '抬眼看你，等待你的决定' },
  { state: 'done', label: '完成', detail: '温和的笑脸，不反复庆祝' },
  { state: 'error', label: '失败', detail: '收起笑脸，等待重新尝试' },
  { state: 'unavailable', label: '未连接', detail: '清晰表达当前不可用' },
]

export function Preview() {
  const [selected, setSelected] = useState(states[0])
  const [profile, setProfile] = useState<BotProfile>({ ...DEFAULT_BOT_PROFILE, shape: 'grok-pebble', color: 'blue' })
  const [expression, setExpression] = useState<BotExpression | 'auto'>('auto')
  const [activity, setActivity] = useState<BotActivity | 'auto'>('auto')
  const [look, setLook] = useState({ x: 0, y: 0 })
  const [greeting, setGreeting] = useState(false)
  const [studio, setStudio] = useState(false)
  const timer = useRef<ReturnType<typeof setTimeout> | undefined>(undefined)
  useEffect(() => () => clearTimeout(timer.current), [])
  return <main className="bot-demo">
    <header><span>LINGTU / MOTION STUDY</span><button onClick={() => setStudio(true)}>打开我的 Bot ↗</button></header>
    <section className="bot-demo-hero">
      <div><p className="bot-demo-kicker">完整形象库 / {Object.keys(BOT_SHAPES).length} 个形象</p><h1>让每个形象，<br />都有自己的节拍。</h1><p className="bot-demo-description">立体外形 · 十八形 · 九种表情 · 十二种活动</p>
        <div className="bot-demo-status"><i />{selected.label}<span>{selected.detail}</span></div>
      </div>
      <button className="bot-demo-stage" aria-label="和 Bot 打招呼" onPointerMove={event => {
        if (event.pointerType !== 'mouse') return
        const bounds = event.currentTarget.getBoundingClientRect()
        setLook({ x: (event.clientX - bounds.left) / bounds.width * 2 - 1, y: (event.clientY - bounds.top) / bounds.height * 2 - 1 })
      }} onPointerLeave={() => setLook({ x: 0, y: 0 })} onClick={() => {
        clearTimeout(timer.current)
        setGreeting(true)
        timer.current = setTimeout(() => setGreeting(false), 700)
      }}><Bot state={selected.state} shape={profile.shape} color={profile.color} size={64} preview look={look} greeting={greeting}
        expression={expression === 'auto' ? undefined : expression} activity={activity === 'auto' ? undefined : activity} /><span>{BOT_SHAPES[profile.shape]} · 移动鼠标看向你 · 点击打招呼</span></button>
    </section>
    <nav className="bot-demo-states" aria-label="模拟工作状态">
      {states.map(item => <button key={item.state} aria-pressed={selected.state === item.state} onClick={() => setSelected(item)}>
        <Bot state={item.state} color={profile.color} size={32} animated={false} /><span>{item.label}</span>
      </button>)}
    </nav>
    <div className="bot-demo-presets">
      <label>表情<select aria-label="预览 Bot 表情" value={expression} onChange={event => setExpression(event.target.value as BotExpression | 'auto')}><option value="auto">跟随状态</option>{Object.entries(BOT_EXPRESSIONS).map(([value, label]) => <option key={value} value={value}>{label}</option>)}</select></label>
      <label>活动<select aria-label="预览 Bot 活动" value={activity} onChange={event => setActivity(event.target.value as BotActivity | 'auto')}><option value="auto">跟随状态</option>{Object.entries(BOT_ACTIVITIES).map(([value, label]) => <option key={value} value={value}>{label}</option>)}</select></label>
    </div>
    {BOT_SHAPE_GROUPS.map(group => <section key={group.label} className="bot-demo-library" aria-label={group.label}>
      <h2>{group.label}<span>{group.shapes.length}</span></h2>
      <div>{group.shapes.map(shape => <button key={shape} data-shape-option={shape} aria-label={`${group.label}：${BOT_SHAPES[shape]}`} aria-pressed={profile.shape === shape} onClick={() => setProfile({ ...profile, shape })}>
        <Bot state="idle" shape={shape} color={profile.color} size={64} animated={false} /><span>{BOT_SHAPES[shape]}</span>
      </button>)}</div>
    </section>)}
    <footer>动作验收页 · 状态由按钮模拟，不连接模型或机器人。项目中的 Bot 由实际会话事件驱动。</footer>
    {studio && <BotStudio profile={profile} state={selected.state} permission="default" permissionsAvailable locked={false} onClose={() => setStudio(false)} onSave={next => { setProfile(next); return true }} />}
  </main>
}

createRoot(document.getElementById('root')!).render(<Preview />)
