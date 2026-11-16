import { useState } from 'react'
import { CalendarDays, BrainCircuit, PlugZap, Search, ShieldCheck } from 'lucide-react'
import { createRoot } from 'react-dom/client'
import { Bot, type BotState } from '../src/components/agentConversation/Bot.tsx'
import { BOT_ACTIVITIES, type BotActivity } from '../src/components/agentConversation/botPresentation.ts'
import { ComputerProgressCard, type ComputerProgressState } from '../src/components/agentConversation/ComputerProgressCard.tsx'
import { FullscreenViewer, type FullscreenTable } from '../src/components/agentConversation/FullscreenViewer.tsx'
import { BOT_SHAPES, DEFAULT_BOT_PROFILE, type BotProfile } from '../src/services/botProfile.ts'
import './grok-product-preview.css'

type ViewerKind = 'image' | 'video' | 'table' | 'map'

const tools: Array<{ key: BotActivity; label: string; detail: string }> = [
  { key: 'searching', label: '检索', detail: 'web_search / MCP search' },
  { key: 'reading', label: '阅读', detail: '网页、相机、传感器' },
  { key: 'coding', label: '编程', detail: 'Python / 点云分析' },
  { key: 'controlling', label: '控制', detail: '导航与目标操作' },
  { key: 'uploading', label: '上传', detail: '附件与上下文' },
  { key: 'writing', label: '书写', detail: '流式生成正文' },
]

const table: FullscreenTable = { columns: ['状态', '来源', '更新时间'], rows: [['已授权', 'MCP', '刚刚'], ['可搜索', 'Web', '12 秒前'], ['等待确认', 'Computer', '现在']] }

export function Preview() {
  const [profile] = useState<BotProfile>({ ...DEFAULT_BOT_PROFILE, shape: 'grok-blob', color: 'blue', size: 64 })
  const [state, setState] = useState<BotState>('working')
  const [activity, setActivity] = useState<BotActivity>('searching')
  const [progressState, setProgressState] = useState<ComputerProgressState>('awaiting_permission')
  const [viewer, setViewer] = useState<ViewerKind | null>(null)
  return <main className="grok-product">
    <header className="grok-product-header"><div><span className="eyebrow">LINGTU / GROK PRODUCT STUDY</span><h1>让任务状态变得可见。</h1><p>形象、工具进度、权限、连接器和查看器，沿用 LingTu 现有真实契约。</p></div><div className="demo-badge">演示数据 · 不连接外部账号</div></header>
    <section className="grok-product-hero">
      <div className="bot-stage"><Bot state={state} shape={profile.shape} color={profile.color} size={profile.size} activity={activity} /><span>{BOT_SHAPES[profile.shape]} · {BOT_ACTIVITIES[activity]}</span></div>
      <div className="hero-copy"><span className="section-kicker">当前活动</span><h2>{tools.find(tool => tool.key === activity)?.label}</h2><p>{tools.find(tool => tool.key === activity)?.detail}</p><div className="state-pills">{(['processing', 'working', 'writing', 'waiting', 'done', 'error'] as BotState[]).map(value => <button key={value} aria-pressed={state === value} onClick={() => setState(value)}>{value}</button>)}</div></div>
    </section>
    <section className="activity-grid" aria-label="工具活动">
      {tools.map(tool => <button key={tool.key} className={activity === tool.key ? 'activity-card active' : 'activity-card'} onClick={() => setActivity(tool.key)}><Bot state="idle" shape={profile.shape} color={profile.color} size={36} activity={tool.key} animated={false} /><strong>{tool.label}</strong><small>{tool.detail}</small></button>)}
    </section>
    <section className="computer-section"><div className="section-title"><div><span className="section-kicker">COMPUTER PROGRESS</span><h2>电脑操作进度</h2></div><div className="demo-badge"><ShieldCheck size={13} /> 权限状态模拟</div></div><ComputerProgressCard state={progressState} action={progressState === 'awaiting_permission' ? '打开浏览器并读取页面' : progressState === 'paused' ? '浏览器动作已暂停' : '完成当前屏幕动作'} detail="真实接入时，这里来自 computer_view 工具结果。" elapsedMs={progressState === 'completed' ? 12800 : 4200} permissionLabel="屏幕操作与辅助功能" error={progressState === 'failed' ? '浏览器会话被用户取消' : undefined} onPause={() => setProgressState('paused')} onResume={() => setProgressState('running')} onRequestPermission={() => setProgressState('running')} /></section>
    <section className="workspace-grid"><article className="workspace-card"><BrainCircuit size={18} /><div><strong>记忆</strong><p>读取偏好、明确保存、可审计。</p></div><button onClick={() => alert('演示入口：连接现有偏好记忆工具')}>查看</button></article><article className="workspace-card"><PlugZap size={18} /><div><strong>连接器</strong><p>MCP、技能和可搜索来源。</p></div><button onClick={() => alert('演示入口：读取 MCP 连接状态')}>查看</button></article><article className="workspace-card"><CalendarDays size={18} /><div><strong>定时计划</strong><p>复用巡检 scheduler 生成计划草稿。</p></div><button onClick={() => alert('演示入口：打开巡检计划')}>查看</button></article><article className="workspace-card"><Search size={18} /><div><strong>跨应用搜索</strong><p>Web search 与 MCP source 卡片。</p></div><button onClick={() => setActivity('searching')}>体验</button></article></section>
    <section className="viewer-section"><div className="section-title"><div><span className="section-kicker">FULLSCREEN VIEWER</span><h2>内容查看器</h2></div><p>图片、视频、表格、地图统一入口</p></div><div className="viewer-buttons">{(['image', 'video', 'table', 'map'] as ViewerKind[]).map(kind => <button key={kind} onClick={() => setViewer(kind)}>{kind === 'image' ? '图片' : kind === 'video' ? '视频' : kind === 'table' ? '表格' : '地图'} 全屏</button>)}</div></section>
    <footer>当前页面使用生产 Bot、ComputerProgressCard 和 FullscreenViewer。演示按钮不会启动电脑、语音、浏览器或外部日历。</footer>
    <FullscreenViewer open={viewer === 'image'} title="相机快照" kind="image" src="/assets/providers/grok.ico" alt="演示图片" onClose={() => setViewer(null)} />
    <FullscreenViewer open={viewer === 'video'} title="实时画面" kind="video" src="" onClose={() => setViewer(null)} />
    <FullscreenViewer open={viewer === 'table'} title="连接器状态" kind="table" table={table} onClose={() => setViewer(null)} />
    <FullscreenViewer open={viewer === 'map'} title="地图查看器" kind="map" onClose={() => setViewer(null)}><div className="map-placeholder"><Bot state="idle" shape="grok-cloud" color="blue" size={96} animated={false} /><span>复用 MapGalleryScene / 点云场景的位置</span></div></FullscreenViewer>
  </main>
}

createRoot(document.getElementById('root')!).render(<Preview />)
