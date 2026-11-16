import { createRoot } from 'react-dom/client'
import { Paperclip, ShieldCheck, SlidersHorizontal } from 'lucide-react'
import { ToolActivityGroup } from '../src/components/agentConversation/ToolActivity.tsx'
import { ComputerProgressCard } from '../src/components/agentConversation/ComputerProgressCard.tsx'
import { PlanCard } from '../src/components/agentConversation/PlanCard.tsx'
import { DecisionInbox } from '../src/components/agentConversation/DecisionInbox.tsx'
import type { ConversationMessage } from '../src/services/agentConversation.ts'
import '../src/App.css'

const now = Date.now() / 1000
const message = (id: string, kind: string, text: unknown, metadata: ConversationMessage['metadata'] = {}): ConversationMessage => ({
  id, seq: Number(id.replace(/\D/g, '')) || 1, role: 'tool', kind, text: typeof text === 'string' ? text : JSON.stringify(text), turn_id: 'feature-turn', created_at: now, metadata,
})
const timeline = [
  message('tool-1', 'web_search', { provider: 'tavily', sources: [{ title: '地图检查规范', url: 'https://example.com/map', content: '读取到 2 个相关来源。' }] }, { status: 'completed', display_name: '联网搜索', duration_ms: 840 }),
  message('tool-2', 'inspect_navigation', { success: true, state: 'PAUSED', reason: 'operator_pause', route: '未启动' }, { status: 'completed', display_name: '检查导航诊断', duration_ms: 1280 }),
  message('tool-3', 'run_python', { error: '执行被权限策略拦截', command: 'python analyze.py', stderr: 'permission_read_only' }, { status: 'failed', display_name: '运行 Python 分析', duration_ms: 420 }),
]
const codex = [
  message('tool-11', 'codex_event', { item: { id: 'reasoning-1', type: 'reasoning' } }, { status: 'completed', method: 'item/completed', codex: true }),
  message('tool-12', 'codex_event', { entries: [{ content: '检查当前地图版本', status: 'completed' }, { content: '生成只读检查报告', status: 'in_progress' }] }, { status: 'completed', method: 'turn/plan/updated', codex: true }),
  message('tool-13', 'codex_event', { item: { id: 'change-1', type: 'fileChange', changes: [{ path: 'web/src/components/AgentConversation.tsx' }] } }, { status: 'completed', method: 'item/completed', codex: true }),
  message('tool-14', 'codex_event', { item: { id: 'exec-1', type: 'commandExecution', command: 'npm run build', output: '✓ built in 31.87s' } }, { status: 'completed', method: 'item/completed', codex: true }),
  message('tool-15', 'codex_event', { usage: { input_tokens: 1840, output_tokens: 612, total_tokens: 2452 } }, { status: 'completed', method: 'turn/usage/updated', codex: true }),
]

export function Section({ id, title, children }: { id: string; title: string; children: React.ReactNode }) {
  return <section data-shot={id} style={{ background: '#fff', border: '1px solid #e7eaf0', borderRadius: 18, padding: 22, boxShadow: '0 12px 32px rgba(35,45,65,.06)' }}>
    <header style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'baseline', marginBottom: 16 }}>
      <h2 style={{ margin: 0, fontSize: 17, fontWeight: 650 }}>{title}</h2><code style={{ color: '#8992a4', fontSize: 11 }}>{id}</code>
    </header>{children}
  </section>
}

export function ModelControls() {
  return <div style={{ display: 'flex', flexWrap: 'wrap', gap: 10 }}>
    <button className="feature-chip"><span>GPT-6.1 Sol · Codex</span><span>⌄</span></button>
    <button className="feature-chip"><SlidersHorizontal size={14} /><span>高</span><span>⌄</span></button>
    <button className="feature-chip"><ShieldCheck size={14} /><span>执行前确认</span><span>⌄</span></button>
  </div>
}

export function AttachmentPanel() {
  return <div style={{ border: '1px solid #e5e8ef', borderRadius: 14, padding: 14, background: '#fbfcfe' }}>
    <div style={{ display: 'flex', alignItems: 'center', gap: 8, color: '#4e5b72', fontSize: 13, marginBottom: 10 }}><Paperclip size={15} />本轮附件 · 2 个</div>
    <div style={{ display: 'grid', gap: 8 }}>
      {['inspection-report.md · 24 KB', 'map-snapshot.png · 1.8 MB'].map(name => <div key={name} style={{ display: 'flex', justifyContent: 'space-between', padding: '10px 12px', borderRadius: 10, background: '#fff', border: '1px solid #edf0f5', fontSize: 12 }}><span>{name}</span><span style={{ color: '#6f7b8e' }}>已上传</span></div>)}
    </div>
  </div>
}

export function App() {
  return <main style={{ maxWidth: 1040, margin: '0 auto', padding: '34px 24px 64px', color: '#1b2433' }}>
    <header style={{ marginBottom: 24 }}><p style={{ fontSize: 11, letterSpacing: '.14em', color: '#7f8a9d', margin: 0 }}>LINGTU / FEATURE EVIDENCE</p><h1 style={{ fontSize: 28, margin: '10px 0 8px' }}>工具过程、Codex 卡片与权限状态</h1><p style={{ color: '#6c7688', margin: 0, lineHeight: 1.7 }}>正式组件的固定回执展示页；这里的内容是可重复的 UI 验收数据，不代表机器人已经执行了这些动作。</p></header>
    <div style={{ display: 'grid', gap: 18 }}>
      <Section id="timeline" title="工具过程时间线"><ToolActivityGroup messages={timeline} active awaiting={[]} now={now} /></Section>
      <Section id="collapsed-summary" title="工具过程折叠摘要"><ToolActivityGroup messages={timeline.map((item, i) => ({ ...item, id: `summary-${i}`, metadata: { status: 'completed', display_name: item.metadata?.display_name } }))} active={false} awaiting={[]} now={now} /></Section>
      <Section id="status-detail" title="工具状态、耗时、失败原因与展开详情"><ToolActivityGroup messages={[timeline[2]]} active awaiting={[]} now={now} /></Section>
      <Section id="codex-cards" title="Codex reasoning / plan / file change / terminal / token usage"><ToolActivityGroup messages={codex} active awaiting={[]} now={now} /></Section>
      <Section id="model-controls" title="模型、推理深度与权限模式"><ModelControls /></Section>
      <Section id="attachments" title="文件与图片附件"><AttachmentPanel /></Section>
      <Section id="computer-progress" title="Computer Use 进度与权限"><ComputerProgressCard state="awaiting_permission" action="打开浏览器并读取巡检页面" detail="当前动作停在权限边界，尚未操作屏幕。" elapsedMs={4200} permissionLabel="屏幕操作与辅助功能" onRequestPermission={() => {}} /></Section>
      <Section id="plan-card" title="方案卡"><PlanCard locale="zh" text={'# 园区东区检查方案\\n\\n1. 核对地图版本\\n2. 读取导航区域\\n3. 输出检查报告'} plan={{ title: '园区东区检查方案', summary: '只读草稿，未执行任何动作。', steps: [{ title: '核对地图版本', detail: '读取当前地图元数据和内容版本。' }, { title: '读取导航区域', detail: '检查禁行区域和可通行范围。' }, { title: '输出检查报告', detail: '把证据与风险整理成可分享结果。' }] }} /></Section>
      <Section id="decision-inbox" title="决策收件箱"><DecisionInbox decisions={[{ id: 'permission-1', title: '是否允许运行 Python 分析？', detail: '将在 Host 执行本轮读取到的观测分析，不会启动后台任务。', primary: '允许执行', secondary: '拒绝' }, { id: 'permission-2', title: '是否另存为新的地图副本？', detail: '原地图保持不变，结果会生成一张新的待检查地图。', primary: '允许另存', secondary: '先不执行' }]} /></Section>
    </div>
  </main>
}

createRoot(document.getElementById('root')!).render(<App />)
