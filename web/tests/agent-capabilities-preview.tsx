import { useEffect } from 'react'
import { createRoot } from 'react-dom/client'
import { AgentConversation } from '../src/components/AgentConversation'
import { useAgentConversation } from '../src/hooks/useAgentConversation'
import { createAgentPreviewFetch } from './fixtures/agentCapabilitiesPreview'
import '../src/App.css'

// This page runs on a separate preview origin; it never contacts Host or OAuth.
window.fetch = createAgentPreviewFetch()

export function Preview() {
  const controller = useAgentConversation()
  const { setOpen } = controller
  useEffect(() => { setOpen(true) }, [setOpen])
  return <main style={{ maxWidth: 860, margin: '40px auto', padding: '0 24px' }}>
    <header style={{ marginBottom: 28 }}>
      <p style={{ fontSize: 11, letterSpacing: '0.12em', color: 'var(--text-dim)' }}>LINGTU / AGENT WORKSPACE</p>
      <h1 style={{ fontSize: 26, fontWeight: 500, margin: '12px 0' }}>连接模型，让对话有据可查。</h1>
      <p role="note" style={{ fontSize: 13, padding: '10px 14px', borderRadius: 9, background: '#f4e8c8', color: '#594d31' }}>交互验收 · 模拟授权与搜索结果，不连接真实账号、模型或机器人。</p>
    </header>
    <nav aria-label="演示步骤" style={{ display: 'flex', flexWrap: 'wrap', gap: 8, marginBottom: 20 }}>
      {['搜索 Hermes Agent 的官方资料', '读取第一个来源的网页正文', '演示搜索未配置'].map(text => <button key={text} onClick={() => controller.setDraft(text)}
        style={{ border: '1px solid var(--border)', padding: '8px 12px', borderRadius: 20, background: 'var(--bg-layer-1)', color: 'var(--text)', font: 'inherit', fontSize: 12, cursor: 'pointer' }}>{text}</button>)}
    </nav>
    <AgentConversation locale="zh" controller={controller} />
    <p style={{ fontSize: 12, lineHeight: 1.8, color: 'var(--text-dim)', marginTop: 18 }}>操作：填入示例后发送 → 查看来源 → 展开工具详情。底栏的“管理模型连接”可演示登录与取消；第二次登录会模拟授权成功。</p>
  </main>
}

createRoot(document.getElementById('root')!).render(<Preview />)
