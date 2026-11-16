import { useEffect } from 'react'
import { createRoot } from 'react-dom/client'
import { AgentConversation } from '../src/components/AgentConversation'
import { useAgentConversation } from '../src/hooks/useAgentConversation'
import '../src/App.css'

/** Recording entry: unlike the fixture preview this page leaves window.fetch untouched. */
export function CodexLivePreview() {
  const controller = useAgentConversation()
  const { setOpen } = controller
  useEffect(() => { setOpen(true) }, [setOpen])
  return <main data-live-codex-preview style={{ maxWidth: 900, margin: '32px auto', padding: '0 24px' }}>
    <header style={{ marginBottom: 24 }}>
      <p style={{ fontSize: 11, letterSpacing: '0.12em', color: 'var(--text-dim)' }}>LINGTU / CODEX LIVE</p>
      <h1 style={{ fontSize: 26, fontWeight: 500, margin: '10px 0' }}>在 LingTu Web 中与 Codex 对话</h1>
      <p role="status" style={{ fontSize: 13, lineHeight: 1.7, color: 'var(--text-dim)' }}>
        当前页面通过 Vite 代理连接 LingTu Gateway 5050；消息和工具步骤来自真实 Codex App Server。
      </p>
    </header>
    <AgentConversation locale="zh" controller={controller} />
  </main>
}

createRoot(document.getElementById('root')!).render(<CodexLivePreview />)
