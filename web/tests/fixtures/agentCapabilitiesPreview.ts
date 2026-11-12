import type { ConversationMessage, ConversationOptions, ConversationSnapshot, ConversationSubmission, ConversationTurn } from '../../src/services/agentConversation.ts'

export function createAgentPreviewFetch(): typeof fetch {
  let connected = false
  let loginCount = 0
  let polls = 0
  let loginState = 'pending'
  const messages: ConversationMessage[] = []
  const turns: ConversationTurn[] = []
  const now = () => Date.now() / 1000
  const metadata = { conversation_id: 'demo', title: '联网能力 · 交互验收', created_at: now(), updated_at: now(), archived: false }
  const json = (data: unknown, status = 200) => Promise.resolve(new Response(JSON.stringify(data), { status, headers: { 'Content-Type': 'application/json' } }))
  const login = () => ({ id: `demo-${loginCount}`, state: loginState, user_code: 'DEMO-ONLY',
    verification_url: 'https://auth.openai.com/codex/device', expires_at: now() + 900, interval: 3 })
  const add = (turn: ConversationTurn, role: ConversationMessage['role'], text: string, kind?: string) => {
    const message: ConversationMessage = { id: `message-${messages.length}`, seq: messages.length + 1,
      turn_id: turn.turn_id, role, text, kind, created_at: now() }
    messages.push(message)
    return message
  }
  return async (input, init) => {
    const url = new URL(String(input), 'http://localhost')
    const path = url.pathname
    const method = init?.method ?? 'GET'
    if (path === '/api/v1/agent/options') {
      const reasoning = [{ id: 'default', label: '默认' }, { id: 'high', label: '高' }]
      const options: ConversationOptions = { available: true, provider: 'moonshot', default_model: 'kimi-demo', default_reasoning: 'default',
        default_permission_mode: 'read_only', permission_modes: ['read_only', 'default', 'full_access'],
        models: [
          { id: 'kimi-demo', label: 'Kimi · 演示', provider: 'moonshot', reasoning, image_input: false, configured: true },
          { id: 'codex:gpt-6.1-sol', label: 'GPT-6.1 Sol · Codex', provider: 'codex', reasoning, image_input: true, configured: connected },
          { id: 'claude-demo', label: 'Claude', provider: 'claude', reasoning, image_input: true, configured: false, key_env: 'ANTHROPIC_API_KEY' },
          { id: 'openai-demo', label: 'OpenAI API', provider: 'openai', reasoning, image_input: true, configured: false, key_env: 'OPENAI_API_KEY' },
          { id: 'qwen-demo', label: 'Qwen', provider: 'qwen', reasoning, image_input: false, configured: false, key_env: 'DASHSCOPE_API_KEY' },
        ], attachments: { accept: '.txt,.png', max_files: 4, max_file_bytes: 1000000 } }
      return json(options)
    }
    if (path.startsWith('/api/v1/agent/connections/codex')) {
      if (path.endsWith('/login')) { loginCount++; polls = 0; loginState = 'pending'; return json(login()) }
      if (path.endsWith('/poll')) {
        if (loginState === 'pending' && ++polls >= 3 && loginCount >= 2) { connected = true; loginState = 'approved' }
        return json(login())
      }
      if (path.includes('/login/')) { loginState = 'cancelled'; return json(login()) }
      if (method === 'DELETE') connected = false
      return json({ provider: 'codex', connected, inference_verified: false })
    }
    if (path === '/api/v1/agent/conversations') return json(method === 'POST' ? { conversation_id: 'demo' } : { conversations: [metadata] })
    if (path.endsWith('/messages') && method === 'POST') {
      const body = JSON.parse(String(init?.body)) as ConversationSubmission
      const turn: ConversationTurn = { turn_id: `turn-${turns.length}`, message_id: body.message_id, state: 'running',
        model: body.model, reasoning: body.reasoning, permission_mode: body.permission_mode, started_at: now() }
      turns.push(turn)
      add(turn, 'user', body.text)
      const extracted = body.text.includes('读取')
      const failure = body.text.includes('未配置')
      const tool = add(turn, 'tool', '执行中', extracted ? 'web_extract' : 'web_search')
      tool.metadata = { status: 'running', started_at: now() }
      setTimeout(() => {
        const sources = extracted ? [{ title: 'Hermes Agent · README', url: 'https://github.com/NousResearch/hermes-agent',
          content: '【演示正文，非实时提取】\nHermes Agent 是 Nous Research 的开源助手项目。\n\n本页演示：网页正文以来源卡片呈现，展开后可阅读；完整内容通过原网页访问。\n\nLingTu 本轮沿用现有 Agent 循环，迁入搜索与提取协议，保留每次调用耗时、失败提示和来源链接。', truncated: false }]
          : [{ title: 'Hermes Agent · 官方代码仓库', url: 'https://github.com/NousResearch/hermes-agent', content: '【演示摘要】官方源代码、README 与工具实现。可继续要求助手读取网页正文。', truncated: false },
            { title: 'Hermes Agent · v0.23.1', url: 'https://github.com/NousResearch/hermes-agent/releases/tag/v0.23.1', content: '【演示摘要】本次能力迁移所对照的固定版本；版本信息应以官方页面为准。', truncated: false }]
        tool.text = JSON.stringify(failure ? { error: '联网工具尚未配置：请在 Host 服务端设置 TAVILY_API_KEY。', provider: 'tavily' } : { provider: 'tavily', sources })
        tool.metadata = { ...tool.metadata, status: 'completed', duration_ms: Math.round((now() - tool.metadata!.started_at!) * 1000) }
        turn.state = 'completed'; turn.kind = 'reply'; turn.finished_at = now()
        turn.text = failure ? '这次没有完成联网搜索。配置 Host 的 TAVILY_API_KEY 后可以重试。'
          : extracted ? '网页正文放在上面的来源卡片中，可展开阅读或打开原网页。这里是模拟响应。'
            : '这里展示两个官方来源的卡片。你可以打开链接，也可以继续让我读取第一个来源。这里是模拟响应。'
        add(turn, 'assistant', turn.text)
      }, 2200)
      return json({ ...turn, conversation_id: 'demo' })
    }
    if (path.startsWith('/api/v1/agent/conversations/')) {
      const id = path.split('/')[5]
      if (id !== 'demo') return json({ reason: 'conversation_not_found' }, 404)
      const snapshot: ConversationSnapshot = { conversation_id: 'demo', messages, turns, approvals: [], metadata, server_time: now() }
      return json(snapshot)
    }
    return json({ error: `演示页面不支持此操作：${method} ${path}` }, 400)
  }
}
