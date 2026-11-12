import { dashboardFetch } from './observationMode.ts'

export interface CodexConnection { provider: 'codex'; connected: boolean; expires_at?: number; inference_verified: boolean }
export interface CodexLogin {
  id: string; state: 'pending' | 'approved' | 'cancelled' | 'expired' | 'failed'
  user_code: string; verification_url: string; expires_at: number; interval: number
}

const path = '/api/v1/agent/connections/codex'
async function request(url: string, method = 'GET') {
  const response = await dashboardFetch(url, { method, signal: AbortSignal.timeout(20_000) })
  const data = await response.json().catch(() => null)
  if (!response.ok) {
    const reason = data?.detail?.reason
    throw new Error(reason === 'agent_unavailable' ? '当前运行服务未启用对话功能，请先启动支持对话的运行模式。'
      : typeof reason === 'string' ? reason : `连接服务暂不可用（HTTP ${response.status}）`)
  }
  return data
}
function connection(value: CodexConnection): CodexConnection {
  if (value?.provider !== 'codex' || typeof value.connected !== 'boolean' || typeof value.inference_verified !== 'boolean') throw new Error('模型连接状态无效')
  return value
}
function login(value: CodexLogin): CodexLogin {
  if (!value || typeof value.id !== 'string' || typeof value.user_code !== 'string'
    || value.verification_url !== 'https://auth.openai.com/codex/device'
    || !['pending', 'approved', 'cancelled', 'expired', 'failed'].includes(value.state)
    || !Number.isFinite(value.expires_at) || !Number.isFinite(value.interval) || value.interval < 3) throw new Error('登录状态无效')
  return value
}
export async function getCodexConnection() { return connection(await request(path)) }
export async function startCodexLogin() { return login(await request(`${path}/login`, 'POST')) }
export async function pollCodexLogin(id: string) { return login(await request(`${path}/login/${encodeURIComponent(id)}/poll`, 'POST')) }
export async function cancelCodexLogin(id: string) { return login(await request(`${path}/login/${encodeURIComponent(id)}`, 'DELETE')) }
export async function disconnectCodex() { return connection(await request(path, 'DELETE')) }
