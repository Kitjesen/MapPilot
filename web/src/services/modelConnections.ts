import { dashboardFetch } from './observationMode.ts'

export interface CodexConnection { provider: 'codex'; connected: boolean; expires_at?: number; inference_verified: boolean; auth_mode?: 'app_server' }
export interface CodexLogin {
  id: string; state: 'pending' | 'approved' | 'cancelled' | 'expired' | 'failed'
  user_code?: string; verification_url: string; auth_url?: string; expires_at: number; interval: number
}
export interface AcpProviderStatus {
  id: string
  label: string
  kind: 'acp'
  command: string | null
  args: string[]
  available: boolean
  version: string | null
  authenticated: boolean | null
  auth_owner: 'provider'
  command_env: string
  args_env: string
  install_url?: string
  install_command?: string
  login_command?: string
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
  if (!value || typeof value.id !== 'string'
    || typeof value.verification_url !== 'string' || !value.verification_url.startsWith('https://auth.openai.com/')
    || (value.user_code != null && typeof value.user_code !== 'string')
    || (value.auth_url != null && typeof value.auth_url !== 'string')
    || !['pending', 'approved', 'cancelled', 'expired', 'failed'].includes(value.state)
    || !Number.isFinite(value.expires_at) || !Number.isFinite(value.interval) || value.interval < 3) throw new Error('登录状态无效')
  return value
}
export async function getCodexConnection() { return connection(await request(path)) }
export async function startCodexLogin() { return login(await request(`${path}/login`, 'POST')) }
export async function pollCodexLogin(id: string) { return login(await request(`${path}/login/${encodeURIComponent(id)}/poll`, 'POST')) }
export async function cancelCodexLogin(id: string) { return login(await request(`${path}/login/${encodeURIComponent(id)}`, 'DELETE')) }
export async function disconnectCodex() { return connection(await request(path, 'DELETE')) }
export async function getAcpProviders(): Promise<AcpProviderStatus[]> {
  const value = await request('/api/v1/agent/providers')
  if (!value || !Array.isArray(value.providers)) throw new Error('Agent 提供商状态无效')
  return value.providers.map((item: AcpProviderStatus) => {
    if (typeof item?.id !== 'string' || item.kind !== 'acp' || typeof item.available !== 'boolean'
      || typeof item.command_env !== 'string' || !Array.isArray(item.args)) throw new Error('Agent 提供商状态无效')
    return item
  })
}
