import { dashboardFetch } from './observationMode.ts'

export const CONVERSATION_PERMISSION_MODES = ['read_only', 'default', 'full_access'] as const
export type ConversationPermission = typeof CONVERSATION_PERMISSION_MODES[number]

export function isConversationPermission(value: unknown): value is ConversationPermission {
  return CONVERSATION_PERMISSION_MODES.some(mode => mode === value)
}

export interface ConversationApproval {
  approval_id: string
  turn_id: string
  tool_name: string
  arguments: Record<string, unknown>
  state: 'pending' | 'approved' | 'denied' | 'cancelled'
}

function isConversationApproval(value: unknown): value is ConversationApproval {
  if (!value || typeof value !== 'object') return false
  const approval = value as ConversationApproval
  return typeof approval.approval_id === 'string' && typeof approval.turn_id === 'string'
    && typeof approval.tool_name === 'string' && Boolean(approval.arguments)
    && typeof approval.arguments === 'object' && !Array.isArray(approval.arguments)
    && ['pending', 'approved', 'denied', 'cancelled'].includes(approval.state)
}

export interface ConversationAttachment {
  id: string
  name: string
  mime_type: string
  size: number
}

export interface ConversationOptions {
  available: true
  provider: string
  default_model: string
  default_reasoning: string
  default_permission_mode: ConversationPermission
  permission_modes: ConversationPermission[]
  models: Array<{ id: string; label: string; provider?: string; configured?: boolean; attachment_accept?: string; key_env?: string; reasoning: Array<{ id: string; label: string }>; image_input: boolean }>
  attachments: { accept: string; max_files: number; max_file_bytes: number }
}

export interface ConversationPlan {
  title: string
  summary?: string
  steps: Array<{ title: string; detail?: string }>
}

export function isConversationPlan(value: unknown): value is ConversationPlan {
  if (!value || typeof value !== 'object') return false
  const plan = value as ConversationPlan
  return typeof plan.title === 'string' && Boolean(plan.title.trim()) && plan.title.length <= 120
    && (plan.summary === undefined || typeof plan.summary === 'string' && Boolean(plan.summary.trim()) && plan.summary.length <= 1000)
    && Array.isArray(plan.steps) && plan.steps.length >= 1 && plan.steps.length <= 12
    && plan.steps.every(step => step && typeof step.title === 'string' && Boolean(step.title.trim()) && step.title.length <= 160
      && (step.detail === undefined || typeof step.detail === 'string' && Boolean(step.detail.trim()) && step.detail.length <= 2000))
}

function isMessageMetadata(value: unknown): boolean {
  if (value == null) return true
  if (typeof value !== 'object' || Array.isArray(value)) return false
  const metadata = value as { plan?: unknown }
  return metadata.plan === undefined || isConversationPlan(metadata.plan)
}

export interface ConversationMessage {
  revision?: number
  id: string
  seq: number
  role: 'user' | 'assistant' | 'tool'
  text: string
  turn_id: string
  run_id?: string
  kind?: string
  attachments?: ConversationAttachment[]
  created_at?: number | null
  metadata?: {
    tool_call_id?: string; display_name?: string; status?: string; started_at?: number; duration_ms?: number; choices?: string[]
    card?: { type?: string; title?: string; subtitle?: string; image_url?: string; price?: string; detail?: string; actions?: Array<{ label: string; href: string }> }
    collaboration?: { name: string; status?: string; summary?: string; href?: string }
    plan?: ConversationPlan
  } | null
}

export interface ConversationTurn {
  queue_position?: number
  turn_id: string
  reply_to_message_id?: string
  reply_quote?: string
  state: 'queued' | 'running' | 'completed' | 'failed' | 'cancelled' | 'interrupted'
  kind?: string
  text?: string
  run_id?: string
  message_id?: string
  failure_reason?: string
  model?: string
  reasoning?: string
  permission_mode?: ConversationPermission
  started_at?: number
  finished_at?: number
}

export interface ConversationMetadata {
  conversation_id: string
  title: string
  created_at: number
  updated_at: number
  archived: boolean
  forked_from?: { conversation_id: string; message_id: string; title: string; seq: number }
}

export interface ConversationSearchResult {
  conversation_id: string
  title: string
  message_id: string
  seq: number
  role: 'user' | 'assistant'
  snippet: string
}

export interface ConversationShare {
  token: string
  created_at: number
  message_count: number
}

export interface SharedConversationSnapshot {
  title: string
  created_at: number
  messages: Array<{ role: 'user' | 'assistant'; text: string; seq: number }>
}

function isConversationMetadata(value: unknown): value is ConversationMetadata {
  if (!value || typeof value !== 'object') return false
  const item = value as ConversationMetadata
  const source = item.forked_from
  return typeof item.conversation_id === 'string' && typeof item.title === 'string'
    && Number.isFinite(item.created_at) && Number.isFinite(item.updated_at) && typeof item.archived === 'boolean'
    && (source === undefined || Boolean(source) && typeof source === 'object'
      && typeof source.conversation_id === 'string' && Boolean(source.conversation_id)
      && typeof source.message_id === 'string' && Boolean(source.message_id) && typeof source.title === 'string'
      && Number.isInteger(source.seq) && source.seq >= 1)
}

function isConversationShare(value: unknown): value is ConversationShare {
  if (!value || typeof value !== 'object') return false
  const share = value as ConversationShare
  return typeof share.token === 'string' && Boolean(share.token) && Number.isFinite(share.created_at)
    && Number.isInteger(share.message_count) && share.message_count >= 0
}

export interface ConversationSnapshot {
  history_cursor?: number | null
  conversation_id: string
  server_time?: number
  messages: ConversationMessage[]
  turns: ConversationTurn[]
  approvals?: ConversationApproval[]
  metadata?: ConversationMetadata
  share?: ConversationShare | null
  active_run?: { run_id: string; instruction?: string } | null
}

export interface ConversationMapContext {
  map_id: string
  content_epoch: number
  selection?: { min: number[]; max: number[] } | null
}

export interface ConversationSubmission {
  enqueue?: boolean
  persona?: string
  map_context?: ConversationMapContext | null
  message_id: string
  text: string
  reply_to_message_id?: string
  reply_quote?: string
  model?: string
  reasoning?: string
  attachment_ids?: string[]
  permission_mode?: ConversationPermission
}

export class ConversationError extends Error {
  readonly status: number
  constructor(message: string, status: number) { super(message); this.status = status }
}

const path = '/api/v1/agent/conversations'

async function request(url: string, body?: object, timeout = 15_000, method = body ? 'POST' : 'GET'): Promise<Record<string, unknown>> {
  const response = await dashboardFetch(url, {
    method,
    headers: body ? { 'Content-Type': 'application/json' } : undefined,
    body: body ? JSON.stringify(body) : undefined,
    signal: AbortSignal.timeout(timeout),
  })
  const data = await response.json().catch(() => null)
  if (!response.ok) {
    const reason = data?.detail?.reason ?? data?.reason ?? data?.error ?? data?.message
    const known: Record<string, string> = {
      conversation_active: '会话仍有执行中的任务，请先结束任务并等待停止确认。',
      conversation_archived: '此会话已归档，恢复后可以继续对话。',
      share_empty: '暂无可分享的已结束对话。',
    }
    throw new ConversationError(known[reason] ?? (reason === 'map_edit_only' ? '当前工作区仅支持地图编辑，未启用对话。'
      : response.status === 503 ? 'Agent 未启用，当前无法对话。'
      : reason ? String(reason) : `会话请求失败（HTTP ${response.status}）。`), response.status)
  }
  if (!data || typeof data !== 'object') throw new Error('会话服务返回了无效数据。')
  return data
}

export function isConversationAttachment(value: unknown): value is ConversationAttachment {
  if (!value || typeof value !== 'object') return false
  const attachment = value as ConversationAttachment
  return typeof attachment.id === 'string' && typeof attachment.name === 'string'
    && typeof attachment.mime_type === 'string' && Number.isFinite(attachment.size) && attachment.size >= 0
}

export async function fetchConversationOptions(): Promise<ConversationOptions> {
  const data = await request('/api/v1/agent/options')
  const options = data as unknown as ConversationOptions
  if (options.available !== true || typeof options.provider !== 'string'
    || !isConversationPermission(options.default_permission_mode) || !Array.isArray(options.permission_modes)
    || options.permission_modes.some(mode => !isConversationPermission(mode))
    || !CONVERSATION_PERMISSION_MODES.every(mode => options.permission_modes.includes(mode))
    || typeof options.default_model !== 'string' || typeof options.default_reasoning !== 'string'
    || !Array.isArray(options.models) || options.models.length === 0
    || options.models.some(model => !model || typeof model.id !== 'string' || typeof model.label !== 'string'
      || typeof model.image_input !== 'boolean' || !Array.isArray(model.reasoning) || !model.reasoning.length
      || model.reasoning.some(item => !item || typeof item.id !== 'string' || typeof item.label !== 'string'))
    || !options.models.some(model => model.id === options.default_model
      && model.reasoning.some(item => item.id === options.default_reasoning))
    || !options.attachments || typeof options.attachments.accept !== 'string' || !options.attachments.accept.trim()
    || !Number.isInteger(options.attachments.max_files) || options.attachments.max_files <= 0
    || !Number.isInteger(options.attachments.max_file_bytes) || options.attachments.max_file_bytes <= 0) {
    throw new Error('Agent 模型能力格式无效，请刷新连接。')
  }
  return options
}

export async function uploadConversationAttachment(id: string, file: File, attachmentId: string,
  limits: ConversationOptions['attachments'], model?: string): Promise<ConversationAttachment> {
  if (!file.size || file.size > limits.max_file_bytes) {
    throw new Error(`“${file.name}”为空或超过单文件 ${Math.round(limits.max_file_bytes / 1024 / 1024)} MB 的限制。`)
  }
  const accepted = limits.accept.split(',').map(item => item.trim().toLowerCase()).filter(Boolean)
  const mime = file.type.toLowerCase()
  if (!accepted.some(type => type.startsWith('.') ? file.name.toLowerCase().endsWith(type)
    : type.endsWith('/*') ? mime.startsWith(type.slice(0, -1)) : mime === type)) {
    throw new Error(`不支持“${file.name}”的文件类型，可上传 ${limits.accept}。`)
  }
  const dataUrl = await new Promise<string>((resolve, reject) => {
    const reader = new FileReader()
    reader.onload = () => typeof reader.result === 'string' ? resolve(reader.result) : reject(new Error('无法读取附件。'))
    reader.onerror = () => reject(new Error(`无法读取“${file.name}”。`))
    reader.onabort = () => reject(new Error('附件读取已取消。'))
    reader.readAsDataURL(file)
  })
  const attachment = await request(`${path}/${encodeURIComponent(id)}/attachments`, {
    attachment_id: attachmentId, model, name: file.name, mime_type: file.type || 'application/octet-stream',
    data_base64: dataUrl.slice(dataUrl.indexOf(',') + 1),
  }, 60_000)
  if (!isConversationAttachment(attachment) || attachment.id !== attachmentId) throw new Error('附件上传结果无效，请重新选择文件。')
  return attachment
}

export function conversationAttachmentUrl(conversationId: string, attachmentId: string): string {
  return `${path}/${encodeURIComponent(conversationId)}/attachments/${encodeURIComponent(attachmentId)}`
}

export async function createConversation(): Promise<string> {
  const data = await request(path, {})
  if (typeof data.conversation_id !== 'string' || !data.conversation_id) throw new Error('会话服务没有返回会话 ID。')
  return data.conversation_id
}

export async function forkConversation(id: string, messageId: string): Promise<ConversationMetadata> {
  const data = await request(`${path}/${encodeURIComponent(id)}/fork`, { message_id: messageId })
  if (!isConversationMetadata(data) || !data.conversation_id || data.conversation_id === id || data.archived
    || !data.forked_from || data.forked_from.conversation_id !== id || data.forked_from.message_id !== messageId) {
    throw new Error('分支创建结果未确认，请先查看历史会话，避免重复创建。')
  }
  return data
}

export async function listConversations(archived = false): Promise<ConversationMetadata[]> {
  const data = await request(`${path}?archived=${archived}`)
  if (!Array.isArray(data.conversations) || !data.conversations.every(isConversationMetadata)) {
    throw new Error('会话列表格式无效，请重试。')
  }
  return data.conversations
}

export async function searchConversations(query: string, archived = false): Promise<ConversationSearchResult[]> {
  const q = query.trim()
  if (!q) return []
  if (q.length > 200) throw new Error('搜索词最多 200 个字符。')
  const data = await request(`${path}/search?q=${encodeURIComponent(q)}&archived=${archived}`)
  if (!Array.isArray(data.results) || data.results.length > 30 || data.results.some(item => !item
    || !['conversation_id', 'title', 'message_id', 'snippet'].every(key => typeof item[key] === 'string')
    || !item.conversation_id || !item.message_id || !Number.isInteger(item.seq) || item.seq < 1
    || !['user', 'assistant'].includes(item.role))) throw new Error('会话搜索结果格式无效，请重试。')
  return data.results
}

export function parseConversationMessageEvent(value: unknown): (ConversationMessage & { conversation_id: string }) | null {
  if (!value || typeof value !== 'object') return null
  const event = value as Record<string, unknown>
  if (event.type !== 'agent_message'
    || !['conversation_id', 'id', 'turn_id'].every(key => typeof event[key] === 'string' && event[key])
    || !['user', 'assistant', 'tool'].includes(String(event.role)) || typeof event.text !== 'string'
    || !Number.isInteger(event.seq) || Number(event.seq) < 1
    || !Number.isInteger(event.revision) || Number(event.revision) < 1
    || ['kind', 'run_id'].some(key => event[key] != null && typeof event[key] !== 'string')
    || (event.created_at != null && !Number.isFinite(event.created_at))
    || !isMessageMetadata(event.metadata)) return null
  return event as unknown as ConversationMessage & { conversation_id: string }
}

export async function updateConversation(id: string, changes: { title?: string; archived?: boolean }): Promise<ConversationMetadata> {
  const data = await request(`${path}/${encodeURIComponent(id)}`, changes, 15_000, 'PATCH')
  if (!isConversationMetadata(data) || data.conversation_id !== id) throw new Error('会话更新结果无效，请刷新后核对。')
  return data
}

export async function createConversationShare(id: string): Promise<ConversationShare> {
  const data = await request(`${path}/${encodeURIComponent(id)}/share`, {})
  if (!isConversationShare(data)) throw new Error('分享结果无效，请重试。')
  return data
}

export async function deleteConversationShare(id: string): Promise<void> {
  const data = await request(`${path}/${encodeURIComponent(id)}/share`, undefined, 15_000, 'DELETE')
  if (data.revoked !== true) throw new Error('撤销分享未确认，请重试。')
}

export async function fetchSharedConversation(token: string): Promise<SharedConversationSnapshot> {
  const data = await request(`/api/v1/agent/shares/${encodeURIComponent(token)}`)
  if (typeof data.title !== 'string' || !Number.isFinite(data.created_at) || !Array.isArray(data.messages)
    || data.messages.some(item => !item || !['user', 'assistant'].includes(item.role)
      || typeof item.text !== 'string' || !Number.isInteger(item.seq))) throw new Error('分享内容格式无效。')
  return data as unknown as SharedConversationSnapshot
}

export async function fetchConversation(id: string, beforeSeq?: number): Promise<ConversationSnapshot> {
  const data = await request(`${path}/${encodeURIComponent(id)}${beforeSeq == null ? '' : `?before_seq=${beforeSeq}`}`)
  if (data.conversation_id !== id || !Array.isArray(data.messages) || !Array.isArray(data.turns)
    || (data.server_time != null && !Number.isFinite(data.server_time))
    || (data.history_cursor != null && (!Number.isInteger(data.history_cursor) || Number(data.history_cursor) < 1))
    || data.messages.some(message => !message || typeof message.id !== 'string' || !Number.isFinite(message.seq)
      || !['user', 'assistant', 'tool'].includes(message.role) || typeof message.text !== 'string' || typeof message.turn_id !== 'string'
      || (message.attachments != null && (!Array.isArray(message.attachments) || !message.attachments.every(isConversationAttachment)))
      || (message.created_at != null && !Number.isFinite(message.created_at))
      || (message.revision != null && (!Number.isInteger(message.revision) || message.revision < 1))
      || !isMessageMetadata(message.metadata)
      || ['run_id', 'kind'].some(key => message[key] != null && typeof message[key] !== 'string'))
    || data.turns.some(turn => !turn || typeof turn.turn_id !== 'string'
      || (turn.queue_position != null && !Number.isInteger(turn.queue_position))
      || !['queued', 'running', 'completed', 'failed', 'cancelled', 'interrupted'].includes(turn.state)
      || (turn.permission_mode != null && !isConversationPermission(turn.permission_mode))
      || ['started_at', 'finished_at'].some(key => turn[key] != null && !Number.isFinite(turn[key]))
      || ['kind', 'text', 'run_id', 'message_id', 'reply_to_message_id', 'reply_quote', 'failure_reason', 'model', 'reasoning'].some(key => turn[key] != null && typeof turn[key] !== 'string'))
    || (data.approvals != null && (!Array.isArray(data.approvals) || !data.approvals.every(isConversationApproval)))
    || (data.metadata != null && (!isConversationMetadata(data.metadata) || data.metadata.conversation_id !== id))
    || (data.share != null && !isConversationShare(data.share))
    || (data.active_run != null && (typeof data.active_run !== 'object'
      || typeof (data.active_run as { run_id?: unknown }).run_id !== 'string'
      || ((data.active_run as { instruction?: unknown }).instruction != null
        && typeof (data.active_run as { instruction?: unknown }).instruction !== 'string')))) {
    throw new Error('会话快照格式无效，历史尚未更新。')
  }
  return data as unknown as ConversationSnapshot
}

export async function sendConversationMessage(id: string, message: ConversationSubmission): Promise<ConversationTurn> {
  const data = await request(`${path}/${encodeURIComponent(id)}/messages`, message)
  if (data.conversation_id !== id || data.message_id !== message.message_id || typeof data.turn_id !== 'string'
    || (data.reply_to_message_id != null && typeof data.reply_to_message_id !== 'string')
    || (data.reply_quote != null && typeof data.reply_quote !== 'string')
    || !['queued', 'running', 'completed', 'failed', 'cancelled', 'interrupted'].includes(String(data.state))
    || ['started_at', 'finished_at'].some(key => data[key] != null && !Number.isFinite(data[key]))) {
    throw new Error('消息受理结果未确认，请使用原消息重试。')
  }
  return data as unknown as ConversationTurn
}

export async function resolveConversationApproval(id: string, approvalId: string, approved: boolean): Promise<ConversationApproval> {
  const data = await request(`${path}/${encodeURIComponent(id)}/approvals/${encodeURIComponent(approvalId)}`, { approved })
  if (!isConversationApproval(data) || data.approval_id !== approvalId) throw new Error('执行确认结果无效，请刷新后核对。')
  return data
}

export async function editQueuedMessage(id: string, turnId: string, text: string | null): Promise<void> {
  await request(`${path}/${encodeURIComponent(id)}/queue/${encodeURIComponent(turnId)}`,
    text === null ? undefined : { text }, 15_000, text === null ? 'DELETE' : 'PATCH')
}

export async function promoteQueuedMessage(id: string, turnId: string): Promise<void> {
  await request(`${path}/${encodeURIComponent(id)}/queue/${encodeURIComponent(turnId)}/promote`, {})
}

export function mergeConversationMessages(previous: ConversationMessage[], incoming: ConversationMessage[]): ConversationMessage[] {
  const messages = new Map(previous.map(message => [message.id, message]))
  for (const message of incoming) {
    const current = messages.get(message.id)
    if (!current || (message.revision ?? 0) >= (current.revision ?? 0)) messages.set(message.id, message)
  }
  return [...messages.values()].sort((left, right) => left.seq - right.seq || left.id.localeCompare(right.id))
}

export const conversationToolNames: Record<string, string> = {
  review_with_specialists: '协作检查', specialist_review: '专门助手检查',
  open_imu_monitor: '打开 IMU 监测', open_sensor_monitor: '打开传感器组件', start_imu_recording: '开始 IMU 记录', stop_imu_recording: '停止并保存 IMU 记录',
  list_imu_records: '查询 IMU 记录', read_imu_record: '读取 IMU 记录',
  get_robot_capabilities: '查询机器人能力', read_robot_sensor: '读取传感器', capture_camera: '拍摄相机快照',
  view_robot_card: '打开机器人卡片', view_live_camera: '观看实时相机', inspect_lidar: '查看 LiDAR 扫描', inspect_navigation: '检查导航诊断', run_python: '运行 Python 分析',
  get_model_connections: '模型连接',
  web_search: '联网搜索',
  web_extract: '读取网页',
  skills_list: '查找技能',
  read_assistant_memory: '查看偏好记忆',
  context_compaction: '整理对话上下文',
  list_mcp_connections: '查看 MCP 连接',
  list_mcp_resources: '查看 MCP 资源',
  read_mcp_resource: '读取 MCP 资源',
  list_mcp_prompts: '查看 MCP 提示模板',
  get_mcp_prompt: '读取 MCP 提示模板',
  search_conversation_history: '查找原始对话',
  read_conversation_turn: '读取原始记录',
  remember_preference: '记住偏好',
  update_preference: '修改偏好',
  forget_preference: '忘记偏好',
  skill_view: '读取技能',
  navigate_to: '前往目标位置', navigate_to_object: '前往目标物体', navigate_to_deg: '转向目标方向',
  find_object: '查找物体', detect_object: '识别物体', query_memory: '查询记忆', ask_user: '补充信息', done: '提交结果',
  list_saved_maps: '查询地图库', inspect_saved_map: '检查地图状态', view_saved_map: '查看三维地图',
  find_inspection_points: '查找巡检点', preview_map_edits: '预览点云编辑',
  save_map_copy: '另存地图', set_navigation_region: '保存导航区域',
  save_inspection_point: '保存巡检点', get_agent_status: '查询任务状态',
}
