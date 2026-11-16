export const BOT_EXPRESSIONS = {
  neutral: '平静', happy: '开心', surprised: '惊讶', sad: '难过', sleepy: '困倦',
  squinted: '眯眼', calm: '放松', inspecting: '好奇', confused: '困惑',
} as const
export type BotExpression = keyof typeof BOT_EXPRESSIONS

export const BOT_ACTIVITIES = {
  idle: '待机', listening: '倾听', thinking: '思考', searching: '检索', reading: '阅读',
  working: '工作', coding: '编程', running: '运行', computing: '计算', controlling: '控制',
  waiting: '等待', blocked: '受阻', finished: '完成', failed: '失败', sleeping: '休眠',
  waking: '唤醒', happy: '开心', excited: '兴奋', laughing: '大笑', proud: '自豪',
  curious: '好奇', playful: '玩耍', shy: '害羞', surprised: '惊讶', suspicious: '警觉',
  angry: '生气', confused: '困惑', sad: '难过', drowsy: '困倦', bored: '无聊', scared: '害怕',
  celebrate: '庆祝', approving: '认可', orbit: '环绕', radar: '雷达', progress: '进度',
  spawning: '生成', loading: '加载', dictating: '倾听', sending: '发送', receiving: '接收',
  uploading: '上传', writing: '书写', alerting: '提醒', bouncing: '弹跳',
  'powering-down': '关机', humming: '嗡鸣', notifying: '通知', dragging: '拖动',
  logo: '标志', showcase: '展示', turntable: '转一圈',
} as const
export type BotActivity = keyof typeof BOT_ACTIVITIES

const SEARCH_TOOLS = new Set(['web_search', 'skills_list', 'search_conversation_history', 'list_mcp_connections', 'list_mcp_resources', 'query_memory', 'list_saved_maps'])
const READ_TOOLS = new Set(['web_extract', 'read_mcp_resource', 'read_mcp_prompt', 'get_mcp_prompt', 'read_assistant_memory', 'read_conversation_turn', 'inspect_saved_map', 'view_saved_map', 'read_robot_sensor', 'inspect_lidar', 'inspect_navigation', 'capture_camera', 'view_live_camera'])
const CODE_TOOLS = new Set(['run_python', 'preview_map_edits'])
const CONTROL_TOOLS = new Set(['navigate_to', 'navigate_to_object', 'navigate_to_deg', 'find_object', 'detect_object', 'save_map_copy', 'set_navigation_region', 'save_inspection_point'])

export function botActivityForTool(toolName: string | undefined): BotActivity | undefined {
  if (!toolName) return undefined
  if (SEARCH_TOOLS.has(toolName)) return 'searching'
  if (READ_TOOLS.has(toolName)) return 'reading'
  if (CODE_TOOLS.has(toolName)) return 'coding'
  if (CONTROL_TOOLS.has(toolName)) return 'controlling'
  return 'working'
}

export const BOT_ACTIVITY_GROUPS = [
  { label: '工作状态', keys: ['idle', 'listening', 'thinking', 'searching', 'reading', 'working', 'coding', 'running', 'computing', 'controlling', 'waiting', 'blocked', 'finished', 'failed', 'sleeping', 'waking'] },
  { label: '表情动作', keys: ['happy', 'excited', 'laughing', 'proud', 'curious', 'playful', 'shy', 'surprised', 'suspicious', 'angry', 'confused', 'sad', 'drowsy', 'bored', 'scared', 'celebrate', 'approving'] },
  { label: '活动效果', keys: ['orbit', 'radar', 'progress', 'spawning', 'loading', 'dictating', 'sending', 'receiving', 'uploading', 'writing', 'alerting', 'bouncing', 'powering-down', 'humming', 'notifying', 'dragging', 'logo', 'showcase', 'turntable'] },
] as const satisfies ReadonlyArray<{ label: string; keys: readonly BotActivity[] }>
