export const BOT_EXPRESSIONS = {
  neutral: '平静', happy: '开心', surprised: '惊讶', sad: '难过', sleepy: '困倦',
  squinted: '眯眼', calm: '放松', inspecting: '好奇', confused: '困惑',
} as const
export type BotExpression = keyof typeof BOT_EXPRESSIONS

export const BOT_ACTIVITIES = {
  orbit: '环绕', radar: '检索', progress: '进度', writing: '书写', sending: '发送',
  receiving: '接收', uploading: '上传', dictating: '倾听', alerting: '提醒',
  bouncing: '弹跳', turntable: '转一圈', showcase: '展示',
} as const
export type BotActivity = keyof typeof BOT_ACTIVITIES
