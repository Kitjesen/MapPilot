import { BOT_SHAPES } from './botShapes.ts'
export { BOT_SHAPES } from './botShapes.ts'

export const BOT_COLORS = {
  graphite: { label: '石墨', fill: '#9aa39c', eyes: '#26352d' },
  blue: { label: '雾蓝', fill: '#88add4', eyes: '#23374f' },
  sage: { label: '苔绿', fill: '#a5bda3', eyes: '#2a3e2d' },
  violet: { label: '浅紫', fill: '#b6a1d1', eyes: '#392849' },
  coral: { label: '珊瑚', fill: '#e8a09c', eyes: '#602e34' },
  peach: { label: '蜜桃', fill: '#edbb9a', eyes: '#57372a' },
  honey: { label: '蜂蜜', fill: '#e7c779', eyes: '#4c3b1a' },
  mint: { label: '薄荷', fill: '#91c7b7', eyes: '#22483e' },
  sky: { label: '晴空', fill: '#91c7df', eyes: '#234553' },
  rose: { label: '玫瑰', fill: '#dba5c1', eyes: '#522c43' },
  indigo: { label: '鸢尾', fill: '#a0a8d9', eyes: '#2e3459' },
  pearl: { label: '珍珠', fill: '#dfded7', eyes: '#44453a' },
} as const

export interface BotProfile {
  name: string
  shape: keyof typeof BOT_SHAPES
  color: keyof typeof BOT_COLORS
  size: 48 | 64
  instructions: string
}
export const DEFAULT_BOT_PROFILE: BotProfile = { name: 'LingTu', shape: 'orb', color: 'graphite', size: 48, instructions: '' }
export const BOT_PROFILE_KEY = 'lingtu.bot-profile.v1'
export function validateBotProfile(value: unknown): BotProfile {
  const p = value as Partial<BotProfile> | null
  if (!p || typeof p.name !== 'string' || !p.name.trim() || p.name.trim().length > 32
    || !Object.hasOwn(BOT_SHAPES, String(p.shape))
    || !Object.hasOwn(BOT_COLORS, String(p.color))
    || (p.size != null && p.size !== 48 && p.size !== 64)
    || typeof p.instructions !== 'string' || p.instructions.length > 1600) throw new Error('请检查名字、形象和角色说明')
  return { name: p.name.trim(), shape: p.shape!, color: p.color!, size: p.size ?? 48, instructions: p.instructions.trim() }
}
export function readBotProfile(): BotProfile {
  try {
    const raw = localStorage.getItem(BOT_PROFILE_KEY)
    return raw ? validateBotProfile(JSON.parse(raw)) : { ...DEFAULT_BOT_PROFILE }
  } catch { return { ...DEFAULT_BOT_PROFILE } }
}
export function saveBotProfile(value: unknown): BotProfile {
  const profile = validateBotProfile(value)
  localStorage.setItem(BOT_PROFILE_KEY, JSON.stringify(profile))
  return profile
}
export function botPersona(profile: BotProfile): string {
  if (profile.name === DEFAULT_BOT_PROFILE.name && !profile.instructions) return ''
  return `称呼：${profile.name}。\n用户设置的角色与回复偏好：${profile.instructions || '遵循当前对话要求。'}`
}
