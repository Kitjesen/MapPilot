import { BOT_SHAPES } from './botShapes.ts'
export { BOT_SHAPES } from './botShapes.ts'

export const BOT_COLORS = {
  graphite: { label: '石墨', fill: '#20252b', eyes: '#f4f7fb' },
  blue: { label: '雾蓝', fill: '#4c9de8', eyes: '#17324b' },
  sage: { label: '苔绿', fill: '#49b86e', eyes: '#173d24' },
  violet: { label: '浅紫', fill: '#8b5cf6', eyes: '#25134d' },
  coral: { label: '珊瑚', fill: '#ef476f', eyes: '#541322' },
  peach: { label: '蜜桃', fill: '#f28f3b', eyes: '#55250d' },
  honey: { label: '蜂蜜', fill: '#d9a514', eyes: '#4a3505' },
  mint: { label: '薄荷', fill: '#21b08a', eyes: '#07382c' },
  sky: { label: '晴空', fill: '#36a9d6', eyes: '#082f44' },
  rose: { label: '玫瑰', fill: '#d64f9b', eyes: '#421231' },
  indigo: { label: '鸢尾', fill: '#5b65db', eyes: '#171d59' },
  pearl: { label: '珍珠', fill: '#d3d4c6', eyes: '#30332d' },
  aqua: { label: '湖蓝', fill: '#43c7c9', eyes: '#07383a' },
  lilac: { label: '丁香', fill: '#a678f2', eyes: '#241247' },
} as const

export interface BotProfile {
  id: string
  name: string
  shape: keyof typeof BOT_SHAPES
  color: keyof typeof BOT_COLORS
  size: 48 | 64
  instructions: string
  role?: string
  capabilities?: string[]
}
export const DEFAULT_BOT_PROFILE: BotProfile = { id: 'lingtu-default', name: 'LingTu', shape: 'orb', color: 'graphite', size: 48, instructions: '', role: '通用助手', capabilities: [] }
export const BOT_PROFILE_KEY = 'lingtu.bot-profile.v1'
export const BOT_ROSTER_KEY = 'lingtu.bot-roster.v1'
export const CURRENT_BOT_KEY = 'lingtu.bot-current.v1'

export const PRESET_BOT_PROFILES: readonly BotProfile[] = [
  { id: 'inspection', name: '现场巡检', shape: 'grok-blob', color: 'blue', size: 48, role: '巡检与证据', instructions: '关注现场巡检、异常记录和证据整理。', capabilities: ['巡检', '证据', '报告'] },
  { id: 'navigation', name: '地图导航', shape: 'grok-pebble', color: 'aqua', size: 48, role: '地图与导航', instructions: '关注地图、定位、路径规划和导航状态。', capabilities: ['地图', '定位', '导航'] },
  { id: 'maintenance', name: '设备维护', shape: 'grok-hex', color: 'honey', size: 48, role: '设备与传感器', instructions: '关注设备状态、传感器诊断和维护记录。', capabilities: ['设备', '传感器', '维护'] },
]

function hasStorage() { return typeof localStorage !== 'undefined' }

function legacyProfile(): BotProfile | null {
  if (!hasStorage()) return null
  try {
    const raw = localStorage.getItem(BOT_PROFILE_KEY)
    return raw ? validateBotProfileWithFallback(JSON.parse(raw), DEFAULT_BOT_PROFILE.id) : null
  } catch { return null }
}

export function validateBotProfile(value: unknown): BotProfile {
  return validateBotProfileWithFallback(value, DEFAULT_BOT_PROFILE.id)
}

function validateBotProfileWithFallback(value: unknown, fallbackId: string): BotProfile {
  const p = value as Partial<BotProfile> | null
  if (!p || typeof p.name !== 'string' || !p.name.trim() || p.name.trim().length > 32
    || !Object.hasOwn(BOT_SHAPES, String(p.shape))
    || !Object.hasOwn(BOT_COLORS, String(p.color))
    || (p.size != null && p.size !== 48 && p.size !== 64)
    || typeof p.instructions !== 'string' || p.instructions.length > 1600
    || (p.id != null && (typeof p.id !== 'string' || !/^[a-z0-9][a-z0-9-]{0,63}$/.test(p.id)))) throw new Error('请检查名字、形象和角色说明')
  return {
    id: typeof p.id === 'string' && p.id.trim() ? p.id.trim() : fallbackId,
    name: p.name.trim(), shape: p.shape!, color: p.color!, size: p.size ?? 48,
    instructions: p.instructions.trim(), role: typeof p.role === 'string' ? p.role.trim().slice(0, 80) : undefined,
    capabilities: Array.isArray(p.capabilities) ? p.capabilities.filter(item => typeof item === 'string').map(item => item.trim()).filter(Boolean).slice(0, 12) : undefined,
  }
}

export function readBotRoster(): BotProfile[] {
  if (!hasStorage()) return PRESET_BOT_PROFILES.map(profile => ({ ...profile, capabilities: [...(profile.capabilities ?? [])] }))
  try {
    const raw = localStorage.getItem(BOT_ROSTER_KEY)
    if (raw) {
      const parsed = JSON.parse(raw)
      if (Array.isArray(parsed) && parsed.length > 0) return parsed.map(item => validateBotProfileWithFallback(item, DEFAULT_BOT_PROFILE.id))
    }
    const old = legacyProfile()
    const hasLegacyConversation = typeof location !== 'undefined'
      && Boolean(localStorage.getItem(`lingtu.agent.conversation:${location.origin}`))
    const custom = old && (old.name !== DEFAULT_BOT_PROFILE.name || old.instructions || old.shape !== DEFAULT_BOT_PROFILE.shape || old.color !== DEFAULT_BOT_PROFILE.color)
      ? [{ ...old, id: DEFAULT_BOT_PROFILE.id }]
      : hasLegacyConversation ? [{ ...DEFAULT_BOT_PROFILE }] : []
    const roster = [...custom, ...PRESET_BOT_PROFILES.filter(profile => !custom.some(item => item.id === profile.id))]
    localStorage.setItem(BOT_ROSTER_KEY, JSON.stringify(roster))
    return roster.map(profile => ({ ...profile, capabilities: [...(profile.capabilities ?? [])] }))
  } catch { return PRESET_BOT_PROFILES.map(profile => ({ ...profile, capabilities: [...(profile.capabilities ?? [])] })) }
}

export function saveBotRoster(value: unknown): BotProfile[] {
  if (!Array.isArray(value) || value.length === 0) throw new Error('Bot 列表不能为空')
  const roster = value.map(item => validateBotProfile(item))
  const ids = new Set(roster.map(profile => profile.id))
  if (ids.size !== roster.length) throw new Error('Bot ID 不能重复')
  if (hasStorage()) localStorage.setItem(BOT_ROSTER_KEY, JSON.stringify(roster))
  return roster
}

export function readCurrentBotId(): string {
  const roster = readBotRoster()
  if (hasStorage()) {
    const id = localStorage.getItem(CURRENT_BOT_KEY)
    if (id && roster.some(profile => profile.id === id)) return id
  }
  return roster[0]?.id ?? DEFAULT_BOT_PROFILE.id
}

export function setCurrentBotId(id: string): BotProfile {
  const profile = readBotRoster().find(item => item.id === id)
  if (!profile) throw new Error('Bot 不存在')
  if (hasStorage()) {
    localStorage.setItem(CURRENT_BOT_KEY, profile.id)
    localStorage.setItem(BOT_PROFILE_KEY, JSON.stringify(profile))
  }
  return profile
}

export const readActiveBotId = readCurrentBotId
export const saveActiveBotId = setCurrentBotId

export function readBotProfile(botId?: string): BotProfile {
  if (botId === undefined && hasStorage()) {
    const raw = localStorage.getItem(BOT_PROFILE_KEY)
    if (raw) {
      try { return validateBotProfileWithFallback(JSON.parse(raw), DEFAULT_BOT_PROFILE.id) }
      catch { return { ...DEFAULT_BOT_PROFILE } }
    }
  }
  const id = botId ?? readCurrentBotId()
  return readBotRoster().find(profile => profile.id === id) ?? { ...DEFAULT_BOT_PROFILE }
}

export function saveBotProfile(value: unknown): BotProfile {
  const profile = validateBotProfile(value)
  const roster = readBotRoster()
  const next = roster.some(item => item.id === profile.id)
    ? roster.map(item => item.id === profile.id ? profile : item)
    : [...roster, profile]
  if (hasStorage()) {
    localStorage.setItem(BOT_PROFILE_KEY, JSON.stringify(profile))
    localStorage.setItem(BOT_ROSTER_KEY, JSON.stringify(next))
  }
  return profile
}
export function botPersona(profile: BotProfile): string {
  if (profile.name === DEFAULT_BOT_PROFILE.name && !profile.instructions) return ''
  return `称呼：${profile.name}。\n用户设置的角色与回复偏好：${profile.instructions || '遵循当前对话要求。'}`
}
