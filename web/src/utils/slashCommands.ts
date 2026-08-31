// Slash command parser, executor, and autocomplete registry.
// Extracts command handling logic from ChatPanel.tsx and adds a
// metadata table that powers the /-command dropdown (Claude-Code style).

import type { PlanPreviewResponse, SSEState } from '../types'
import * as api from '../services/api'

const NAV_STATE_ZH: Record<string, string> = {
  PLANNING:  '规划路线中',
  EXECUTING: '导航中',
  ARRIVED:   '已到达目的地。',
  IDLE:      '导航空闲。',
  FAILED:    '导航失败。',
  CANCELLED: '导航已取消。',
}

// ── Command registry — single source of truth for dropdown + help ──

export interface SlashCommandSpec {
  /** Command trigger including the leading slash, e.g. '/go' */
  name: string
  /** Full argument template shown in the dropdown, e.g. '/go <x> <y>' */
  usage: string
  /** One-line description in Chinese */
  description: string
  /** Optional concrete example */
  example?: string
  /** Optional emoji-free single-letter category tag */
  group?: '导航' | '系统' | '地图'
}

export const SLASH_COMMANDS: SlashCommandSpec[] = [
  {
    name: '/go',
    usage: '/go <x> <y>',
    description: '发送导航目标到指定世界坐标',
    example: '/go 3.5 -1.2',
    group: '导航',
  },
  {
    name: '/stop',
    usage: '/stop',
    description: '立即停止机器人并取消当前导航',
    group: '导航',
  },
  {
    name: '/status',
    usage: '/status',
    description: '查看当前位置、导航状态、安全状态',
    group: '系统',
  },
  {
    name: '/map',
    usage: '/map',
    description: '跳转到地图管理（也可直接点击 "地图" 标签）',
    group: '地图',
  },
  {
    name: '/help',
    usage: '/help',
    description: '显示全部可用指令',
    group: '系统',
  },
]

const HELP_TEXT =
  '可用指令：\n' +
  SLASH_COMMANDS.map(c => `${c.usage.padEnd(16)} — ${c.description}`).join('\n')

function formatPlanSummary(preview: PlanPreviewResponse | null | undefined): string {
  if (!preview) return ''
  return [
    preview.planner ? `planner=${preview.planner}` : null,
    `points=${preview.count}`,
  ].filter((v): v is string => Boolean(v)).join(' | ')
}

function formatPlanPreviewFailure(
  preview: PlanPreviewResponse | null | undefined,
  reasons: string[] = [],
  error?: string | null,
): string {
  const safety = formatPlanSummary(preview)
  const reason = reasons.slice(0, 3).join(' / ') || error || preview?.error || '目标预检未通过'
  return safety ? `${reason} (${safety})` : reason
}

/**
 * Return commands whose name matches the user's current partial input.
 * "fuzzy" means: substring match on the command name, case-insensitive.
 * Typing nothing ('/') returns all commands.
 */
export function matchSlashCommands(partial: string): SlashCommandSpec[] {
  const q = partial.trim().toLowerCase()
  if (!q.startsWith('/')) return []
  const needle = q.slice(1)
  if (needle.length === 0) return SLASH_COMMANDS
  return SLASH_COMMANDS.filter(c =>
    c.name.slice(1).toLowerCase().includes(needle)
  )
}

export interface SlashCommandResult {
  command: string
  response: string
}

export function isSlashCommand(input: string): boolean {
  return input.trimStart().startsWith('/')
}

export async function executeSlashCommand(raw: string, sseState: SSEState): Promise<string> {
  const parts = raw.trim().split(/\s+/)
  const cmd = parts[0].toLowerCase()

  if (cmd === '/help') {
    return HELP_TEXT
  }

  if (cmd === '/status') {
    const ms = sseState.missionStatus
    const odom = sseState.odometry
    const safety = sseState.safetyState
    const stateZh = ms ? (NAV_STATE_ZH[ms.state] ?? ms.state) : '未知'
    const pos =
      typeof odom?.x === 'number' && typeof odom?.y === 'number'
        ? `(${odom.x.toFixed(2)}, ${odom.y.toFixed(2)})`
        : '--'
    const estop = safety?.estop ? '急停激活' : '正常'
    return `状态：${stateZh}\n位置：${pos}\n安全：${estop}`
  }

  if (cmd === '/map') {
    return '地图管理请切换到地图标签。'
  }

  if (cmd === '/stop') {
    try {
      const res = await api.sendStop()
      return api.formatCommandAck(res, '急停指令')
    } catch (e: unknown) {
      return api.formatCommandError(e, '急停失败')
    }
  }

  if (cmd === '/go') {
    const x = parseFloat(parts[1])
    const y = parseFloat(parts[2])
    if (isNaN(x) || isNaN(y)) {
      return '用法：/go <x> <y>  （示例：/go 3.5 -1.2）'
    }
    try {
      const candidate = await api.constructGoalCandidate({
        x,
        y,
        source: 'coordinate',
        target_type: 'coordinate',
        label: 'slash_go',
      })
      if (!candidate.ok || candidate.preview?.feasible === false) {
        const reason = formatPlanPreviewFailure(
          candidate.preview,
          candidate.reasons,
          candidate.error,
        )
        return `目标不可达：${reason}`
      }
      const res = await api.sendGoal(x, y, {
        source: 'coordinate',
        target_type: 'coordinate',
        label: 'slash_go',
      })
      return api.formatCommandAck(res, `目标 (${x}, ${y})`)
    } catch (e: unknown) {
      return api.formatCommandError(e, '导航失败')
    }
  }

  return `未知指令：${cmd}。输入 /help 查看可用指令。`
}
